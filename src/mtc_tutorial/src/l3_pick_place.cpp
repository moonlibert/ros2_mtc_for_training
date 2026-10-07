// ============================================================
// Level 3：真正的 Pick & Place
// 完整任务：在场景里放一个方块 → 抓取 → 搬运 → 放置 → 回 ready
// 新知识点：
//   1. ModifyPlanningScene：往场景加物体、附着/分离物体、允许/禁止碰撞
//   2. ComputeIK：把笛卡尔目标位姿翻译成关节角（MoveTo+Pose 内部也用它）
//   3. 嵌套 SerialContainer：pick 子容器 + place 子容器
//   4. 抓取三要素：允许碰撞（夹爪碰物体不报错）+ 附着（物体跟随夹爪）+ 抬起
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace mtc = moveit::task_constructor;

// 构造一个方块碰撞物体（5cm × 5cm × 10cm），放在世界系指定位置
moveit_msgs::msg::CollisionObject makeBox(const std::string& id, double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = "world";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.10};  // x, y, z

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;

  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

// 末端朝下的抓取姿态：panda_link8 绕 x 轴转 180°，使夹爪指向 -z
geometry_msgs::msg::PoseStamped makeGraspPose(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "world";
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.position.z = z;
  ps.pose.orientation.x = 1.0;  // 180° about X → 夹爪朝下
  ps.pose.orientation.y = 0.0;
  ps.pose.orientation.z = 0.0;
  ps.pose.orientation.w = 0.0;
  return ps;
}

mtc::Task createTask(const rclcpp::Node::SharedPtr& node)
{
  mtc::Task task;
  task.stages()->setName("l3 pick and place");
  task.loadRobotModel(node);

  auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  auto joint_interp = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // 抓取点 / 放置点（世界系）
  const double pick_x = 0.45, pick_y = 0.0;
  const double place_x = 0.30, place_y = 0.30;
  const double obj_z = 0.05;        // 方块中心高度
  const double pregrasp_z = 0.30;   // 预抓取高度（夹爪 link8）
  const double approach_dist = 0.12; // 下探距离

  auto root = std::make_unique<mtc::SerialContainer>("pick and place");

  // ---- 1. 起点 ----
  root->add(std::make_unique<mtc::stages::CurrentState>("current"));

  // 先回到 ready，给后续 IK 一个良好的起始构型
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("to ready", ompl);
    stage->setGroup("panda_arm");
    stage->setGoal("ready");
    root->add(std::move(stage));
  }

  // ---- 2. 往场景里加一个方块 ----
  // ModifyPlanningScene 不产生运动，只修改 PlanningScene（碰撞物体、附着关系等）
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("add object");
    stage->addObject(makeBox("object", pick_x, pick_y, obj_z));
    root->add(std::move(stage));
  }

  // ---- 3. 张开夹爪 ----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", joint_interp);
    stage->setGroup("hand");
    stage->setGoal("open");
    root->add(std::move(stage));
  }

  // ---- 4. 走到预抓取位姿（方块上方）----
  // MoveTo + PoseStamped 内部调用 IK 求解器，把笛卡尔位姿翻译成关节角
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("to pre-grasp", ompl);
    stage->setGroup("panda_arm");
    stage->setIKFrame("panda_link8");
    stage->setGoal(makeGraspPose(pick_x, pick_y, pregrasp_z));
    root->add(std::move(stage));
  }

  // ---- 5. Pick 子容器：允许碰撞 → 下探 → 合手 → 附着 → 抬起 ----
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick");

    // 5a. 允许夹爪与方块碰撞（下探时手指会碰到方块，必须先放行）
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
      const std::vector<std::string> finger_links = {"panda_leftfinger", "panda_rightfinger", "panda_hand"};
      stage->allowCollisions("object", finger_links, true);
      pick->add(std::move(stage));
    }

    // 5b. 沿世界 -z 下探（末端走直线）
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian);
      stage->setGroup("panda_arm");
      stage->setIKFrame("panda_link8");
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "world";
      dir.vector.z = -approach_dist;
      stage->setDirection(dir);
      stage->setMinDistance(approach_dist);
      stage->setMaxDistance(approach_dist);
      pick->add(std::move(stage));
    }

    // 5c. 闭合夹爪（抓住方块）
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", joint_interp);
      stage->setGroup("hand");
      stage->setGoal("close");
      pick->add(std::move(stage));
    }

    // 5d. 把方块附着到 panda_hand，之后方块跟随夹爪运动
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", "panda_hand");
      pick->add(std::move(stage));
    }

    // 5e. 抬起
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian);
      stage->setGroup("panda_arm");
      stage->setIKFrame("panda_link8");
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "world";
      dir.vector.z = approach_dist;
      stage->setDirection(dir);
      stage->setMinDistance(approach_dist);
      stage->setMaxDistance(approach_dist);
      pick->add(std::move(stage));
    }

    root->add(std::move(pick));
  }

  // ---- 6. 搬运到放置点上方（带着方块做自由空间避障运动）----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("to pre-place", ompl);
    stage->setGroup("panda_arm");
    stage->setIKFrame("panda_link8");
    stage->setGoal(makeGraspPose(place_x, place_y, pregrasp_z));
    root->add(std::move(stage));
  }

  // ---- 7. Place 子容器：下探 → 张手 → 分离 → 抬起 ----
  {
    auto place = std::make_unique<mtc::SerialContainer>("place");

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian);
      stage->setGroup("panda_arm");
      stage->setIKFrame("panda_link8");
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "world";
      dir.vector.z = -approach_dist;
      stage->setDirection(dir);
      stage->setMinDistance(approach_dist);
      stage->setMaxDistance(approach_dist);
      place->add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", joint_interp);
      stage->setGroup("hand");
      stage->setGoal("open");
      place->add(std::move(stage));
    }

    // 分离方块（不恢复碰撞禁止——手指还在物体旁边，retreat 后才安全）
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", "panda_hand");
      place->add(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian);
      stage->setGroup("panda_arm");
      stage->setIKFrame("panda_link8");
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "world";
      dir.vector.z = approach_dist;
      stage->setDirection(dir);
      stage->setMinDistance(approach_dist);
      stage->setMaxDistance(approach_dist);
      place->add(std::move(stage));
    }

    root->add(std::move(place));
  }

  // ---- 8. 回到 ready ----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return to ready", ompl);
    stage->setGroup("panda_arm");
    stage->setGoal("ready");
    root->add(std::move(stage));
  }

  task.add(std::move(root));
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("l3_pick_place");

  std::thread spinning_thread([node] { rclcpp::spin(node); });

  auto task = createTask(node);

  try
  {
    task.init();

    if (!task.plan(5))
    {
      RCLCPP_ERROR(node->get_logger(), "规划失败，检查各 Stage 状态：");
      // 遍历打印每个 Stage 有无解
      task.stages()->traverseChildren(
          [&node](const mtc::Stage& s, unsigned int /*depth*/) -> bool {
            RCLCPP_ERROR(node->get_logger(), "  Stage '%s': %s", s.name().c_str(),
                         s.solutions().empty() ? "无解" : "有解");
            return true;
          });
      rclcpp::shutdown();
      spinning_thread.join();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "规划成功，共 %zu 个解", task.numSolutions());
    task.introspection().publishSolution(*task.solutions().front());

    RCLCPP_INFO(node->get_logger(), "开始执行 ...");
    auto result = task.execute(*task.solutions().front());
    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      RCLCPP_INFO(node->get_logger(), "执行完成");
    else
      RCLCPP_ERROR(node->get_logger(), "执行失败，错误码 %d", result.val);
  }
  catch (const mtc::InitStageException& e)
  {
    RCLCPP_ERROR(node->get_logger(), "任务初始化失败: %s", e.what());
  }

  rclcpp::shutdown();
  if (spinning_thread.joinable())
    spinning_thread.join();
  return 0;
}
