// ============================================================
// Level 4：进阶抓取 —— 自定义 Stage + 属性传播 + Fallbacks 备选
// 新知识点：
//   1. 自定义 Stage：继承 MonitoringGenerator，自己生成解
//   2. 属性传播（Property Propagation）：上游 Stage 设置 "target_pose" 属性，
//      下游 ComputeIK 自动读取该属性做 IK 求解
//   3. Fallbacks 容器：按顺序尝试多个备选方案，第一个成功就用它
//   4. ComputeIK：把笛卡尔目标位姿（属性）反解成关节角
//
// ⚠️ 关键接口规则（踩坑总结）：
//   ComputeIK 包裹的是 Generator（接口 ← →），所以它只能放在
//   Connector（Connect）之后，不能直接跟在 Propagator（MoveTo 等）后面。
//   因为 Propagator 之后的外部接口是 → →，和 Generator 的 ← → 不匹配。
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <deque>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace mtc = moveit::task_constructor;

// ---------------- 工具函数 ----------------

moveit_msgs::msg::CollisionObject makeBox(const std::string& id, double x, double y, double z)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = "world";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.10};

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

// 夹爪朝下的四元数（绕 X 转 180°）
geometry_msgs::msg::Quaternion graspOrient()
{
  geometry_msgs::msg::Quaternion q;
  q.x = 1.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 0.0;
  return q;
}

// ============================================================
// 自定义 Stage：GraspPoseGenerator
// 继承 MonitoringGenerator，监听某个上游 Stage 的解，
// 在其场景基础上生成抓取目标位姿。
//
// 关键点：把位姿作为 "target_pose" 属性写到 InterfaceState 上。
//         下游的 ComputeIK 会自动读取这个属性做 IK 求解。
//         这就是「属性传播」。
// ============================================================
class GraspPoseGenerator : public mtc::MonitoringGenerator
{
public:
  GraspPoseGenerator(const std::string& name, double x, double y, double z,
                     const geometry_msgs::msg::Quaternion& orient)
    : mtc::MonitoringGenerator(name), x_(x), y_(y), z_(z), orient_(orient)
  {
    // 声明本 Stage 会产出的属性（下游 ComputeIK 按名字 "target_pose" 读取）
    auto& p = properties();
    p.declare<geometry_msgs::msg::PoseStamped>("target_pose", "grasp target pose");
  }

  void reset() override
  {
    upstream_solutions_.clear();
    mtc::MonitoringGenerator::reset();
  }

  // 还有没有未处理的上游解？
  bool canCompute() const override { return !upstream_solutions_.empty(); }

  // 核心：生成一个 InterfaceState，带上 target_pose 属性
  void compute() override
  {
    if (upstream_solutions_.empty())
      return;

    const mtc::SolutionBase* s = upstream_solutions_.front();
    upstream_solutions_.pop_front();

    // 构造目标位姿
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "world";
    ps.pose.position.x = x_;
    ps.pose.position.y = y_;
    ps.pose.position.z = z_;
    ps.pose.orientation = orient_;

    // 继承上游状态的场景（机器人构型 + 碰撞物体），
    // 在其上设置 target_pose 属性，然后 spawn 出去
    mtc::InterfaceState state(s->end()->scene());
    state.properties().set("target_pose", ps);
    spawn(std::move(state), 0.0);
  }

protected:
  // 被监听的 Stage 每产出一个解，就回调这里
  void onNewSolution(const mtc::SolutionBase& s) override
  {
    upstream_solutions_.push_back(&s);
  }

private:
  double x_, y_, z_;
  geometry_msgs::msg::Quaternion orient_;
  std::deque<const mtc::SolutionBase*> upstream_solutions_;
};

// ============================================================
// 构建任务
// ============================================================
mtc::Task createTask(const rclcpp::Node::SharedPtr& node)
{
  mtc::Task task;
  task.stages()->setName("l4 advanced grasp");
  task.loadRobotModel(node);

  auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  auto joint_interp = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  const double pick_x = 0.45, pick_y = 0.0;
  const double place_x = 0.30, place_y = 0.30;
  const double obj_z = 0.05;
  const double approach_dist = 0.12;

  // 两个备选预抓取高度
  const double pregrasp_high = 0.32;
  const double pregrasp_low = 0.28;

  auto root = std::make_unique<mtc::SerialContainer>("pick and place");

  // ---- 1. 起点 ----
  mtc::Stage* add_object_ptr = nullptr;
  root->add(std::make_unique<mtc::stages::CurrentState>("current"));
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("add object");
    stage->addObject(makeBox("object", pick_x, pick_y, obj_z));
    add_object_ptr = stage.get();
    root->add(std::move(stage));
  }

  // ---- 2. Connect：把当前状态和后面的 IK 解连接起来 ----
  // 这里必须用 Connect（连接器），因为后面紧跟的是 ComputeIK（包裹 Generator）。
  // Connect 的接口是 CONNECT，它后面的外部接口才是 GENERATE，能匹配 Generator。
  {
    std::vector<std::pair<std::string, mtc::solvers::PlannerInterfacePtr>> planners;
    planners.push_back({"panda_arm", ompl});
    root->add(std::make_unique<mtc::stages::Connect>("connect to pre-grasp", planners));
  }

  // ---- 3. Fallbacks：先试高位预抓取，失败再试低位 ----
  // Fallbacks 按顺序尝试子 Stage，第一个成功的就采用，后面的不再执行。
  // 每个备选是一个 ComputeIK，包裹自定义的 GraspPoseGenerator。
  {
    auto fallbacks = std::make_unique<mtc::Fallbacks>("pre-grasp alternatives");

    // 备选 1：高位预抓取
    {
      auto gen = std::make_unique<GraspPoseGenerator>("gen high pose", pick_x, pick_y,
                                                       pregrasp_high, graspOrient());
      // 监听 "add object" 阶段，拿到包含物体的场景
      gen->setMonitoredStage(add_object_ptr);
      auto ik = std::make_unique<mtc::stages::ComputeIK>("ik high", std::move(gen));
      ik->setGroup("panda_arm");
      ik->setIKFrame("panda_link8");
      ik->setMaxIKSolutions(1);
      fallbacks->add(std::move(ik));
    }

    // 备选 2：低位预抓取（高位 IK 无解或碰撞时才会用到）
    {
      auto gen = std::make_unique<GraspPoseGenerator>("gen low pose", pick_x, pick_y,
                                                       pregrasp_low, graspOrient());
      gen->setMonitoredStage(add_object_ptr);
      auto ik = std::make_unique<mtc::stages::ComputeIK>("ik low", std::move(gen));
      ik->setGroup("panda_arm");
      ik->setIKFrame("panda_link8");
      ik->setMaxIKSolutions(1);
      fallbacks->add(std::move(ik));
    }

    root->add(std::move(fallbacks));
  }

  // ---- 4. 张开手 ----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", joint_interp);
    stage->setGroup("hand");
    stage->setGoal("open");
    root->add(std::move(stage));
  }

  // ---- 5. Pick 子容器 ----
  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick");

    // 允许手指和物体碰撞（必须在 approach 之前，否则下降时手指撞物体）
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision");
      const std::vector<std::string> finger_links = {"panda_leftfinger", "panda_rightfinger", "panda_hand"};
      stage->allowCollisions("object", finger_links, true);
      pick->add(std::move(stage));
    }
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
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", joint_interp);
      stage->setGroup("hand");
      stage->setGoal("close");
      pick->add(std::move(stage));
    }
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", "panda_hand");
      pick->add(std::move(stage));
    }
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

  // ---- 6. 搬到放置点上方 ----
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("to pre-place", ompl);
    stage->setGroup("panda_arm");
    stage->setIKFrame("panda_link8");
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "world";
    ps.pose.position.x = place_x;
    ps.pose.position.y = place_y;
    ps.pose.position.z = pregrasp_high;
    ps.pose.orientation = graspOrient();
    stage->setGoal(ps);
    root->add(std::move(stage));
  }

  // ---- 7. Place 子容器 ----
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

  // ---- 8. 回 ready ----
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
  auto node = rclcpp::Node::make_shared("l4_advanced_grasp");

  std::thread spinning_thread([node] { rclcpp::spin(node); });
  auto task = createTask(node);

  try
  {
    task.init();

    if (!task.plan(5))
    {
      RCLCPP_ERROR(node->get_logger(), "规划失败，各 Stage 状态：");
      task.stages()->traverseChildren(
          [&node](const mtc::Stage& s, unsigned int /*depth*/) -> bool {
            RCLCPP_ERROR(node->get_logger(), "  Stage '%s': %s", s.name().c_str(),
                         s.solutions().empty() ? "无解" : "有解");
            return true;
          });
      rclcpp::shutdown();
      if (spinning_thread.joinable())
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
    RCLCPP_ERROR_STREAM(node->get_logger(), "任务初始化失败: " << e);
  }

  rclcpp::shutdown();
  if (spinning_thread.joinable())
    spinning_thread.join();
  return 0;
}
