// ============================================================
// Level 2：完整动作链 + 真执行
// 结构：显式 SerialContainer 串起 6 个 Stage
//   张手 → 走到 ready → 下探 15cm → 合手 → 抬起 15cm
// 新知识点：
//   1. SerialContainer 显式串行容器（L1 的 task.add 是隐式容器）
//   2. MoveRelative：笛卡尔空间的相对移动（走直线）
//   3. 三种规划器（solver）的分工
//   4. task.execute()：把解发给 move_group 真执行
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <thread>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>  // SerialContainer
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>      // OMPL
#include <moveit/task_constructor/solvers/cartesian_path.h>        // 笛卡尔直线
#include <moveit/task_constructor/solvers/joint_interpolation.h>   // 关节插值

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace mtc = moveit::task_constructor;

mtc::Task createTask(const rclcpp::Node::SharedPtr& node)
{
  mtc::Task task;
  task.stages()->setName("l2 夹取动作链");
  task.loadRobotModel(node);

  // ---- 三种规划器（solver）：Stage 决定"做什么"，solver 决定"怎么算" ----
  // OMPL：自由空间大范围运动，会避障、路径随机采样
  auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
  // CartesianPath：末端走严格直线，用于接近/离开这类短距离精确动作
  auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
  // JointInterpolation：关节空间直接插值，简单可靠，夹爪开合专用
  auto joint_interp = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // ---- 显式串行容器：子阶段按添加顺序首尾相接 ----
  // L1 里直接 task.add() 其实是隐式创建了一个串行容器
  auto container = std::make_unique<mtc::SerialContainer>("pick 序列");

  // 1. 起点：读取当前状态
  container->add(std::make_unique<mtc::stages::CurrentState>("current"));

  // 2. 张开夹爪（关节插值，1 个自由度不需要采样规划）
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", joint_interp);
    stage->setGroup("hand");   // 夹爪规划组
    stage->setGoal("open");    // SRDF 命名状态
    container->add(std::move(stage));
  }

  // 3. 走到 ready 位姿（大范围运动，交给 OMPL）
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to ready", ompl);
    stage->setGroup("panda_arm");
    stage->setGoal("ready");
    container->add(std::move(stage));
  }

  // 4. 沿世界系 z 轴下探 15cm（末端走直线）
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian);
    stage->setGroup("panda_arm");
    stage->setIKFrame("panda_link8");  // 以哪个连杆为"末端参考点"
    // 方向向量：frame_id 决定在哪个坐标系里描述运动
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.z = -0.15;  // 向下
    stage->setDirection(dir);
    // 距离窗口：min=max 表示必须走满 15cm，走不到就算失败
    stage->setMinDistance(0.15);
    stage->setMaxDistance(0.15);
    container->add(std::move(stage));
  }

  // 5. 闭合夹爪
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", joint_interp);
    stage->setGroup("hand");
    stage->setGoal("close");
    container->add(std::move(stage));
  }

  // 6. 沿 z 轴抬起 15cm
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian);
    stage->setGroup("panda_arm");
    stage->setIKFrame("panda_link8");
    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = "world";
    dir.vector.z = 0.15;  // 向上
    stage->setDirection(dir);
    stage->setMinDistance(0.15);
    stage->setMaxDistance(0.15);
    container->add(std::move(stage));
  }

  // 整个容器作为一个阶段挂到任务根部
  task.add(std::move(container));
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("l2_grasp_sequence");

  // 规划前就启动 spin：OMPL 管线 / MoveGroup 客户端都需要节点事件循环
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  // task 必须存活到进程结束，否则 introspection 节点提前析构，RViz 看不到任务树
  auto task = createTask(node);

  try
  {
    task.init();

    if (!task.plan(5))
    {
      RCLCPP_ERROR(node->get_logger(), "规划失败：检查每个 Stage 的属性与求解器");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "规划成功，共 %zu 个解", task.numSolutions());
    task.introspection().publishSolution(*task.solutions().front());

    // execute()：把解的各段轨迹通过 action 发给 move_group 执行
    // 底层调用的是 ExecuteTaskSolution action（execute_task_solution_capability）
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

  spinning_thread.join();
  rclcpp::shutdown();
  return 0;
}
