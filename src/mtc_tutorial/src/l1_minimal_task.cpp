// ============================================================
// Level 1：最小 MTC 任务
// 结构：CurrentState(Generator) + MoveTo(Propagator)
// 功能：读取 Panda 当前关节状态，规划一条到 SRDF 命名位姿
//       "ready" 的轨迹，并把解发布给 RViz 查看（不执行）
// ============================================================

#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

namespace mtc = moveit::task_constructor;

// 把任务构建封装成函数：Task 的"定义"与 ROS 节点生命周期解耦
mtc::Task createTask(const rclcpp::Node::SharedPtr& node)
{
  mtc::Task task;
  task.stages()->setName("l1 最小任务");

  // 加载机器人模型（读取 /robot_description 参数），
  // 所有 Stage 内部做 IK、碰撞检测都依赖这个模型
  task.loadRobotModel(node);

  // ---- Stage 1: CurrentState (Generator) ----
  // 从 move_group 监控的规划场景中读取"当前关节状态"，
  // 生成一个 InterfaceState，作为整条链路的起点
  auto current = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(current));

  // ---- Stage 2: MoveTo (Propagator) ----
  // 规划器（solver）决定"怎么算"：这里用 OMPL 的 RRTConnect
  // 它消费 CurrentState 产出的状态，计算到目标位姿的轨迹
  auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
  auto move = std::make_unique<mtc::stages::MoveTo>("move to ready", pipeline);
  move->setGroup("panda_arm");  // SRDF 中定义的规划组
  move->setGoal("ready");       // SRDF 中预定义的命名位姿
  task.add(std::move(move));

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("l1_minimal_task");

  try
  {
    auto task = createTask(node);

    // init()：连接相邻 Stage 的接口，检查属性配置是否完整
    // 接线错误（如 Propagator 前面没有状态来源）会在这里抛异常
    task.init();

    // plan()：让整个任务树开始求解，最多保留 5 个解
    if (task.plan(5))
    {
      RCLCPP_INFO(node->get_logger(), "规划成功，共 %zu 个解，最优代价 %.3f", task.numSolutions(),
                  task.solutions().front()->cost());

      // 把最优解发布到 introspection 话题，
      // RViz 的 "Motion Planning Tasks" 面板可以逐阶段回放
      task.introspection().publishSolution(*task.solutions().front());
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "规划失败：没有任何一个 Stage 算出可行解");
    }
  }
  catch (const mtc::InitStageException& e)
  {
    RCLCPP_ERROR(node->get_logger(), "任务初始化失败: %s", e.what());
  }

  // 保持节点存活：RViz 通过该节点提供的 introspection 服务回看解
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
