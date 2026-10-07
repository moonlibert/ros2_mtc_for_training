// ============================================================
// Level 1：最小 MTC 任务
// 结构：CurrentState(Generator) + MoveTo(Propagator)
// 功能：读取 Panda 当前关节状态，规划一条到 SRDF 命名位姿
//       "ready" 的轨迹，并把解发布给 RViz 查看（不执行）
// ============================================================

#include <rclcpp/rclcpp.hpp>

#include <thread>

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

  // 关键：规划前就在独立线程中 spin 主节点。
  // plan() 期间 OMPL 规划管线会向主节点注册发布器，CurrentState 也要靠
  // 它接收 planning scene / joint_states，主节点必须保持事件循环运转
  std::thread spinning_thread([node] { rclcpp::spin(node); });

  // 关键：Task 生命周期必须覆盖到进程结束——
  // introspection 节点/话题随 Task 析构而注销，
  // 若 task 死在 spin 之前，RViz 将永远看不到任务树
  auto task = createTask(node);

  try
  {
    task.init();

    if (task.plan(5))
    {
      RCLCPP_INFO(node->get_logger(), "规划成功，共 %zu 个解，最优代价 %.3f", task.numSolutions(),
                  task.solutions().front()->cost());

      task.introspection().publishSolution(*task.solutions().front());
      task.execute(*task.solutions().front());
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

  // 此时 task 仍然活着，RViz 随时可以来回看解
  spinning_thread.join();
  rclcpp::shutdown();
  return 0;
}
