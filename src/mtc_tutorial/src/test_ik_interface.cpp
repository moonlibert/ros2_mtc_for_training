// 测试：CurrentState → add object → Connect → Fallbacks(ComputeIK x2) → open hand
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

namespace mtc = moveit::task_constructor;

moveit_msgs::msg::CollisionObject makeBox()
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = "object"; obj.header.frame_id = "world";
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.10};
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.45; pose.position.y = 0.0; pose.position.z = 0.05;
  pose.orientation.w = 1.0;
  obj.primitives.push_back(box);
  obj.primitive_poses.push_back(pose);
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  return obj;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_ik");
  std::thread spin([node] { rclcpp::spin(node); });

  mtc::Task task;
  task.stages()->setName("test");
  task.loadRobotModel(node);

  auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
  auto jip = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cart = std::make_shared<mtc::solvers::CartesianPath>();

  auto root = std::make_unique<mtc::SerialContainer>("root");
  mtc::Stage* current = nullptr;
  {
    auto s = std::make_unique<mtc::stages::CurrentState>("current");
    current = s.get();
    root->add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("add object");
    s->addObject(makeBox());
    root->add(std::move(s));
  }
  {
    std::vector<std::pair<std::string, mtc::solvers::PlannerInterfacePtr>> planners;
    planners.push_back({"panda_arm", ompl});
    root->add(std::make_unique<mtc::stages::Connect>("connect", planners));
  }
  // Fallbacks: 两个高度的 ComputeIK
  {
    auto fb = std::make_unique<mtc::Fallbacks>("pre-grasp alternatives");
    for (double z : {0.32, 0.28}) {
      auto gen = std::make_unique<mtc::stages::GeneratePose>("gen " + std::to_string(z));
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "world";
      ps.pose.position.x = 0.45; ps.pose.position.y = 0.0; ps.pose.position.z = z;
      ps.pose.orientation.x = 1.0; ps.pose.orientation.w = 0.0;
      gen->properties().set("pose", ps);
      gen->setMonitoredStage(current);
      auto ik = std::make_unique<mtc::stages::ComputeIK>("ik " + std::to_string(z), std::move(gen));
      ik->setGroup("panda_arm");
      ik->setIKFrame("panda_link8");
      ik->setMaxIKSolutions(1);
      fb->add(std::move(ik));
    }
    root->add(std::move(fb));
  }
  {
    auto s = std::make_unique<mtc::stages::MoveTo>("open hand", jip);
    s->setGroup("hand"); s->setGoal("open");
    root->add(std::move(s));
  }
  {
    auto s = std::make_unique<mtc::stages::MoveRelative>("approach", cart);
    s->setGroup("panda_arm"); s->setIKFrame("panda_link8");
    geometry_msgs::msg::Vector3Stamped d; d.header.frame_id = "world"; d.vector.z = -0.12;
    s->setDirection(d); s->setMinDistance(0.12); s->setMaxDistance(0.12);
    root->add(std::move(s));
  }
  task.add(std::move(root));

  try {
    task.init();
    RCLCPP_INFO(node->get_logger(), "init OK");
    auto ok = task.plan(5);
    RCLCPP_INFO(node->get_logger(), "plan: %s, solutions=%zu", ok?"success":"FAIL", task.numSolutions());
  } catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "init FAIL: " << e);
  }

  rclcpp::shutdown();
  if (spin.joinable()) spin.join();
  return 0;
}
