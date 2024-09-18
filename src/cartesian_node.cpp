#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/move_to.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial2");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
	MTCTaskNode(const rclcpp::NodeOptions& options);

	rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

	void doTask();

	void setupPlanningScene();

private:
	// Compose an MTC task from a series of stages.
	mtc::Task createTask();
	mtc::Task task_;
	rclcpp::Node::SharedPtr node_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  	: node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  	return node_->get_node_base_interface();
}

void MTCTaskNode::setupPlanningScene()
{
	/*	
	*  	--------------------------
	*	Origin One Collision Object
	*	--------------------------
 	*/
	
	// Create collision object, in the way of moving the robot wrt to Origin One
	moveit_msgs::msg::CollisionObject collision_object;
	collision_object.header.frame_id = "base_link";
	// collision_object.header.frame_id = "base_link";
	collision_object.id = "box";

	shape_msgs::msg::SolidPrimitive floor;
	floor.type = floor.BOX;
	floor.dimensions = { 1.8, 1.8, 0.03 };

	geometry_msgs::msg::Pose floor_pose;
	floor_pose.position.x = 0.0;
	floor_pose.position.y = 0.0;
	floor_pose.position.z = -0.21;
	// floor_pose.position.z = 0.08;

	shape_msgs::msg::SolidPrimitive robot;
	robot.type = robot.BOX;
	robot.dimensions = { 0.7, 0.4, 0.25 };

	geometry_msgs::msg::Pose robot_pose;
	robot_pose.position.x = 0.0;
	robot_pose.position.y = 0.0;
	robot_pose.position.z = -0.125;

	shape_msgs::msg::SolidPrimitive wheel_fl;
	wheel_fl.type = wheel_fl.BOX;
	wheel_fl.dimensions = { 0.24, 0.1, 0.24 };

	geometry_msgs::msg::Pose wheel_fl_pose;
	wheel_fl_pose.position.x = 0.205;
	wheel_fl_pose.position.y = 0.25;
	wheel_fl_pose.position.z = -0.09;

	shape_msgs::msg::SolidPrimitive wheel_fr;
	wheel_fr.type = wheel_fr.BOX;
	wheel_fr.dimensions = { 0.24, 0.1, 0.24 };

	geometry_msgs::msg::Pose wheel_fr_pose;
	wheel_fr_pose.position.x = 0.205;
	wheel_fr_pose.position.y = -0.25;
	wheel_fr_pose.position.z = -0.09;

	shape_msgs::msg::SolidPrimitive wheel_rl;
	wheel_rl.type = wheel_rl.BOX;
	wheel_rl.dimensions = { 0.24, 0.1, 0.24 };

	geometry_msgs::msg::Pose wheel_rl_pose;
	wheel_rl_pose.position.x = -0.205;
	wheel_rl_pose.position.y = 0.25;
	wheel_rl_pose.position.z = -0.09;

	shape_msgs::msg::SolidPrimitive wheel_rr;
	wheel_rr.type = wheel_rr.BOX;
	wheel_rr.dimensions = { 0.24, 0.1, 0.24 };

	geometry_msgs::msg::Pose wheel_rr_pose;
	wheel_rr_pose.position.x = -0.205;
	wheel_rr_pose.position.y = -0.25;
	wheel_rr_pose.position.z = -0.09;

	collision_object.primitives.push_back(floor);
	collision_object.primitive_poses.push_back(floor_pose);
	collision_object.primitives.push_back(robot);
	collision_object.primitive_poses.push_back(robot_pose);
	collision_object.primitives.push_back(wheel_fl);
	collision_object.primitive_poses.push_back(wheel_fl_pose);
	collision_object.primitives.push_back(wheel_fr);
	collision_object.primitive_poses.push_back(wheel_fr_pose);
	collision_object.primitives.push_back(wheel_rl);
	collision_object.primitive_poses.push_back(wheel_rl_pose);
	collision_object.primitives.push_back(wheel_rr);
	collision_object.primitive_poses.push_back(wheel_rr_pose);
	collision_object.operation = collision_object.ADD;

	/*	
	*  	--------------------------
	*	Cylinder Object
	*	--------------------------
 	*/

	moveit_msgs::msg::CollisionObject object;
	object.id = "object";
	object.header.frame_id = "world";
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = { 0.1, 0.02 };

	geometry_msgs::msg::Pose pose;
	pose.position.x = 0.7;
	pose.position.z = 0;
	pose.orientation.w = 1;
	object.pose = pose;

	/*	
	*  	--------------------------
	*	Add Objects to Planning Scene
	*	--------------------------
 	*/

	moveit::planning_interface::PlanningSceneInterface psi;
	psi.applyCollisionObject(collision_object);
	psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask()
{
	task_ = createTask();

	try
	{
		task_.init();
	}
	catch (mtc::InitStageException& e)
	{
		RCLCPP_ERROR_STREAM(LOGGER, e);
		return;
	}

	if (!task_.plan(5))
	{
		RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
		return;
	}
	task_.introspection().publishSolution(*task_.solutions().front());

	auto result = task_.execute(*task_.solutions().front());
	if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
	{
		RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
		return;
	}

	return;
}

mtc::Task MTCTaskNode::createTask()
{
	mtc::Task task;
	task.stages()->setName("demo task");
	task.loadRobotModel(node_);

	const auto& arm_group_name = "manipulator";
	const auto& hand_group_name = "gripper";
	const auto& hand_frame = "end_effector_link";

	// Set task properties
	task.setProperty("group", arm_group_name);
	task.setProperty("eef", hand_group_name);
	task.setProperty("ik_frame", hand_frame);

	// Disable warnings for this line, as it's a variable that's set but not used in this example
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
	#pragma GCC diagnostic pop

	auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
	current_state_ptr = stage_state_current.get();
	task.add(std::move(stage_state_current));

	auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
	auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
	interpolation_planner->setMaxVelocityScalingFactor(0.25);
	interpolation_planner->setMaxAccelerationScalingFactor(0.2);

	auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(0.25);
	cartesian_planner->setMaxAccelerationScalingFactor(0.25);
	cartesian_planner->setStepSize(.01);

	/*	
	*  	--------------------------
	*	CLOSE HAND
	*	--------------------------
 	*/

	{
		auto stage_close_hand =
			std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
		stage_close_hand->setGroup(hand_group_name);
		stage_close_hand->setGoal("Close");
		task.add(std::move(stage_close_hand));
	}

	/*	
	*  	--------------------------
	*	MOVE FORWARD X+
	*	--------------------------
 	*/

 	{
		auto stage = std::make_unique<mtc::stages::MoveRelative>("x +0.2", cartesian_planner);
		stage->setGroup(arm_group_name);
		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = 0.2;
		stage->setDirection(direction);
		task.add(std::move(stage));
	}

	/*	
	*  	--------------------------
	*	MOVE TO SIDE Z-
	*	--------------------------
 	*/

 	{
 		auto stage = std::make_unique<mtc::stages::MoveRelative>("z -0.3", cartesian_planner);
		stage->setGroup(arm_group_name);
		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = -0.3;
		stage->setDirection(direction);
		task.add(std::move(stage));
	}

	/*	
	*  	--------------------------
	*	Rotate about TCP RZ +45°
	*	--------------------------
 	*/

	{  // rotate about TCP
		auto stage = std::make_unique<mtc::stages::MoveRelative>("rz +45°", interpolation_planner);
		stage->setGroup(arm_group_name);
		geometry_msgs::msg::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = M_PI / 4.;
		stage->setDirection(twist);
		task.add(std::move(stage));
	}

	/*	
	*  	--------------------------
	*	MOVE TO HOME
	*	--------------------------
 	*/

	{ 
		auto stage = std::make_unique<mtc::stages::MoveTo>("move to home", interpolation_planner);
		stage->setGroup(arm_group_name);
		stage->setGoal("Home");
		stage->setTimeout(5.0);
		task.add(std::move(stage));
	}

	/*	
	*  	--------------------------
	*	OPEN HAND
	*	--------------------------
 	*/

	{
		auto stage_open_hand =
		std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
		stage_open_hand->setGroup(hand_group_name);
		stage_open_hand->setGoal("Open");
		task.add(std::move(stage_open_hand));
	}

	return task;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(true);

	auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
	rclcpp::executors::MultiThreadedExecutor executor;

	auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
		executor.add_node(mtc_task_node->getNodeBaseInterface());
		executor.spin();
		executor.remove_node(mtc_task_node->getNodeBaseInterface());
	});

	mtc_task_node->setupPlanningScene();
	mtc_task_node->doTask();

	spin_thread->join();
	rclcpp::shutdown();
	return 0;
}