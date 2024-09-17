from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    launch_arguments = {
        "robot_ip": "yyy.yyy.yyy.yyy",
        "use_fake_hardware": "true",
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "7",
        "gripper_max_velocity": "100.0",
        "gripper_max_force": "100.0",
        "use_internal_bus_gripper_comm": "true",
    }

    moveit_config = MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config").to_dict()
    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_tutorial2",
        executable="cartesian_node",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])