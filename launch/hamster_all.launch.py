from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mmi_hamster",
            executable="cam_reader",
            name="cam_reader",
            output="screen",
        ),
        Node(
            package="mmi_hamster",
            executable="face_smile_processor",
            name="face_smile_processor",
            output="screen",
        ),
        Node(
            package="mmi_hamster",
            executable="microcontroller_communicator",
            name="microcontroller_communicator",
            output="screen",
        ),
        Node(
            package="mmi_hamster",
            executable="math_quiz_node",
            name="math_quiz_node",
            output="screen",
        ),
        Node(
            package="mmi_hamster",
            executable="blow_challenge_node",
            name="blow_challenge_node",
            output="screen",
        ),
        Node(
            package="mmi_hamster",
            executable="game_master",
            name="game_master",
            output="screen",
        ),
    ])
