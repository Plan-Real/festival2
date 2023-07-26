import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    yolox_ros_share_dir = get_package_share_directory("yolov5_ros")

    # realsense = launch_ros.actions.Node(
    #     package="realsense2_camera", executable="rs_launch.py",
    #     parameters=[
    #         {"image_size": [640,480]},
    #     ],
    # )

    # realsense = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #     [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']
    #     ),
    #     launch_arguments = [
    #         ("depth_module.profile", "1280x720x30"),
    #         ("pointcloud.enable", "True")
    #     ]
    # )

    yolov5_ros = launch_ros.actions.Node(
        package="yolov5_ros",
        executable="yolov5_ros",
        parameters=[
            {"view_img": True},
        ],
    )

    yolo_people = launch_ros.actions.Node(
        package="yolo_people",
        executable="yolo_people",
        # parameters=[
        #     {"view_img":True},
        # ],
    )

    rqt_graph = launch_ros.actions.Node(
        package="rqt_graph",
        executable="rqt_graph",
    )

    return launch.LaunchDescription(
        [
            # realsense,
            yolov5_ros,
            yolo_people,
        ]
    )
