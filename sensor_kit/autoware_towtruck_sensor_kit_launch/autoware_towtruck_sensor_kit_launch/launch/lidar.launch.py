# Single-Livox MID-360 preprocessing chain.
#
# All nodes load into pointcloud_container as composable nodes with
# intra-process comms — zero-copy from livox_bridge through the filter
# chain to the concatenated-topic publisher.
#
# Pipeline:
#   /livox/lidar (inter-process, from livox_ros_driver2)
#     -> LivoxToAutoware (C++)        -> pointcloud_raw_ex (PointXYZIRCAEDT, frame velodyne_left)
#     -> CropBoxFilter "self"         -> self_cropped/pointcloud_ex
#     -> CropBoxFilter "mirror"       -> mirror_cropped/pointcloud_ex
#     -> DistortionCorrector          -> rectified/pointcloud_ex
#     -> PointCloudExToXyzirc (C++)   publishes both:
#                                       /sensing/lidar/left/pointcloud_before_sync
#                                       /sensing/lidar/concatenated/pointcloud
#
# Why no concatenator: PointCloudConcatenateDataSynchronizerComponent refuses
# to start with a single input topic ("Need at least two topics to continue").
# The format-stripper double-publishes for now; add the concat node back when
# a second lidar comes online.
#
# ring_outlier_filter is intentionally skipped — the MID-360's non-repetitive
# scan has no rings; replacing it with the format stripper keeps the layout
# check happy without dropping good points.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


VEHICLE_DESCRIPTION_PKG = "autoware_towtruck_vehicle_description"
COMMON_SENSOR_PKG = "common_sensor_launch"
LIDAR_NAMESPACE = "/sensing/lidar/left"


def _load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]


def _vehicle_self_bbox():
    """Vehicle outer bbox (with 5 cm margin) for the self-crop filter."""
    info = _load_yaml(
        os.path.join(
            get_package_share_directory(VEHICLE_DESCRIPTION_PKG),
            "config",
            "vehicle_info.param.yaml",
        )
    )
    margin = 0.05
    return {
        "min_x": -info["rear_overhang"] - margin,
        "max_x": info["front_overhang"] + info["wheel_base"] + margin,
        "min_y": -(info["wheel_tread"] / 2.0 + info["right_overhang"]) - margin,
        "max_y": info["wheel_tread"] / 2.0 + info["left_overhang"] + margin,
        "min_z": 0.0,
        "max_z": info["vehicle_height"] + margin,
    }


def launch_setup(context, *args, **kwargs):
    container = LaunchConfiguration("pointcloud_container_name")

    distortion_param = ParameterFile(
        os.path.join(
            get_package_share_directory(COMMON_SENSOR_PKG),
            "config",
            "distortion_corrector_node.param.yaml",
        ),
        allow_substs=True,
    )

    cropbox_common = {
        "input_frame": "base_link",
        "output_frame": "base_link",
        "negative": True,
        "processing_time_threshold_sec": 0.01,
    }

    self_box = _vehicle_self_bbox()
    self_params = {**cropbox_common, **self_box}

    mirror_box = _load_yaml(
        LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    )
    mirror_params = {
        **cropbox_common,
        "min_x": mirror_box["min_longitudinal_offset"],
        "max_x": mirror_box["max_longitudinal_offset"],
        "min_y": mirror_box["min_lateral_offset"],
        "max_y": mirror_box["max_lateral_offset"],
        "min_z": mirror_box["min_height_offset"],
        "max_z": mirror_box["max_height_offset"],
    }

    composable_nodes = [

        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            namespace=LIDAR_NAMESPACE,
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[self_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            namespace=LIDAR_NAMESPACE,
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[mirror_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            namespace=LIDAR_NAMESPACE,
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            parameters=[distortion_param],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        # Format stripper + N=1 stand-in for the concatenator (C++).
        ComposableNode(
            package="towtruck_interface",
            plugin="towtruck_interface::PointCloudExToXyzirc",
            name="pointcloud_ex_to_xyzirc",
            parameters=[{
                "input_topic":       f"{LIDAR_NAMESPACE}/rectified/pointcloud_ex",
                "output_topic":      f"{LIDAR_NAMESPACE}/pointcloud_before_sync",
                "concatenated_topic": "/sensing/lidar/concatenated/pointcloud",
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        # Livox -> Autoware bridge (C++, intra-process to the cropbox below).
        ComposableNode(
            package="towtruck_interface",
            plugin="towtruck_interface::LivoxToAutoware",
            name="livox_to_autoware",
            parameters=[{
                "lidar_frame_id": "velodyne_left",
                "imu_frame_id":   "tamagawa/imu_link",
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]

    load_into_container = LoadComposableNodes(
        target_container=container,
        composable_node_descriptions=composable_nodes,
    )

    return [load_into_container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("launch_driver", default_value="true"),
        DeclareLaunchArgument("vehicle_mirror_param_file"),
        DeclareLaunchArgument("pointcloud_container_name", default_value="/pointcloud_container"),
        OpaqueFunction(function=launch_setup),
    ])
