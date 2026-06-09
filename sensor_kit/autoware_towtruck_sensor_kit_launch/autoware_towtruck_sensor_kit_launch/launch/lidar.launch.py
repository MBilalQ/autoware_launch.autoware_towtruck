# Dual Livox MID-360 preprocessing chain.
#
# All nodes load into pointcloud_container as composable nodes with
# intra-process comms — zero-copy from each livox_bridge through the
# per-lidar filter chain into the concatenator.
#
# Pipeline (mirrored per side):
#   /livox/lidar    (left  driver)    -> LivoxToAutoware  -> /sensing/lidar/left/pointcloud_raw_ex   (frame velodyne_left)
#   /livox/lidar2   (right driver)    -> LivoxToAutoware  -> /sensing/lidar/right/pointcloud_raw_ex  (frame velodyne_right)
#
#   per side:
#     pointcloud_raw_ex
#       -> CropBoxFilter "self"       -> self_cropped/pointcloud_ex
#       -> CropBoxFilter "mirror"     -> mirror_cropped/pointcloud_ex
#       -> DistortionCorrector        -> rectified/pointcloud_ex
#       -> PointCloudExToXyzirc       -> pointcloud_before_sync
#
#   PointCloudConcatenateDataSynchronizerComponent
#       inputs:  /sensing/lidar/{left,right}/pointcloud_before_sync
#       output:  /sensing/lidar/concatenated/pointcloud  (output_frame = base_link)
#
# Only the left MID-360's onboard IMU is bridged to tamagawa/imu_link
# (enable_imu_bridge=true on left, false on right) — tamagawa/imu_link is
# physically tied to the left unit per sensor_kit.xacro.
#
# ring_outlier_filter is intentionally skipped — the MID-360's non-repetitive
# scan has no rings; PointCloudExToXyzirc replaces it functionally by doing
# the AEDT->XYZIRC layout strip the concatenator wants.

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
SENSOR_KIT_PKG = "autoware_towtruck_sensor_kit_launch"


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


def _side_chain(
    *,
    side: str,
    lidar_frame: str,
    input_cloud_topic: str,
    input_imu_topic: str,
    enable_imu_bridge: bool,
    self_params: dict,
    mirror_params: dict,
    distortion_param,
):
    """Build the 5-node preprocessing chain for one lidar side."""
    ns = f"/sensing/lidar/{side}"
    return [
        # Livox -> Autoware bridge (C++, intra-process to the cropbox below).
        ComposableNode(
            package="towtruck_interface",
            plugin="towtruck_interface::LivoxToAutoware",
            name=f"livox_to_autoware_{side}",
            parameters=[{
                "lidar_frame_id":     lidar_frame,
                "imu_frame_id":       "tamagawa/imu_link",
                "input_cloud_topic":  input_cloud_topic,
                "output_cloud_topic": f"{ns}/pointcloud_raw_ex",
                "input_imu_topic":    input_imu_topic,
                "enable_imu_bridge":  enable_imu_bridge,
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            namespace=ns,
            remappings=[
                ("input",  "pointcloud_raw_ex"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[self_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            namespace=ns,
            remappings=[
                ("input",  "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[mirror_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            namespace=ns,
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            parameters=[distortion_param],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        # Format stripper: AEDT (32 B) -> XYZIRC (16 B) for the concatenator.
        ComposableNode(
            package="towtruck_interface",
            plugin="towtruck_interface::PointCloudExToXyzirc",
            name=f"pointcloud_ex_to_xyzirc_{side}",
            parameters=[{
                "input_topic":  f"{ns}/rectified/pointcloud_ex",
                "output_topic": f"{ns}/pointcloud_before_sync",
            }],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]


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

    concat_param = ParameterFile(
        os.path.join(
            get_package_share_directory(SENSOR_KIT_PKG),
            "config",
            "concatenate_and_time_sync_node.param.yaml",
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

    # Topic names follow the multi-lidar driver convention: with multi_topic=1
    # the livox_ros_driver2 publishes per-lidar on /livox/lidar_<ip-with-underscores>.
    # See livox_ros_driver2/src/lddc.cpp line 650.
    composable_nodes = []
    composable_nodes += _side_chain(
        side="left",
        lidar_frame="velodyne_left",
        input_cloud_topic="/livox/lidar_192_168_1_137",
        input_imu_topic="/livox/imu_192_168_1_137",
        enable_imu_bridge=True,
        self_params=self_params,
        mirror_params=mirror_params,
        distortion_param=distortion_param,
    )
    composable_nodes += _side_chain(
        side="right",
        lidar_frame="velodyne_right",
        input_cloud_topic="/livox/lidar_192_168_2_105",
        input_imu_topic="/livox/imu_192_168_2_105",
        enable_imu_bridge=False,
        self_params=self_params,
        mirror_params=mirror_params,
        distortion_param=distortion_param,
    )

    # Multi-lidar concatenator. Subscribes to the per-side
    # pointcloud_before_sync topics (listed in the YAML) and emits the
    # combined cloud on /sensing/lidar/concatenated/pointcloud, transformed
    # into base_link via TF (sensor_kit_calibration.yaml has both
    # velodyne_{left,right}_base_link static transforms).
    composable_nodes.append(
        ComposableNode(
            package="autoware_pointcloud_preprocessor",
            plugin="autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
            name="concatenate_data",
            namespace="/sensing/lidar",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("output", "concatenated/pointcloud"),
                ("output_info", "concatenated/pointcloud_info"),
            ],
            parameters=[concat_param],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

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
