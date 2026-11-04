"""10/25 1·2단계: 클러스터 파라미터 쓸기를 한 컨테이너에서 한 번에 돌리는 런치. s69_sweep_offline.sh 가 컨테이너 안에서 부름
흐름(주행 중 clustering 과 같은 노드·순서, tier4_perception_launch 0.52.0 pointcloud_map_filter.launch.py no_compare_map + lidar_rule_detector):
  /perception/obstacle_segmentation/pointcloud(bag) -> ApproximateDownsampleFilter 0.1m(use_pointcloud_map false 일 때와 같음) -> /s69/ds
  -> 점 지움 노드 둘(s67_facility_crop.py, 컨테이너 밖 프로세스, 이 런치가 띄움): 시설만 /s69/crop_f, 시설+건물 /s69/crop_fb
  -> 지움마다 voxel_grid_based_euclidean_cluster(min_points_per_cluster x voxel_leaf_size_m, 나머지는 autoware_launch 0.52.0 값)
  -> shape_estimation(패키지 기본 설정) -> /s69/<지움>_m<최소 점>_l<복셀>/objects
  feature remover 는 모양·위치를 안 바꿔서 뺌(objects 의 중심은 같음)
인자: mins:=3,5,7,10 leafs:=0.3,0.2 crops:=f,fb
"""
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CL = "/opt/autoware/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml"
SE = "/opt/autoware/autoware_shape_estimation/share/autoware_shape_estimation/config/shape_estimation.param.yaml"
CROP = "/p/tools/s67_facility_crop.py"


def name(crop, m, l):
    return f"{crop}_m{m}_l{str(l).replace('.', '')}"


def setup(context):
    mins = [int(x) for x in LaunchConfiguration("mins").perform(context).split(",")]
    leafs = [float(x) for x in LaunchConfiguration("leafs").perform(context).split(",")]
    crops = LaunchConfiguration("crops").perform(context).split(",")
    log = LaunchConfiguration("log").perform(context)
    clp = yaml.safe_load(open(CL))["/**"]["ros__parameters"]
    sep = yaml.safe_load(open(SE))["/**"]["ros__parameters"]
    nodes = [ComposableNode(package="autoware_pointcloud_preprocessor", plugin="autoware::pointcloud_preprocessor::ApproximateDownsampleFilterComponent",
                            name="voxel_grid_downsample_filter", namespace="/s69",
                            remappings=[("input", "/perception/obstacle_segmentation/pointcloud"), ("output", "/s69/ds")],
                            parameters=[{"voxel_size_x": 0.1, "voxel_size_y": 0.1, "voxel_size_z": 0.1}])]
    for crop in crops:
        for m in mins:
            for l in leafs:
                n = name(crop, m, l); ns = f"/s69/{n}"
                nodes.append(ComposableNode(package="autoware_euclidean_cluster", plugin="autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterNode",
                                            name="euclidean_cluster", namespace=ns,
                                            remappings=[("input", f"/s69/crop_{crop}"), ("output", f"{ns}/clusters")],
                                            parameters=[clp, {"min_points_per_cluster": m, "voxel_leaf_size_m": l}]))
                nodes.append(ComposableNode(package="autoware_shape_estimation", plugin="autoware::shape_estimation::ShapeEstimationNode",
                                            name="shape_estimation", namespace=ns,
                                            remappings=[("input", f"{ns}/clusters"), ("objects", f"{ns}/objects")],
                                            parameters=[sep, {"model_path": ""}]))
    acts = [ComposableNodeContainer(name="s69_container", namespace="/s69", package="rclcpp_components", executable="component_container",
                                    composable_node_descriptions=nodes, output="screen")]
    for crop in crops:
        extra = ["--buildings", "/p/config/s69_building_rects.json", "--bpad", "0.3"] if crop == "fb" else []
        acts.append(ExecuteProcess(cmd=["python3", "-u", CROP, "--input", "/s69/ds", "--output", f"/s69/crop_{crop}",
                                        "--polygons", "/p/config/s67_facility_polygons.json", "--log", f"{log}_crop_{crop}.csv"] + extra, output="screen"))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("mins", default_value="3,5,7,10"), DeclareLaunchArgument("leafs", default_value="0.3,0.2"),
                              DeclareLaunchArgument("crops", default_value="f,fb"), DeclareLaunchArgument("log", default_value="/tmp/s69"),
                              OpaqueFunction(function=setup)])
