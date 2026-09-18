import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, PushLaunchConfigurations, PopLaunchConfigurations
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_realsense(context, *args, **kwargs):
    use_pc_z = context.launch_configurations.get('use_pointcloud_z', 'true')
    percept = context.launch_configurations.get('perception_type', 'yolo_pnp')
    need_depth = 'true' if (use_pc_z == 'true' or percept != 'yolo_pnp') else 'false'

    keys_to_remove = ['orms_config_file', 'perception_type', 'use_pointcloud_z', 'sim_mode']
    for key in keys_to_remove:
        if key in context.launch_configurations:
            del context.launch_configurations[key]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'pointcloud.enable': need_depth,
                'align_depth.enable': need_depth,
            }.items()
        )
    ]

def launch_eye_to_hand_transform(context, *args, **kwargs):
    config_file = context.launch_configurations.get('orms_config_file', 'orms_config_fr3.yaml')
    
    if not os.path.isabs(config_file):
        orms_lib_share = get_package_share_directory('orms_lib')
        config_path = os.path.join(orms_lib_share, 'config', config_file)
    else:
        config_path = config_file
        
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            transform = config.get('eye_to_hand_transform', None)
            if transform is None:
                # Default values if not present
                node_args = ['-0.061272', '0.009180', '-0.046419', '0.007725', '0.707307', '-0.007172', '-0.706828', 'fr3_hand_tcp', 'camera_link']
            else:
                translation = transform['translation']
                quaternion = transform['quaternion']
                parent = transform['parent_frame']
                child = transform['child_frame']
                node_args = [str(translation[0]), str(translation[1]), str(translation[2]),
                             str(quaternion[0]), str(quaternion[1]), str(quaternion[2]), str(quaternion[3]),
                             parent, child]
    except Exception as e:
        print(f"Failed to read eye_to_hand_transform from {config_path}: {e}")
        node_args = ['-0.061272', '0.009180', '-0.046419', '0.007725', '0.707307', '-0.007172', '-0.706828', 'fr3_hand_tcp', 'camera_link']

    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='eye2hand',
            output='screen',
            arguments=node_args
        )
    ]

def generate_launch_description():
    # ----------------------------------------------------------------
    # Launch arguments — all values come from CLI or defaults.
    # No Python-level config file reading here; the task_server_node
    # reads the YAML at runtime once it knows its own package share dir.
    # ----------------------------------------------------------------
    orms_config_file_arg = DeclareLaunchArgument(
        'orms_config_file',
        default_value='orms_config_fr3.yaml',
        description='Config filename (relative to orms_lib/config/) or absolute path'
    )
    perception_type_arg = DeclareLaunchArgument(
        'perception_type',
        default_value='yolo_pnp',
        description='Perception pipeline: yolo_pnp | frcnn_pointcloud | frcnn_sam'
    )
    use_pointcloud_z_arg = DeclareLaunchArgument(
        'use_pointcloud_z',
        default_value='true',
        description='Enable pointcloud depth for YOLO PnP (requires RealSense depth stream)'
    )
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='False',
        description='Run in simulation mode (bypasses force guards)'
    )

    orms_config_file = LaunchConfiguration('orms_config_file')
    perception_type = LaunchConfiguration('perception_type')
    use_pointcloud_z = LaunchConfiguration('use_pointcloud_z')
    sim_mode        = LaunchConfiguration('sim_mode')

    # ----------------------------------------------------------------
    # 1. Realsense Camera
    # Enable pointcloud if use_pointcloud_z=true OR perception != yolo_pnp
    # ----------------------------------------------------------------
    realsense_camera = GroupAction([
        PushLaunchConfigurations(),
        OpaqueFunction(function=launch_realsense),
        PopLaunchConfigurations()
    ])

    # ----------------------------------------------------------------
    # 2. Eye-to-hand static transform (camera_link → fr3_hand_tcp)
    # ----------------------------------------------------------------
    eye_to_hand_transform = GroupAction([
        PushLaunchConfigurations(),
        OpaqueFunction(function=launch_eye_to_hand_transform),
        PopLaunchConfigurations()
    ])

    # ----------------------------------------------------------------
    # 3. Perception Node — selected by perception_type arg
    # ----------------------------------------------------------------
    yolo_pnp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tum_tb_perception2'),
                'launch',
                'yolo_pnp_pose_estimator.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression(["'", perception_type, "' == 'yolo_pnp'"]))
    )
    frcnn_pc_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tum_tb_perception2'),
                'launch',
                'pose_estimator.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression(["'", perception_type, "' == 'frcnn_pointcloud'"]))
    )
    frcnn_sam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tum_tb_perception2'),
                'launch',
                'sam_pose_estimator.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression(["'", perception_type, "' == 'frcnn_sam'"]))
    )

    # ----------------------------------------------------------------
    # 4. ORMS Task Server — reads config_file at runtime
    # ----------------------------------------------------------------
    orms_action_server = Node(
        package='orms_lib',
        executable='task_server_node',
        name='task_server_node',
        output='screen',
        parameters=[
            {'config_file': orms_config_file},
            {'sim_mode': sim_mode},
        ]
    )

    return LaunchDescription([
        orms_config_file_arg,
        perception_type_arg,
        use_pointcloud_z_arg,
        sim_mode_arg,
        realsense_camera,
        eye_to_hand_transform,
        yolo_pnp_node,
        frcnn_pc_node,
        frcnn_sam_node,
        orms_action_server,
    ])
