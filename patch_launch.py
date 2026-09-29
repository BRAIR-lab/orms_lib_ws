import re

with open('/home/brairlab/orms_lib_ws/src/orms_lib/orms_lib/launch/system_bringup.launch.py', 'r') as f:
    content = f.read()

new_func = """
def launch_yolo_pnp(context, *args, **kwargs):
    config_file = context.launch_configurations.get('orms_config_file', 'orms_config_fr3.yaml')
    perception_type = context.launch_configurations.get('perception_type', 'yolo_pnp')

    if perception_type != 'yolo_pnp':
        return []

    if not os.path.isabs(config_file):
        orms_lib_share = get_package_share_directory('orms_lib')
        config_path = os.path.join(orms_lib_share, 'config', config_file)
    else:
        config_path = config_file

    desired_frame = 'fr3_link0'
    try:
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
                percept_cfg = cfg.get('perception', {})
                if isinstance(percept_cfg, dict):
                    desired_frame = percept_cfg.get('desired_reference_frame', 'fr3_link0')
    except Exception as e:
        print(f"Failed to read perception config from {config_path}: {e}")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('tum_tb_perception2'),
                    'launch',
                    'yolo_pnp_pose_estimator.launch.py'
                ])
            ]),
            launch_arguments={'desired_reference_frame': desired_frame}.items()
        )
    ]
"""

content = content.replace("def generate_launch_description():", new_func + "\ndef generate_launch_description():")

old_node_def = """    yolo_pnp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tum_tb_perception2'),
                'launch',
                'yolo_pnp_pose_estimator.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression(["'", perception_type, "' == 'yolo_pnp'"]))
    )"""

new_node_def = """    yolo_pnp_node = GroupAction([
        PushLaunchConfigurations(),
        OpaqueFunction(function=launch_yolo_pnp),
        PopLaunchConfigurations()
    ])"""

content = content.replace(old_node_def, new_node_def)

with open('/home/brairlab/orms_lib_ws/src/orms_lib/orms_lib/launch/system_bringup.launch.py', 'w') as f:
    f.write(content)
