with open('/home/brairlab/orms_lib_ws/src/tum-tb-perception2/ros/tum_tb_perception2/yolo_pnp_pose_estimator_node.py', 'r') as f:
    content = f.read()

content = content.replace(
    'fallback_frames = [self.desired_reference_frame, "map", "world", camera_frame_id]',
    'fallback_frames = list(dict.fromkeys([self.desired_reference_frame, "world", "right_base_link", "map", camera_frame_id]))'
)

with open('/home/brairlab/orms_lib_ws/src/tum-tb-perception2/ros/tum_tb_perception2/yolo_pnp_pose_estimator_node.py', 'w') as f:
    f.write(content)
