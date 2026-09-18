from orms_lib.tasks.task_base import TaskBase
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import copy

class OpenDoorTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting OpenDoor", 0.1)

        request = goal_handle.request
        target_frame = "hatch_handle_frame"
        if "target_frame" in request.param_keys:
            idx = request.param_keys.index("target_frame")
            target_frame = request.param_values[idx]

        # 1. Pre-open the gripper
        self.publish_feedback(goal_handle, "Opening Gripper", 0.15)
        if not self.gripper.move(0.04):
            return False

        # 2. Get target frame
        target_tf = self.perception.wait_for_frame(target_frame, timeout_sec=3.0)
        if target_tf:
        #     # Apply a 1.5cm offset in the negative Y direction of the target's local frame
        #     r_tf = R.from_quat([
        #         target_tf.transform.rotation.x,
        #         target_tf.transform.rotation.y,
        #         target_tf.transform.rotation.z,
        #         target_tf.transform.rotation.w
        #     ])
        #     y_offset = r_tf.apply([0.0, -0.015, 0.0])
            
        #     target_pose = PoseStamped()
        #     target_pose.pose.position.x = target_tf.transform.translation.x + y_offset[0]
        #     target_pose.pose.position.y = target_tf.transform.translation.y + y_offset[1]
        #     target_pose.pose.position.z = target_tf.transform.translation.z + y_offset[2]
            target_pose = PoseStamped()
            target_pose.pose.position.x = target_tf.transform.translation.x 
            target_pose.pose.position.y = target_tf.transform.translation.y 
            target_pose.pose.position.z = target_tf.transform.translation.z 
            target_pose.pose.orientation = target_tf.transform.rotation
        else:
            self.node.get_logger().warn(
                f"Frame '{target_frame}' not found. Using fallback test pose."
            )
            target_pose = PoseStamped()
            target_pose.pose.position.x = 0.4
            target_pose.pose.position.y = 0.0
            target_pose.pose.position.z = 0.4
            target_pose.pose.orientation.x = 1.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 0.0

        # 3. Approach 5cm above target (along local Z-axis)
        self.publish_feedback(goal_handle, "Moving overhead", 0.3)
        r_target = R.from_quat([
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        ])
        approach_offset = r_target.apply([0.0, 0.0, 0.05])
        
        approach_pose = PoseStamped()
        approach_pose.pose.position.x = target_pose.pose.position.x + approach_offset[0]
        approach_pose.pose.position.y = target_pose.pose.position.y + approach_offset[1]
        approach_pose.pose.position.z = target_pose.pose.position.z + approach_offset[2]

        # Orient the TCP.
        # TCP Y aligned with hatch X, and TCP X in direction of hatch Y.
        r_rot_tcp = R.from_euler('zx', [-90, 180], degrees=True)
        r_approach = r_target * r_rot_tcp
        
        quat = r_approach.as_quat()
        approach_pose.pose.orientation.x = quat[0]
        approach_pose.pose.orientation.y = quat[1]
        approach_pose.pose.orientation.z = quat[2]
        approach_pose.pose.orientation.w = quat[3]
        
        if not self.robot.move_cartesian(approach_pose, 5.0):
            self.node.get_logger().error("Robot action failed.")
            return False

        # 4. Lower to grasp handle
        self.publish_feedback(goal_handle, "Lowering to grasp", 0.4)
        
        # Drop Z stiffness for contact
        STIFFNESS_DEFAULT = [2000.0, 2000.0, 2000.0, 200.0, 200.0, 200.0]
        STIFFNESS_CONTACT_Z = [2000.0, 2000.0, 200.0, 200.0, 200.0, 200.0]
        self.robot.set_cartesian_stiffness(STIFFNESS_CONTACT_Z)

        # Move slightly below the detected frame to ensure a firm contact/grasp
        contact_offset = r_target.apply([0.0, 0.0, -0.005])
        contact_pose = copy.deepcopy(approach_pose)
        contact_pose.pose.position.x = target_pose.pose.position.x + contact_offset[0]
        contact_pose.pose.position.y = target_pose.pose.position.y + contact_offset[1]
        contact_pose.pose.position.z = target_pose.pose.position.z + contact_offset[2]
        
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.1, 0.1)
        
        if not self.robot.move_cartesian_path([contact_pose]):
            self.node.get_logger().error("Robot action failed.")
            return False

        # 5. Grasp
        self.publish_feedback(goal_handle, "Grasping handle", 0.5)
        if not self.gripper.close(width=0.016):
            return False

        # 6. Compliant opening
        self.publish_feedback(goal_handle, "Compliant opening", 0.6)
        
        # Drop stiffness to be highly compliant in all directions
        # This allows the robot to follow the physical arc of the door hinge.
        STIFFNESS_PULL = [10.0, 10.0, 10.0, 1.0, 1.0, 1.0]
        self.robot.set_cartesian_stiffness(STIFFNESS_PULL)
        self.robot.set_speed_scaling(0.05, 0.05)

        # Command an arc pull.
        # The center of the hinge is 6cm in the -X direction in the hatch's local frame.
        # We move in a quarter circle (90 degrees).
        import math
        import numpy as np
        
        num_waypoints = 15
        angles_deg = np.linspace(0, -100, num_waypoints)
        
        arc_poses = []
        # The center of the hinge in the local frame relative to the contact pose is [0.0, -0.065, 0.0]
        # In the global frame, the vector from the center to the contact pose is [0.0, 0.065, 0.0] (local).
        center_to_contact_local = [0.0, 0.065, 0.0]
        center_offset = r_target.apply(center_to_contact_local)
        
        center_global = [
            contact_pose.pose.position.x - center_offset[0],
            contact_pose.pose.position.y - center_offset[1],
            contact_pose.pose.position.z - center_offset[2]
        ]

        r_rot_tcp = R.from_euler('zx', [-90, 180], degrees=True)

        for phi in angles_deg:
            # New position: rotate the center_to_contact vector by -phi around local X
            phi_rad = math.radians(phi)
            v_local = [
                0.0,
                0.065 * math.cos(phi_rad),
                -0.06 * math.sin(phi_rad)
            ]
            v_global = r_target.apply(v_local)
            
            wp_pose = copy.deepcopy(contact_pose)
            wp_pose.pose.position.x = center_global[0] + v_global[0]
            wp_pose.pose.position.y = center_global[1] + v_global[1]
            wp_pose.pose.position.z = center_global[2] + v_global[2]
            
            # New orientation: rotate TCP along with the door
            r_door = r_target * R.from_euler('x', -phi, degrees=True)
            r_new_tcp = r_door * r_rot_tcp
            quat = r_new_tcp.as_quat()
            
            wp_pose.pose.orientation.x = quat[0]
            wp_pose.pose.orientation.y = quat[1]
            wp_pose.pose.orientation.z = quat[2]
            wp_pose.pose.orientation.w = quat[3]
            
            arc_poses.append(wp_pose)

        if not self.robot.move_cartesian_path(arc_poses):
            self.node.get_logger().error("Robot action failed.")
            return False
        
        open_pose = arc_poses[-1]

        # 7. Release and retract
        self.publish_feedback(goal_handle, "Releasing and retracting", 0.9)
        if not self.gripper.move(0.04):
            return False
        
        retract_pose = copy.deepcopy(open_pose)
        # Retract back up in the Z direction (relative to target frame)
        retract_offset = r_target.apply([0.0, 0.0, 0.05])
        retract_pose.pose.position.x += retract_offset[0]
        retract_pose.pose.position.y += retract_offset[1]
        retract_pose.pose.position.z += retract_offset[2]
        
        if not self.robot.move_cartesian_path([retract_pose]):
            self.node.get_logger().error("Robot action failed.")
            return False

        # Restore parameters
        self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)
        self.robot.set_speed_scaling(old_vel, old_acc)

        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
