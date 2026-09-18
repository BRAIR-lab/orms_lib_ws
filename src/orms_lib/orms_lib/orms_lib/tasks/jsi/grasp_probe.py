from orms_lib.tasks.task_base import TaskBase

class GraspProbeTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting GraspProbe", 0.1)
        
        # Get frame
        probe_frame = self.perception.wait_for_frame('multimeter_probe_frame')
        if not probe_frame:
            self.node.get_logger().error("Probe frame not found")
            return False
            
        # 1. Move joint 7 to -135 to avoid camera collision
        self.publish_feedback(goal_handle, "Rotating wrist to avoid camera", 0.2)
        # self.robot.move_joints(..., 2.0)
        
        # 2. Pre-open gripper to 2cm
        self.publish_feedback(goal_handle, "Pre-opening gripper", 0.3)
        self.gripper.move(0.02)
        
        # 3. Approach 8cm overhead
        self.publish_feedback(goal_handle, "Approaching probe", 0.4)
        # self.robot.approach_in_object_frame(probe_frame.transform, [0, 0, 0.08], 2.0)
        
        # 4. Lower to grasp pose
        self.publish_feedback(goal_handle, "Lowering to grasp pose", 0.6)
        # self.robot.move_in_object_frame(probe_frame.transform, 1.5)
        
        # 5. Close gripper
        self.publish_feedback(goal_handle, "Grasping", 0.7)
        self.gripper.close()
        
        # Verify grasp
        width = self.gripper.get_width()
        if width < 0.004:
            self.node.get_logger().error("Grasp failed, gripper closed too much (empty)")
            self.gripper.move(0.04) # open and abort
            return False
            
        # 6. Extract (pull back 3cm, lift 3cm)
        self.publish_feedback(goal_handle, "Extracting probe", 0.9)
        # self.robot.move_relative_in_object_frame([-0.03, 0, 0], 1.0)
        # self.robot.move_relative_in_object_frame([0, 0, 0.03], 1.0)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
