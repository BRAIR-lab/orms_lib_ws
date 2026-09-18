import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint, JointConstraint,
    BoundingVolume, PlanningOptions, RobotState
)
from shape_msgs.msg import SolidPrimitive
import tf2_ros
import numpy as np
import time


from orms_lib.interfaces.robot_base import RobotBase

class FrankaRobot(RobotBase):
    """
    Hardware abstraction layer for Franka robot via MoveIt 2 MoveGroup action.
    Provides methods mimicking the MATLAB robot API for Cartesian, Joint, and Force control.
    """

    # FR3 specific constants
    GROUP_NAME = 'fr3_arm'
    EE_LINK = 'fr3_hand_tcp'
    BASE_FRAME = 'fr3_link0'
    JOINT_NAMES = [
        'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
        'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
    ]

    def __init__(self, node: Node, velocity_scaling: float = 0.1, acceleration_scaling: float = 0.1):
        self.node = node
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling

        from rclpy.callback_groups import ReentrantCallbackGroup
        self._cb_group = ReentrantCallbackGroup()

        # MoveGroup action client
        self._move_group_client = ActionClient(self.node, MoveGroup, '/move_action', callback_group=self._cb_group)
        self.node.get_logger().info("Waiting for MoveGroup action server...")
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().warn(
                "MoveGroup action server not available after 10s. "
                "Motion commands will fail until it comes up."
            )
        else:
            self.node.get_logger().info("MoveGroup action server connected.")

        # Franka stiffness service client
        from franka_msgs.srv import SetCartesianStiffness
        self._stiffness_client = self.node.create_client(
            SetCartesianStiffness, '/service_server/set_cartesian_stiffness',
            callback_group=self._cb_group)
            
        # Cartesian path service and action clients
        from moveit_msgs.srv import GetCartesianPath
        from moveit_msgs.action import ExecuteTrajectory
        self._cartesian_client = self.node.create_client(
            GetCartesianPath, '/compute_cartesian_path', callback_group=self._cb_group)
        self._execute_client = ActionClient(
            self.node, ExecuteTrajectory, '/execute_trajectory', callback_group=self._cb_group)

        # TF for reading current pose
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)

        # Wrench subscription (live hardware forces)
        from geometry_msgs.msg import WrenchStamped
        from rclpy.qos import qos_profile_sensor_data
        self.current_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._wrench_sub = self.node.create_subscription(
            WrenchStamped,
            '/franka_robot_state_broadcaster/external_wrench_in_base_frame',
            self._wrench_cb,
            qos_profile_sensor_data,
            callback_group=self._cb_group
        )

        # Object frame for relative motions
        self.object_frame_pose = None

        self.node.get_logger().info(
            f"RobotInterface ready. Speed: {self.velocity_scaling*100:.0f}%, "
            f"Accel: {self.acceleration_scaling*100:.0f}%"
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def set_speed_scaling(self, velocity: float, acceleration: float):
        """
        Set motion speed scaling factors (0.0 to 1.0).
        0.1 = 10% speed (safe for testing), 1.0 = full speed.
        """
        self.velocity_scaling = max(0.01, min(1.0, velocity))
        self.acceleration_scaling = max(0.01, min(1.0, acceleration))
        self.node.get_logger().info(
            f"Speed scaling updated: vel={self.velocity_scaling*100:.0f}%, "
            f"accel={self.acceleration_scaling*100:.0f}%"
        )

    def _build_move_group_goal(self) -> MoveGroup.Goal:
        """Create a MoveGroup.Goal with common defaults."""
        goal = MoveGroup.Goal()
        goal.request.group_name = self.GROUP_NAME
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        # Empty start state = plan from current state
        goal.request.start_state = RobotState()
        goal.request.start_state.is_diff = True
        # Plan and execute
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        return goal

    def _wait_for_future(self, future, timeout_sec: float = 10.0) -> bool:
        """Wait for a future to complete without re-spinning the executor."""
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                self.node.get_logger().error(f"Future timed out after {timeout_sec}s")
                return False
            time.sleep(0.05)
        return True

    def _send_move_group_goal(self, goal: MoveGroup.Goal, timeout_sec: float = 30.0, stop_condition=None) -> bool:
        """Send a MoveGroup goal and wait for the result. Returns True on success."""
        if not self._move_group_client.server_is_ready():
            self.node.get_logger().error("MoveGroup action server not available.")
            return False

        future = self._move_group_client.send_goal_async(goal)
        if not self._wait_for_future(future, timeout_sec=10.0):
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup goal was rejected.")
            return False

        self.node.get_logger().info("MoveGroup goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        
        start = time.time()
        while not result_future.done():
            if time.time() - start > timeout_sec:
                self.node.get_logger().error(f"Action timed out after {timeout_sec}s")
                return False
            if stop_condition is not None and stop_condition():
                self.node.get_logger().info("Stop condition met. Canceling goal.")
                goal_handle.cancel_goal_async()
                # Wait for cancellation to complete
                self._wait_for_future(result_future, timeout_sec=5.0)
                return True
            time.sleep(0.05)

        result = result_future.result()
        if result is None:
            self.node.get_logger().error("MoveGroup returned no result (timeout?).")
            return False

        error_code = result.result.error_code.val
        if error_code == 1:  # MoveItErrorCodes.SUCCESS
            self.node.get_logger().info("MoveGroup motion succeeded.")
            return True
        else:
            self.node.get_logger().error(f"MoveGroup motion failed with error code: {error_code}")
            return False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    
    def set_cartesian_stiffness(self, stiffness_values: list) -> bool:
        """Set Franka Cartesian stiffness [x, y, z, roll, pitch, yaw]."""
        from franka_msgs.srv import SetCartesianStiffness
        if not self._stiffness_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().warn('Cartesian stiffness service not available, skipping.')
            return False
            
        req = SetCartesianStiffness.Request()
        req.cartesian_stiffness = [float(v) for v in stiffness_values]
        
        future = self._stiffness_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=5.0):
            return False
            
        result = future.result()
        if result is not None and result.success:
            self.node.get_logger().info(f'Cartesian stiffness set to Z={stiffness_values[2]}')
            return True
            
        self.node.get_logger().warn('Failed to set stiffness.')
        return False

    def set_object_frame(self, t_obj: Pose):
        """ Equivalent to r.SetObject(T_obj) """
        self.node.get_logger().info("Setting object frame reference.")
        self.object_frame_pose = t_obj

    def move_cartesian(self, target_pose: PoseStamped, duration: float = 5.0, stop_condition=None) -> bool:
        """
        Move end-effector to an absolute Cartesian pose (in base frame).
        Equivalent to r.CMove(T, time).
        Uses OMPL (MoveGroup) for free-space motion.
        """
        self.node.get_logger().info(f"Moving Cartesian to target over {duration}s")
        goal = self._build_move_group_goal()

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.BASE_FRAME
        pos_constraint.link_name = self.EE_LINK
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        
        target_point = target_pose.pose.position
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm tolerance

        pos_constraint.constraint_region.primitives.append(primitive)
        pose_in_region = Pose()
        pose_in_region.position = target_point
        pose_in_region.orientation.w = 1.0
        pos_constraint.constraint_region.primitive_poses.append(pose_in_region)
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self.BASE_FRAME
        ori_constraint.link_name = self.EE_LINK
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(constraints)

        return self._send_move_group_goal(goal, timeout_sec=max(duration * 3, 15.0), stop_condition=stop_condition)

    def move_cartesian_path(self, waypoints: list) -> bool:
        """
        Execute a strict straight-line Cartesian path through waypoints.
        Bypasses OMPL and uses the Cartesian interpolator.
        """
        self.node.get_logger().info(f"Executing Cartesian path with {len(waypoints)} waypoints.")
        from moveit_msgs.srv import GetCartesianPath
        from moveit_msgs.action import ExecuteTrajectory
        from moveit_msgs.msg import RobotState

        if not self._cartesian_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error('Cartesian path service not available!')
            return False

        req = GetCartesianPath.Request()
        req.header.frame_id = self.BASE_FRAME
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.group_name = self.GROUP_NAME
        req.link_name = self.EE_LINK
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = self.velocity_scaling
        req.max_acceleration_scaling_factor = self.acceleration_scaling
        req.start_state = RobotState()
        req.start_state.is_diff = True
        
        for wp in waypoints:
            req.waypoints.append(wp.pose)
            
        future = self._cartesian_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=10.0):
            return False
            
        result = future.result()
        if result is None or result.fraction < 0.9:
            frac = result.fraction if result else 0.0
            self.node.get_logger().error(f'Cartesian path only {frac*100:.1f}% achieved')
            return False
            
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = result.solution
        
        future = self._execute_client.send_goal_async(exec_goal)
        if not self._wait_for_future(future, timeout_sec=5.0):
            return False
            
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.node.get_logger().error("ExecuteTrajectory goal was rejected")
            return False
            
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=30.0):
            return False
            
        res = result_future.result()
        return res.result.error_code.val == 1

    def move_cartesian_relative(self, delta_pos: list, duration: float = 3.0, stop_condition=None) -> bool:
        """
        Move end-effector by a relative Cartesian offset [dx, dy, dz].
        Equivalent to r.CMoveFor(delta, time).
        """
        self.node.get_logger().info(f"Relative Cartesian move {delta_pos} over {duration}s")
        current = self.get_ee_pose()
        if current is None:
            self.node.get_logger().error("Cannot get current pose for relative move.")
            return False

        target = PoseStamped()
        target.header = current.header
        target.pose.position.x = current.pose.position.x + delta_pos[0]
        target.pose.position.y = current.pose.position.y + delta_pos[1]
        target.pose.position.z = current.pose.position.z + delta_pos[2]
        target.pose.orientation = current.pose.orientation
        return self.move_cartesian(target, duration, stop_condition=stop_condition)

    def move_joints(self, joint_positions: list, duration: float = 5.0) -> bool:
        """
        Move to a joint configuration.
        Equivalent to r.JnMove(q, time).
        """
        self.node.get_logger().info(f"Moving joints to {joint_positions}")

        goal = self._build_move_group_goal()

        constraints = Constraints()
        for name, pos in zip(self.JOINT_NAMES, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        return self._send_move_group_goal(goal, timeout_sec=max(duration * 3, 15.0))

    def move_in_object_frame(self, target_pose_obj: PoseStamped, duration: float = 5.0) -> bool:
        """ Equivalent to r.OMove(T, time) """
        self.node.get_logger().info(f"Moving to pose in object frame over {duration}s")
        # TODO: Transform target_pose_obj from object frame to base frame
        # For now, treat as base-frame pose
        return self.move_cartesian(target_pose_obj, duration)

    def approach_in_object_frame(self, target_pose_obj: PoseStamped, offset: list, duration: float = 5.0) -> bool:
        """ Equivalent to r.OApproach(T, offset, time) """
        self.node.get_logger().info(f"Approaching pose in object frame with offset {offset}")
        approach_pose = PoseStamped()
        approach_pose.header = target_pose_obj.header
        approach_pose.pose.position.x = target_pose_obj.pose.position.x + offset[0]
        approach_pose.pose.position.y = target_pose_obj.pose.position.y + offset[1]
        approach_pose.pose.position.z = target_pose_obj.pose.position.z + offset[2]
        approach_pose.pose.orientation = target_pose_obj.pose.orientation
        return self.move_cartesian(approach_pose, duration)

    def move_relative_in_object_frame(self, delta_pos: list, duration: float = 3.0) -> bool:
        """ Equivalent to r.OMoveFor(delta, time) """
        self.node.get_logger().info(f"Relative move in object frame {delta_pos}")
        return self.move_cartesian_relative(delta_pos, duration)

    def set_impedance(self, k_p: list, k_r: list):
        """ Equivalent to r.SetCartesianCompliance(kp, kr) """
        self.node.get_logger().info(f"Setting Cartesian impedance Kp={k_p}, Kr={k_r}")
        # No-op in MoveIt mode — impedance requires a real-time controller

    def apply_wrench(self, force: list, torque: list):
        """ Equivalent to r.ApplyFT(wrench) """
        self.node.get_logger().info(f"Applying wrench F={force}, T={torque}")
        # No-op in MoveIt mode

    def get_ee_pose(self) -> PoseStamped:
        """Returns current end-effector pose as PoseStamped by reading from TF."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self.BASE_FRAME,
                self.EE_LINK,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
            pose_msg = PoseStamped()
            pose_msg.header = transform.header
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            return pose_msg
        except tf2_ros.TransformException as ex:
            self.node.get_logger().warn(f"Could not get current EE pose: {ex}")
            return None

    def _wrench_cb(self, msg):
        self.current_wrench[0] = msg.wrench.force.x
        self.current_wrench[1] = msg.wrench.force.y
        self.current_wrench[2] = msg.wrench.force.z
        self.current_wrench[3] = msg.wrench.torque.x
        self.current_wrench[4] = msg.wrench.torque.y
        self.current_wrench[5] = msg.wrench.torque.z

    def get_wrench(self) -> list:
        """ Returns current F/T readings [fx, fy, fz, tx, ty, tz] """
        return list(self.current_wrench)
