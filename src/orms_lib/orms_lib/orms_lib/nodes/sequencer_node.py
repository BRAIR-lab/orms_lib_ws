import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from orms_lib_interfaces.action import TaskAction

class SequencerNode(Node):
    def __init__(self):
        super().__init__('sequencer_node')
        
        # Define the task sequence
        self.sequence = [
            ("press_button",    ["target_frame"], ["blue_button_frame"]),
            #("slider_move",     [], []),
            ("switch_sockets",  ["source", "target"], ["black_socket", "red_socket"]),
            ("open_door",       [], []),
            ("grasp_probe",     [], []),
            ("measure_probe",   [], []),
            ("drop_probe",      [], []),
            ("wrap_cable",      [], []),
            ("return_probe",    [], []),
            ("press_button",    ["target_frame"], ["red_button_frame"]),
        ]
        
    def run_sequence(self):
        for task_name, p_keys, p_vals in self.sequence:
            self.get_logger().info(f"== Starting {task_name} ==")
            client = ActionClient(self, TaskAction, f'/orms/{task_name}')
            
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"Action server /orms/{task_name} not available. Aborting sequence.")
                return
                
            goal_msg = TaskAction.Goal()
            goal_msg.task_name = task_name
            goal_msg.param_keys = p_keys
            goal_msg.param_values = p_vals
            
            send_goal_future = client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f"Goal for {task_name} rejected.")
                return
                
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future)
            result = get_result_future.result().result
            
            if not result.success:
                self.get_logger().error(f"Task {task_name} failed. Aborting sequence.")
                return
                
            self.get_logger().info(f"Task {task_name} succeeded in {result.execution_time_sec:.2f} seconds.")
            
        self.get_logger().info("== All tasks completed successfully ==")

def main(args=None):
    rclpy.init(args=args)
    node = SequencerNode()
    node.run_sequence()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
