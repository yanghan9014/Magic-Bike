import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_interface.action import Stm
import time

class MainActionClient(Node):
    def __init__(self):
        super().__init__('stm_action_client')
        self._action_client = ActionClient(self, Stm, 'stm_action')
        self.success = False
        print("Init action client node")

    def send_goal(self, cmd):
        goal_msg = Stm.Goal()
        goal_msg.cmd = cmd
        print("Sent goal:", cmd)
        
        # print("Connecting to action server...")
        self._action_client.wait_for_server()
        # print("Connected. Sending action goal...")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.success = result.done
        # self.get_logger().info('Result: {}'.format(result.done))
        print("getting result callback")
        # rclpy.shutdown()

#def main(args=None):
#    rclpy.init()
#    action_client = MainActionClient()
#    action_client.send_goal("l100")
#    
#    rclpy.spin(action_client)
#    if action_client.success:
#        print("action success")
#    else :
#        print("action failed")
