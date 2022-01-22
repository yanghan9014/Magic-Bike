import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

#from action_tutorials_interfaces.action import Stm
from action_interface.action import Stm
# from interfaces.action import Action
import time
import serial


class STMNodeServer(Node):

    def __init__(self):
        super().__init__('stm_node_server')
        self.get_logger().info("Starting stm action server")
        self._action_server = ActionServer(
            self,
            Stm,
            'stm_action',
            self.execute_callback)
        
        self.serial_port = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=115200,
            timeout=0.5
        )
        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        print(goal_handle.request.cmd)
        sendMessage = goal_handle.request.cmd
        self.serial_port.write(sendMessage.encode('ascii'))
        start = time.time()
        getMessage = ""
        ret = True
        while True:
            if time.time() - start > 10 :
                ret = False
                # print("Time out")
                break
            
            if self.serial_port.inWaiting() > 0 :
                if time.time() - start > 10 :
                    ret = False
                    # print("Time out")
                    break
                data = self.serial_port.read()
                getMessage = getMessage + data.decode('ascii')
                if data == "\n".encode():
                    print("Get message: " + getMessage)
                    break
        
        goal_handle.succeed()
        result = Stm.Result()
        # print("len(getMessage): " + str(len(getMessage)))
        # print(getMessage)
        if ret == False :
            result.done = False
            return result
        if len(getMessage) != 4:
            result.done = False
            return result
        result.done = (getMessage[1] == 'd')
        if result.done :
            print("ret true")
        else :
            print("ret false")
        return result

def main(args=None):
    rclpy.init(args=args)

    stm = STMNodeServer()

    rclpy.spin(stm)


if __name__ == '__main__':
    main()
