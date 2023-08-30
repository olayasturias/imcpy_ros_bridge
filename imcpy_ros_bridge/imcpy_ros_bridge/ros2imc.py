import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import imcpy
from imcpy.actors.dynamic import DynamicActor
from imcpy.decorators import Periodic, RunOnce, Subscribe
import numpy as np

# Acknowledgements: Laszlo u da best


class Ros2Imc(DynamicActor):
    def __init__(self, node_name='ros2imc', target_name='lauv-simulator-1'):
        super().__init__()  
        self.connected = False

        self.ros_node = Node(node_name)

        # IMC STUFF
        self.target_name = target_name
        # This list contains the target systems to maintain communications with
        self.heartbeat.append(target_name)        
        # This command starts the asyncio event loop
        self.run()

    @Periodic(10)
    def send_state(self):
        try:
            # This function resolves the map of connected nodes
            node = self.resolve_node_id(self.target_name)

            # CReate a new logbook entry
            log_book_entry = imcpy.LogBookEntry()
            log_book_entry.type = 0
            log_book_entry.timestamp = time.time()
            log_book_entry.context = 'CONTROL_STATE'
            log_book_entry.text = 'SURFACED' + str(time.time())

            # print(log_book_entry)
            # print(type(log_book_entry))

            # Send the IMC message to the node
            self.send(node, log_book_entry)
            self.ros_node.get_logger().info('Logbook connected: ' + str(self.connected))
            return True

        except KeyError as e:
            # Target system is not connected
            self.ros_node.get_logger().info('Target system is not connected.')
            return False
        
    @Periodic(10)
    def send_PlanControl(self):
        try:
            # This function resolves the map of connected nodes
            node = self.resolve_node_id(self.target_name)

            # Create PlanControl msg
            plan_control_msg = imcpy.PlanControl()
            plan_control_msg.type = 0 # Request
            plan_control_msg.op = 0 # Start
            plan_control_msg.plan_id = 'plan1' 
    
            plan_control_msg.flags = 0
            plan_control_msg.info = 'info'
            


            # CReate goto msg
            goto_msg = imcpy.Goto()
            goto_msg.lat = 41.1854111111111*np.pi/180
            goto_msg.lon = -8.705886111111111*np.pi/180
            goto_msg.z = 2.0
            goto_msg.z_units = 1
            goto_msg.speed = 1.
            goto_msg.speed_units = 0

            plan_control_msg.arg = goto_msg

            # print(log_book_entry)
            # print(type(log_book_entry))

            # Send the IMC message to the node
            self.send(node, goto_msg)
            self.ros_node.get_logger().info('GOTO sent')

        except KeyError as e:
            # Target system is not connected
            self.ros_node.get_logger().info('Could not deliver GOTO.')

    # @Subscribe(imcpy.ManeuverControlState)
    # def recv_plandbstate(self, msg: imcpy.ManeuverControlState):
    #     self.ros_node.get_logger().info('Maneuver control state: ' + str(msg))  

    @Subscribe(imcpy.PlanControlState)
    def recv_plandbstate(self, msg: imcpy.PlanControlState):
        self.ros_node.get_logger().info('Plan control state: ' + str(msg))  

    # @Subscribe(imcpy.ManeuverDone)
    # def recv_plandbstate(self, msg: imcpy.ManeuverDone):
    #     self.ros_node.get_logger().info('Maneuver DONE WEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE' )
    # @Subscribe(imcpy.TCPStatus)
    # def recv_tcpstatus(self, msg: imcpy.TCPStatus):
    #     self.ros_node.get_logger().info('TCP status: ' + str(msg))

    # @Subscribe(imcpy.EstimatedState)
    # def recv_estate(self, msg: imcpy.EstimatedState):
    #     self.ros_node.get_logger().info('Estimated state: ' + str(msg))
  


def main():
    print('Hi from ros2imc.')
    rclpy.init()
    ros2imc = Ros2Imc(node_name='ros2imc', target_name='lauv-simulator-1')
    
    while not ros2imc.send_state():
        ros2imc.ros_node.get_logger().info('Connecting to target system.')
    ros2imc.ros_node.get_logger().info('Connected to target system.')
    ros2imc.send_PlanControl()

    rclpy.spin(ros2imc.ros_node)

    ros2imc.ros_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
