import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import imcpy
from imcpy.actors.dynamic import DynamicActor
from imcpy.decorators import Periodic, Subscribe

from imcpy.network.utils import get_interfaces

# Acknowledgements: Laszlo u da best


class Imc2Ros(DynamicActor):
    def __init__(self, node_name='imc2ros', target_name='lauv-simulator-1'):
        super().__init__()  
        self.ros_node = Node(node_name)

        # IMC STUFF
        self.target_name = target_name
        # This list contains the target systems to maintain communications with
        self.heartbeat.append(target_name)

        # ROS STUFF
        self.pose_publisher_ = self.ros_node.create_publisher(PoseStamped, 'base_link', 10)

        # debugging stuff
        
        # if self._port_imc:  # Port must be ready to build IMC service string
        #     services = ['imc+udp://{}:{}/'.format(adr[1], self._port_imc) for adr in get_interfaces()]
        #     self.ros_node.get_logger().info('services: {}'.format(services))
        #     if not self.services:
        #         # No external interfaces available, announce localhost/loopback
        #         self.services = ['imc+udp://{}:{}/'.format(adr[1], self._port_imc) for adr in get_interfaces(False)]
        #         self.ros_node.get_logger().info('services: {}'.format(self.services))
        
        # This command starts the asyncio event loop
        self.run()


    @Subscribe(imcpy.EstimatedState)
    def recv_estate(self, msg: imcpy.EstimatedState):
        self.imc_pose_to_ros(msg)

    def imc_pose_to_ros(self, imc_msg: imcpy.EstimatedState):
        msg = PoseStamped()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'dune_map'
        msg.pose.position.x = imc_msg.x
        msg.pose.position.y = imc_msg.y
        msg.pose.position.z = imc_msg.z
        msg.pose.orientation.x = imc_msg.phi
        msg.pose.orientation.y = imc_msg.theta
        msg.pose.orientation.z = imc_msg.psi
        self.pose_publisher_.publish(msg)

    @Periodic(0.5)
    def send_state(self):
        self.ros_node.get_logger().info('helo omelettes')
        try:
            # self.ros_node.get_logger().info(self.__dict__)
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
            self.ros_node.get_logger().info('message sent to node')

        except KeyError as e:
            # Target system is not connected
            self.ros_node.get_logger().info('Target system is not connected.')


def main():
    print('Hi from imcpy_ros_bridge.')
    rclpy.init()
    imc2ros = Imc2Ros(node_name='imc2ros', target_name='lauv-simulator-1')
    
    rclpy.spin(imc2ros.ros_node)

    imc2ros.ros_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
