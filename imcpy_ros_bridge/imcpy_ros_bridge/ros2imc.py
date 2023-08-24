import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import imcpy
from imcpy.actors.dynamic import DynamicActor
from imcpy.decorators import Periodic, Subscribe

# Acknowledgements: Laszlo u da best


class Ros2Imc(DynamicActor):
    def __init__(self, node_name='ros2imc', target_name='lauv-simulator-1'):
        super().__init__()  
        self.ros_node = Node(node_name)

        # IMC STUFF
        self.target_name = target_name
        # This list contains the target systems to maintain communications with
        self.heartbeat.append(target_name)        
        # This command starts the asyncio event loop
        self.run()


    @Subscribe(imcpy.EstimatedState)
    def recv_estate(self, msg: imcpy.EstimatedState):
        self.imc_pose_to_ros(msg)


    @Periodic(0.5)
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
            print('message sent to node')

        except KeyError as e:
            # Target system is not connected
            print('Target system is not connected.')


def main():
    print('Hi from ros2imc.')
    rclpy.init()
    ros2imc = Ros2Imc(node_name='ros2imc', target_name='lauv-simulator-1')
    
    rclpy.spin(ros2imc.ros_node)

    ros2imc.ros_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
