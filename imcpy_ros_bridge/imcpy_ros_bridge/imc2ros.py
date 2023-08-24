import rclpy
import time
from rclpy.node import Node
from imc_ros_msgs.msg import DesiredHeading, DesiredHeadingRate, DesiredPitch, DesiredRoll, DesiredSpeed, DesiredZ
from imc_ros_msgs.msg import EstimatedState, Maneuver, PolygonVertex, RemoteState, SonarData, VehicleState
from imc_ros_msgs.msg import PlanControl, PlanControlState, PlanDB, PlanDBInformation,PlanDBState, PlanManeuver, PlanSpecification
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
        self.desired_heading_publisher_     = self.ros_node.create_publisher(DesiredHeading,    'desired_heading', 10)
        self.desired_heading_rate_publisher_= self.ros_node.create_publisher(DesiredHeadingRate,'desired_heading_rate', 10)
        self.desired_pitch_publisher_       = self.ros_node.create_publisher(DesiredPitch,      'desired_pitch',10)
        self.desired_roll_publisher_        = self.ros_node.create_publisher(DesiredRoll,       'desired_roll',10)
        self.desired_speed_publisher_       = self.ros_node.create_publisher(DesiredSpeed,      'desired_speed',10)
        self.desired_z_publisher_           = self.ros_node.create_publisher(DesiredZ,          'desired_Z',10)
        self.pose_publisher_                = self.ros_node.create_publisher(PoseStamped,       'base_link', 10)
        self.EE_publisher_                  = self.ros_node.create_publisher(EstimatedState,    'estimated_state', 10)
        self.maneuver_publisher_            = self.ros_node.create_publisher(Maneuver,          'maneuver',10)
        self.plan_control_publisher_        = self.ros_node.create_publisher(PlanControl,       'plan_control',10)
        self.plan_control_state_publisher_  = self.ros_node.create_publisher(PlanControlState,  'plan_control_state',10)
        self.plan_db_publisher_             = self.ros_node.create_publisher(PlanDB,            'plan_DB',10)
        self.plan_db_information_publisher_ = self.ros_node.create_publisher(PlanDBInformation, 'plan_DB_information',10)
        self.plan_db_state_publisher_       = self.ros_node.create_publisher(PlanDBState,       'plan_db_state',10)
        self.plan_maneuver_publisher_       = self.ros_node.create_publisher(PlanManeuver,      'plan_maneuver',10)
        self.plan_specification_publisher_  = self.ros_node.create_publisher(PlanSpecification, 'plan_specification',10)
        self.polygon_vertex_publisher_      = self.ros_node.create_publisher(PolygonVertex,     'polygon_vertex',10)
        self.remote_state_publisher_        = self.ros_node.create_publisher(RemoteState,       'remote_state',10)
        self.sonar_data_publisher_          = self.ros_node.create_publisher(SonarData,         'sonar_data',10)
        self.vehicle_state_publisher_       = self.ros_node.create_publisher(VehicleState,      'vehicle_state',10)
        
        # This command starts the asyncio event loop
        self.run()

    @Subscribe(imcpy.DesiredHeading)
    def recv_heading(self, msg: imcpy.DesiredHeading):
        self.imc_DH_to_ros(msg)

    @Subscribe(imcpy.DesiredHeadingRate)
    def recv_headingrate(self, msg: imcpy.DesiredHeadingRate):
        self.imc_DHR_to_ros(msg)

    @Subscribe(imcpy.DesiredPitch)
    def recv_pitch(self, msg: imcpy.DesiredPitch):
        self.imc_DP_to_ros(msg)

    @Subscribe(imcpy.DesiredRoll)
    def recv_roll(self, msg: imcpy.DesiredRoll):
        self.imc_DR_to_ros(msg)

    @Subscribe(imcpy.DesiredSpeed)
    def recv_speed(self, msg: imcpy.DesiredSpeed):
        self.imc_DS_to_ros(msg)

    @Subscribe(imcpy.DesiredZ)
    def recv_depth(self, msg: imcpy.DesiredZ):
        self.imc_DZ_to_ros(msg)

    @Subscribe(imcpy.EstimatedState)
    def recv_estate(self, msg: imcpy.EstimatedState):
        self.imc_EE_to_ros(msg)

    @Subscribe(imcpy.Maneuver)
    def recv_maneuver(self, msg: imcpy.Maneuver):
        self.imc_M_to_ros(msg)

    @Subscribe(imcpy.PlanControl)
    def recv_plan(self, msg: imcpy.PlanControl):
        self.imc_PC_to_ros(msg)

    @Subscribe(imcpy.PlanControlState)
    def recv_planstate(self, msg: imcpy.PlanControlState):
        self.imc_PCS_to_ros(msg)

    @Subscribe(imcpy.PlanDB)
    def recv_plandb(self, msg: imcpy.PlanDB):
        self.imc_PDB_to_ros(msg)

    @Subscribe(imcpy.PlanDBState)
    def recv_plandbstate(self, msg: imcpy.PlanDBState):
        self.imc_PDBS_to_ros(msg)

    @Subscribe(imcpy.PlanManeuver)
    def recv_planmaneuver(self, msg: imcpy.PlanManeuver):
        self.imc_PM_to_ros(msg)

    @Subscribe(imcpy.PlanSpecification)
    def recv_planspec(self, msg: imcpy.PlanSpecification):
        self.imc_PS_to_ros(msg)

    @Subscribe(imcpy.PolygonVertex)
    def recv_polygonvertex(self, msg: imcpy.PolygonVertex):
        self.imc_PV_to_ros(msg)

    @Subscribe(imcpy.RemoteState)
    def recv_remotestate(self, msg: imcpy.RemoteState):
        self.imc_RS_to_ros(msg)

    @Subscribe(imcpy.SonarData)
    def recv_sonar(self, msg: imcpy.SonarData):
        self.imc_SD_to_ros(msg)

    @Subscribe(imcpy.VehicleState)
    def recv_vstate(self, msg: imcpy.VehicleState):
        self.imc_VS_to_ros(msg)
    

    


    def imc_DH_to_ros(self, imc_msg: imcpy.DesiredHeading):
        msg = DesiredHeading()
        msg.value = imc_msg.value
        self.desired_heading_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Desired Heading {}.'.format(imc_msg.value))


    def imc_DHR_to_ros(self, imc_msg: imcpy.DesiredHeadingRate):
        msg = DesiredHeadingRate()
        msg.value = imc_msg.value
        self.desired_heading_rate_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Desired Heading Rate {}.'.format(imc_msg.value))

    def imc_DP_to_ros(self, imc_msg: imcpy.DesiredPitch):
        msg = DesiredPitch()
        msg.value = imc_msg.value
        self.desired_pitch_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Desired Pitch {}.'.format(imc_msg.value))

    def imc_DR_to_ros(self, imc_msg: imcpy.DesiredRoll):
        msg = DesiredRoll()
        msg.value = imc_msg.value
        self.desired_roll_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Desired Roll {}.'.format(imc_msg.value))
    
    def imc_DS_to_ros(self, imc_msg: imcpy.DesiredSpeed):
        msg = DesiredSpeed()
        msg.value = imc_msg.value
        msg.speed_units = imc_msg.speed_units
        self.desired_speed_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Desired Speed {}.'.format(imc_msg.value))
    
    def imc_DZ_to_ros(self, imc_msg: imcpy.DesiredZ):
        msg = DesiredZ()
        msg.value = imc_msg.value
        msg.z_units = imc_msg.z_units
        self.desired_z_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Desired Z {}.'.format(imc_msg.value))
    
    def imc_EE_to_ros(self, imc_msg: imcpy.EstimatedState):
        # Publish on PoseStamped (for UNavSim)
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
        # Publish on Estimated State (for ROS2)
        msg = EstimatedState()
        msg.x = imc_msg.x
        msg.y = imc_msg.y
        msg.z = imc_msg.z
        msg.phi = imc_msg.phi
        msg.theta = imc_msg.theta
        msg.psi = imc_msg.psi
        msg.u = imc_msg.u
        msg.v = imc_msg.v
        msg.w = imc_msg.w
        msg.p = imc_msg.p
        msg.q = imc_msg.q
        msg.r = imc_msg.r
        msg.depth = imc_msg.depth
        msg.alt = imc_msg.alt
        msg.lat = imc_msg.lat
        msg.lon = imc_msg.lon
        msg.height = imc_msg.height
        self.EE_publisher_.publish(msg)

    def imc_M_to_ros(self, imc_msg: imcpy.Maneuver):
        msg = Maneuver()
        msg.maneuver_name = imc_msg.maneuver_name
        msg.maneuver_id = imc_msg.maneuver_id
        msg.lat = imc_msg.lat
        msg.lon = imc_msg.lon
        msg.z = imc_msg.z
        msg.z_units = imc_msg.z_units
        msg.speed = imc_msg.speed
        msg.speed_units = imc_msg.speed_units
        msg.roll = imc_msg.roll
        msg.pitch = imc_msg.pitch
        msg.yaw = imc_msg.yaw
        msg.timeout = imc_msg.timeout
        msg.custom_string = imc_msg.custom_string
        msg.syringe0 = imc_msg.syringe0
        msg.syringe1 = imc_msg.syringe1
        msg.syringe2 = imc_msg.syringe2
        msg.polygon = imc_msg.polygon

        self.maneuver_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published maneuver {}.'.format(imc_msg.maneuver_name))
    
    def imc_PC_to_ros(self, imc_msg: imcpy.PlanControl):
        msg = PlanControl()
        msg.plan_id = imc_msg.plan_id
        msg.request_id = imc_msg.request_id
        msg.op = int(imc_msg.op)
        msg.type = int(imc_msg.type)
        msg.flags = imc_msg.flags
        msg.info = imc_msg.info

        self.plan_control_publisher_.publish(msg)

        self.ros_node.get_logger().debug('Published Plan Control {}.'.format(imc_msg.plan_id))
    
    def imc_PCS_to_ros(self, imc_msg: imcpy.PlanControlState):
        msg = PlanControlState()
        msg.plan_id = imc_msg.plan_id
        msg.plan_eta = imc_msg.plan_eta
        msg.plan_progress = imc_msg.plan_progress
        msg.man_id = imc_msg.man_id
        msg.man_type = imc_msg.man_type
        msg.man_eta = imc_msg.man_eta
        msg.last_outcome = int(imc_msg.last_outcome)

        self.plan_control_state_publisher_.publish(msg)

        self.ros_node.get_logger().debug('Published Plan Control State {}.'.format(imc_msg.plan_id))
    
    def imc_PDB_to_ros(self, imc_msg: imcpy.PlanDB):
        msg = PlanDB()
        msg.plan_id = imc_msg.plan_id
        msg.op = int(imc_msg.op)
        msg.type = int(imc_msg.type)
        msg.request_id = imc_msg.request_id
        # msg.plan_spec = self.imc_PS_to_ros(imc_msg.plan_spec)
        # msg.plan_spec_md5 = imc_msg.plan_spec_md5
        # msg.plandb_information = imc_msg.plandb_information
        # msg.plandb_state = imc_msg.plandb_state

        self.plan_db_publisher_.publish(msg)

        self.ros_node.get_logger().debug('Published Plan DB {}.'.format(imc_msg.plan_id))
    
    def imc_PDBI_to_ros(self, imc_msg: imcpy.PlanDBInformation):
        msg = PlanDBInformation()
        msg.plan_id = imc_msg.plan_id
        msg.plan_size = imc_msg.plan_size
        msg.change_time = imc_msg.change_time
        msg.change_sid = imc_msg.change_sid
        msg.change_sname = imc_msg.change_sname
        msg.md5 = imc_msg.md5

        self.plan_db_information_publisher_.publish(msg)

        self.ros_node.get_logger().debug('Published Plan DB Information {}.'.format(imc_msg.plan_id))
    
    def imc_PDBS_to_ros(self, imc_msg: imcpy.PlanDBState):
        msg = PlanDBState()
        msg.plan_count = imc_msg.plan_count
        msg.plan_size = imc_msg.plan_size
        msg.change_time = imc_msg.change_time
        msg.change_sid = imc_msg.change_sid
        msg.change_sname = imc_msg.change_sname
        msg.md5 = imc_msg.md5

        msg.plans_info = imc_msg.plans_info

        self.plan_db_state_publisher_.publish(msg)

        self.ros_node.get_logger().debug('Published Plan DB State {}.'.format(imc_msg.plan_count))

    def imc_PM_to_ros(self, imc_msg: imcpy.PlanManeuver):
        msg = PlanManeuver()
        msg.maneuver = imc_msg.maneuver

        self.plan_maneuver_publisher_.publish(msg)

        self.ros_node.get_logger().debug('Published Plan Maneuver {}.'.format(imc_msg.maneuver.maneuver_name))

    def imc_PS_to_ros(self, imc_msg: imcpy.PlanSpecification):
        msg = PlanSpecification()
        msg.plan_id = imc_msg.plan_id
        msg.description = imc_msg.description
        msg.vnamespace = imc_msg.vnamespace
        msg.start_man_id = imc_msg.start_man_id
        msg.maneuvers = imc_msg.maneuvers
        
        self.plan_specification_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Plan Specification {}.'.format(imc_msg.plan_id))
        return msg

    def imc_PV_to_ros(self, imc_msg: imcpy.PolygonVertex):
        msg = PolygonVertex()
        msg.lat = imc_msg.lat
        msg.lon = imc_msg.lon

        self.polygon_vertex_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Polygon Vertex {}.'.format(imc_msg.lat))


    def imc_RS_to_ros(self, imc_msg: imcpy.RemoteState):
        msg = RemoteState()
        msg.lat = imc_msg.lat
        msg.lon = imc_msg.lon
        msg.depth = imc_msg.depth
        msg.speed = imc_msg.speed
        msg.psi = imc_msg.psi

        self.remote_state_publisher_.publish(msg)

        self.ros_node.get_logger().debug('Published Remote State {}.'.format(imc_msg.lat))

    def imc_SD_to_ros(self, imc_msg: imcpy.SonarData):
        msg = SonarData()
        msg.type = int(imc_msg.type)
        msg.frequency = imc_msg.frequency
        msg.min_range = imc_msg.min_range
        msg.max_range = imc_msg.max_range
        msg.bits_per_point = imc_msg.bits_per_point
        msg.scale_factor = imc_msg.scale_factor
        msg.data = imc_msg.data

        self.sonar_data_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Sonar Data {}.'.format(imc_msg.type))

    def imc_VS_to_ros(self, imc_msg: imcpy.VehicleState):
        msg = VehicleState()
        msg.op_mode = int(imc_msg.op_mode)
        msg.error_count = imc_msg.error_count
        msg.error_ents = imc_msg.error_ents
        msg.maneuver_type = imc_msg.maneuver_type
        msg.maneuver_stime = imc_msg.maneuver_stime
        msg.maneuver_eta = imc_msg.maneuver_eta
        msg.control_loops = imc_msg.control_loops
        msg.flags = imc_msg.flags
        msg.last_error = imc_msg.last_error
        msg.last_error_time = imc_msg.last_error_time

        self.vehicle_state_publisher_.publish(msg)
        self.ros_node.get_logger().debug('Published Vehicle State {}.'.format(imc_msg.op_mode))


    @Periodic(0.5)
    def send_state(self):
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
            # self.ros_node.get_logger().info('message sent to node')

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
