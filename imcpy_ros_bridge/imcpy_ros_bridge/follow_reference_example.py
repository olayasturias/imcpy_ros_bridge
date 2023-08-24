import logging, math, sys

import imcpy
from imcpy.actors import DynamicActor
from imcpy.decorators import Subscribe, Periodic
import rclpy

logger = logging.getLogger('examples.FollowRef')


class FollowRef(DynamicActor):
    def __init__(self, target):
        super().__init__()
        self.target = target
        self.heartbeat.append(target)
        self.state = None
        self.lat = 0.0
        self.lon = 0.0
        self.last_ref = False
        self.wp = [(50., 0.), (0.0, 50.), (-50, 0.), (0., -50.)]  # North/east offsets for waypoints
        self.wp_next = 0

    def send_reference(self, node_id, final=False):
        """
        After the FollowReferenceManeuver is started, references must be sent continously
        """
        try:
            next_coord = self.wp[self.wp_next % len(self.wp)]
            lat, lon = imcpy.coordinates.WGS84.displace(self.lat, self.lon, n=next_coord[0], e=next_coord[1])
            self.wp_next += 1

            node = self.resolve_node_id(node_id)
            r = imcpy.Reference()
            r.lat = lat  # Target waypoint
            r.lon = lon  # Target waypoint

            # Assign z
            dz = imcpy.DesiredZ()
            dz.value = 0.0
            dz.z_units = imcpy.ZUnits.DEPTH
            r.z = dz

            # Assign the speed
            ds = imcpy.DesiredSpeed()
            ds.value = 1.6
            ds.speed_units = imcpy.SpeedUnits.METERS_PS
            r.speed = ds

            # Bitwise flags (see IMC spec for explanation)
            flags = imcpy.Reference.FlagsBits.LOCATION | imcpy.Reference.FlagsBits.SPEED | imcpy.Reference.FlagsBits.Z
            flags = flags | imcpy.Reference.FlagsBits.MANDONE if final else flags
            r.flags = flags
            logger.info('Sending reference')
            self.last_ref = r
            self.send(node, r)
        except KeyError:
            pass

    def is_from_target(self, msg):
        """
        Check that incoming message is from the target system
        """
        try:
            node = self.resolve_node_id(msg)
            return node.sys_name == self.target
        except KeyError:
            return False

    @Periodic(10)
    def init_followref(self):
        """
        If target is connected, start the FollowReferenceManeuver
        """
        if not self.state:
            # Check if target system is connected
            try:
                node = self.resolve_node_id(self.target)
                fr = imcpy.FollowReference()
                fr.control_src = 0xFFFF  # Controllable from all IMC adresses
                fr.control_ent = 0xFF  # Controllable from all entities
                fr.timeout = 10.0  # Maneuver stops when time since last Reference message exceeds this value
                fr.loiter_radius = 0  # Default loiter radius when waypoint is reached
                fr.altitude_interval = 0

                # Add to PlanManeuver message
                pman = imcpy.PlanManeuver()
                pman.data = fr
                pman.maneuver_id = 'FollowReferenceManeuver'

                # Add to PlanSpecification
                spec = imcpy.PlanSpecification()
                spec.plan_id = 'FollowReference'
                spec.maneuvers.append(pman)
                spec.start_man_id = 'FollowReferenceManeuver'
                spec.description = 'A test plan sent from imcpy'

                # Start plan
                pc = imcpy.PlanControl()
                pc.type = imcpy.PlanControl.TypeEnum.REQUEST
                pc.op = imcpy.PlanControl.OperationEnum.START
                pc.plan_id = 'FollowReference'
                pc.arg = spec

                self.send(node, pc)

                logger.info('Started FollowRef command')
            except KeyError:
                pass

    @Subscribe(imcpy.EstimatedState)
    def recv_estate(self, msg):
        if self.is_from_target(msg):
            self.lat, self.lon, _ = imcpy.coordinates.toWGS84(msg)

    @Subscribe(imcpy.FollowRefState)
    def recv_followrefstate(self, msg: imcpy.FollowRefState):
        if not self.is_from_target(msg):
            return

        logger.info('Received FollowRefState')
        self.state = msg.state

        if msg.state == imcpy.FollowRefState.StateEnum.GOTO:
            # In goto maneuver
            logger.info('Goto')
            if msg.proximity & imcpy.FollowRefState.ProximityBits.XY_NEAR:
                # Near XY - send next reference
                logger.info('-- Near XY')
                self.send_reference(node_id=self.target)
        elif msg.state in (imcpy.FollowRefState.StateEnum.LOITER, imcpy.FollowRefState.StateEnum.HOVER, imcpy.FollowRefState.StateEnum.WAIT):
            # Loitering/hovering/waiting - send next reference
            logger.info('Waiting')
            self.send_reference(node_id=self.target)
        elif msg.state == imcpy.FollowRefState.StateEnum.ELEVATOR:
            # Moving in z-direction after reaching reference cylinder
            logger.info('Elevator')
        elif msg.state == imcpy.FollowRefState.StateEnum.TIMEOUT:
            # Controlling system timed out
            logger.info('Timeout')

    @Periodic(1.0)
    def periodic_ref(self):
        if self.last_ref:
            try:
                self.send(self.target, self.last_ref)
            except KeyError:
                pass

def main():
    #ROS stuff
    rclpy.init()
    node = rclpy.create_node('follow_reference_example')
    

    # IMCPY stuff
    # Setup logging level and console output
    logging.basicConfig(stream=sys.stdout, level=logging.INFO)

    # Run actor
    x = FollowRef('lauv-simulator-1')
    x.run()
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

