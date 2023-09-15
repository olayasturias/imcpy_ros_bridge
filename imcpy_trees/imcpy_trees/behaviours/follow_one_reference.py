
##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
import imcpy
import geometry_msgs.msg as geometry_msgs
from imc_ros_msgs.msg import FollowReference, Reference, DesiredSpeed, DesiredZ


##############################################################################
# Behaviours
##############################################################################

class FollowOneReference(py_trees.behaviour.Behaviour):
    """
    This behaviour simply shoots a command off to the AUV to follow
    a single point returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command if it is cancelled or interrupted by a higher
    priority behaviour.

    Publishers:
        * **/led_strip/command** (:class:`std_msgs.msg.String`)

          * colourised string command for the led strip ['red', 'green', 'blue']

    Args:
        name: name of the behaviour
        topic_name : name of the battery state topic
        colour: colour to flash ['red', 'green', blue']
    """
    def __init__(
            self,
            name: str,
            lat: float,
            lon: float,
            z: float,
            speed: float,
            radius: float
    ):
        super(FollowOneReference, self).__init__(name=name)
        self.r = Reference()
        self.r.lat = lat
        self.r.lon = lon
        self.radius = radius
        self.r.z.value = z
        self.r.z.z_units = imcpy.ZUnits.DEPTH
        
        
        

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        # First we need to send FollowReference only once
        fr_publisher = self.node.create_publisher(
            msg_type = FollowReference,
            topic ='to_imc/follow_reference',
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )

        self.reference_pub = self.node.create_publisher(
            msg_type = Reference,
            topic = 'to_imc/reference',
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )

        fr = FollowReference()
        fr.control_src = 0xFFFF  # Controllable from all IMC adresses
        fr.control_ent = 0xFF  # Controllable from all entities
        fr.timeout = 10.0  # Maneuver stops when time since last Reference message exceeds this value
        fr.loiter_radius = -1  # Default loiter radius when waypoint is reached
        fr.altitude_interval = 0
        fr_publisher.publish(fr)

        self.feedback_message = "FollowReference sent"

    def update(self) -> py_trees.common.Status:
        """
        Annoy the led strip to keep firing every time it ticks over (the led strip will clear itself
        if no command is forthcoming within a certain period of time).
        This behaviour will only finish if it is terminated or priority interrupted from above.

        Returns:
            Always returns :attr:`~py_trees.common.Status.RUNNING`
        """
        if msg.state == imcpy.FollowRefState.StateEnum.GOTO:
            # In goto maneuver
            self.feedback_message = "Goto"
            if msg.proximity & imcpy.FollowRefState.ProximityBits.XY_NEAR:
                # Near XY - send next reference
                self.feedback_message = "------ Near XY"
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING

        elif msg.state in (imcpy.FollowRefState.StateEnum.LOITER, imcpy.FollowRefState.StateEnum.HOVER):
            # Loitering/hovering/waiting - send next reference
            self.feedback_message = "Point reached: Loiter/Hover"
            self.reference_pub.publish(self.r)
            self.send_reference(node_id=self.target_name)
            return py_trees.common.Status.SUCCESS

        elif msg.state == imcpy.FollowRefState.StateEnum.WAIT:
            # Loitering/hovering/waiting - send next reference
            self.feedback_message = "Waiting"
            self.send_reference(node_id=self.target_name)
            return py_trees.common.Status.RUNNING
        
        elif msg.state == imcpy.FollowRefState.StateEnum.ELEVATOR:
            # Moving in z-direction after reaching reference cylinder
            self.feedback_message = "Elevator"
            return py_trees.common.Status.RUNNING

        elif msg.state == imcpy.FollowRefState.StateEnum.TIMEOUT:
            # Controlling system timed out
            self.feedback_message = "TIMEOUT"
            return py_trees.common.Status.FAILURE


        

    def terminate(self, new_status: py_trees.common.Status):
        """
        Shoot off a clearing command to the led strip.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        self.feedback_message = "cleared"
