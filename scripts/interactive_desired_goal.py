import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from geometry_msgs.msg import Point, Pose


# This script is to create a 6-dof interactive marker to be used to indicate a user's desired pose.
# Later on can be changed to be used with an omni.

class DesiredGoalInteractiveMarker(object):
    def __init__(self):
        rospy.init_node("desired_goal_interactive_marker")

        self.server = InteractiveMarkerServer("desired_goal_interactive_marker")
        self.interactive_marker_pub = rospy.Publisher("desired_goal", Pose, queue_size=10)
        self.current_pose = Pose()

        pos = Point(0, 0, 3)
        self.make_6dof_desired_goal_marker(pos)

        self.server.applyChanges()

        # create a timer to update the published transforms
        rospy.Timer(rospy.Duration(0.01), self.publish_callback)

        rospy.spin()

    def publish_callback(self, msg):
        self.interactive_marker_pub.publish(self.current_pose)

    def process_feedback(self, feedback):
        self.current_pose = feedback.pose

    def normalize_quaternion(self, quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def make_box_control(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.make_box(msg))
        msg.controls.append(control)
        return control

    def make_box(self, msg):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 1.0 
        marker.scale.y = msg.scale * 1.0 
        marker.scale.z = msg.scale * 1.0 
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        return marker

    def make_6dof_desired_goal_marker(self, position):
        # Visual marker settings
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.pose.position = position
        int_marker.scale = 1.5
        int_marker.name = "desired_goal"
        int_marker.description = "Desired goal marker for CTR"

        # insert a box
        self.make_box_control(int_marker)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        self.normalize_quaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        self.normalize_quaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        self.normalize_quaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        self.normalize_quaternion(control.orientation)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        self.normalize_quaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        self.normalize_quaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.process_feedback)


if __name__ == '__main__':
    desired_goal_marker = DesiredGoalInteractiveMarker()
