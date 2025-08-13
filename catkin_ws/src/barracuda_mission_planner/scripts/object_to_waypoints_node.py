#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Point
from barracuda_msgs.msg import ObjectPoseArray, Waypoints


class ObjectToWaypointsNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~objectposearray_topic", "/objectposearray")
        self.output_topic = rospy.get_param("~planned_path_topic", "planned_path")

        self.pub = rospy.Publisher(self.output_topic, Waypoints, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, ObjectPoseArray, self._objects_cb, queue_size=10)

        rospy.loginfo("object_to_waypoints_node listening on %s, publishing to %s",
                      self.input_topic, self.output_topic)

    def _objects_cb(self, msg: ObjectPoseArray):
        if not msg.objects:
            rospy.logdebug("No objects in ObjectPoseArray; skipping")
            return

        obj = msg.objects[0]
        obj_id = obj.object_id
        obj_class = (obj.object_class or "").strip().lower()
        pose = obj.pose

        rospy.loginfo_throttle(1.0, "First object -> id: %s, class: %s", obj_id, obj.object_class)

        if obj_class != "gate":
            rospy.logdebug("First object is not a gate (%s); skipping", obj.object_class)
            return

        # Build a single-point Waypoints message at the gate position
        wp = Waypoints()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = msg.header.frame_id  # assume same frame as detections

        p = Point()
        p.x = pose.position.x
        p.y = pose.position.y
        p.z = pose.position.z
        wp.points = [p]

        self.pub.publish(wp)
        rospy.loginfo("Published planned_path with 1 waypoint to gate at (%.2f, %.2f, %.2f)", p.x, p.y, p.z)


def main():
    rospy.init_node("object_to_waypoints_node")
    _node = ObjectToWaypointsNode()
    rospy.spin()


if __name__ == "__main__":
    main()

