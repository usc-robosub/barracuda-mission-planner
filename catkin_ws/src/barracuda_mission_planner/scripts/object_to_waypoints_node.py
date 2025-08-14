#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Point
from barracuda_msgs.msg import ObjectPoseArray
from barracuda_msgs.srv import (
    GetObjectPosition,
    GetObjectPositionRequest,
    GetObjectPositionResponse,
)


class ObjectToWaypointsNode:
    """
    Refactored: this node now provides a service to query the
    latest known position of an object class, instead of publishing
    waypoints directly. The mission planner should call the service
    and handle publishing.
    """

    def __init__(self):
        self.input_topic = rospy.get_param("~objectposearray_topic", "/objectposearray")
        self.service_name = rospy.get_param("~get_position_service", "get_object_position")

        self._latest_objects = None  # type: ObjectPoseArray | None
        self.sub = rospy.Subscriber(self.input_topic, ObjectPoseArray, self._objects_cb, queue_size=10)
        self.srv = rospy.Service(self.service_name, GetObjectPosition, self._handle_get_position)

        rospy.loginfo("object_to_waypoints_node listening on %s, providing service %s",
                      self.input_topic, self.service_name)

    def _objects_cb(self, msg: ObjectPoseArray):
        # Cache the latest detections for service queries
        self._latest_objects = msg

    def _handle_get_position(self, req: GetObjectPositionRequest) -> GetObjectPositionResponse:
        """Return the first detected object's position for the requested class."""
        want = (req.object_class or "").strip().lower()
        res = GetObjectPositionResponse()
        res.found = False
        res.frame_id = ""
        res.position = Point()  # defaults to zeros

        if self._latest_objects is None or not self._latest_objects.objects:
            rospy.logdebug("No cached ObjectPoseArray available for query '%s'", want)
            return res

        for obj in self._latest_objects.objects:
            obj_class = (obj.object_class or "").strip().lower()
            if obj_class == want:
                res.found = True
                res.frame_id = self._latest_objects.header.frame_id
                res.position.x = obj.pose.position.x
                res.position.y = obj.pose.position.y
                res.position.z = obj.pose.position.z
                rospy.loginfo("Service %s: found %s at (%.2f, %.2f, %.2f) in %s",
                              self.service_name, want, res.position.x, res.position.y, res.position.z, res.frame_id)
                return res

        rospy.loginfo("Service %s: no object of class '%s' found", self.service_name, want)
        return res


def main():
    rospy.init_node("object_to_waypoints_node")
    _node = ObjectToWaypointsNode()
    rospy.spin()


if __name__ == "__main__":
    main()
