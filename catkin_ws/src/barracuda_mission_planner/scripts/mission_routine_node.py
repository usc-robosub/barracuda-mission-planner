#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Point
from barracuda_msgs.msg import Waypoints
from barracuda_msgs.srv import GetObjectPosition, GetObjectPositionRequest


class MissionRoutineNode:

    def __init__(self):
        # Topics and service names
        self.status_topic = rospy.get_param("~status_topic", "/mission/status")
        self.command_topic = rospy.get_param("~command_topic", "/mission/command")
        self.planned_path_topic = rospy.get_param("~planned_path_topic", "planned_path")
        self.get_pos_service = rospy.get_param("~get_position_service", "get_object_position")
        self.target_class = rospy.get_param("~target_object_class", "gate")

        self._running = False
        self._idx = 0

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.path_pub = rospy.Publisher(self.planned_path_topic, Waypoints, queue_size=10)
        self.cmd_sub = rospy.Subscriber(self.command_topic, String, self._command_cb, queue_size=10)

        rospy.loginfo("mission_routine_node ready; listening for %s and will publish %s",
                      self.command_topic, self.planned_path_topic)

    def _publish_waypoint_to(self, point: Point, frame_id: str):
        wp = Waypoints()
        wp.header.stamp = rospy.Time.now()
        wp.header.frame_id = frame_id or ""
        wp.points = [point]
        self.path_pub.publish(wp)
        rospy.loginfo("Published planned_path with 1 waypoint to (%.2f, %.2f, %.2f) in %s",
                      point.x, point.y, point.z, frame_id)

    def _command_cb(self, msg: String):
        cmd = msg.data.strip().lower().split(' ')[0]
        if cmd == "start":
            if not self._running:
                self._running = True
                self._idx = 0
                rospy.loginfo("Mission routine started")
                self.status_pub.publish(String(data="started"))

                # Query object position via service and publish waypoint
                try:
                    rospy.wait_for_service(self.get_pos_service, timeout=3.0)
                    proxy = rospy.ServiceProxy(self.get_pos_service, GetObjectPosition)
                    req = GetObjectPositionRequest(object_class=self.target_class)
                    res = proxy(req)
                    if res.found:
                        self._publish_waypoint_to(res.position, res.frame_id)
                    else:
                        rospy.logwarn("Object class '%s' not found; no waypoint published", self.target_class)
                except rospy.ROSException:
                    rospy.logerr("Service %s not available; cannot get object position", self.get_pos_service)
                except rospy.ServiceException as e:
                    rospy.logerr("Service %s call failed: %s", self.get_pos_service, str(e))

                # Signal ready for the next command
                self.status_pub.publish(String(data="ready"))

            else:
                rospy.loginfo("Mission routine already running")
        elif cmd == "stop":
            if self._running:
                self._running = False
                rospy.loginfo("Mission routine stopped")
                self.status_pub.publish(String(data="stopped"))
                # Also signal readiness (idle state)
                self.status_pub.publish(String(data="ready"))
            else:
                rospy.loginfo("Mission routine already idle")
                self.status_pub.publish(String(data="ready"))
        else:
            rospy.logwarn("Unknown command on %s: %s", self.command_topic, cmd)
            # Unknown command processed; still allow progression
            self.status_pub.publish(String(data="ready"))


def main():
    rospy.init_node("mission_routine_node")
    _node = MissionRoutineNode()
    rospy.spin()


if __name__ == "__main__":
    main()
