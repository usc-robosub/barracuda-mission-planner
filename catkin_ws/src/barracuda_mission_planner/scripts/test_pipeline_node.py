#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from barracuda_msgs.msg import ObjectPose, ObjectPoseArray, Waypoints

try:
    import yaml  # For parsing YAML-style dicts from launch params
except Exception:  # yaml may not be available, handle gracefully
    yaml = None


class PipelineTestNode:
    def __init__(self):
        self.namespace = rospy.get_param("~namespace", "/barracuda")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 2.0))
        # gate_position may arrive as a dict (preferred) or a YAML string from <param>.
        raw_gate = rospy.get_param("~gate_position", {"x": 5.0, "y": 1.5, "z": 0.0})
        self.gate_position = self._normalize_gate_position(raw_gate)

        self.objects_topic = self.namespace.rstrip('/') + "/object_pose_array"
        self.path_topic = self.namespace.rstrip('/') + "/planned_path"

        self.pub_objects = rospy.Publisher(self.objects_topic, ObjectPoseArray, queue_size=10)
        self.sub_path = rospy.Subscriber(self.path_topic, Waypoints, self._path_cb, queue_size=10)

        self._timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 0.1)), self._tick)
        rospy.loginfo("test_pipeline_node publishing fake objects to %s; listening for path on %s",
                      self.objects_topic, self.path_topic)

    def _tick(self, _evt):
        # Publish a single fake 'gate' detection
        arr = ObjectPoseArray()
        arr.header = Header()
        arr.header.stamp = rospy.Time.now()
        arr.header.frame_id = self.frame_id

        obj = ObjectPose()
        obj.header = arr.header
        obj.object_id = 1
        obj.object_class = "gate"
        obj.pose = Pose()
        obj.pose.position = Point(self.gate_position.get("x", 0.0),
                                  self.gate_position.get("y", 0.0),
                                  self.gate_position.get("z", 0.0))
        obj.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        arr.objects = [obj]
        self.pub_objects.publish(arr)

    def _path_cb(self, msg: Waypoints):
        if msg.points:
            p = msg.points[0]
            rospy.loginfo("Received planned path waypoint: (%.2f, %.2f, %.2f) frame=%s",
                          p.x, p.y, p.z, msg.header.frame_id)

    def _normalize_gate_position(self, value):
        """Return a dict with x,y,z floats given various param encodings.
        Accepts a dict (already parsed) or a YAML string like "{x: 5.0, y: 1.5, z: 0.0}".
        Falls back to sensible defaults on failure.
        """
        default = {"x": 5.0, "y": 1.5, "z": 0.0}
        # If already a dict, ensure it has numeric x/y/z
        if isinstance(value, dict):
            result = {
                "x": float(value.get("x", default["x"])),
                "y": float(value.get("y", default["y"])),
                "z": float(value.get("z", default["z"]))
            }
            return result
        # If provided as a string, attempt to parse as YAML (e.g., from <param value="{x: 1, y: 2, z: 3}")
        if isinstance(value, str):
            if yaml is not None:
                try:
                    parsed = yaml.safe_load(value)
                    if isinstance(parsed, dict):
                        return {
                            "x": float(parsed.get("x", default["x"])),
                            "y": float(parsed.get("y", default["y"])),
                            "z": float(parsed.get("z", default["z"]))
                        }
                    else:
                        rospy.logwarn("~gate_position YAML did not parse to dict; using defaults: %s", value)
                except Exception as e:
                    rospy.logwarn("Failed to parse ~gate_position as YAML (%s); using defaults. raw=\"%s\"", e, value)
            else:
                rospy.logwarn("PyYAML not available; cannot parse string ~gate_position. Using defaults. raw=\"%s\"", value)
        else:
            rospy.logwarn("Unexpected type for ~gate_position (%s); using defaults.", type(value).__name__)
        return default


def main():
    rospy.init_node("test_pipeline_node")
    _node = PipelineTestNode()
    rospy.spin()


if __name__ == "__main__":
    main()
