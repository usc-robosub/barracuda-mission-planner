#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


class MissionRoutineNode:

    def __init__(self):
        self.pub = rospy.Publisher("/mission/status", String, queue_size=10)

    def _command_cb(self, msg: String):
        cmd = msg.data.strip().lower().split(' ')[0]
        if cmd == "start":
            if not self._running:
                self._running = True
                self._idx = 0
                rospy.loginfo("Mission routine started")
                self.pub.publish(String(data="started"))
            else:
                rospy.loginfo("Mission routine already running")
        elif cmd == "stop":
            if self._running:
                self._running = False
                rospy.loginfo("Mission routine stopped")
                self.pub.publish(String(data="stopped"))
            else:
                rospy.loginfo("Mission routine already idle")
        else:
            rospy.logwarn("Unknown command on /mission/command: %s", cmd)



def main():
    rospy.init_node("mission_routine_node")
    _node = MissionRoutineNode()
    rospy.spin()


if __name__ == "__main__":
    main()

