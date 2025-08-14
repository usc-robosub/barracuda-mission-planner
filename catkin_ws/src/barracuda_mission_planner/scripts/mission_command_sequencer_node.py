#!/usr/bin/env python3
import os
import rospy

from std_msgs.msg import String


class MissionCommandSequencer:
    def __init__(self):
        # Parameters
        self.commands_file = rospy.get_param("~commands_file", "")
        self.command_topic = rospy.get_param("~command_topic", "/mission/command")
        self.status_topic = rospy.get_param("~status_topic", "/mission/status")
        self.ready_token = rospy.get_param("~ready_token", "ready").strip().lower()
        self.initial_delay = float(rospy.get_param("~initial_delay", 0.5))

        if not self.commands_file:
            rospy.logfatal("Parameter ~commands_file is required")
            raise SystemExit(2)
        if not os.path.isabs(self.commands_file):
            # Resolve relative to this package share/config
            pkg_dir = rospy.get_param("~package_share_dir", "")
            if pkg_dir:
                self.commands_file = os.path.join(pkg_dir, self.commands_file)

        # Load commands
        self.commands = self._load_commands(self.commands_file)
        if not self.commands:
            rospy.logfatal("No commands found in %s", self.commands_file)
            raise SystemExit(2)

        # Pub/Sub
        self.pub_cmd = rospy.Publisher(self.command_topic, String, queue_size=10)
        self.sub_status = rospy.Subscriber(self.status_topic, String, self._status_cb, queue_size=10)

        self._idx = 0
        self._waiting = False

        # Kickoff timer to send the first command after a small delay
        rospy.Timer(rospy.Duration(self.initial_delay), self._initial_send, oneshot=True)
        rospy.loginfo("Sequencer ready: %d commands, waiting to publish first on %s",
                      len(self.commands), self.command_topic)

    def _load_commands(self, path):
        cmds = []
        try:
            with open(path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    cmds.append(line)
        except Exception as e:
            rospy.logerr("Failed to read commands file %s: %s", path, str(e))
            return []
        return cmds

    def _publish_current(self):
        if self._idx >= len(self.commands):
            rospy.loginfo("Sequencer completed all commands")
            return
        cmd = self.commands[self._idx]
        self.pub_cmd.publish(String(data=cmd))
        rospy.loginfo("Sequencer published command [%d/%d]: %s",
                      self._idx + 1, len(self.commands), cmd)
        self._idx += 1
        self._waiting = True

    def _initial_send(self, _evt):
        # Ensure there is at least one subscriber before sending
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and self.pub_cmd.get_num_connections() == 0:
            if (rospy.Time.now() - t0).to_sec() > 5.0:
                break
            rospy.sleep(0.05)
        self._publish_current()

    def _status_cb(self, msg: String):
        token = (msg.data or "").strip().lower()
        if token == self.ready_token and self._waiting:
            self._waiting = False
            if self._idx < len(self.commands):
                self._publish_current()
            else:
                rospy.loginfo("Sequencer done; no further commands to publish")


def main():
    rospy.init_node("mission_command_sequencer")
    _node = MissionCommandSequencer()
    rospy.spin()


if __name__ == "__main__":
    main()

