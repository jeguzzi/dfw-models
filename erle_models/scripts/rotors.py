#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import numpy as np
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State

MIN_OMEGA = 0.5
THROTTLE2OMEGA = 70.0


class Propellers(object):
    def __init__(self):
        rospy.init_node('propellers')
        self.angle = np.random.random(4) * np.pi * 2
        self.direction = np.array([-1.0, -1.0, 1.0, 1.0])
        self.omega = self.direction * 0
        self.throttle = 0.0
        self.status = 3
        self.armed = False
        self.msg = JointState()
        self.msg.header.frame_id = 'base_link'
        self.msg.name = ['rotor_0_joint', 'rotor_1_joint', 'rotor_2_joint', 'rotor_3_joint']
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        rospy.Subscriber('omega', Float32, self.update_omega)
        rospy.Subscriber('mavros/state', State, self.update_state)
        rospy.Subscriber('mavros/vfr_hud', VFR_HUD, self.update_throttle)
        rospy.Timer(rospy.Duration(0.01), self.update)
        rospy.spin()

    def omega_from_state(self):
        if self.armed:
            if self.status == 3:
                return MIN_OMEGA
            else:
                return max(MIN_OMEGA, THROTTLE2OMEGA * self.throttle)
        else:
            return 0

    def update_state(self, msg):
        self.armed = msg.armed
        self.status = msg.system_status
        self.omega = self.omega_from_state()

    def update_throttle(self, msg):
        self.throttle = msg.throttle
        self.omega = self.omega_from_state()

    def update_omega(self, msg):
        self.omega = self.direction * msg.data

    def update(self, evt):
        if evt.last_real:
            dt = (evt.current_real - evt.last_real).to_sec()
        else:
            dt = 0.0
        self.angle += self.omega * dt
        self.msg.header.stamp = rospy.Time.now()
        self.msg.position = self.angle
        self.pub.publish(self.msg)


if __name__ == '__main__':
    Propellers()
