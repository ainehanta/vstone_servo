#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

from vstone_servo.msg import *
from vstone_servo.srv import *
from std_msgs.msg import UInt16
from std_msgs.msg import Bool

servo_target_pub = None

def base_callback(motor_config, msg):
    servo_target_msg = ServoTarget(sid = motor_config['id'], target_position = msg.data)
    servo_target_pub.publish(servo_target_msg)

def get_callback(motor_config):
    return lambda msg: base_callback(motor_config, msg)

def controller_spawner():
    rospy.init_node("controller_spawner")

    params = rospy.get_param('~config')
    rospy.loginfo(params)

    global servo_target_pub
    servo_target_pub = rospy.Publisher('servo_target', ServoTarget, queue_size=10)
    servo_lock_pub = rospy.Publisher('servo_lock', Bool, queue_size=10)
    servo_enable_pub = rospy.Publisher('servo_enable', Bool, queue_size=10)

    for motor_name, motor_config in params.items():
        # 各サーボ専用のトピックを立ち上げていく
        servo_lock_pub.publish(False)
        servo_target_pub.publish(ServoTarget(sid = motor_config['id'], target_position = motor_config['init']))
        servo_enable_pub.publish(True)
        callback_method = get_callback(motor_config)
        rospy.Subscriber(motor_name + '_controller', UInt16, callback_method)

    rospy.spin()

if __name__ == '__main__':
      try:
        controller_spawner()
      except rospy.ROSInterruptException:
        pass
