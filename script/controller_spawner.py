#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

from vstone_servo.msg import *
from vstone_servo.srv import *
from std_msgs.msg import UInt16

servo_target_pub = None

def base_callback(motor_config, msg):
    target = msg.data
    target = min(target, motor_config['max'])
    target = max(target, motor_config['min'])
    servo_target_msg = ServoTarget(sid = motor_config['id'], target_position = target)
    servo_target_pub.publish(servo_target_msg)

def get_callback(motor_config):
    return lambda msg: base_callback(motor_config, msg)

def controller_spawner():

    global servo_target_pub
    servo_target_pub = rospy.Publisher('/robovie/vstone_servo/servo_target', ServoTarget, queue_size=1000)
    servo_lock_pub = rospy.Publisher('/robovie/vstone_servo/servo_lock', ServoLock, queue_size=1000)
    servo_enable_pub = rospy.Publisher('/robovie/vstone_servo/servo_enable', ServoEnable, queue_size=1000)

    rospy.init_node("controller_spawner")
    params = rospy.get_param('~config')
    rospy.loginfo(params)

    for motor_name, motor_config in params.items():
        # 各サーボ専用のトピックを立ち上げていく

        servo_lock_msg = ServoLock()
        servo_lock_msg.sid = motor_config['id']
        servo_lock_msg.lock_state = False

        servo_enable_msg = ServoEnable()
        servo_enable_msg.sid = motor_config['id']
        servo_enable_msg.enable_state = True

        servo_lock_pub.publish(servo_lock_msg)
        servo_target_pub.publish(ServoTarget(sid = motor_config['id'], target_position = motor_config['init']))
        servo_enable_pub.publish(servo_enable_msg)

        callback_method = get_callback(motor_config)
        rospy.Subscriber(motor_name + '_controller', UInt16, callback_method)

    rospy.spin()

if __name__ == '__main__':
      try:
        controller_spawner()
      except rospy.ROSInterruptException:
        pass
