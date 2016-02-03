#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import rospy
import binascii
import struct
from vstone_servo.srv import *
from vstone_servo.msg import *

def servo_read_client(sid, data_length, address, data):
    rospy.wait_for_service('servo_read')
    try:
        servo_read = rospy.ServiceProxy('servo_read', ServoRead)

        servo_read_request = ServoReadRequest()
        servo_read_request.write_parameters.sid = sid
        servo_read_request.write_parameters.data_length = data_length
        servo_read_request.write_parameters.address = address
        servo_read_request.write_parameters.data = data

        read_value = servo_read(servo_read_request)

        return read_value
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [sid data_length address data]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        sid = int(sys.argv[1], 16)
        data_length = int(sys.argv[2], 16)
        address = int(sys.argv[3], 16)
        data = [byte for byte in bytearray(sys.argv[4].decode('hex'))]
    else:
        print usage()
        sys.exit(1)
    print "Requesting sid:%X data_length:%X address:%X data:%s" % (sid, data_length, address, data)
    response = servo_read_client(sid, data_length, address, data)
    print('response')
    print(response)
