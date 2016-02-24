#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy

from vstone_servo.msg import *
from vstone_servo.srv import *
from std_msgs.msg import Bool

import serial
import binascii
import struct

_serial = None

class DummySerial:
    def write(self, data):
        rospy.loginfo("[Debug] DummySerial:" + binascii.hexlify(data))

    def is_open(self):
        return False

    def read(self, data_length):
        return bytearray([0xFF for i in range(data_length)])

def sanitize_sid(sid):
    return sid & 0x3F

def access_servo_ram(_serial, sid, address, data, write_flag=True, read_flag=False):
    WRF = 0x40
    RDF = 0x20

    sid = sanitize_sid(sid)
    command = 0x80 | sid

    data_length = len(data)

    flags = data_length
    if read_flag == True:
        flags |= RDF
    if write_flag == True:
        flags |= WRF

    raw_packet = bytearray([command, flags, address])
    raw_packet.extend(encode_data(data))

    _serial.flush()
    _serial.write(raw_packet)
    _serial.flush()
    rospy.loginfo("Write Serial Packet :" + binascii.hexlify(raw_packet))

    if read_flag == True:
        read_data = _serial.read(data_length)
        _serial.flush()
        return decode_data(read_data)

    return None

def enable_callback(msg):
    rospy.loginfo("ServoEnable Message Received :")
    rospy.loginfo(msg)

    if msg.enable_state == True:
        access_servo_ram(_serial, msg.sid, 0x3b, [0x01])
    elif msg.enable_state == False:
        access_servo_ram(_serial, msg.sid, 0x3b, [0x00])

def lock_callback(msg):
    rospy.loginfo("ServoLock Message Received :")
    rospy.loginfo(msg)

    if msg.lock_state == True:
        access_servo_ram(_serial, msg.sid, 0x14, [0x00])
    elif msg.lock_state == False:
        access_servo_ram(_serial, msg.sid, 0x14, [0x55])

def activate_callback(msg):
    rospy.loginfo("ServoActivate Message Received :")
    rospy.loginfo(msg)

    if msg.data == False:
        # disable
        access_servo_ram(_serial, msg.sid, 0x3b, [0x00])
        # lock
        access_servo_ram(_serial, msg.sid, 0x14, [0x00])
    elif msg.data == True:
        # unlock
        access_servo_ram(_serial, msg.sid, 0x14, [0x55])
        # enable
        access_servo_ram(_serial, msg.sid, 0x3b, [0x01])

def write_callback(msg):
    rospy.loginfo("ServoWrite Message Received :")
    rospy.loginfo(msg)
    access_servo_ram(_serial, msg.sid, msg.address, msg.data)

def target_callback(msg):
    rospy.loginfo("ServoTarget Message Received :")
    rospy.loginfo(msg)
    data = [(msg.target_position>>8)&0xFF, msg.target_position&0xFF]
    access_servo_ram(_serial, msg.sid, 0x30, data)

def handle_read(req):
    rospy.loginfo("ServoRead Service Called :")
    rospy.loginfo(req)

    raw_read_data = access_servo_ram(_serial, req.write_parameters.sid, req.write_parameters.address, req.write_parameters.data, read_flag=True, write_flag=False)
    read_data = [byte for byte in raw_read_data]
    print(read_data)
    # [struct.unpack('B', byte)[0] for byte in sys.argv[4].decode('hex')]

    return ServoReadResponse(data=read_data, data_length=len(read_data))

# vservo用にビックエンディアン、整数のリストからリトルエンディアン、7ビット区切りのbytearrayを生成
def encode_data(source_data):
    raw_data = 0
    # [int] -> [byte]
    source_data = bytearray(source_data)

    # [byte] -> int
    if len(source_data) == 1:
        raw_data = struct.unpack('B', source_data)[0]
    elif len(source_data) == 2:
        raw_data = struct.unpack('>H', source_data)[0]
    elif len(source_data) == 4:
        raw_data = struct.unpack('>I', source_data)[0]
    else:
        return None

    # align 7bit
    encoded_data = []
    for i in range(len(source_data)):
        encoded_data.append(raw_data&0x7F)
        raw_data >>= 7

    return bytearray(encoded_data)

# vservoのパケット（リトルエンディアン、７ビット区切り、bytearray）からビックエンディアン、整数のリストを生成
def decode_data(packet_data):
    raw_data = 0
    # [byte] -> 7bit aligned int
    if len(packet_data) == 1:
        raw_data = struct.unpack('B', packet_data)[0]
    elif len(packet_data) == 2:
        raw_data = struct.unpack('<H', packet_data)[0]
    elif len(packet_data) == 4:
        raw_data = struct.unpack('<I', packet_data)[0]
    else:
        return None

    # realign 8bit
    decoded_data_int = 0
    for i in reversed(range(len(packet_data))):
        decoded_data_int <<= 7
        decoded_data_int |= (raw_data>>(i*8)) & 0x7F

    # 8bit aligned int -> [int]
    decoded_data = []
    for i in range(len(packet_data)):
        decoded_data.append(decoded_data_int&0xFF)
        decoded_data_int >>= 8
    decoded_data = list(reversed(decoded_data))

    return decoded_data

def controller():
  rospy.init_node("serial_controller")

  port = rospy.get_param('~port', '/dev/ttyUSB0')
  baud = rospy.get_param('~baud', 115200)
  debug_flag = rospy.get_param('~debug', False)

  global _serial
  if debug_flag == True:
      _serial = DummySerial()
  else:
      _serial = serial.Serial(port, int(baud), timeout=0.5)

  # サブスクライバー
  rospy.Subscriber("servo_enable",    ServoEnable, enable_callback)
  rospy.Subscriber("servo_lock",      ServoLock,   lock_callback)
  rospy.Subscriber("servo_activate",  Bool,        activate_callback)
  rospy.Subscriber("servo_write",     ServoWrite,  write_callback)
  rospy.Subscriber("servo_target",    ServoTarget, target_callback)

  # サービス
  rospy.Service('servo_read', ServoRead, handle_read)

  rospy.spin()

  if _serial.is_open():
      _serial.close()

if __name__ == "__main__":
  try:
    controller()
  except rospy.ROSInterruptException:
    pass
