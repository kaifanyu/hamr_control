#!/usr/bin/env python3
"""Exercise actual relay serial parsing/covariances/node clock using a local PTY.

No physical serial device or robot command topics are used. Run with sourced ROS:
ROS_DOMAIN_ID=191 /usr/bin/python3 bridge_integration_test.py /path/to/relay_node
"""
import json
import math
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import tempfile
import time
import zlib
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, MagneticField

binary=sys.argv[1];results=[]
# Invalid covariance overrides must fail before opening any serial device.
for param in ['imu_gyro_z_variance','imu_yaw_variance']:
 for invalid in ['0.0','-0.001','.nan','.inf']:
    with tempfile.NamedTemporaryFile('w',suffix='.yaml') as cfg:
        cfg.write('/**:\n  ros__parameters:\n    '+param+': '+invalid+'\n');cfg.flush()
        result=subprocess.run([binary,'--ros-args','--params-file',cfg.name],capture_output=True,text=True,timeout=8)
    expected=param+' must be finite and positive'
    assert expected in result.stderr,(param,invalid,result.stdout,result.stderr)
    assert 'open serial' not in result.stderr
    results.append({'parameter':param,'value':invalid,'rejected_before_serial_open':True})

master,slave=pty.openpty();device=os.ttyname(slave)
rclpy.init();node=Node('hamr_bridge_covariance_test');received=[];magnetic=[]
node.create_subscription(Imu,'/imu/data',received.append,10)
node.create_subscription(MagneticField,'/imu/mag',magnetic.append,10)
pub=node.create_publisher(Clock,'/clock',10)
cmd=[binary,'--ros-args','-p','serial_port:='+device,'-p','use_sim_time:=true',
     '-p','imu_gyro_z_variance:=0.0123','-p','imu_yaw_variance:=0.0456']
relay=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
clock=Clock();clock.clock.sec=123;clock.clock.nanosec=456000000
try:
    deadline=time.monotonic()+8;seq=0
    while time.monotonic()<deadline:
        pub.publish(clock)
        # TYPE_IMU_EXT, wire version 4; complete sensor+mag+calibration payload.
        seq+=1
        payload=struct.pack('<HHHIQ12f4B',0xCAFE,4,0x0005,seq,1000000,
                            0.,0.,.1, .1,.2,.3, .01,.02,.3, 10.,20.,30., 0,3,1,0)
        packet=payload+struct.pack('<H',zlib.crc32(payload)&0xFFFF)
        os.write(master,packet)
        rclpy.spin_once(node,timeout_sec=.02)
        if received and magnetic and received[-1].header.stamp.sec==123 and magnetic[-1].header.stamp.sec==123:
            break
    assert received,'No IMU output decoded from test serial packet'
    assert magnetic,'No magnetometer output decoded from test serial packet'
    imu=received[-1];mag=magnetic[-1]
    assert imu.angular_velocity_covariance[8]==.0123
    assert imu.orientation_covariance[8]==.0456
    assert imu.angular_velocity_covariance[0]==.0002
    assert imu.orientation_covariance[0]==.0003
    assert math.isclose(imu.angular_velocity.z,.3,abs_tol=1e-6)
    assert math.isclose(mag.magnetic_field.x,10e-6,abs_tol=1e-10)
    for msg in [imu,mag]:
        assert msg.header.stamp.sec==123,msg.header.stamp
        assert msg.header.stamp.nanosec==456000000,msg.header.stamp
    results.append(dict(serial_packet_decoding=True,gyro_override=.0123,yaw_override=.0456,
      default_other_axes_preserved=True,imu_sim_time=True,mag_sim_time=True))
finally:
    relay.send_signal(signal.SIGINT)
    try:stdout,stderr=relay.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        relay.kill();stdout,stderr=relay.communicate(timeout=5)
    os.close(master);os.close(slave);node.destroy_node();rclpy.shutdown()
Path(__file__).with_name('bridge_integration_test.json').write_text(json.dumps(results,indent=2))
print(json.dumps(results,indent=2))
