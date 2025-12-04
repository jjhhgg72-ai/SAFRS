#!/usr/bin/env python3
"""
motor_command_sub.py
---------------------
Main Pi에서 생성되는 /cmd_vel → JSON 명령을
Motor Control RPi에서 수신하여 Arduino로 전달하기 위한 브릿지.

동작:
1) Main Pi에서 ZMQ PUB(5000)로 lx, az(선속도/각속도) JSON 전송
2) MotorPi에서 SUB 후 cmd 해석
3) lx, az 값을 기반으로 ‘w/a/s/d/x’ 단일 문자 생성
4) motor_serial_bridge.py 로 전달 (ZMQ PUB → 6003)
5) motor_serial_bridge.py 가 Arduino에 시리얼로 전송
"""

import zmq
import json

MAIN_PI_IP = "172.30.1.78"
ZMQ_MAIN_CMD_PORT = 5000

ctx = zmq.Context()

# -------- Main Pi 명령 구독 --------
main_sub = ctx.socket(zmq.SUB)
main_sub.connect(f"tcp://{MAIN_PI_IP}:{ZMQ_MAIN_CMD_PORT}")
main_sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("[MotorPi] main CMD SUB ready (5000)")

# -------- Arduino 브리지 노드로 송신 --------
bridge_pub = ctx.socket(zmq.PUB)
bridge_pub.bind("tcp://*:6003")
print("[MotorPi] bridge CMD PUB bind (6003)")

while True:
    # 1) Main Pi → JSON 명령 수신
    raw = main_sub.recv_string()
    print("[MotorPi] Received:", raw)

    try:
        data = json.loads(raw)
        lx = data["lx"]   # linear.x
        az = data["az"]   # angular.z
    except:
        continue

    # 2) /cmd_vel → 단일 문자 명령 변환
    if lx > 0.1:
        cmd = "w"          # forward
    elif lx < -0.1:
        cmd = "s"          # backward
    elif az > 0.1:
        cmd = "a"          # turn left
    elif az < -0.1:
        cmd = "d"          # turn right
    else:
        cmd = "x"          # stop

    # 3) 브리지로 송신 → Arduino에 전달됨
    print("[MotorPi] SEND TO BRIDGE:", cmd)
    bridge_pub.send_string(cmd)
