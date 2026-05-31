#!/usr/bin/env python3
import rospy
import pigpio
import time
from servo_controller.msg import ServoCmd

# --- 硬件配置 (BCM编码)---
# 舵机ID -> GPIO引脚
SERVO_PINS = {
    1: 12,   # 舵机1接GPIO12 (硬件PWM)
    2: 13,   # 舵机2接GPIO13 (硬件PWM)
    3: 26    # 舵机3接GPIO26 (普通GPIO，软件PWM)
}
HARDWARE_PWM_PINS = {12, 13}   # 硬件PWM引脚集合

def angle_to_pulsewidth(angle):
    """
    将角度(0~180)转换为脉宽(微秒)
    0°  -> 500us
    90° -> 1500us
    180°-> 2500us (线性关系)
    """
    if angle == 0:
        return 500
    elif angle == 90:
        return 1500
    else:
        # 其他角度可扩展，目前仅用0和90
        return 500 + (angle / 180.0) * 2000

def driver_node():
    rospy.init_node('servo_driver_node', anonymous=False)

    # 连接pigpio守护进程 (必须先运行 sudo pigpiod)
    pi = pigpio.pi()
    if not pi.connected:
        rospy.logerr("无法连接到pigpio守护进程，请先运行 'sudo pigpiod'")
        return

    # 设置所有舵机引脚为输出模式（pigpio会在使用PWM时自动设置，但预先设置无妨）
    for pin in SERVO_PINS.values():
        pi.set_mode(pin, pigpio.OUTPUT)

    rospy.loginfo("舵机驱动节点已启动，等待指令...")
    rospy.loginfo("硬件PWM引脚: 12,13   软件PWM引脚: 26")

    def callback(msg):
        servo_id = msg.servo_id
        pos = msg.position      # 约定: 0->0度, 1->90度

        if servo_id not in SERVO_PINS:
            rospy.logerr("无效的舵机ID: %d", servo_id)
            return

        pin = SERVO_PINS[servo_id]
        target_angle = 90 if pos == 1 else 0
        pulsewidth_us = angle_to_pulsewidth(target_angle)

        rospy.loginfo("控制舵机 %d (GPIO%d) 转到 %d 度 (脉宽 %d us)",
                      servo_id, pin, target_angle, pulsewidth_us)

        if pin in HARDWARE_PWM_PINS:
            # 硬件PWM: 频率50Hz，占空比 = 脉宽 / 20000us * 1e6
            # duty = pulsewidth_us * 50  (因为 1e6/20000 = 50)
            duty = pulsewidth_us * 50
            pi.hardware_PWM(pin, 50, duty)      # 启动硬件PWM
            rospy.sleep(0.5)                   # 等待舵机转动
            pi.hardware_PWM(pin, 50, 0)        # 停止输出，防止抖动
        else:
            # 软件PWM: 使用set_servo_pulsewidth，支持所有引脚
            pi.set_servo_pulsewidth(pin, pulsewidth_us)
            rospy.sleep(0.5)
            pi.set_servo_pulsewidth(pin, 0)    # 停止输出

    # 订阅话题 /servo_control
    sub = rospy.Subscriber('/servo_control', ServoCmd, callback, queue_size=10)

    def cleanup():
        rospy.loginfo("正在清理GPIO...")
        for servo_id, pin in SERVO_PINS.items():
            if pin in HARDWARE_PWM_PINS:
                pi.hardware_PWM(pin, 50, 0)    # 停止硬件PWM
            else:
                pi.set_servo_pulsewidth(pin, 0) # 停止软件PWM
            pi.set_mode(pin, pigpio.INPUT)     # 恢复为输入，释放引脚
        pi.stop()
        rospy.loginfo("清理完成")

    rospy.on_shutdown(cleanup)
    rospy.spin()

if __name__ == '__main__':
    driver_node()