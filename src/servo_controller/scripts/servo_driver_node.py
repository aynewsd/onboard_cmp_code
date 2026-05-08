#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from servo_controller.msg import ServoCmd

# --- 硬件配置 (BCM编码)---
# 舵机引脚映射表: 舵机ID (servo_id) -> GPIO引脚编号
SERVO_PINS = {
    1: 17, # 舵机1接GPIO17
    2: 27, # 舵机2接GPIO27
    3: 22  # 舵机3接GPIO22
}
# 舵机角度对应的占空比 (基于50Hz PWM)
# 经验值，若不准可在2.5~12.5范围内微调[reference:6]
DUTY_ANGLE_MAP = {
    0: 2.5,   # 0度: 脉宽0.5ms -> 占空比2.5%
    90: 7.5   # 90度: 脉宽1.5ms -> 占空比7.5%[reference:7]
}

def setup_gpio():
    """初始化GPIO和PWM"""
    GPIO.setmode(GPIO.BCM)
    pwms = {}
    for servo_id, pin in SERVO_PINS.items():
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, 50) # 频率50Hz
        pwm.start(0)
        pwms[servo_id] = pwm
    return pwms

def callback(msg, pwms):
    """ROS订阅回调函数"""
    servo_id = msg.servo_id
    pos = msg.position

    if servo_id not in pwms:
        rospy.logerr("无效的舵机ID: %d", servo_id)
        return

    # 根据约定将 position 字段(0或1)转换为目标角度(0或90)
    target_angle = 90 if pos == 1 else 0
    duty_cycle = DUTY_ANGLE_MAP.get(target_angle)

    if duty_cycle is not None:
        rospy.loginfo("控制舵机 %d 转到 %d 度，占空比: %.1f%%", servo_id, target_angle, duty_cycle)
        pwms[servo_id].ChangeDutyCycle(duty_cycle)
        rospy.sleep(0.5) # 等待舵机转动到位
        # 清理占空比以防止舵机抖动
        pwms[servo_id].ChangeDutyCycle(0)
    else:
        rospy.logwarn("收到无效角度指令: %d", target_angle)

def driver_node():
    """ROS节点主函数"""
    rospy.init_node('servo_driver_node', anonymous=False)
    pwms = setup_gpio()

    # 注册清理函数，确保程序退出时释放GPIO
    rospy.on_shutdown(lambda: [p.stop() for p in pwms.values()] or GPIO.cleanup())

    # 订阅自定义话题 /servo_control
    sub = rospy.Subscriber('/servo_control', ServoCmd, callback, callback_args=pwms, queue_size=10)

    rospy.loginfo("舵机驱动节点已启动，等待指令...")
    rospy.spin() # 保持节点运行

if __name__ == '__main__':
    driver_node()