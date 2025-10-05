import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')

        # ---- Pin map (BCM) ----
        self.right_motor_a = 24
        self.right_motor_b = 23
        self.right_motor_en = 25

        self.left_motor_a  = 15   # consider changing off UART pins if needed
        self.left_motor_b  = 14   # consider changing off UART pins if needed
        self.left_motor_en = 4

        # ---- GPIO init ----
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.right_motor_a, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_motor_b, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.right_motor_en, GPIO.OUT)

        GPIO.setup(self.left_motor_a,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.left_motor_b,  GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.left_motor_en, GPIO.OUT)

        # 1 kHz PWM, start STOPPED (0% duty)
        self.pwm_r = GPIO.PWM(self.right_motor_en, 1000)
        self.pwm_l = GPIO.PWM(self.left_motor_en, 1000)
        self.pwm_r.start(0)
        self.pwm_l.start(0)

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_to_pwm_callback, 10
        )

        # Failsafe: stop if no cmd_vel for 0.5 s
        self.last_cmd_time = time.time()
        self.timeout_sec = 0.5
        self.create_timer(0.1, self._failsafe_tick)

        # Scale: |velocity| >= 1.0 -> 100% duty
        self.max_abs_vel = 1.0

    def _set_motor(self, a_pin, b_pin, duty):
        # duty is 0..100 float
        # Note: direction pins LOW/LOW is brake/coast depending on driver;
        # adjust if your H-bridge expects a different scheme.
        if duty == 0:
            GPIO.output(a_pin, GPIO.LOW)
            GPIO.output(b_pin, GPIO.LOW)
        # Direction pins are set in callback; enable PWM handles speed.

    def _failsafe_tick(self):
        if time.time() - self.last_cmd_time > self.timeout_sec:
            # stop both motors
            self.pwm_r.ChangeDutyCycle(0)
            self.pwm_l.ChangeDutyCycle(0)
            GPIO.output(self.right_motor_a, GPIO.LOW)
            GPIO.output(self.right_motor_b, GPIO.LOW)
            GPIO.output(self.left_motor_a,  GPIO.LOW)
            GPIO.output(self.left_motor_b,  GPIO.LOW)

    def cmd_to_pwm_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        # Simple differential drive mapping
        right_wheel_vel = (msg.linear.x + msg.angular.z) / 2.0
        left_wheel_vel  = (msg.linear.x - msg.angular.z) / 2.0

        # Direction pins
        GPIO.output(self.right_motor_a, GPIO.HIGH if right_wheel_vel > 0 else GPIO.LOW)
        GPIO.output(self.right_motor_b, GPIO.HIGH if right_wheel_vel < 0 else GPIO.LOW)
        GPIO.output(self.left_motor_a,  GPIO.HIGH if left_wheel_vel  > 0 else GPIO.LOW)
        GPIO.output(self.left_motor_b,  GPIO.HIGH if left_wheel_vel  < 0 else GPIO.LOW)

        # Duty cycle from magnitude (clamped 0..100)
        duty_r = max(0.0, min(100.0, abs(right_wheel_vel) / self.max_abs_vel * 100.0))
        duty_l = max(0.0, min(100.0, abs(left_wheel_vel)  / self.max_abs_vel * 100.0))

        self.pwm_r.ChangeDutyCycle(duty_r)
        self.pwm_l.ChangeDutyCycle(duty_l)
        print(right_wheel_vel, "/", left_wheel_vel)
    def destroy_node(self):
        # Stop and cleanup safely
        try:
            self.pwm_r.ChangeDutyCycle(0)
            self.pwm_l.ChangeDutyCycle(0)
            self.pwm_r.stop()
            self.pwm_l.stop()
        except Exception:
            pass
        GPIO.output(self.right_motor_a, GPIO.LOW)
        GPIO.output(self.right_motor_b, GPIO.LOW)
        GPIO.output(self.left_motor_a,  GPIO.LOW)
        GPIO.output(self.left_motor_b,  GPIO.LOW)
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VelocitySubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
