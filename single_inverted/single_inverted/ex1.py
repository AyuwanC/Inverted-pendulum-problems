from numpy import pi, tan
import time
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from custom_msgs.msg import States, TorqueInput
def limited(x, value):
    if x > value:
        return value
    elif x < -value:
        return -value
    else:
        return x


class controller(Node):
    aim = pi
    eprev = 0
    I = 0
    kp, ki, kd = 100, 1, 1
    def __init__(self):
        super().__init__('controller')
        self.subscription = self.create_subscription(
            States,
            "/state_feedback",
            self.callback,
            10
        )
        self.torque = self.create_publisher(
            TorqueInput,
            "/torque_input",
            10
        )
    def callback(self, msgin:States):
        msgout = TorqueInput()

        # PID
        tdiff = 1/500

        if msgin.theta > 0:
            enow = self.aim - abs(msgin.theta)
        else:
            enow = -(msgin.theta + self.aim)
        
        P = self.kp * enow

        D = self.kd * (enow - self.eprev)/tdiff
        print(enow)
        torque = P + self.I + D
        self.I += self.ki * enow
        torque = limited(torque, 20)

        self.eprev = enow
        msgout.torque_value = torque *1.0
        self.torque.publish(msgout)

def main(args = None):
    rclpy.init(args = args)
    control = controller()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()