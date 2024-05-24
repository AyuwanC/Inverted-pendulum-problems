from numpy import pi, cos
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
    kp, ki, kd = 200, 1, 1
    flag = False
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
    def currEnergy(theta, omega):
        pot = 9.81 * (1 - cos(theta))
        kin = 0.5 * omega * omega
        return pot + kin
    
    def callback(self, msgin:States):
        msgout = TorqueInput()

        # PID
        if self.flag:
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
            torque = limited(torque, 5)

            self.eprev = enow
            msgout.torque_value = torque * 1.0
        #/PID
        else:
            omega = msgin.theta_dot
            pot = 9.81 * (1 - cos(msgin.theta))
            kin = 0.5 * omega * omega
            energy = pot + kin
            if ((-pi <= msgin.theta <= -0.88*pi) and msgin.theta_dot >= 0) or ((0.88*pi <= msgin.theta <= pi) and msgin.theta_dot <=0):
                self.flag = True
                msgout.torque_value = 0.0
            else:
                if 0 <= msgin.theta < pi/4 and msgin.theta_dot >= 0:
                    msgout.torque_value = 4.0
                elif -pi/4 <= msgin.theta < 0 and msgin.theta_dot <= 0:
                    msgout.torque_value = -4.0
                else: 
                    msgout.torque_value = 0.0
       
        self.torque.publish(msgout)

def main(args = None):
    rclpy.init(args = args)
    control = controller()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()