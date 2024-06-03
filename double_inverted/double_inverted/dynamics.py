import numpy as np
import time
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from custom_msgs.msg import TorqueInput, States

class double_pendulum(Node):

    theta1_0 = pi - 0.2
    theta1_dot0 = 0.0
    theta2_0 = pi - 0.1
    theta2_dot0 = 0.0

    mass1 = 1.0
    mass2 = 1.0
    g = 9.81
    l1 = 1.0
    l2 = 1.0
    state_update_frequency = 500
    state_update_timeperiod = 1/state_update_frequency
    feedback_frequency = 50

    def __init__(self):
        super().__init__('main')

        # Timers
        update_states_timer = self.create_timer(1/self.state_update_frequency, self.update_pendulum_states)
        #feedback_timer = self.create_timer(1/self.feedback_frequency, self.feedback)

        # pubsubs
        self.visualizer = self.create_publisher(Marker, '/pendulum_viz', 1)
        # self.feedback_pub = self.create_publisher(States, '/state_feedback', 1)
        # self.input = self.create_subscription(TorqueInput, '/torque_input', self.update_input_torque, 5)


        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001
        self.obj_id = 0

        self.theta1 = self.theta1_0
        self.theta2 = self.theta2_0
        self.theta1_dot = self.theta1_dot0
        self.theta2_dot = self.theta2_dot0
        self.get_logger().info('Double Pendulum node initialized')
        self.get_logger().info('Accepting Input')
        self.get_logger().info('Publishing Feedback')

    
    def update_pendulum_states(self):

        dt = time.time() - self.t_prev
        self.t_prev = time.time()


        num1 = -self.g * (2 * self.mass1 + self.mass2) * sin(self.theta1)
        num2 = -self.mass2 * self.g * sin(self.theta1 - 2 * self.theta2)
        num3 = -2 * sin(self.theta1 - self.theta2) * self.mass2 *((self.theta2_dot**2)*self.l2 + (self.theta1_dot**2)*self.l1*cos(self.theta1 - self.theta2))
        den = self.l1 * (2 * self.mass1 + self.mass2 - self.mass2*cos(2*self.theta1 - 2*self.theta2))
        alpha1 = (num1 + num2 + num3)/den
        num1 = 2 * sin(self.theta1 - self.theta2) 
        num2 = (self.theta1_dot**2)*self.l1*(self.mass1 + self.mass2)
        num3 = self.g * cos(self.theta1) * (self.mass1 + self.mass2)
        num4 = (self.theta2_dot**2)* self.l2 * self.mass2 * cos(self.theta1 - self.theta2)
        den = self.l2 * (2*self.mass1 + self.mass2 -self.mass2*cos(2*self.theta1 - 2*self.theta2))
        alpha2 = num1 * (num2 + num3 + num4) / den
        
        self.theta1_dot += alpha1 * dt
        self.theta2_dot += alpha2 * dt
        self.theta1 += self.theta1_dot * dt + 0.5 * dt * dt * alpha1
        self.theta2 += self.theta2_dot * dt + 0.5 * dt * dt * alpha2
        

        self.theta1 = (self.theta1 + np.pi)%(2*np.pi) - np.pi
        self.theta2 = (self.theta2 + np.pi)%(2*np.pi) - np.pi

        self.visualize_pendulum()

        return
    
    def visualize_pendulum(self):
        pendulum_marker = Marker()
        pendulum_marker.header.frame_id = "map"
        pendulum_marker.id = self.obj_id
        pendulum_marker.type = Marker.LINE_LIST
        pendulum_marker.action = Marker.ADD
        pendulum_marker.pose.orientation.w = 1.0
        pendulum_marker.scale.x = 0.05  

        point_1 = Point()
        point_1.x = 0.0
        point_1.y = 0.0
        point_1.z = 0.0

        point_2 = Point()
        point_2.x = self.l1 * sin(self.theta1)
        point_2.y = - self.l1 * cos(self.theta1)
        point_2.z = 0.0

        point_3 = Point()
        point_3.x = point_2.x + self.l2 * sin(self.theta2)
        point_3.y = point_2.y - self.l2 * cos(self.theta2)
        point_3.z = 0.0
        
        pendulum_marker.points = [point_1, point_2, point_2, point_3]

        pendulum_marker.color.r = 1.0
        pendulum_marker.color.a = 1.0
        Duration_of_pendulum_marker = Duration()
        Duration_of_pendulum_marker.sec = 0
        Duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum_marker.lifetime = Duration_of_pendulum_marker 
        self.visualizer.publish(pendulum_marker)

        self.obj_id += 1


def main(args = None):
    rclpy.init(args=args)
    pendulum = double_pendulum()
    rclpy.spin(pendulum)
    
    pendulum.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


