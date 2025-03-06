import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
import signal
import math

class CircleWalker(Node):
    def __init__(self):
        super().__init__('circlewalker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.radius = 1.0  # Radius of the circle (adjust as needed)
        self.angular_speed = 0.2  # Angular speed (adjust as needed)
        self.linear_speed = self.radius * self.angular_speed # Linear speed to match circle

    def move_in_circle(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed  # Linear velocity
        twist_msg.angular.z = self.angular_speed # Angular velocity

        while rclpy.ok(): # Loop to continue publishing the Twist message
            self.publisher.publish(twist_msg)
            self.rate.sleep()

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        desired_velocity.angular.z = 0.0
        self.publisher.publish(desired_velocity)

def main():
    def signal_handler(sig, frame):
        first_walker.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    first_walker = CircleWalker()

    signal.signal(signal.SIGINT, signal_handler)

    thread = threading.Thread(target=rclpy.spin, args=(first_walker,), daemon=True)
    thread.start()

    try:
        first_walker.move_in_circle() # Call the circle movement function
    except ROSInterruptException:
        pass

if __name__ == "__main__":
    main()