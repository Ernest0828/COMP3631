import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
import signal
import math
import time

class SquareWalker(Node):
    def __init__(self):
        super().__init__('square_walker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.side_length = 1.0  # Length of each side of the square (adjust as needed)
        self.linear_speed = 0.2  # Linear speed (adjust as needed)
        self.angular_speed = math.pi / 2  # 90 degrees turn in 1 second for testing. Adjust as needed.

    def move_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        for _ in range(int(self.side_length / self.linear_speed * 10)):  # Calculate steps for side length
            self.publisher.publish(twist_msg)
            self.rate.sleep()

    def turn_left(self):
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_speed
        for _ in range(10): # Adjust this number of steps to control the turn. It is related to the angular speed.
            self.publisher.publish(twist_msg)
            self.rate.sleep()

    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

    def move_in_square(self):
        while rclpy.ok():
            for _ in range(4):  # Repeat 4 times for a square
                self.move_forward()
                self.turn_left()


def main():
    def signal_handler(sig, frame):
        square_walker.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    square_walker = SquareWalker()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(square_walker,), daemon=True)
    thread.start()

    try:
        square_walker.move_in_square()
    except ROSInterruptException:
        pass

if __name__ == "__main__":
    main()