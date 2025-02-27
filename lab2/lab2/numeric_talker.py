import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8


class Talker(Node):
    def __init__(self):
        super().__init__('numeric_talker')
        self.string_publisher = self.create_publisher(String, 'chatter', 10)
        self.int_publisher = self.create_publisher(Int8, 'numeric_chatter', 10)

        timer_in_seconds = 0.5
        self.string_timer = self.create_timer(timer_in_seconds, self.string_talker_callback)
        self.int_timer = self.create_timer(timer_in_seconds, self.int_talker_callback)
        self.counter = 0

    def string_talker_callback(self):
        msg = String()
        msg.data = f'Hello World, {self.counter}'
        self.string_publisher.publish(msg)
        self.get_logger().info(f'Publishing to chatter: {msg.data}')

    def int_talker_callback(self):
        msg = Int8()
        msg.data = self.counter
        self.int_publisher.publish(msg)
        self.get_logger().info(f'Publishing to numeric_chatter: {msg.data}')

        self.counter += 1
        if self.counter > 127:  
            self.counter = 0


def main(args=None):
    rclpy.init(args=args)

    talker = Talker()
    rclpy.spin(talker)


if __name__ == '__main__':
    main()


