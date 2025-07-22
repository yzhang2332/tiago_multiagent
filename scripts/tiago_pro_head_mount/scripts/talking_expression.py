import rclpy
from rclpy.node import Node
from hri_msgs.msg import Expression
from std_msgs.msg import Header


class ExpressionLoopPublisher(Node):
    def __init__(self):
        super().__init__('expression_loop_publisher')
        self.publisher_ = self.create_publisher(Expression, '/robot_face/expression', 10)

        self.expressions = [
            # ('surprised', 0.0, 0.0),
            ('excited', 0.0, 0.0)
        ]

        self.index = 0
        self.timer = self.create_timer(0.3, self.publish_expression)  # 1 Hz

    def publish_expression(self):
        label, valence, arousal = self.expressions[self.index]
        msg = Expression()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'face'

        msg.expression = label
        msg.valence = valence
        msg.arousal = arousal
        msg.confidence = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {label}')
        self.index = (self.index + 1) % len(self.expressions)


def main(args=None):
    rclpy.init(args=args)
    node = ExpressionLoopPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
