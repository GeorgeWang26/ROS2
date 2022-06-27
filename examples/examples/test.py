import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from interfaces.action import Fib


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self.action_client = ActionClient(self, Fib, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fib.Goal()
        goal_msg.order = order
        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print("request rejected")
            return
        print("request accepted")

        result_future = goal_handle.get_result_async()
        # wait 0.1 sec for future to complete, if future is not compelte after 0.1 sec, end the blocking caused by spin
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
        print("result spin with timeout complete")
        rclpy.spin_until_future_complete(self, result_future)
        print("result spin without timeout complete")

    # feedback function
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()