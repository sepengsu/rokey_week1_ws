import rclpy
from rclpy.node import Node
import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append("/home/jaenote/rokey_ros/src")
from ros_msgs.srv import OrderService

class MenuNode(Node):
    def __init__(self):
        super().__init__('menu_node')
        self.order_client = self.create_client(OrderService, 'order_service')
        while not self.order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for order_service...')

    def send_order_request(self, order_detail):
        """KitNode에 주문 요청을 전송합니다."""
        request = OrderService.Request()
        request.order_detail = order_detail  # 예: "테이블 5번 주문 요청"
        future = self.order_client.call_async(request)
        future.add_done_callback(self.order_response_callback)
 
    def order_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Order response: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MenuNode()
    # 예시로 주문 요청을 전송
    node.send_order_request("테이블 5번 주문 요청")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
