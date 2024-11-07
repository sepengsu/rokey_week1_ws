import rclpy
from rclpy.node import Node
import sys, os
from ros_msgs.srv import OrderService

class MenuNode(Node):
    def __init__(self):
        super().__init__('menu_node')
        self.order_client = self.create_client(OrderService, 'order_service')
        while not self.order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for order_service...')

    def send_order_request(self, table_index: int,order_detail: str):
        """KitNode에 주문 요청을 전송합니다."""
        request = OrderService.Request()
        request.table_index = table_index
        request.order_detail = order_detail
        future = self.order_client.call_async(request)
        future.add_done_callback(self.order_response_callback)
 
    def order_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"정보도착!!! {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MenuNode()
    # 예시로 주문 요청을 전송
    node.send_order_request(1,"(아메리카노, 2)")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
