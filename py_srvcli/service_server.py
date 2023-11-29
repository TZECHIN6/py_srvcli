import rclpy
from rclpy.node import Node

from tutorial_interfaces.srv import OrderDetail
from tutorial_interfaces.msg import TaskRequest

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.order_detail_srv = self.create_service(OrderDetail, 'order_detail', self.order_detail_callback)
        self.order_reset_srv = self.create_service(OrderDetail, 'order_reset', self.order_reset_callback)
        self.order_detail = TaskRequest()
        self.order_detail.header.stamp = self.get_clock().now().to_msg()
        self.order_detail.order_number = "AB12345"
        self.order_detail.location = "LT1"
        self.order_detail.charge_time = 30

    def order_detail_callback(self, request, response):
        response.success = True
        response.order_detail = self.order_detail

        self.get_logger().info('Request received')

        return response
    
    def order_reset_callback(self, request, response):
        self.order_detail = TaskRequest()

        self.get_logger().info('Order detail reset')

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()