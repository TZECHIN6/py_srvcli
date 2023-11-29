import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from tutorial_interfaces.srv import OrderDetail

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        cb_group = ReentrantCallbackGroup()
        self.cli = self.create_client(OrderDetail, 'order_detail', callback_group=cb_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OrderDetail.Request()
        
        timer_period = 3
        self.timer = self.create_timer(timer_period, self.send_request, callback_group=cb_group)

    async def send_request(self):
        try:
            self.future = self.cli.call_async(self.req)
            result: OrderDetail.Response = await self.future
            self.get_logger().info(f'{result.order_detail.order_number}')
        except Exception:
            pass
        

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()