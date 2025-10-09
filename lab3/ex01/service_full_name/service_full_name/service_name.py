from full_name_package.srv import FullNameSumService

import rclpy
from rclpy.node import Node


class NameService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(FullNameSumService, 'service_name', self.service_name_callback)

    def service_name_callback(self, request, response):
        response.full_name = request.last_name + request.first_name + request.name
        self.get_logger().info('Incoming request\nLast name: %s first name: %s name: %s' % (request.last_name, request.first_name, request.name))

        return response


def main():
    rclpy.init()

    name_service = NameService()

    rclpy.spin(name_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()