# import sys

# from select_table_interfaces.srv import TableSelect
# import rclpy
# from rclpy.node import Node
# from SelectedTable import SelectedTable


# class MinimalClientAsync(Node):

#     def __init__(self):
#         super().__init__('minimal_client_async')
#         self.cli = self.create_client(TableSelect, 'select_table')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Table Selection service not available, waiting again...')
#         self.req = TableSelect.Request()

#     def send_request(self):
#         # get Variable from global Variable
#         self.req.table_id = SelectedTable.table_id

#         self.future = self.cli.call_async(self.req)
#         rclpy.spin_until_future_complete(self, self.future)
#         return self.future.result()
    
#     def sendServiceRequest():
    
#         rclpy.init(args=args)

#         minimal_client = MinimalClientAsync()
#         response = minimal_client.send_request()
#         print(response.message)

#         minimal_client.destroy_node()
#         rclpy.shutdown()



# def main(args=None):
#     #rclpy.init(args=args)

#     #minimal_client = MinimalClientAsync()
#     #response = minimal_client.send_request(int(sys.argv[1]))
#     print("bin fälschlicherweise in der Main")

#     #minimal_client.destroy_node()
#     #rclpy.shutdown()
    


# if __name__ == '__main__':
#     main()