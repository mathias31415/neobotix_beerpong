# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int16  # Importiere die Nachrichtenart, die du verwenden möchtest
# from SelectedTable import SelectedTable

# class SelectTablePublisher(Node):
#     def __init__(self):
#         print("bin in der Init")
#         super().__init__('selected_table_publisher')
#         self.publisher_ = self.create_publisher(Int16, 'selected_table', 1)
#         timer_period = 2.0  # Sekunden
#         self.timer_callback()
#         self.get_logger().info('Init ausgeführt')


#         #self.i = 0
        
#     def timer_callback(self):
#         msg = Int16()
#         msg.data = SelectedTable.table_id
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Publishing: "{msg.data}"')

#         #self.i += 1
    
#     @staticmethod
#     def main(args=None):
            
#         try:
#             rclpy.init(args=args)
#             node_website = SelectTablePublisher()
#             rclpy.spin(node_website)
#             node_website.destroy_node()
#             rclpy.shutdown()
#         except:
#             #rclpy.init(args=args)
#             node_website = SelectTablePublisher()
#             rclpy.spin(node_website)
#             node_website.destroy_node()
#             rclpy.shutdown()
#             pass


# if __name__ == '__main__':
#     main()