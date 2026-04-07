import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class MyRobot(Node):
    def __init__(self):
        super().__init__('my_robot')
        
        # Create a client for the /get_task service
        self.task_client = self.create_client(Trigger, 'get_task')
        
        # Wait until the service is available
        while not self.task_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_task service...')
        
        # Call it once on startup
        self.request_task()

    def request_task(self):
        req = Trigger.Request()  # Empty request — no fields to fill
        future = self.task_client.call_async(req)
        future.add_done_callback(self.task_response_callback)

    def task_response_callback(self, future):
        response = future.result()
        
        if response.success:
            # Parse the comma-separated string
            parts = response.message.split(',')
            start_x = float(parts[0])
            start_y = float(parts[1])
            goal_x  = float(parts[2])
            goal_y  = float(parts[3])
            
            self.get_logger().info(
                f"Task received — Start: ({start_x}, {start_y}) | Goal: ({goal_x}, {goal_y})"
            )
            
            # Now hand off to your navigation logic
            self.navigate_to(goal_x, goal_y)
        else:
            self.get_logger().error("Task request failed.")

    # def navigate_to(self, goal_x, goal_y):
        # Your navigation logic here
        # pass

def main():
    rclpy.init()
    node = MyRobot()
    rclpy.spin(node)
    rclpy.shutdown()