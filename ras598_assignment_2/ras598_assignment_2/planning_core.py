import rclpy                                                              # ROS2 Python client library
from rclpy.node import Node                                               # Base class for ROS2 nodes
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy     # QoS settings for publishers/subscribers
from geometry_msgs.msg import Twist, Point                                # Twist for velocity commands, Point for visualization
from std_msgs.msg import Float32                                          # Energy consumption topic type
from nav_msgs.msg import Odometry                                         # Odometry messages (robot pose)
from visualization_msgs.msg import MarkerArray, Marker                    # RViz visualization
from example_interfaces.srv import Trigger                                # Service used to request task
import math, os                                                           # Math utilities and filesystem operations

# Custom utilities for map processing and planning
from ras598_assignment_2.map_utils import load_map_image, build_occupancy_grid, inflate_obstacles
from ras598_assignment_2.astar_utils import astar
from ras598_assignment_2.prune_utils import prune_path


class PlannerNode(Node):
    def __init__(self):
        # Initialize ROS2 node with name 'planner_node'
        super().__init__('planner_node')
 
        # Load map file from workspace
        home = os.path.expanduser('~')
        map_path = os.path.join(home, 'ros2_ws/src/ras598_assignment_2/cave_filled.png')

        # Load and process map
        self.map_image = load_map_image(map_path)  # grayscale or binary map image
        
        # Map origin (must match map.yaml)
        self.map_origin_x = -8.0
        self.map_origin_y = -8.0

        # Task variables
        self.have_task = False      # whether a valid task has been received
        self.task_message = ""      # raw string from service
        self.start_x = 0.0          # start x (world coordinates)
        self.start_y = 0.0          # start y
        self.goal_x = 0.0           # goal x
        self.goal_y = 0.0           # goal y
        self.goal_reached = False   # flag to stop execution when goal is reached

        # Robot state (updated via odometry)
        self.x = 0.0                # robot x position (world frame)
        self.y = 0.0                # robot y position (world frame)
        self.yaw = 0.0              # robot heading (orientation in radians)
        self.energy = 0.0           # accumulated energy consumption
        
        # Path execution variables
        self.current_waypoint_idx = 0       # index of current waypoint in path
        self.path_ready = False             # becomes True after planning completes

        # Finite state machine for motion control
        self.state = 'ROTATE'               # either ROTATE or DRIVE
        self.heading_threshold = 0.2        # allowable yaw error (rad)
        self.waypoint_tolerance = 0.2       # distance threshold to consider waypoint reached

        # Robot velocity limits
        self.max_linear_speed = 1.5
        self.max_angular_speed = 0.75

        # Logging timers (used to throttle console output)
        self.last_odom_log_time = 0.0
        self.last_energy_log_time = 0.0

        # Resolution parameters
        self.source_resolution = 0.032  # original map resolution (m/pixel)
        self.grid_resolution = 0.2      # planning grid resolution (coarser for efficiency)

        # Obstacle inflation (robot safety margin)
        self.inflation_radius = 0.601   # physical radius to keep away from obstacles
        self.inflation_cells = int(self.inflation_radius / self.grid_resolution)  # convert to grid cells
        
        # Convert image into occupancy grid at desired resolution
        self.grid = build_occupancy_grid(
            self.map_image,
            source_resolution = self.source_resolution,
            target_resolution = self.grid_resolution
        )
        # Inflate obstacles to account for robot footprint
        self.inflated_grid = inflate_obstacles(self.grid, self.inflation_cells)

        # Print grid size for debugging
        self.get_logger().info(f'Grid shape: {self.inflated_grid.shape}')


        # ------------------


        # SERVICE CLIENT: request start/goal from environment
        self.client = self.create_client(Trigger, '/get_task')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_task...') 

        # SUBSCRIBER: ground truth odometry
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth', self.odom_callback, 10)

        # SUBSCRIBER: energy consumption
        self.energy_sub = self.create_subscription(Float32, '/energy_consumed', self.energy_callback, 10)

        # PUBLISHER: velocity commands
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # QoS settings for visualization markers (persistent in RViz)
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # PUBLISHER: path visualization
        self.planner_marker = self.create_publisher(MarkerArray, '/planner_markers', marker_qos)   

        # Control loop timer (runs at 10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Visualization timer (1 Hz)
        self.marker_timer = self.create_timer(1.0, self.publish_path_timer)

        # Request task from service
        self.send_request()


    def send_request(self):
        # Send asynchronous request to /get_task service
        req = Trigger.Request()
        self.future = self.client.call_async(req)

        # When response arrives, handle it
        self.future.add_done_callback(self.handle_response)


    # Periodically publish markers if path is ready
    def publish_path_timer(self):
        if self.path_ready:
            self.publish_markers()

    # Callback for service response
    def handle_response(self, future):
        try:
            response = future.result()

            # Log response
            self.get_logger().info(f'success: {response.success}')
            self.get_logger().info(f'message: {response.message}')

            # Parse response string: "start_x, start_y, goal_x, goal_y"
            self.task_message = response.message.strip()
            parts = [p.strip() for p in self.task_message.split(',')]

            if len(parts) != 4:
                self.get_logger().error(f'Unexpected /get_task format: {self.task_message}')
                return

            # Extract start and goal
            self.start_x = float(parts[0])
            self.start_y = float(parts[1])
            self.goal_x = float(parts[2])
            self.goal_y = float(parts[3])
            self.have_task = True

            # Convert world coordinates to grid indices
            self.start_cell = self.world_to_grid(self.start_x, self.start_y)
            self.goal_cell = self.world_to_grid(self.goal_x, self.goal_y)

            # Run A* path planning on inflated grid
            self.raw_path = astar(self.inflated_grid, self.start_cell, self.goal_cell)

            if not self.raw_path:
                self.get_logger().error("A* returned no path")
                return

            # Path pruning removes unnecessary intermediate points
            self.pruned_path = prune_path(self.inflated_grid, self.raw_path, margin=0)

            if not self.pruned_path:
                self.get_logger().error("Pruned path is empty")
                return

            # Publish visualization
            self.publish_markers()

            # Initialize waypoint tracking
            self.current_waypoint_idx = 1 if len(self.pruned_path) > 1 else 0
            self.state = 'ROTATE'

            self.path_ready = True

        except Exception as e:
            self.get_logger().error(f'get_task failed: {e}')


    def odom_callback(self, msg):
        # Extract robot position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw (Euler angle)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Throttled logging
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_odom_log_time > 1.0:
            self.get_logger().info(
                f'Robot position: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}'
            )
            self.last_odom_log_time = now


    def energy_callback(self, msg):
        # Update energy consumption
        self.energy = msg.data

        # Throttled logging
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_energy_log_time > 1.0:
            self.get_logger().info(f'Energy consumed: {self.energy:.2f}')
            self.last_energy_log_time = now


    def publish_velocity(self, v, w):
        # Publish linear (v) and angular (w) velocity commands
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.velocity_pub.publish(msg)   


    def world_to_grid(self, x, y):
        # Convert world coordinates to grid indices
        gx = int((x - self.map_origin_x) / self.grid_resolution)
        gy = int((y - self.map_origin_y) / self.grid_resolution)
        return gx, gy


    def grid_to_world(self, gx, gy):
        # Convert grid indices to world coordinates (center of cell)
        x = gx * self.grid_resolution + self.map_origin_x + self.grid_resolution / 2.0
        y = gy * self.grid_resolution + self.map_origin_y + self.grid_resolution / 2.0
        return x, y

    
    def publish_markers(self):
        # Create a MarkerArray container to hold all RViz visualization markers
        markers = MarkerArray()

        # Get current ROS time for timestamping all markers (ensures RViz consistency)
        now = self.get_clock().now().to_msg()

        # Marker used to clear previously published markers in RViz
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL  # instruct RViz to remove all existing markers in this namespace
        markers.markers.append(delete_all)



        # -----------------------------
        # GREEN: Raw A* path marker
        # -----------------------------
        raw_marker = Marker()                       # Create a line strip marker to visualize the raw A* output path
        raw_marker.header.frame_id = "map"          # Frame in which the marker is defined (map frame for global consistency)
        raw_marker.header.stamp = now               # Timestamp ensures RViz updates correctly with latest transform context
        raw_marker.ns = "planner_paths"             # Namespace groups related markers together in RViz
        raw_marker.id = 0                           # Unique ID for this marker (raw path = 0)
        raw_marker.type = Marker.LINE_STRIP         # LINE_STRIP connects all points into a continuous path visualization
        raw_marker.action = Marker.ADD              # ADD means this marker is created/updated in RViz
        raw_marker.scale.x = 0.03                   # Line thickness in meters

        # RGB color for raw A* path (green)
        raw_marker.color.r = 0.0    # No red
        raw_marker.color.g = 1.0    # Full green
        raw_marker.color.b = 0.0    # No blue
        raw_marker.color.a = 1.0    # full opacity

        # Convert each grid cell in raw path into world coordinates and add as a point
        for gx, gy in self.raw_path:
            p = Point()  # ROS geometry message for a single point in space
            p.x, p.y = self.grid_to_world(gx, gy)      # Convert grid indices (gx, gy) into world coordinates (x, y
            p.z = 0.05   # Slight elevation to avoid z-fighting with map plane in RViz
            raw_marker.points.append(p)   # Append point to line strip

        markers.markers.append(raw_marker)        # Add completed raw path marker to MarkerArray



        # -----------------------------
        # BLUE: Pruned path marker
        # -----------------------------
        pruned_marker = Marker()        # Create optimized/pruned path marker (fewer waypoints, smoother motion)
        pruned_marker.header.frame_id = "map"        # Same map frame for consistency
        pruned_marker.header.stamp = now        # Timestamp for synchronization with RViz
        pruned_marker.ns = "planner_paths"        # Same namespace as raw path for grouping
        pruned_marker.id = 1        # Unique ID for pruned path visualization
        pruned_marker.type = Marker.LINE_STRIP        # Line strip visualization for pruned trajectory
        pruned_marker.action = Marker.ADD        # Add/update marker in RViz
        pruned_marker.scale.x = 0.05        # Slightly thicker line than raw path for emphasis

        # Blue color indicates processed/optimized path
        pruned_marker.color.r = 0.0
        pruned_marker.color.g = 0.0
        pruned_marker.color.b = 1.0
        pruned_marker.color.a = 1.0  # fully visible

        # Convert pruned grid path into world coordinates and add points
        for gx, gy in self.pruned_path:
            p = Point()                                     # individual waypoint in world space
            p.x, p.y = self.grid_to_world(gx, gy)           # Convert grid cell to continuous world coordinate
            p.z = 0.07                                      # Slightly higher z-value to visually separate from raw path
            pruned_marker.points.append(p)                  # Append waypoint to pruned path line

        markers.markers.append(pruned_marker)        # Add pruned path marker to MarkerArray



        # -----------------------------
        # RED: Goal marker (sphere)
        # -----------------------------
        goal_marker = Marker()                      # Create spherical marker representing final goal location
        goal_marker.header.frame_id = "map"         # Map frame for consistency with paths
        goal_marker.header.stamp = now              # Timestamp for visualization sync
        goal_marker.ns = "planner_goal"             # Separate namespace for goal visualization
        goal_marker.id = 2                          # Unique ID for goal marker
        goal_marker.type = Marker.SPHERE            # Sphere represents a point goal in space
        goal_marker.action = Marker.ADD             # Add/update marker in RViz

        # Set goal position in world coordinates (directly from service request)
        goal_marker.pose.position.x = self.goal_x
        goal_marker.pose.position.y = self.goal_y   
        goal_marker.pose.position.z = 0.1           # slightly above ground plane

        # Size of goal sphere (uniform in all axes)
        goal_marker.scale.x = 0.25
        goal_marker.scale.y = 0.25
        goal_marker.scale.z = 0.25

        # Red color indicates goal location
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0  # fully visible

        # Add goal marker to array
        markers.markers.append(goal_marker)

        # Publish all markers (raw path, pruned path, goal) to RViz
        self.planner_marker.publish(markers)


    # Main feedback control loop (runs periodically)
    def control_loop(self):
        # Stop if no valid path
        if not self.path_ready or not self.pruned_path or self.goal_reached:
            self.publish_velocity(0.0, 0.0)
            return

        # Check if all waypoints are completed
        if self.current_waypoint_idx >= len(self.pruned_path):
            if not self.goal_reached:
                self.goal_reached = True
                self.stop_planner()
            return

        # Get current target waypoint
        target_gx, target_gy = self.pruned_path[self.current_waypoint_idx]
        target_x, target_y = self.grid_to_world(target_gx, target_gy)

        # Compute position error
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        # Final goal check
        if self.current_waypoint_idx == len(self.pruned_path) - 1 and dist < 0.3:
            self.publish_velocity(0.0, 0.0)
            self.goal_reached = True
            self.stop_planner()
            return

        # Desired heading
        desired_yaw = math.atan2(dy, dx)
        yaw_error = desired_yaw - self.yaw

        # Normalize angle to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2.0 * math.pi
        
        while yaw_error < -math.pi:
            yaw_error += 2.0 * math.pi

        # Waypoint reached → advance
        if dist < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            self.state = 'ROTATE'
            self.publish_velocity(0.0, 0.0)
            return

        # State machine for motion
        if self.state == 'ROTATE':
            if abs(yaw_error) > self.heading_threshold:
                v = 0.0
                w = self.max_angular_speed if yaw_error > 0.0 else -self.max_angular_speed
            else:
                self.state = 'DRIVE'
                v = 0.0
                w = 0.0
        
        elif self.state == 'DRIVE':
            if abs(yaw_error) > self.heading_threshold:
                self.state = 'ROTATE'
                v = 0.0
                w = 0.0
            else:
                v = self.max_linear_speed
                w = 0.0
        
        else:   # Fallback safety state
            self.state = 'ROTATE'
            v = 0.0
            w = 0.0


        self.publish_velocity(v, w)        # Send command

    # Stop robot motion
    def stop_planner(self):
        self.publish_velocity(0.0, 0.0)

        # Stop timers to end execution loop
        if hasattr(self, "control_timer"):
            self.control_timer.cancel()
        if hasattr(self, "marker_timer"):
            self.marker_timer.cancel()

        self.destroy_node()


def main():    # Standard ROS2 node lifecycle
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
