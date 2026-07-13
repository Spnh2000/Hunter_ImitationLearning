#!/usr/bin/env python3
"""
Lateral Path-Following PID Controller Node.
Implements a ROS 2 node that tracks a hardcoded path using a PID controller
acting on the heading error towards a look-ahead target waypoint.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
class LateralPIDController(Node):
    """
    ROS 2 Node implementing a lateral path-following PID controller.
    """
    
    # =========================================================================
    # Configurable placeholder variables (easy to tune)
    # =========================================================================
    KP = 1.5
    KI = 0.05
    KD = 0.2
    
    TARGET_SPEED = 3.0         # Target linear speed in m/s
    LOOKAHEAD_DISTANCE = 1.2   # Lookahead distance in meters
    MAX_STEER = 0.461          # Max steering angle/angular velocity in rad (~26.4 deg)
    MAX_INTEGRAL = 0.5         # Anti-windup limit for the integral term
    
    # Hardcoded 2D waypoints [x, y] representing a trajectory path (IMS centerline downsampled by 2)
    WAYPOINTS = [
        [0.0, 0.0], [0.015, -0.728], [0.03, -1.456], [0.044, -2.185], [0.059, -2.913], [0.074, -3.641],
        [0.089, -4.369], [0.104, -5.097], [0.119, -5.825], [0.134, -6.554], [0.149, -7.282], [0.163, -8.01],
        [0.178, -8.738], [0.193, -9.466], [0.208, -10.194], [0.222, -10.922], [0.237, -11.651], [0.251, -12.379],
        [0.266, -13.107], [0.28, -13.835], [0.294, -14.563], [0.308, -15.291], [0.322, -16.019], [0.336, -16.748],
        [0.35, -17.476], [0.364, -18.204], [0.377, -18.932], [0.393, -19.66], [0.414, -20.388], [0.442, -21.115],
        [0.48, -21.842], [0.532, -22.568], [0.6, -23.294], [0.686, -24.018], [0.793, -24.74], [0.924, -25.457],
        [1.08, -26.168], [1.263, -26.872], [1.477, -27.567], [1.722, -28.251], [2.0, -28.923], [2.31, -29.581],
        [2.652, -30.222], [3.023, -30.847], [3.419, -31.458], [3.836, -32.057], [4.273, -32.641], [4.73, -33.21],
        [5.206, -33.763], [5.701, -34.298], [6.214, -34.814], [6.746, -35.309], [7.295, -35.783], [7.861, -36.236],
        [8.444, -36.667], [9.043, -37.078], [9.657, -37.468], [10.286, -37.838], [10.929, -38.186], [11.585, -38.51],
        [12.254, -38.807], [12.933, -39.074], [13.623, -39.308], [14.322, -39.506], [15.029, -39.667], [15.742, -39.793],
        [16.461, -39.888], [17.185, -39.957], [17.911, -40.003], [18.639, -40.03], [19.368, -40.044], [20.097, -40.046],
        [20.826, -40.039], [21.554, -40.025], [22.283, -40.008], [23.011, -39.988], [23.739, -39.969], [24.467, -39.951],
        [25.195, -39.935], [25.923, -39.92], [26.651, -39.907], [27.378, -39.894], [28.106, -39.882], [28.834, -39.871],
        [29.562, -39.861], [30.29, -39.85], [31.019, -39.839], [31.747, -39.829], [32.476, -39.817], [33.206, -39.805],
        [33.935, -39.789], [34.663, -39.763], [35.39, -39.723], [36.115, -39.665], [36.837, -39.583], [37.555, -39.472],
        [38.269, -39.332], [38.977, -39.161], [39.677, -38.961], [40.369, -38.732], [41.049, -38.473], [41.718, -38.186],
        [42.373, -37.871], [43.014, -37.529], [43.642, -37.161], [44.254, -36.768], [44.852, -36.351], [45.434, -35.912],
        [45.999, -35.452], [46.548, -34.971], [47.08, -34.471], [47.594, -33.954], [48.09, -33.419], [48.567, -32.869],
        [49.025, -32.303], [49.462, -31.723], [49.878, -31.129], [50.269, -30.518], [50.634, -29.892], [50.973, -29.25],
        [51.282, -28.591], [51.562, -27.917], [51.815, -27.231], [52.045, -26.536], [52.251, -25.835], [52.429, -25.129],
        [52.573, -24.417], [52.68, -23.7], [52.751, -22.978], [52.791, -22.253], [52.804, -21.526], [52.799, -20.797],
        [52.785, -20.067], [52.772, -19.338], [52.762, -18.609], [52.754, -17.881], [52.747, -17.153], [52.741, -16.424],
        [52.735, -15.696], [52.727, -14.968], [52.718, -14.24], [52.707, -13.512], [52.694, -12.784], [52.681, -12.056],
        [52.666, -11.328], [52.65, -10.6], [52.634, -9.872], [52.616, -9.144], [52.599, -8.416], [52.581, -7.688],
        [52.563, -6.959], [52.545, -6.231], [52.527, -5.503], [52.509, -4.775], [52.491, -4.047], [52.473, -3.319],
        [52.456, -2.591], [52.438, -1.863], [52.421, -1.134], [52.403, -0.406], [52.386, 0.322], [52.369, 1.05],
        [52.352, 1.778], [52.335, 2.506], [52.318, 3.234], [52.301, 3.962], [52.284, 4.69], [52.267, 5.419],
        [52.251, 6.147], [52.234, 6.875], [52.218, 7.603], [52.201, 8.331], [52.185, 9.059], [52.168, 9.787],
        [52.152, 10.515], [52.135, 11.243], [52.119, 11.972], [52.102, 12.7], [52.086, 13.428], [52.07, 14.156],
        [52.053, 14.884], [52.037, 15.612], [52.02, 16.34], [52.004, 17.068], [51.988, 17.796], [51.971, 18.524],
        [51.955, 19.253], [51.938, 19.981], [51.922, 20.709], [51.905, 21.437], [51.889, 22.165], [51.872, 22.893],
        [51.855, 23.621], [51.838, 24.349], [51.822, 25.077], [51.805, 25.806], [51.788, 26.534], [51.771, 27.262],
        [51.754, 27.99], [51.737, 28.718], [51.719, 29.446], [51.702, 30.174], [51.685, 30.902], [51.667, 31.631],
        [51.65, 32.359], [51.632, 33.087], [51.614, 33.815], [51.597, 34.543], [51.58, 35.271], [51.562, 35.999],
        [51.545, 36.727], [51.528, 37.456], [51.511, 38.184], [51.494, 38.912], [51.478, 39.64], [51.461, 40.368],
        [51.446, 41.096], [51.43, 41.824], [51.415, 42.552], [51.4, 43.281], [51.385, 44.009], [51.371, 44.737],
        [51.357, 45.465], [51.344, 46.193], [51.331, 46.921], [51.319, 47.649], [51.308, 48.377], [51.297, 49.105],
        [51.285, 49.833], [51.269, 50.561], [51.246, 51.289], [51.214, 52.016], [51.17, 52.744], [51.11, 53.47],
        [51.033, 54.196], [50.935, 54.919], [50.814, 55.639], [50.667, 56.352], [50.492, 57.057], [50.286, 57.752],
        [50.047, 58.437], [49.776, 59.109], [49.474, 59.769], [49.143, 60.416], [48.784, 61.049], [48.399, 61.668],
        [47.989, 62.272], [47.556, 62.86], [47.101, 63.433], [46.626, 63.989], [46.132, 64.528], [45.621, 65.05],
        [45.094, 65.553], [44.553, 66.038], [43.996, 66.503], [43.424, 66.947], [42.836, 67.371], [42.232, 67.773],
        [41.612, 68.153], [40.975, 68.51], [40.324, 68.841], [39.658, 69.143], [38.981, 69.416], [38.292, 69.655],
        [37.593, 69.858], [36.886, 70.023], [36.172, 70.153], [35.453, 70.251], [34.729, 70.322], [34.002, 70.37],
        [33.273, 70.4], [32.543, 70.416], [31.814, 70.422], [31.086, 70.419], [30.357, 70.41], [29.629, 70.395],
        [28.901, 70.378], [28.173, 70.359], [27.445, 70.341], [26.718, 70.324], [25.99, 70.307], [25.263, 70.291],
        [24.535, 70.275], [23.807, 70.261], [23.08, 70.247], [22.351, 70.235], [21.623, 70.223], [20.894, 70.213],
        [20.165, 70.205], [19.436, 70.197], [18.706, 70.191], [17.976, 70.183], [17.247, 70.168], [16.519, 70.139],
        [15.794, 70.091], [15.072, 70.018], [14.354, 69.914], [13.641, 69.775], [12.935, 69.604], [12.235, 69.403],
        [11.544, 69.172], [10.861, 68.914], [10.189, 68.631], [9.528, 68.324], [8.879, 67.994], [8.243, 67.641],
        [7.62, 67.264], [7.011, 66.865], [6.417, 66.442], [5.838, 65.998], [5.278, 65.53], [4.74, 65.04],
        [4.227, 64.527], [3.738, 63.992], [3.273, 63.437], [2.827, 62.864], [2.401, 62.274], [1.995, 61.668],
        [1.611, 61.047], [1.249, 60.413], [0.91, 59.765], [0.596, 59.106], [0.308, 58.436], [0.045, 57.755],
        [-0.189, 57.066], [-0.396, 56.368], [-0.572, 55.663], [-0.718, 54.952], [-0.834, 54.236], [-0.921, 53.515],
        [-0.983, 52.791], [-1.024, 52.064], [-1.048, 51.336], [-1.058, 50.607], [-1.058, 49.877], [-1.051, 49.149],
        [-1.037, 48.42], [-1.019, 47.692], [-0.997, 46.964], [-0.975, 46.236], [-0.952, 45.509], [-0.93, 44.781],
        [-0.909, 44.053], [-0.889, 43.325], [-0.87, 42.597], [-0.851, 41.869], [-0.834, 41.141], [-0.816, 40.413],
        [-0.799, 39.685], [-0.783, 38.957], [-0.767, 38.228], [-0.751, 37.5], [-0.736, 36.772], [-0.72, 36.044],
        [-0.705, 35.316], [-0.689, 34.588], [-0.674, 33.859], [-0.659, 33.131], [-0.644, 32.403], [-0.629, 31.675],
        [-0.614, 30.947], [-0.599, 30.219], [-0.584, 29.49], [-0.569, 28.762], [-0.554, 28.034], [-0.539, 27.306],
        [-0.525, 26.578], [-0.51, 25.85], [-0.496, 25.122], [-0.481, 24.393], [-0.467, 23.665], [-0.452, 22.937],
        [-0.438, 22.209], [-0.423, 21.481], [-0.409, 20.753], [-0.395, 20.024], [-0.38, 19.296], [-0.366, 18.568],
        [-0.352, 17.84], [-0.338, 17.112], [-0.323, 16.384], [-0.309, 15.655], [-0.295, 14.927], [-0.281, 14.199],
        [-0.266, 13.471], [-0.252, 12.743], [-0.238, 12.015], [-0.224, 11.287], [-0.209, 10.558], [-0.195, 9.83],
        [-0.181, 9.102], [-0.167, 8.374], [-0.152, 7.646], [-0.138, 6.918], [-0.124, 6.189], [-0.109, 5.461],
        [-0.095, 4.733], [-0.08, 4.005], [-0.066, 3.277], [-0.051, 2.549], [-0.037, 1.82], [-0.022, 1.092],
        [-0.007, 0.364]
    ]
    def __init__(self):
        super().__init__('lateral_pid_controller')
        self.get_logger().info("Initializing Lateral PID Controller Node...")
        # Declare parameters (default to class constants)
        self.declare_parameter('kp', self.KP)
        self.declare_parameter('ki', self.KI)
        self.declare_parameter('kd', self.KD)
        self.declare_parameter('target_speed', self.TARGET_SPEED)
        self.declare_parameter('lookahead_distance', self.LOOKAHEAD_DISTANCE)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', -1.57)
        # Retrieve parameter values
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.target_speed = self.get_parameter('target_speed').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.spawn_yaw = self.get_parameter('spawn_yaw').value
        # Controller states
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.last_time = None
        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ackermann_like_controller/odometry',
            self.odom_callback,
            10
        )
        # Publisher
        self.ref_pub = self.create_publisher(
            TwistStamped,
            '/ackermann_like_controller/reference',
            10
        )
        self.get_logger().info(f"Subscribed to '/ackermann_like_controller/odometry'")
        self.get_logger().info(f"Publishing to '/ackermann_like_controller/reference'")
        self.get_logger().info(f"PID Parameters -> Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}")
        self.get_logger().info(f"Lookahead distance: {self.lookahead_distance} meters")
    def odom_callback(self, msg: Odometry):
        """
        Callback triggered upon receiving a new Odometry message.
        Processes states, computes PID tracking, and publishes TwistStamped command.
        """
        # 1. Parse current robot position (X, Y) from odom
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y
        # 2. Convert quaternion orientation into a yaw angle (theta_odom) in odom frame
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_odom = math.atan2(siny_cosp, cosy_cosp)
        # 2.5 Transform odom pose to world frame to align with WAYPOINTS
        cos_yaw = math.cos(self.spawn_yaw)
        sin_yaw = math.sin(self.spawn_yaw)
        x = x_odom * cos_yaw - y_odom * sin_yaw + self.spawn_x
        y = x_odom * sin_yaw + y_odom * cos_yaw + self.spawn_y
        theta = theta_odom + self.spawn_yaw
        theta = math.atan2(math.sin(theta), math.cos(theta))
        # 3. Find the closest waypoint on the path
        min_dist = float('inf')
        closest_idx = 0
        for i, waypoint in enumerate(self.WAYPOINTS):
            dist = math.hypot(waypoint[0] - x, waypoint[1] - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        # Calculate Cross-Track Error (CTE) relative to closest waypoint for logging
        closest_waypoint = self.WAYPOINTS[closest_idx]
        next_idx = (closest_idx + 1) % len(self.WAYPOINTS)
        next_waypoint = self.WAYPOINTS[next_idx]
        
        path_yaw = math.atan2(next_waypoint[1] - closest_waypoint[1], next_waypoint[0] - closest_waypoint[0])
        dx = x - closest_waypoint[0]
        dy = y - closest_waypoint[1]
        
        # Left of path is positive, right of path is negative
        cte = -dx * math.sin(path_yaw) + dy * math.cos(path_yaw)
        # 3.5 Speed logic: slow down on tight corners or high lateral error to maintain traction
        speed = self.target_speed
        if abs(cte) > 0.15 or abs(self.prev_error) > 0.10:
            speed = max(1.2, speed * 0.6)
        # Calculate dynamic lookahead distance based on current target speed
        dynamic_lookahead = max(1.0, self.lookahead_distance * (speed / self.target_speed))
        # 4. Find look-ahead target waypoint based on dynamic_lookahead (in meters)
        accum_dist = 0.0
        lookahead_idx = closest_idx
        n_wpts = len(self.WAYPOINTS)
        
        while accum_dist < dynamic_lookahead:
            n_idx = (lookahead_idx + 1) % n_wpts
            p1 = self.WAYPOINTS[lookahead_idx]
            p2 = self.WAYPOINTS[n_idx]
            step_dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            accum_dist += step_dist
            lookahead_idx = n_idx
            if lookahead_idx == closest_idx:
                break  # Prevent infinite loop if lookahead exceeds path length
        target_waypoint = self.WAYPOINTS[lookahead_idx]
        # 5. Determine the target yaw required to head towards the look-ahead point
        target_yaw = math.atan2(target_waypoint[1] - y, target_waypoint[0] - x)
        # Calculate heading error relative to target waypoint
        heading_error = target_yaw - theta
        # 6. Normalize heading error cleanly between -pi and pi using math.atan2
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        # 7. Compute Time Step (dt)
        curr_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = curr_time
            return
        dt = (curr_time - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 0.01  # Fallback to prevent division by zero
        # 8. Calculate continuous PID loop equations
        self.integral_error += heading_error * dt
        # Anti-windup clamping
        self.integral_error = max(min(self.integral_error, self.MAX_INTEGRAL), -self.MAX_INTEGRAL)
        derivative_error = (heading_error - self.prev_error) / dt
        # Continuous PID logic: Output = (Kp * error) + (Ki * integral_error) + (Kd * derivative_error)
        steering_output = (self.kp * heading_error) + (self.ki * self.integral_error) + (self.kd * derivative_error)
        # 9. Clamp/limit the maximum absolute angular output to prevent over-steering
        steering_output = max(min(steering_output, self.MAX_STEER), -self.MAX_STEER)
        # Log tracking state details periodically
        self.get_logger().info(
            f"CTE: {cte:.3f}m | Heading Err: {heading_error:.3f}rad | "
            f"Steer: {steering_output:.3f}rad | Speed: {speed:.2f}m/s | Wpt: #{lookahead_idx}",
            throttle_duration_sec=0.5
        )
        # 10. Populate and publish TwistStamped message
        msg_out = TwistStamped()
        msg_out.header.stamp = curr_time.to_msg()
        msg_out.header.frame_id = 'base_link'
        msg_out.twist.linear.x = speed
        msg_out.twist.angular.z = steering_output
        self.ref_pub.publish(msg_out)
        # Save states for next loop
        self.prev_error = heading_error
        self.last_time = curr_time
def main(args=None):
    rclpy.init(args=args)
    node = LateralPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down PID controller node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()