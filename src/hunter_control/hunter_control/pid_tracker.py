import os
import math
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float64

class SimplePIDTracker(Node):
    def __init__(self):
        super().__init__('pid_tracker')

        # 1. Parameter Declarations (with sensible defaults)
        self.declare_parameter('csv_path', '')
        self.declare_parameter('target_speed', 2.0)
        self.declare_parameter('kp_heading', 1.2)
        self.declare_parameter('kp_lateral', 0.8)
        self.declare_parameter('kd_lateral', 0.15)
        self.declare_parameter('lookahead_index_offset', 4)  # Look ahead 4 waypoints (~1.5 meters)
        self.declare_parameter('max_laps', 1)
        self.declare_parameter('metrics_output_path', '')

        # 2. Dynamic path resolution for macOS compatibility
        workspace_root = self.find_workspace_root()
        
        csv_param = self.get_parameter('csv_path').value
        self.csv_path = csv_param if csv_param else os.path.join(workspace_root, 'src', 'Monza', 'Monza_centerline.csv')
        
        metrics_param = self.get_parameter('metrics_output_path').value
        self.metrics_output_path = metrics_param if metrics_param else os.path.join(workspace_root, 'metrics_baseline.json')

        self.target_speed = self.get_parameter('target_speed').value
        self.kp_heading = self.get_parameter('kp_heading').value
        self.kp_lateral = self.get_parameter('kp_lateral').value
        self.kd_lateral = self.get_parameter('kd_lateral').value
        self.lookahead_offset = self.get_parameter('lookahead_index_offset').value
        self.max_laps = self.get_parameter('max_laps').value

        # Load reference path
        self.path_points = self.load_path(self.csv_path)
        self.get_logger().info(f"Loaded {len(self.path_points)} path waypoints from: {self.csv_path}")

        # 3. State Variables
        self.prev_error_y = 0.0
        self.prev_steering = 0.0
        self.last_time = None
        self.is_completed = False
        
        # Lap state machine
        self.completed_laps = 0
        self.lap_start_time = None
        self.halfway_passed = False
        self.lap_times = []

        # History arrays for calculating baseline metrics
        self.history_ey = []
        self.history_epsi = []
        self.history_steering = []
        self.history_steering_rate = []

        # 4. ROS 2 Publishers & Subscribers
        self.cmd_pub = self.create_publisher(TwistStamped, '/ackermann_like_controller/reference', 10)
        self.pub_ey = self.create_publisher(Float64, '/tracking_metrics/lateral_error', 10)
        self.pub_epsi = self.create_publisher(Float64, '/tracking_metrics/heading_error', 10)
        self.pub_progress = self.create_publisher(Float64, '/tracking_metrics/lap_progress', 10)

        self.odom_sub = self.create_subscription(Odometry, '/ackermann_like_controller/odometry', self.odom_callback, 10)
        self.get_logger().info("Simple PID Tracker Node started.")

    def find_workspace_root(self):
        """Resolves workspace folder dynamically from installation share directory."""
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('hunter_control')
            parent = share_dir
            for _ in range(4):
                parent = os.path.dirname(parent)
            return parent
        except Exception:
            return os.getcwd()

    def load_path(self, file_path):
        """Simple CSV parser supporting centerline (comma) and raceline (semicolon)."""
        points = []
        is_raceline = "raceline" in file_path.lower()
        delimiter = ';' if is_raceline else ','

        with open(file_path, 'r') as f:
            for line in f:
                if line.startswith('#') or not line.strip():
                    continue
                parts = [float(p.strip()) for p in line.split(delimiter)]
                if is_raceline: # format: s; x; y; psi; kappa; vx; ax
                    points.append({'x': parts[1], 'y': parts[2], 'psi': parts[3]})
                else: # format: x, y, width_right, width_left
                    points.append({'x': parts[0], 'y': parts[1], 'psi': 0.0})

        # Generate tangent heading angles for centerline path
        if not is_raceline and len(points) > 1:
            for i in range(len(points)):
                next_i = (i + 1) % len(points)
                dx = points[next_i]['x'] - points[i]['x']
                dy = points[next_i]['y'] - points[i]['y']
                points[i]['psi'] = math.atan2(dy, dx)

        return points

    def wrap_angle(self, angle):
        """Helper to clamp angles between -pi and pi."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        if self.is_completed:
            self.publish_command(0.0, 0.0)
            return

        # 1. Parse simulation time and delta time
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = curr_time
            self.lap_start_time = curr_time
            return

        dt = curr_time - self.last_time
        if dt <= 0.0:
            dt = 0.01

        # 2. Parse robot pose
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # 3. Find closest path waypoint (Simple scan)
        dists = [math.hypot(pt['x'] - rx, pt['y'] - ry) for pt in self.path_points]
        closest_dist = min(dists)
        closest_idx = dists.index(closest_dist)

        pt_ref = self.path_points[closest_idx]
        psi_ref = pt_ref['psi']

        # Signed lateral (cross-track) error: left of path is positive, right is negative
        dx = rx - pt_ref['x']
        dy = ry - pt_ref['y']
        error_y = - dx * math.sin(psi_ref) + dy * math.cos(psi_ref)

        # Heading error at closest point (for metrics)
        error_psi = self.wrap_angle(psi_ref - yaw)

        # 4. PID Steering Control with Lookahead Waypoint
        lookahead_idx = (closest_idx + self.lookahead_offset) % len(self.path_points)
        pt_look = self.path_points[lookahead_idx]

        # Calculate target heading angle to lookahead point
        target_psi = math.atan2(pt_look['y'] - ry, pt_look['x'] - rx)
        heading_error_look = self.wrap_angle(target_psi - yaw)

        # PID control law
        p_term = self.kp_heading * heading_error_look
        derivative_error_y = (error_y - self.prev_error_y) / dt
        
        # Steering = P(heading to lookahead) - P(lateral error) - D(lateral error change rate)
        steering = p_term - (self.kp_lateral * error_y + self.kd_lateral * derivative_error_y)
        steering = max(min(steering, 0.461), -0.461)  # Clamp to steering limits

        # Speed logic (slow down slightly on tight corners or high lateral error)
        speed = self.target_speed
        if abs(error_y) > 0.5 or abs(steering) > 0.3:
            speed = max(0.5, speed * 0.6)

        # Send command to Ackermann controller
        self.publish_command(speed, steering)

        # 5. Gated Lap Completion Logic
        n_points = len(self.path_points)
        
        # Mark halfway point passed when vehicle reaches indices in the middle half
        if closest_idx > int(0.4 * n_points) and closest_idx < int(0.8 * n_points):
            self.halfway_passed = True

        # Wrap-around check: index transitions from near the end to near the start
        prev_idx = getattr(self, 'prev_closest_idx', closest_idx)
        if prev_idx > int(0.8 * n_points) and closest_idx < int(0.2 * n_points):
            if self.halfway_passed:
                lap_time = curr_time - self.lap_start_time
                self.completed_laps += 1
                self.lap_times.append(lap_time)
                self.get_logger().info(f"Lap {self.completed_laps} COMPLETED! Time: {lap_time:.2f} s")
                
                self.lap_start_time = curr_time
                self.halfway_passed = False

                if self.completed_laps >= self.max_laps:
                    self.get_logger().info(f"Finished target of {self.max_laps} lap(s). Exiting node...")
                    self.is_completed = True
                    self.save_metrics_baseline()
            
        self.prev_closest_idx = closest_idx

        # 6. Metrics Recording
        self.history_ey.append(error_y)
        self.history_epsi.append(error_psi)
        self.history_steering.append(steering)
        self.history_steering_rate.append((steering - self.prev_steering) / dt)

        # Stream metrics to ROS 2 topics
        self.pub_ey.publish(Float64(data=error_y))
        self.pub_epsi.publish(Float64(data=error_psi))
        self.pub_progress.publish(Float64(data=closest_idx / n_points))

        # Save previous states
        self.prev_error_y = error_y
        self.prev_steering = steering
        self.last_time = curr_time

    def publish_command(self, speed, steering):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = speed
        cmd.twist.angular.z = steering
        self.cmd_pub.publish(cmd)

    def save_metrics_baseline(self):
        """Calculates running stats and saves a simple JSON benchmark summary."""
        if not self.history_ey:
            return

        abs_ey = [abs(e) for e in self.history_ey]
        abs_epsi = [abs(e) for e in self.history_epsi]
        abs_chatter = [abs(r) for r in self.history_steering_rate]

        # Calculate simple statistics
        mean_abs_ey = sum(abs_ey) / len(abs_ey)
        rmse_ey = math.sqrt(sum(e**2 for e in self.history_ey) / len(self.history_ey))
        max_ey = max(abs_ey)
        
        mean_abs_epsi = sum(abs_epsi) / len(abs_epsi)
        max_epsi = max(abs_epsi)
        
        chatter = sum(abs_chatter) / len(abs_chatter)
        
        # Calculate standard deviation
        mean_steer = sum(self.history_steering) / len(self.history_steering)
        var_steer = sum((s - mean_steer)**2 for s in self.history_steering) / len(self.history_steering)
        std_steer = math.sqrt(var_steer)

        metrics = {
            "reference_path": self.csv_path,
            "laps_completed": self.completed_laps,
            "lap_times": self.lap_times,
            "lateral_error": {
                "mae": mean_abs_ey,
                "rmse": rmse_ey,
                "max": max_ey
            },
            "heading_error": {
                "mae": mean_abs_epsi,
                "max": max_epsi
            },
            "oscillation": {
                "steering_chatter_radps": chatter,
                "steering_std_rad": std_steer
            }
        }

        # Print baseline report to console
        self.get_logger().info("========================================")
        self.get_logger().info("      TRACKING BENCHMARK METRICS        ")
        self.get_logger().info("========================================")
        self.get_logger().info(f"Laps Completed:      {self.completed_laps}")
        self.get_logger().info(f"Lap Times (s):       {self.lap_times}")
        self.get_logger().info(f"Lateral Error MAE:   {mean_abs_ey:.4f} m")
        self.get_logger().info(f"Lateral Error RMSE:  {rmse_ey:.4f} m")
        self.get_logger().info(f"Lateral Error Max:   {max_ey:.4f} m")
        self.get_logger().info(f"Heading Error MAE:   {mean_abs_epsi:.4f} rad")
        self.get_logger().info(f"Steering Chatter:    {chatter:.4f} rad/s")
        self.get_logger().info("========================================")

        try:
            with open(self.metrics_output_path, 'w') as f:
                json.dump(metrics, f, indent=4)
            self.get_logger().info(f"Saved benchmark baseline JSON to: {self.metrics_output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write JSON: {e}")

    def destroy_node(self):
        if not self.is_completed and len(self.history_ey) > 100:
            self.save_metrics_baseline()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimplePIDTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
