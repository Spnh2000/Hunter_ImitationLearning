import os
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float64


class PIDTracker(Node):
    def __init__(self):
        super().__init__('pid_tracker')

        # Resolve dynamic paths relative to the package installation to remain highly portable on macOS
        self.workspace_root = self.find_workspace_root()
        default_csv = os.path.join(self.workspace_root, 'src', 'Monza', 'Monza_centerline.csv')
        default_metrics = os.path.join(self.workspace_root, 'metrics_baseline.json')

        # Declare parameters
        self.declare_parameter('csv_path', default_csv)
        self.declare_parameter('target_speed', 2.0)  # m/s (use negative to follow speed profile in raceline CSV)
        self.declare_parameter('kp_lateral', 1.0)
        self.declare_parameter('ki_lateral', 0.0)
        self.declare_parameter('kd_lateral', 0.15)
        self.declare_parameter('kp_heading', 1.2)
        self.declare_parameter('lookahead_distance', 1.5)  # m (set to 0.0 to track closest point directly)
        self.declare_parameter('max_laps', 1)
        self.declare_parameter('metrics_output_path', default_metrics)

        # Retrieve parameters
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value
        self.kp_lateral = self.get_parameter('kp_lateral').get_parameter_value().double_value
        self.ki_lateral = self.get_parameter('ki_lateral').get_parameter_value().double_value
        self.kd_lateral = self.get_parameter('kd_lateral').get_parameter_value().double_value
        self.kp_heading = self.get_parameter('kp_heading').get_parameter_value().double_value
        self.lookahead_dist = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.max_laps = self.get_parameter('max_laps').get_parameter_value().integer_value
        self.metrics_output_path = self.get_parameter('metrics_output_path').get_parameter_value().string_value

        self.get_logger().info(f"Workspace Root resolved to: {self.workspace_root}")
        self.get_logger().info(f"Loading reference path from: {self.csv_path}")
        self.path_points = self.load_path(self.csv_path)
        self.get_logger().info(f"Successfully loaded {len(self.path_points)} path points.")

        # Tracker state
        self.closest_idx = None
        self.integral_error_y = 0.0
        self.prev_error_y = 0.0
        self.prev_steering = 0.0
        self.last_time = None
        self.is_completed = False

        # Metrics collection lists
        self.history_ey = []
        self.history_epsi = []
        self.history_steering = []
        self.history_steering_rate = []
        self.history_time = []

        # Lap completion state
        self.completed_laps = 0
        self.lap_start_time = None
        self.lap_times = []
        
        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/ackermann_like_controller/reference', 10)
        self.metrics_summary_pub = self.create_publisher(String, '/tracking_metrics/summary', 10)
        
        # Individual metric publishers for PlotJuggler / RViz
        self.pub_lateral_error = self.create_publisher(Float64, '/tracking_metrics/lateral_error', 10)
        self.pub_heading_error = self.create_publisher(Float64, '/tracking_metrics/heading_error', 10)
        self.pub_lap_progress = self.create_publisher(Float64, '/tracking_metrics/lap_progress', 10)
        self.pub_steering_chatter = self.create_publisher(Float64, '/tracking_metrics/steering_chatter', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ackermann_like_controller/odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info("PID Tracker Node Initialized.")

    def find_workspace_root(self):
        """Resolves workspace root path dynamically by traversing up from the install share path."""
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
        """Loads path points from centerline or raceline CSV files dynamically."""
        path_points = []
        if not os.path.exists(file_path):
            self.get_logger().error(f"Path file does not exist: {file_path}")
            return path_points

        with open(file_path, 'r') as f:
            lines = f.readlines()

        delimiter = ','
        header_line = None
        
        # Identify structure and delimiter
        for line in lines:
            if line.startswith('#'):
                clean = line.lstrip('#').strip()
                if 'x_m' in clean or 'y_m' in clean:
                    header_line = clean
                    delimiter = ';' if ';' in clean else ','
                continue
            if not header_line and (',' in line or ';' in line):
                delimiter = ';' if ';' in line else ','
                break

        self.get_logger().info(f"Detected CSV delimiter: '{delimiter}'")

        for line in lines:
            if line.startswith('#') or not line.strip():
                continue
            parts = [p.strip() for p in line.split(delimiter)]
            if len(parts) < 2:
                continue
            try:
                vals = [float(p) for p in parts]
            except ValueError:
                continue

            if delimiter == ';':  # Raceline format: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
                if len(vals) >= 7:
                    path_points.append({
                        's': vals[0],
                        'x': vals[1],
                        'y': vals[2],
                        'psi': vals[3],
                        'kappa': vals[4],
                        'v': vals[5],
                        'a': vals[6]
                    })
            else:  # Centerline format: x_m, y_m, w_tr_right_m, w_tr_left_m
                if len(vals) >= 2:
                    path_points.append({
                        'x': vals[0],
                        'y': vals[1],
                        's': 0.0,
                        'psi': 0.0,
                        'v': 0.0
                    })

        # Precompute orientation (psi) and cumulative distance (s) if centerline CSV
        if delimiter == ',':
            n = len(path_points)
            if n > 1:
                s_accum = 0.0
                path_points[0]['s'] = 0.0
                for i in range(n):
                    next_i = (i + 1) % n
                    dx = path_points[next_i]['x'] - path_points[i]['x']
                    dy = path_points[next_i]['y'] - path_points[i]['y']
                    dist = math.hypot(dx, dy)
                    
                    # Compute path angle
                    path_points[i]['psi'] = math.atan2(dy, dx)
                    
                    if i < n - 1:
                        s_accum += dist
                        path_points[i+1]['s'] = s_accum
                
                # Close the loop tangent
                dx = path_points[0]['x'] - path_points[-1]['x']
                dy = path_points[0]['y'] - path_points[-1]['y']
                path_points[-1]['psi'] = math.atan2(dy, dx)

        return path_points

    def find_closest_point(self, rx, ry):
        """Finds closest point index and distance to path using a local window search."""
        if not self.path_points:
            return 0, 0.0

        n = len(self.path_points)
        if self.closest_idx is None:
            # Full scan on first callback
            min_dist = float('inf')
            best_idx = 0
            for i, pt in enumerate(self.path_points):
                dist = math.hypot(pt['x'] - rx, pt['y'] - ry)
                if dist < min_dist:
                    min_dist = dist
                    best_idx = i
            self.closest_idx = best_idx
            return best_idx, min_dist

        # Local window search (100 points around previous index)
        w = 100
        min_dist = float('inf')
        best_idx = self.closest_idx
        for offset in range(-w, w + 1):
            idx = (self.closest_idx + offset) % n
            pt = self.path_points[idx]
            dist = math.hypot(pt['x'] - rx, pt['y'] - ry)
            if dist < min_dist:
                min_dist = dist
                best_idx = idx

        self.closest_idx = best_idx
        return best_idx, min_dist

    def get_lookahead_point(self, start_idx, lookahead_dist):
        """Finds path point at a lookahead distance along the track."""
        n = len(self.path_points)
        accum_dist = 0.0
        curr_idx = start_idx
        while accum_dist < lookahead_dist:
            next_idx = (curr_idx + 1) % n
            p1 = self.path_points[curr_idx]
            p2 = self.path_points[next_idx]
            dist = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
            accum_dist += dist
            curr_idx = next_idx
            if next_idx == start_idx:
                break  # Prevent infinite loop if lookahead distance exceeds track length
        return self.path_points[curr_idx]

    def wrap_angle(self, angle):
        """Wraps angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg):
        if self.is_completed:
            # Send stop command
            self.publish_command(0.0, 0.0)
            return

        # Get simulation time
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = curr_time
            self.lap_start_time = curr_time
            return

        dt = curr_time - self.last_time
        if dt <= 0.0:
            dt = 0.01  # Fallback to avoid division by zero

        # Parse robot pose
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        
        # Quaternion to Euler yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # 1. Closest Point Calculations
        closest_idx, dist = self.find_closest_point(rx, ry)
        pt_close = self.path_points[closest_idx]
        psi_ref = pt_close['psi']

        # Signed lateral (cross-track) error: Left of path is positive, right is negative
        dx = rx - pt_close['x']
        dy = ry - pt_close['y']
        error_y = - dx * math.sin(psi_ref) + dy * math.cos(psi_ref)

        # Heading error at closest point (for metrics)
        error_psi = self.wrap_angle(psi_ref - yaw)

        # 2. Control Calculations (with optional Lookahead)
        if self.lookahead_dist > 0.0:
            pt_look = self.get_lookahead_point(closest_idx, self.lookahead_dist)
            # Reference steering target points to lookahead position
            target_psi = math.atan2(pt_look['y'] - ry, pt_look['x'] - rx)
            ctrl_error_psi = self.wrap_angle(target_psi - yaw)
            p_term = self.kp_heading * ctrl_error_psi
        else:
            p_term = self.kp_heading * error_psi

        # PID on Lateral Error
        self.integral_error_y += error_y * dt
        derivative_error_y = (error_y - self.prev_error_y) / dt
        
        # Anti-windup clamping for integral term
        self.integral_error_y = max(min(self.integral_error_y, 1.0), -1.0)
        
        lateral_feedback = self.kp_lateral * error_y + self.ki_lateral * self.integral_error_y + self.kd_lateral * derivative_error_y
        
        # Steering command = heading control term - lateral correction term
        if self.lookahead_dist > 0.0:
            # Lookahead target heading already integrates lateral error correction, but we add derivative/integral terms to resolve offsets
            steering = p_term - (self.ki_lateral * self.integral_error_y + self.kd_lateral * derivative_error_y)
        else:
            steering = p_term - lateral_feedback

        # Clamp steering command to physical limits (±0.461 rad / ~26.4 deg)
        steering = max(min(steering, 0.461), -0.461)

        # Speed target
        if self.target_speed >= 0.0:
            speed = self.target_speed
        else:
            # Follow profile speed from raceline CSV
            speed = pt_close.get('v', 2.0)

        # Safety slowdown: slow down if lateral error or steering is very high
        if abs(error_y) > 0.5 or abs(steering) > 0.3:
            speed = max(0.5, speed * 0.6)

        # Publish steering/speed
        self.publish_command(speed, steering)

        # 3. Lap Completion Logic
        n_points = len(self.path_points)
        # Check wrap-around from end of loop to start
        if self.closest_idx is not None:
            prev_idx = getattr(self, 'prev_closest_idx', closest_idx)
            # Detect forward crossing of start/finish line (wrapping from near n_points to near 0)
            if prev_idx > int(0.8 * n_points) and closest_idx < int(0.2 * n_points):
                lap_time = curr_time - self.lap_start_time
                self.completed_laps += 1
                self.lap_times.append(lap_time)
                self.get_logger().info(f"Lap {self.completed_laps} COMPLETED! Time: {lap_time:.3f} s")
                self.lap_start_time = curr_time

                if self.completed_laps >= self.max_laps:
                    self.get_logger().info(f"Target of {self.max_laps} lap(s) reached. Stopping vehicle and saving metrics...")
                    self.is_completed = True
                    self.save_metrics_baseline()
                    
            self.prev_closest_idx = closest_idx

        # 4. Metrics Recording
        self.history_ey.append(error_y)
        self.history_epsi.append(error_psi)
        self.history_steering.append(steering)
        self.history_time.append(curr_time)
        
        steering_rate = (steering - self.prev_steering) / dt
        self.history_steering_rate.append(steering_rate)

        # Publish metrics to topics
        self.publish_metrics_topics(error_y, error_psi, closest_idx / n_points, steering_rate)

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

    def publish_metrics_topics(self, error_y, error_psi, progress, steering_rate):
        self.pub_lateral_error.publish(Float64(data=error_y))
        self.pub_heading_error.publish(Float64(data=error_psi))
        self.pub_lap_progress.publish(Float64(data=progress))
        self.pub_steering_chatter.publish(Float64(data=abs(steering_rate)))

    def save_metrics_baseline(self):
        """Calculates running stats and saves them to a JSON file."""
        if not self.history_ey:
            self.get_logger().warn("No tracking history recorded. Metrics cannot be saved.")
            return

        abs_ey = np.abs(self.history_ey)
        abs_epsi = np.abs(self.history_epsi)
        abs_steering_rate = np.abs(self.history_steering_rate)

        metrics = {
            "reference_path": self.csv_path,
            "target_laps": self.max_laps,
            "total_laps_completed": self.completed_laps,
            "lap_times": self.lap_times,
            "lateral_error": {
                "mae": float(np.mean(abs_ey)),
                "rmse": float(math.sqrt(np.mean(np.square(self.history_ey)))),
                "max": float(np.max(abs_ey)),
                "std": float(np.std(self.history_ey))
            },
            "heading_error": {
                "mae": float(np.mean(abs_epsi)),
                "max": float(np.max(abs_epsi)),
                "std": float(np.std(self.history_epsi))
            },
            "oscillation": {
                "steering_chatter_radps": float(np.mean(abs_steering_rate)),
                "steering_std_rad": float(np.std(self.history_steering)),
                "lateral_error_std_m": float(np.std(self.history_ey))
            }
        }

        # Print to console
        self.get_logger().info("========================================")
        self.get_logger().info("      TRACKING BENCHMARK METRICS        ")
        self.get_logger().info("========================================")
        self.get_logger().info(f"Laps Completed:      {self.completed_laps}")
        self.get_logger().info(f"Lap Times (s):       {self.lap_times}")
        self.get_logger().info(f"Lateral Error MAE:   {metrics['lateral_error']['mae']:.4f} m")
        self.get_logger().info(f"Lateral Error RMSE:  {metrics['lateral_error']['rmse']:.4f} m")
        self.get_logger().info(f"Lateral Error Max:   {metrics['lateral_error']['max']:.4f} m")
        self.get_logger().info(f"Heading Error MAE:   {metrics['heading_error']['mae']:.4f} rad")
        self.get_logger().info(f"Heading Error Max:   {metrics['heading_error']['max']:.4f} rad")
        self.get_logger().info(f"Steering Chatter:    {metrics['oscillation']['steering_chatter_radps']:.4f} rad/s")
        self.get_logger().info(f"Steering Std Dev:    {metrics['oscillation']['steering_std_rad']:.4f} rad")
        self.get_logger().info("========================================")

        # Publish JSON summary string
        summary_msg = String()
        summary_msg.data = json.dumps(metrics, indent=2)
        self.metrics_summary_pub.publish(summary_msg)

        # Write to JSON file
        try:
            with open(self.metrics_output_path, 'w') as f:
                json.dump(metrics, f, indent=4)
            self.get_logger().info(f"Successfully saved tracking baseline metrics to: {self.metrics_output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write metrics baseline file: {e}")

    def destroy_node(self):
        # Force a metrics save if we are shutting down and haven't finished the laps yet
        if not self.is_completed and len(self.history_ey) > 100:
            self.get_logger().info("Node shutting down early. Saving partial metrics...")
            self.save_metrics_baseline()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
