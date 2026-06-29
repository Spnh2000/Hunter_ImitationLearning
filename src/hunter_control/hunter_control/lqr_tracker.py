import os
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Float64

# Import exact functions from local utils
from .utils import nearest_point
from .utils import update_matrix
from .utils import solve_lqr
from .utils import pi_2_pi

# ==============================================================================
# Exact LQRPlanner Class from f1tenth_planning Repository
# ==============================================================================
class LQRPlanner():
    """
    Lateral Controller using LQR
    Args:
        wheelbase (float, optional, default=0.33): wheelbase of the vehicle
        waypoints (numpy.ndarray [N, 5], optional, default=None): waypoints to track, columns are [x, y, velocity, heading, curvature]
    Attributes:
        wheelbase (float, optional, default=0.33): wheelbase of the vehicle
        waypoints (numpy.ndarray [N, 5], optional, default=None): waypoints to track, columns are [x, y, velocity, heading, curvature]
        vehicle_control_e_cog (float): lateral error of cog to ref trajectory
        vehicle_control_theta_e (float): yaw error to ref trajectory
    """
    def __init__(self, wheelbase=0.33, waypoints=None):
        self.wheelbase = 0.33
        self.waypoints = waypoints
        self.vehicle_control_e_cog = 0 # e_cg: lateral error of CoG to ref trajectory
        self.vehicle_control_theta_e = 0 # theta_e: yaw error to ref trajectory

    def calc_control_points(self, vehicle_state, waypoints):
        """
        Calculate the heading and cross-track errors and target velocity and curvature
        Args:
            vehicle_state (numpy.ndarray [4, ]): [x, y, heading, velocity] of the vehicle
            waypoints (numpy.ndarray [N, 5]): waypoints to track [x, y, velocity, heading, curvature]
        Returns:
            theta_e (float): heading error
            e_cog (float): lateral crosstrack error
            theta_raceline (float): target heading
            kappa_ref (float): target curvature
            goal_veloctiy (float): target velocity
        """
        # distance to the closest point to the front axle center
        fx = vehicle_state[0] + self.wheelbase * math.cos(vehicle_state[2])
        fy = vehicle_state[1] + self.wheelbase * math.sin(vehicle_state[2])
        position_front_axle = np.array([fx, fy])

        nearest_point_front, nearest_dist, t, target_index = nearest_point(position_front_axle, self.waypoints[:, 0:2])
        vec_dist_nearest_point = position_front_axle - nearest_point_front

        # crosstrack error
        front_axle_vec_rot_90 = np.array([[math.cos(vehicle_state[2] - math.pi / 2.0)], [math.sin(vehicle_state[2] - math.pi / 2.0)]])
        ef = np.dot(vec_dist_nearest_point.T, front_axle_vec_rot_90)

        # heading error
        # NOTE: If your raceline is based on a different coordinate system you need to -+ pi/2 = 90 degrees
        theta_raceline = waypoints[target_index, 3]
        theta_e = pi_2_pi(theta_raceline - vehicle_state[2])

        # target velocity
        goal_veloctiy = waypoints[target_index, 2]

        # reference curvature
        kappa_ref = self.waypoints[target_index, 4]

        # saving control errors
        self.vehicle_control_e_cog = ef[0]
        self.vehicle_control_theta_e = theta_e

        return theta_e, ef[0], theta_raceline, kappa_ref, goal_veloctiy

    def controller(self, vehicle_state, waypoints, ts, matrix_q, matrix_r, max_iteration, eps):
        """
        Compute lateral control command.
        Args:
            vehicle_state (numpy.ndarray [4, ]): [x, y, heading, velocity] of the vehicle
            waypoints (numpy.ndarray [N, 5]): waypoints to track
            ts (float): discretization time step
            matrix_q ([float], len=4): weights on the states
            matrix_r ([float], len=1): weights on control input
            max_iteration (int): maximum iteration for solving
            eps (float): error tolerance for solving
        Returns:
            steer_angle (float): desired steering angle
            v_ref (float): desired velocity
        """
        # size of controlled states
        state_size = 4

        # Saving lateral error and heading error from previous timestep
        e_cog_old = self.vehicle_control_e_cog
        theta_e_old = self.vehicle_control_theta_e

        # Calculating current errors and reference points from reference trajectory
        theta_e, e_cg, yaw_ref, k_ref, v_ref = self.calc_control_points(vehicle_state, waypoints)

        #Update the calculation matrix based on the current vehicle state
        matrix_ad_, matrix_bd_ = update_matrix(vehicle_state, state_size, ts, self.wheelbase)

        matrix_state_ = np.zeros((state_size, 1))

        matrix_r_ = np.diag(matrix_r)
        matrix_q_ = np.diag(matrix_q)

        matrix_k_ = solve_lqr(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, eps, max_iteration)

        matrix_state_[0][0] = e_cg
        matrix_state_[1][0] = (e_cg - e_cog_old) / ts
        matrix_state_[2][0] = theta_e
        matrix_state_[3][0] = (theta_e - theta_e_old) / ts

        steer_angle_feedback = (matrix_k_ @ matrix_state_)[0][0]

        #Compute feed forward control term to decrease the steady error.
        steer_angle_feedforward = k_ref * self.wheelbase

        # Calculate final steering angle in [rad]
        steer_angle = steer_angle_feedback + steer_angle_feedforward

        return steer_angle, v_ref

    def plan(self, pose_x, pose_y, pose_theta, velocity, timestep=0.01, matrix_q_1=0.999, matrix_q_2=0.0, matrix_q_3=0.0066, matrix_q_4=0.0, matrix_r=0.75, iterations=50, eps=0.001, waypoints=None):
        """
        Compute lateral control command.
        Args:
            pose_x (float):
            pose_y (float):
            pose_theta (float):
            velocity (float):
            timestep (float, optional, default=0.01):
            matrix_q_1 (float, optional, default=1.0):
            matrix_q_2 (float, optional, default=0.95):
            matrix_q_3 (float, optional, default=0.0066):
            matrix_q_4 (float, optional, default=0.0257):
            matrix_r (float, optional, default=0.0062):
            iterations (int, optional, default=50):
            eps (float, optional, default=0.01):
            waypoints (numpy.ndarray [N x 5], optional, default=None):
        Returns:
            steering_angle (float): desired steering angle
            speed (float): desired speed
        """
        if waypoints is not None:
            if waypoints.shape[1] < 5 or len(waypoints.shape) != 2:
                raise ValueError('Waypoints needs to be a (Nxm), m >= 5, numpy array!')
            self.waypoints = waypoints
        else:
            if self.waypoints is None:
                raise ValueError('Please set waypoints to track during planner instantiation or when calling plan()')

        #Define LQR Matrix and Parameter
        matrix_q = [matrix_q_1, matrix_q_2, matrix_q_3, matrix_q_4]
        matrix_r = [matrix_r]

        #Define a numpy array that includes the current vehicle state: x,y, theta, veloctiy
        vehicle_state = np.array([pose_x, pose_y, pose_theta, velocity])

        #Calculate the steering angle and the speed in the controller
        steering_angle, speed = self.controller(vehicle_state, self.waypoints, timestep, matrix_q, matrix_r, iterations, eps)

        return steering_angle, speed


# ==============================================================================
# ROS 2 Wrapper Node calling the exact LQRPlanner class
# ==============================================================================
class LQRTracker(Node):
    def __init__(self):
        super().__init__('lqr_tracker')

        # Trajectory and Output Path Dynamic Resolution
        self.workspace_root = self.find_workspace_root()
        default_csv = os.path.join(self.workspace_root, 'src', 'Monza', 'Monza_raceline.csv')
        default_metrics = os.path.join(self.workspace_root, 'metrics_lqr_baseline.json')

        # Declare parameters
        self.declare_parameter('csv_path', default_csv)
        self.declare_parameter('target_speed', 2.5)
        self.declare_parameter('wheelbase', 0.65)  # AgileX Hunter V2 wheelbase (m)
        self.declare_parameter('max_laps', 1)
        self.declare_parameter('metrics_output_path', default_metrics)
        
        # Q and R weights parameter mapping
        self.declare_parameter('q_lateral', 2.0)
        self.declare_parameter('q_lateral_dot', 0.2)
        self.declare_parameter('q_heading', 1.5)
        self.declare_parameter('q_heading_dot', 0.1)
        self.declare_parameter('r_steering', 1.0)

        # Retrieve parameters
        self.csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.target_speed = self.get_parameter('target_speed').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_laps = self.get_parameter('max_laps').get_parameter_value().integer_value
        self.metrics_output_path = self.get_parameter('metrics_output_path').get_parameter_value().string_value

        self.q_lat = self.get_parameter('q_lateral').value
        self.q_lat_dot = self.get_parameter('q_lateral_dot').value
        self.q_head = self.get_parameter('q_heading').value
        self.q_head_dot = self.get_parameter('q_heading_dot').value
        self.r_steer = self.get_parameter('r_steering').value

        self.get_logger().info(f"Workspace Root resolved to: {self.workspace_root}")
        self.get_logger().info(f"Loading reference path from: {self.csv_path}")
        self.path_points = self.load_path(self.csv_path)
        self.get_logger().info(f"Successfully loaded {len(self.path_points)} path waypoints.")

        # Convert waypoints to NumPy (N, 5) array for the planner
        self.waypoints_np = np.zeros((len(self.path_points), 5))
        for i, pt in enumerate(self.path_points):
            self.waypoints_np[i, 0] = pt['x']
            self.waypoints_np[i, 1] = pt['y']
            self.waypoints_np[i, 2] = pt.get('v', self.target_speed)
            self.waypoints_np[i, 3] = pt['psi']
            self.waypoints_np[i, 4] = pt.get('kappa', 0.0)

        # Instantiate the exact repo LQRPlanner
        self.planner = LQRPlanner(wheelbase=self.wheelbase, waypoints=self.waypoints_np)
        self.planner.wheelbase = self.wheelbase # Overrides default 0.33 with parameter (0.65)

        # Tracker state
        self.prev_steering = 0.0
        self.last_time = None
        self.is_completed = False

        # Lap completion state
        self.completed_laps = 0
        self.lap_start_time = None
        self.halfway_passed = False
        self.lap_times = []

        # History arrays for calculating metrics
        self.history_ey = []
        self.history_epsi = []
        self.history_steering = []
        self.history_steering_rate = []

        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/ackermann_like_controller/reference', 10)
        self.pub_ey = self.create_publisher(Float64, '/tracking_metrics/lateral_error', 10)
        self.pub_epsi = self.create_publisher(Float64, '/tracking_metrics/heading_error', 10)
        self.pub_progress = self.create_publisher(Float64, '/tracking_metrics/lap_progress', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ackermann_like_controller/odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info("LQR Tracker Node calling exact repository code started.")

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
                        'x': vals[1],
                        'y': vals[2],
                        'psi': vals[3],
                        'kappa': vals[4],
                        'v': vals[5]
                    })
            else:  # Centerline format: x_m, y_m, w_tr_right_m, w_tr_left_m
                if len(vals) >= 2:
                    path_points.append({
                        'x': vals[0],
                        'y': vals[1],
                        's': 0.0,
                        'psi': 0.0,
                        'kappa': 0.0,
                        'v': self.target_speed
                    })

        # Precompute orientation (psi) and curvature (kappa) if centerline CSV
        if delimiter == ',':
            n = len(path_points)
            if n > 2:
                for i in range(n):
                    next_i = (i + 1) % n
                    dx = path_points[next_i]['x'] - path_points[i]['x']
                    dy = path_points[next_i]['y'] - path_points[i]['y']
                    path_points[i]['psi'] = math.atan2(dy, dx)
                    
                # Compute approximate curvature
                for i in range(n):
                    prev_i = (i - 1) % n
                    next_i = (i + 1) % n
                    p1 = path_points[prev_i]
                    p2 = path_points[i]
                    p3 = path_points[next_i]
                    
                    area = p1['x']*(p2['y'] - p3['y']) + p2['x']*(p3['y'] - p1['y']) + p3['x']*(p1['y'] - p2['y'])
                    side1 = math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])
                    side2 = math.hypot(p2['x'] - p3['x'], p2['y'] - p3['y'])
                    side3 = math.hypot(p3['x'] - p1['x'], p3['y'] - p1['y'])
                    
                    if side1 * side2 * side3 > 1e-6:
                        # Signed curvature (using 2.0 * det / (a*b*c))
                        path_points[i]['kappa'] = 2.0 * area / (side1 * side2 * side3)
                    else:
                        path_points[i]['kappa'] = 0.0

        return path_points

    def odom_callback(self, msg):
        if self.is_completed:
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
            dt = 0.01

        # Parse robot pose
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        v_current = msg.twist.twist.linear.x

        # We clamp speed slightly in the state-space matrices internally in the solver to avoid divide-by-zero
        v_m = max(abs(v_current), 0.1)

        # Call exact f1tenth LQR planner
        try:
            # We pass velocity as v_m for solver matrix updates to ensure stability at standstill
            steering, target_speed = self.planner.plan(
                pose_x=rx,
                pose_y=ry,
                pose_theta=yaw,
                velocity=v_m,
                timestep=dt,
                matrix_q_1=self.q_lat,
                matrix_q_2=self.q_lat_dot,
                matrix_q_3=self.q_head,
                matrix_q_4=self.q_head_dot,
                matrix_r=self.r_steer,
                iterations=150,
                eps=0.01,
                waypoints=self.waypoints_np
            )
        except Exception as e:
            self.get_logger().error(f"LQR planner calculation failed: {e}")
            steering = 0.0
            target_speed = self.target_speed

        # Steering limits clamp
        steering = max(min(steering, 0.461), -0.461)

        # Extract errors calculated by the planner class for logging consistency
        error_y = self.planner.vehicle_control_e_cog
        # Keep consistent heading error sign relative to target
        error_psi = -self.planner.vehicle_control_theta_e

        # Speed logic (slow down slightly on tight corners or high lateral error)
        speed = self.target_speed
        if abs(error_y) > 0.5 or abs(steering) > 0.3:
            speed = max(0.5, speed * 0.6)

        # Send command to Ackermann controller
        self.publish_command(speed, steering)

        # Gated Lap Completion Logic using closest index to track progress
        dists = [math.hypot(pt[0] - rx, pt[1] - ry) for pt in self.waypoints_np]
        closest_idx = dists.index(min(dists))

        n_points = len(self.path_points)
        if closest_idx > int(0.4 * n_points) and closest_idx < int(0.8 * n_points):
            self.halfway_passed = True

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

        # Metrics Recording
        self.history_ey.append(error_y)
        self.history_epsi.append(error_psi)
        self.history_steering.append(steering)
        self.history_steering_rate.append((steering - self.prev_steering) / dt)

        # Stream metrics to ROS 2 topics
        self.pub_ey.publish(Float64(data=error_y))
        self.pub_epsi.publish(Float64(data=error_psi))
        self.pub_progress.publish(Float64(data=closest_idx / n_points))

        # Save previous states
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

        mean_abs_ey = sum(abs_ey) / len(abs_ey)
        rmse_ey = math.sqrt(sum(e**2 for e in self.history_ey) / len(self.history_ey))
        max_ey = max(abs_ey)
        
        mean_abs_epsi = sum(abs_epsi) / len(abs_epsi)
        max_epsi = max(abs_epsi)
        
        chatter = sum(abs_chatter) / len(abs_chatter)
        
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
        self.get_logger().info("      TRACKING BENCHMARK METRICS (LQR)  ")
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
    node = LQRTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
