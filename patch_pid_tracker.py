import re

with open('src/hunter_control/hunter_control/pid_tracker.py', 'r') as f:
    content = f.read()

# 1. Update imports
content = content.replace(
    'import math\nimport rclpy\nfrom rclpy.node import Node',
    'import math\nimport rclpy\nimport csv\nimport os\nfrom rclpy.node import Node'
)

# 2. Replace WAYPOINTS and __init__
pattern_waypoints = re.compile(r'    # Hardcoded 2D waypoints.*?\n    def __init__\(self\):\n        super\(\)\.__init__\(\'lateral_pid_controller\'\)\n        self\.get_logger\(\)\.info\("Initializing Lateral PID Controller Node\.\.\."\)', re.DOTALL)

replacement_init = """    def __init__(self):
        super().__init__('lateral_pid_controller')
        self.get_logger().info("Initializing Lateral PID Controller Node...")

        self.WAYPOINTS = []
        self.TARGET_VELOCITIES = []

        # Load CSV waypoints
        csv_path = './IMS_raceline.csv'
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f, delimiter=';')
                # Skip the first 3 rows as requested
                for _ in range(3):
                    next(reader, None)
                for row in reader:
                    # Expected columns: x_m; y_m; w_tr_right_m; w_tr_left_m; v_m_s
                    if len(row) >= 5:
                        self.WAYPOINTS.append([float(row[0]), float(row[1])])
                        self.TARGET_VELOCITIES.append(float(row[4]))
            self.get_logger().info(f"Loaded {len(self.WAYPOINTS)} waypoints with dynamic velocity profiling from {csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV from {csv_path}: {e}")
            self.WAYPOINTS = [[0.0, 0.0], [1.0, 0.0]]  # Fallback to prevent immediate crash
            self.TARGET_VELOCITIES = [1.0, 1.0]"""

content = pattern_waypoints.sub(replacement_init, content)

# 3. Replace speed logic
pattern_speed = re.compile(r'        # 3\.5 Speed logic.*?dynamic_lookahead = max\(1\.0, self\.lookahead_distance \* \(speed / self\.target_speed\)\)', re.DOTALL)

replacement_speed = """        # 3.5 Dynamic velocity profiling logic
        base_speed = self.TARGET_VELOCITIES[closest_idx]

        # Safety fallback: slow down on high lateral error to maintain traction
        speed = base_speed
        if abs(cte) > 0.15 or abs(self.prev_error) > 0.10:
            speed = max(1.2, speed * 0.6)

        # Calculate dynamic lookahead distance based on the speed profile
        # self.lookahead_distance is treated as the baseline lookahead distance at self.target_speed
        lookahead_time = self.lookahead_distance / max(0.1, self.target_speed)
        dynamic_lookahead = max(1.0, speed * lookahead_time)"""

content = pattern_speed.sub(replacement_speed, content)

with open('src/hunter_control/hunter_control/pid_tracker.py', 'w') as f:
    f.write(content)

print("Patch applied successfully.")
