"""Robot health monitor: battery state, diagnostics and a one-line status for the forklift.

Publishes
  /battery_state   sensor_msgs/BatteryState      simulated pack (see battery_model.py)
  /diagnostics     diagnostic_msgs/DiagnosticArray   battery, drive, fork and docking status
Subscribes
  /odom            nav_msgs/Odometry      speed and yaw rate
  /joint_states    sensor_msgs/JointState fork height (fork_lift_joint)
  /dock/state      std_msgs/String        docking node state (latched)
  /battery/charging  std_msgs/Bool        publish true to simulate being on a charger

Real trucks expose exactly this kind of data to a fleet manager, so a dashboard (rqt_robot_monitor,
Foxglove, Grafana) works on the simulated robot the same way as on a real one.

  ros2 launch forklift_nav monitor.launch.py
  ros2 launch forklift_nav monitor.launch.py time_scale:=60.0     # drain an hour per minute
"""
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, String

from forklift_nav.battery_model import BatteryModel, BatteryParams

FORK_JOINT = 'fork_lift_joint'


class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')
        self.declare_parameter('start_soc', 0.95)
        self.declare_parameter('time_scale', 1.0)
        self.declare_parameter('max_speed', 0.4)          # m/s, same as nav2 max_vel_x
        self.declare_parameter('publish_rate', 1.0)       # Hz for /battery_state and /diagnostics

        params = BatteryParams(time_scale=float(self.get_parameter('time_scale').value))
        self.battery = BatteryModel(params, soc=float(self.get_parameter('start_soc').value))
        self.max_speed = float(self.get_parameter('max_speed').value)

        self.speed = 0.0
        self.yaw_rate = 0.0
        self.fork_height = None
        self.fork_moving = False
        self.fork_last = None            # (height, time_s)
        self.fork_still_since = None
        self.dock_state = 'unknown'
        self.charging = False
        self.last_level = 'ok'
        self.last_step = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.batt_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/dock/state', self.on_dock_state, latched)
        self.create_subscription(Bool, '/battery/charging', self.on_charging, 10)
        self.create_timer(0.1, self.integrate)
        self.create_timer(1.0 / max(float(self.get_parameter('publish_rate').value), 0.1), self.publish)
        self.get_logger().info('robot_monitor ready: /battery_state and /diagnostics')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- subscriptions ----------------------------------------------------------------
    def on_odom(self, msg):
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)
        self.yaw_rate = msg.twist.twist.angular.z

    def on_joints(self, msg):
        if FORK_JOINT not in msg.name:
            return
        height = msg.position[msg.name.index(FORK_JOINT)]
        now = self.now_s()
        if self.fork_last is not None:
            dt = now - self.fork_last[1]
            if dt > 0.0 and abs(height - self.fork_last[0]) / dt > 0.005:   # > 5 mm/s
                self.fork_moving = True
                self.fork_still_since = None
            elif self.fork_still_since is None:
                self.fork_still_since = now
            elif now - self.fork_still_since > 0.5:
                self.fork_moving = False
        self.fork_last = (height, now)
        self.fork_height = height

    def on_dock_state(self, msg):
        self.dock_state = msg.data

    def on_charging(self, msg):
        self.charging = bool(msg.data)

    # ---- timers -----------------------------------------------------------------------
    def integrate(self):
        now = self.now_s()
        if self.last_step is not None:
            dt = now - self.last_step
            if 0.0 < dt < 1.0:
                self.battery.step(dt, self.speed, self.yaw_rate, self.fork_moving, self.charging)
        self.last_step = now
        level = self.battery.level
        if level != self.last_level:
            text = f'battery {level}: {self.battery.soc * 100:.0f} %'
            if level == 'ok':
                self.get_logger().info(text)
            elif level == 'low':
                self.get_logger().warn(text + ', go charge')
            else:
                self.get_logger().error(text + ', stop and charge')
            self.last_level = level

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self.batt_pub.publish(self.battery_msg(stamp))
        self.diag_pub.publish(self.diagnostics_msg(stamp))

    def battery_msg(self, stamp):
        b = self.battery
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.voltage = float(b.voltage)
        msg.current = float(b.current)
        msg.design_capacity = float(b.p.capacity_wh / b.p.v_full)       # Ah
        msg.capacity = msg.design_capacity
        msg.charge = float(msg.capacity * b.soc)
        msg.percentage = float(b.soc)
        if b.charging:
            msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL if b.soc >= 0.999
                                       else BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = (BatteryState.POWER_SUPPLY_HEALTH_GOOD if b.level == 'ok'
                                   else BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        msg.present = True
        msg.location = 'forklift'
        msg.serial_number = 'SIM-LFP-24V-60AH'
        return msg

    def status(self, name, level, message, **values):
        s = DiagnosticStatus()
        s.name = f'forklift: {name}'
        s.hardware_id = 'forklift'
        s.level = level
        s.message = message
        s.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        return s

    def diagnostics_msg(self, stamp):
        b = self.battery
        level = {'ok': DiagnosticStatus.OK, 'low': DiagnosticStatus.WARN,
                 'critical': DiagnosticStatus.ERROR}[b.level]
        battery = self.status('battery', level, f'{b.level}, {b.soc * 100:.0f} %',
                              percentage=f'{b.soc * 100:.1f}', voltage=f'{b.voltage:.2f}',
                              current=f'{b.current:.1f}', power_w=f'{b.power_w:.0f}',
                              charging=b.charging)
        over = self.speed > self.max_speed * 1.1
        drive = self.status('drive', DiagnosticStatus.WARN if over else DiagnosticStatus.OK,
                            'over speed limit' if over else 'ok',
                            speed=f'{self.speed:.2f}', yaw_rate=f'{self.yaw_rate:.2f}',
                            speed_limit=self.max_speed)
        height = 'n/a' if self.fork_height is None else f'{self.fork_height:.3f}'
        fork = self.status('fork', DiagnosticStatus.OK, 'moving' if self.fork_moving else 'idle',
                           height_m=height)
        failed = self.dock_state.startswith('failed')
        dock = self.status('docking', DiagnosticStatus.ERROR if failed else DiagnosticStatus.OK,
                           self.dock_state, state=self.dock_state)
        arr = DiagnosticArray()
        arr.header.stamp = stamp
        arr.status = [battery, drive, fork, dock]
        return arr


def main():
    rclpy.init()
    node = RobotMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
