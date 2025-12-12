#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray

class HealthMonitor(Node):
    def __init__(self):
        super().__init__("tb3_health_monitor")
        self.create_subscription(BatteryState, "/battery_state", self.on_battery, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diag, 10)
        # Tune threshold for your battery. TB3 publishes battery voltage on /battery_state.
        self.low_voltage = 10.8
        self.get_logger().info("Health monitor started: /battery_state, /diagnostics")

    def on_battery(self, msg: BatteryState):
        v = float(msg.voltage)
        pct = float(msg.percentage) if msg.percentage is not None else -1.0
        if v and v < self.low_voltage:
            self.get_logger().warn(f"LOW BATTERY: voltage={v:.2f}V percentage={pct:.2f}")
        else:
            self.get_logger().info(f"Battery OK: voltage={v:.2f}V percentage={pct:.2f}")

    def on_diag(self, msg: DiagnosticArray):
        for st in msg.status:
            if int(st.level) >= 2:  # 2=ERROR
                self.get_logger().error(f"DIAGNOSTIC ERROR: {st.name}: {st.message}")

def main():
    rclpy.init()
    node = HealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
