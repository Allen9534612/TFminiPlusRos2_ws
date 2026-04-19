import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math
import message_filters

class MultiSideAngleCalculator(Node):
    def __init__(self):
        super().__init__('multi_angle_calculator')
        
        # --- 機器人幾何參數 ---
        self.L_right = 0.17 # 右側前後雷達間距
        self.L_front = 0.175  # 前方左右雷達間距
        
        # 暫存最新數據
        self.data = {
            'right': {'dist': 0.0, 'angle': 0.0}, # 改為 right
            'front': {'dist': 0.0, 'angle': 0.0}
        }
        
        # 1. 建立四個訂閱者 (依照你修改後的名稱)
        self.sub_fl = message_filters.Subscriber(self, Range, 'range/front_left')
        self.sub_fr = message_filters.Subscriber(self, Range, 'range/front_right')
        self.sub_rf = message_filters.Subscriber(self, Range, 'range/right_front')
        self.sub_rr = message_filters.Subscriber(self, Range, 'range/right_rear')

        # 2. 建立同步器
        self.ts_right = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rf, self.sub_rr], 10, 0.1)
        self.ts_right.registerCallback(self.right_side_callback)

        self.ts_front = message_filters.ApproximateTimeSynchronizer(
            [self.sub_fl, self.sub_fr], 10, 0.1)
        self.ts_front.registerCallback(self.front_side_callback)

        # 定時器刷新螢幕
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info("多側邊角度監測已啟動 (右側 & 前方)...")

    def calculate_angle(self, d1, d2, L):
        pass

    def right_side_callback(self, msg_rf, msg_rr):
        """ 計算與右側牆壁的角度 """
        self.data['right']['angle'] = self.calculate_angle(msg_rf.range, msg_rr.range, self.L_right)
        self.data['right']['dist'] = (msg_rf.range + msg_rr.range) / 2.0

    def front_side_callback(self, msg_fl, msg_fr):
        """ 計算與前方牆壁的角度 """
        self.data['front']['angle'] = self.calculate_angle(msg_fl.range, msg_fr.range, self.L_front)
        self.data['front']['dist'] = (msg_fl.range + msg_fr.range) / 2.0

    def timer_callback(self):
        r_d = self.data['right']['dist']
        r_a = self.data['right']['angle']
        f_d = self.data['front']['dist']
        f_a = self.data['front']['angle']
        
        self.get_logger().info(
            f"R: {r_d:.2f}m, {r_a:5.1f}° | F: {f_d:.2f}m, {f_a:5.1f}°"
        )

def main():
    rclpy.init()
    node = MultiSideAngleCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()