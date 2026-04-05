import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math
import message_filters

class MultiSideAngleCalculator(Node):
    def __init__(self):
        super().__init__('multi_angle_calculator')
        
        # --- 機器人幾何參數 (公尺) ---
        self.L_side = 0.4  # 左側前後雷達的間距
        self.L_front = 0.3 # 前方左右雷達的間距
        
        # 1. 建立四個訂閱者
        self.sub_fl = message_filters.Subscriber(self, Range, 'range/front_left')
        self.sub_rl = message_filters.Subscriber(self, Range, 'range/rear_left')
        self.sub_fr = message_filters.Subscriber(self, Range, 'range/front_right')
        # self.sub_rr = message_filters.Subscriber(self, Range, 'range/rear_right') # 如果右側也要算再開啟

        # 2. 建立同步器 - 左側組 (FL + RL)
        self.ts_left = message_filters.ApproximateTimeSynchronizer(
            [self.sub_fl, self.sub_rl], queue_size=10, slop=0.1)
        self.ts_left.registerCallback(self.left_side_callback)

        # 3. 建立同步器 - 前側組 (FL + FR)
        self.ts_front = message_filters.ApproximateTimeSynchronizer(
            [self.sub_fl, self.sub_fr], queue_size=10, slop=0.1)
        self.ts_front.registerCallback(self.front_side_callback)

        self.get_logger().info("多側邊角度監測已啟動...")

    def calculate_angle(self, d1, d2, L):
        """ 基礎三角函數計算 """
        return math.degrees(math.atan2(d1 - d2, L))

    def left_side_callback(self, msg_fl, msg_rl):
        """ 計算與左側牆壁的角度 """
        angle = self.calculate_angle(msg_fl.range, msg_rl.range, self.L_side)
        dist = (msg_fl.range + msg_rl.range) / 2.0
        # 使用 get_logger 輸出，方便在終端機查看
        self.get_logger().info(f"【左側牆】距離: {dist:.2f}m, 角度: {angle:6.2f}°")

    def front_side_callback(self, msg_fl, msg_fr):
        """ 計算與前方牆壁的角度 """
        # 注意：前方角度通常是用來判斷機器人是否垂直於前牆
        angle = self.calculate_angle(msg_fl.range, msg_fr.range, self.L_front)
        dist = (msg_fl.range + msg_fr.range) / 2.0
        self.get_logger().info(f"【前側牆】距離: {dist:.2f}m, 角度: {angle:6.2f}°")

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