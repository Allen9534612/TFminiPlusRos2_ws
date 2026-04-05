import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import threading

class TFminiPublisher(Node):
    def __init__(self):
        super().__init__('tfmini_publisher')
        
        # 定義四個埠口與對應的 Topic 名稱
        self.port_configs = [
            {'port': '/dev/ttyUSB0', 'topic': 'range/front_left',  'frame': 'lidar_fl'},
            {'port': '/dev/ttyUSB1', 'topic': 'range/front_right', 'frame': 'lidar_fr'},
            {'port': '/dev/ttyUSB2', 'topic': 'range/rear_left',   'frame': 'lidar_rl'},
            {'port': '/dev/ttyUSB3', 'topic': 'range/rear_right',  'frame': 'lidar_rr'}
        ]
        
        self.publishers_list = []
        self.serial_threads = []

        for cfg in self.port_configs:
            try:
                ser = serial.Serial(cfg['port'], 115200, timeout=0.1)
                pub = self.create_publisher(Range, cfg['topic'], 10)
                
                # 啟動獨立執行緒讀取數據
                thread = threading.Thread(
                    target=self.read_loop, 
                    args=(ser, pub, cfg['frame']), 
                    daemon=True
                )
                thread.start()
                
                self.get_logger().info(f"已成功連接 {cfg['port']} -> Topic: {cfg['topic']}")
                self.publishers_list.append(ser)
            except Exception as e:
                self.get_logger().error(f"無法開啟 {cfg['port']}: {e}")

    def read_loop(self, ser, pub, frame_id):
        """ 每個光達專屬的讀取迴圈 """
        while rclpy.ok():
            if ser.in_waiting >= 9:
                # 尋找標頭 0x59 0x59
                if ser.read(1) == b'\x59' and ser.read(1) == b'\x59':
                    data = ser.read(7)
                    dist_cm = data[0] + data[1] * 256
                    
                    # 封裝成 ROS 2 Range 訊息
                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = frame_id
                    msg.radiation_type = Range.INFRARED
                    msg.field_of_view = 0.035 # TFmini 約 2度 (弧度)
                    msg.min_range = 0.3      # 30cm
                    msg.max_range = 12.0     # 12m
                    msg.range = float(dist_cm) / 100.0 # 轉為公尺 (m)
                    
                    pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFminiPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()