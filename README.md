# TFmini Plus ROS 2 多光達角度校正系統 (Multi-LiDAR Alignment)

這是一個標準的 **ROS 2 Humble** 功能包。透過四個 **TFmini Plus** 光達 (LiDAR)，即時計算機器人相對於牆面的 **側邊偏航角 (Side Yaw)** 與 **前方垂直角 (Frontal Angle)**。

---

## 📌 功能特點
* **多執行緒驅動**：`tfmini_publisher` 節點同時監控 4 路 UART 序列埠，解決數據阻塞。
* **精準數據同步**：利用 `message_filters` 同步不同雷達的時間戳記，計算結果更穩定。
* **標準訊息格式**：採用 `sensor_msgs/Range` 格式，可直接與 Rviz2 或其他導航節點對接。

## 🛠 硬體與安裝
* **感測器**: TFmini Plus (UART 版本) x 4
* **連線**: USB 轉 TTL 模組
* **幾何配置**:
    * **右側**: 前後各一個（間距 $L_{side}$）
    * **前方**: 左右各一個（間距 $L_{front}$）



---

## 🚀 快速上手

### 1. 安裝依賴環境
請確保已安裝 ROS 2 數據同步套件：
```bash
sudo apt update
sudo apt install ros-humble-message-filters
```

### 2. 編譯工作空間
將此 Repo 放進你的 colcon 工作空間（例如 ~/ros2_ws/src）並編譯：
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 3. 設定硬體權限
將當前用戶加入 dialout 群組以讀取 USB 埠口：
```bash
sudo usermod -a -G dialout $USER
```

### 4. 執行節點
開啟兩個終端機，分別啟動驅動與計算節點：

啟動硬體讀取 (Publisher)
```bash
ros2 run tfmini_plus tfmini_pub
```

啟動角度計算 (Calculator)
```bash
ros2 run lidar_processor calculate_angle
```

## 📐 數學運算原理
### 系統透過兩點距離差 $\Delta d$ 與安裝間距 $L$，利用反餘弦函數計算夾角：
$$ \theta = \arctan\left(\frac{d_{front} - d_{rear}}{L}\right) $$

## 📂 檔案結構
 tfmini_publisher.py: 解析 TFmini 協議並發佈 Range 訊息。

 angle_calculator.py: 訂閱數據並進行三角函數運算，輸出校正 Log。

 setup.py: 定義 ROS 2 節點入口點（Entry Points）。

## 💡 使用小技巧
 udev rules: 由於多個 USB 轉 TTL 模組插拔順序可能變動，建議撰寫 /etc/udev/rules.d/ 規則固定埠口名稱（如 ttyLidar_FL）。

 參數調整: 若計算出的角度正負號與預期相反，請交換 calculate_angle 中的 d1 與 d2 參數位置。

