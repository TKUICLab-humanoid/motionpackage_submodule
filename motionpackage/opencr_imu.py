import rclpy
from rclpy.node import Node
from tku_msgs.msg import SensorPackage, SensorSet
import serial
import threading


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # 初始化串口鎖
        self.serial_lock = threading.Lock()

        # 嘗試配置串口與 OpenCR 通信
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
            self.get_logger().info("Serial port initialized successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize serial port: {e}")
            self.serial_port = None

        # 創建 Publisher：發佈 IMU 數據到 '/package/sensorpackage'
        self.sensor_data_pub = self.create_publisher(SensorPackage, '/package/sensorpackage', 10)

        # 創建 Subscriber：接收指令控制 IMU 重置
        self.sensor_set_sub = self.create_subscription(SensorSet, '/sensorset', self.SensorSetFunction, 10)

        # 啟動 IMU 資料讀取執行緒（如果串口初始化成功）
        if self.serial_port:
            self.serial_thread = threading.Thread(target=self.imu_port, daemon=True)
            self.serial_thread.start()

    def SensorSetFunction(self, msg):
        """
        Subscriber 回調函式，用於處理從 '/sensorset' 主題接收到的指令。
        當接收到 pitch 為 0 的指令時，向 OpenCR 發送 'R' 指令以重置 IMU。
        """
        if self.serial_port:
            with self.serial_lock:
                if msg.pitch == 0:
                    self.get_logger().info("Received reset command. Sending 'R' to OpenCR...")
                    self.serial_port.write(b'R')

    def imu_port(self):
        """
        獨立執行緒函式，持續讀取 IMU 數據，並將其發佈到 ROS 2 主題。
        """
        try:
            self.get_logger().info("Listening for IMU data...")
            while rclpy.ok():  # 保持執行緒運行，直到 ROS 2 停止
                with self.serial_lock:
                    if self.serial_port.in_waiting > 0:  # 檢查是否有可用數據
                        line = self.serial_port.readline().decode('utf-8').strip()
                        if line:
                            # 解析 IMU 數據
                            parsed_data = self.parse_imu_data(line)
                            if parsed_data:
                                roll, pitch, yaw = parsed_data
                                # 發佈數據到 '/package/sensorpackage'
                                msg = SensorPackage()
                                msg.x = roll
                                msg.y = pitch
                                msg.z = yaw
                                self.sensor_data_pub.publish(msg)
                                self.get_logger().info(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
                            else:
                                self.get_logger().warn(f"Invalid data received: {line}")
        except Exception as e:
            self.get_logger().error(f"IMU port error: {e}")
        finally:
            with self.serial_lock:
                if self.serial_port:
                    self.serial_port.close()
                    self.get_logger().info("Serial port closed.")

    def parse_imu_data(self, data):
        """
        解析 IMU 傳來的 roll, pitch, yaw 數據。
        數據格式應為 "roll,pitch,yaw"（用逗號分隔）。
        """
        try:
            # 將數據字符串分割並轉換為浮點數
            roll, pitch, yaw = map(float, data.split(","))
            return roll, pitch, yaw
        except ValueError:
            return None


def main(args=None):
    """
    主入口函式：初始化 ROS 2 節點並開始執行。
    """
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()

    try:
        rclpy.spin(imu_publisher)  # ROS 2 主事件循環
    except KeyboardInterrupt:
        imu_publisher.get_logger().info("Keyboard interrupt detected. Shutting down node.")
    finally:
        imu_publisher.destroy_node()  # 銷毀節點
        rclpy.shutdown()


if __name__ == '__main__':
    main()
