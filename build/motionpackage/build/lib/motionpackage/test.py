import serial
import time

# 設定 Jetson UART 串口
uart = serial.Serial(
    port="/dev/ttyTHS1",  # Jetson Orin Nano UART2 (TX = Pin 8)
    baudrate=115200,
    timeout=1
)

if uart.isOpen():
    print("UART 已開啟，開始發送數據...")

try:
    while True:
        data = "Hello OpenCR\n"
        uart.write(data.encode())  # 發送字串
        print(f"發送: {data.strip()}")
        time.sleep(1)
except KeyboardInterrupt:
    uart.close()
    print("傳輸結束")
