#!/usr/bin/env python3
# coding=utf-8
import serial
import time
import threading
import binascii
import rclpy.logging
from std_msgs.msg import Int16,Bool
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tku_msgs.msg import SensorPackage,SensorSet,HeadPackage,InterfaceSend2Sector,SaveMotion,SaveMotionVector,Location,Parametermessage,Interface,Dio
from tku_msgs.srv import ReadMotion,CheckSector,WalkingGaitParameter
import rclpy
from collections import namedtuple
import os
import sys
import configparser
import struct
#宣告tsRobotis的資料結構
Motor = namedtuple("Motor", ["ID", "position", "speed"])
from dataclasses import dataclass
import numpy as np
import select
import toml
import Jetson.GPIO as GPIO



@dataclass
class GaitParameters:
    com_y_swing: float
    com_y_swing_range: float
    period_t: int
    osc_lockrange: float
    base_default_z: float
    now_stand_height: float
    now_com_height: float
    stand_balance: bool

class Motionpackage(Node):
    
    def __init__(self):
        super().__init__('motionpackage_node')
        qos_profile = QoSProfile(depth=1000)
        self.sensor_data_pub    = self.create_publisher(SensorPackage, '/package/sensorpackage', qos_profile)
        # self.Dio_pub    = self.create_publisher(Dio, '/package/FPGAack', qos_profile)
        self.Dio_pub    = self.create_publisher(Dio, '/package/FPGAack', 1)
        self.sensor_set_sub     = self.create_subscription(SensorSet, '/sensorset', self.SensorSetFunction, 1000)   
        self.sensor_set_sub
        self.headmotor_sub      = self.create_subscription(HeadPackage, '/package/HeadMotor', self.HeadMotorFunction, 1000)
        self.headmotor_sub
        self.InterfaceSend2Sector = self.create_subscription(InterfaceSend2Sector, '/package/InterfaceSend2Sector', self.InterfaceSend2SectorFunction, 1000)
        self.InterfaceSend2Sector
        self.SectorSend2FPGA = self.create_subscription(Int16, '/package/Sector', self.SectorSendOpenCRFunction, 1000)
        self.SectorSend2FPGA
        self.InterfaceSaveData = self.create_subscription(SaveMotion, '/package/InterfaceSaveMotion', self.InterfaceSaveDataDataFunction, 1000)
        self.InterfaceSaveData
        ############################  location  ##############################
        self.location_subscription = self.create_subscription(
            Location,
            '/location',
            self.location_callback,
            10
        )
        self.location_subscription  # prevent unused variable warning
        ############################  location  ##############################
        self.InterfaceReadData_service = self.create_service(ReadMotion, '/package/InterfaceReadSaveMotion', self.InterfaceReadDataFunction)
        self.loadingwalkinggait_service = self.create_service(WalkingGaitParameter, '/web/LoadWalkingGaitParameter', self.LoadingWalkingGaitFunction)
        self.InterfaceCheckSectorFunction_service = self.create_service(CheckSector, '/package/InterfaceCheckSector', self.InterfaceCheckSectorFunction)
        self.InterfaceCallback_Publish = self.create_publisher(Bool, '/package/motioncallback', 1000)
        self.ExecuteCallBack_Publish = self.create_publisher(Bool,'/package/executecallback',1000)
        self.Continousback_sub = self.create_subscription(Bool, '/walkinggait/Continuousback', self.ContinousbackFunction, 1000)
        self.Continousback_sub 
        self.ChangeContinuousValue_sub = self.create_subscription(Interface, '/ChangeContinuousValue_Topic', self.ChangeContinuousValueFunction, 1000)
        self.ChangeContinuousValue_sub
        self.SaveWalkingGaitParameter = self.create_subscription(Parametermessage, '/web/parameter_Topic', self.SaveWalkingGaitFunction, 1000)
        self.SaveWalkingGaitParameter
        self.Gerente = self.create_subscription(Int16, '/ContinousMode_Topic', self.GerenteFunction, 1000)
        self.Gerente
        self.Send = self.create_subscription(Bool, '/Send_parameter', self.SendtoOpenCR, 1000)
        self.Send
        self.serial_init()
        self.start_imu_thread()
        self.motionsavedata = SaveMotionVector()
        self.savemotionvector = []

        self.serial_head = False
        self.serial_motor = False

        self.dio_tmpstatus = 0
        self.walkdata_receive = False
        self.robotislist = []
        # self.tool = Tool()
        self.SendSectorPackage = []
        self.packageMotorData = []

        self.checkSectorPackage = []
        self.SaveSectorPackage = []

        self.interface_ack = Bool()
        self.execut_ack = Bool()

        GPIO.setmode(GPIO.BOARD)
        # 2. 要讀的腳位清單
        self.pins = [18, 19, 21, 22]
        for p in self.pins:
            # Orin 上內部 pull-up/down 可能不生效，建議硬體拉定
            GPIO.setup(p, GPIO.IN)
        self.timer = self.create_timer(0.1, self.dio)
        self.prev_pin22_val = None


    ######################         walking      ###############################
    def location_callback(self, msg):
        self.location = msg.data
        print(f"Location: {self.location}")

    def LoadingWalkingGaitFunction(self, request, response):
        print(f"Mode: {request.mode}")
        if request.mode == 0:
            if self.back_falg:
                self.path = f"{self.location}/Continuous_Back.ini"
                print(f"Path: {self.path}")
            else:
                self.path = f"{self.location}/{'Continuous_Parameter.ini'}"
                print(f"Path: {self.path}")
            config = configparser.ConfigParser()
            config.read(self.path)
            general = config["General"]

            # 初始化儲存字典
            self.gait_params = {
                "com_y_swing":      float   (general["com_y_swing"]     ),
                "y_swing_range":    float   (general["Y_Swing_Range"]   ),
                "period_t":         int     (general["Period_T"]        ),
                "osc_lockrange":    float   (general["OSC_LockRange"]   ),
                "base_default_z":   float   (general["BASE_Default_Z"]  ),
                "now_stand_height": float   (general["now_stand_height"]),
                "now_com_height":   float   (general["now_com_height"]  ),
                "stand_balance":    bool    (general["Stand_Balance"]   )
            }

            # 使用字典自動設定回傳值
            for key, value in self.gait_params.items():
                setattr(response, key, value)
            
            print(f"Response: {response}")
            self.SendtoOpenCR(Bool(data=True))

        elif request.mode == 3:
            self.path = f"{self.location}/Single_Parameter.ini"
            config = configparser.ConfigParser()
            config.read(self.path)
            general = config["General"]
            # 讀取參數
            response.x_swing_range = float(general["X_Swing_Range"])
            response.y_swing_range = float(general["Y_Swing_Range"])
            response.z_swing_range = float(general["Z_Swing_Range"])
            response.period_t = int(general["Period_T"])
            response.period_t2 = int(general["Period_T2"])
            response.sample_time = int(general["Sample_Time"])
            response.osc_lockrange = float(general["OSC_LockRange"])
            response.base_default_z = float(general["BASE_Default_Z"])
            response.x_swing_com = float(general["X_Swing_COM"])
            response.base_lift_z = float(general["BASE_LIFT_Z"])
            response.rightfoot_shift_z = float(general["rightfoot_shift_z"])
            response.com_y_swing = float(general["com_y_swing"])
            response.now_stand_height = float(general["now_stand_height"])
            response.now_com_height = float(general["now_com_height"])
            response.stand_balance = bool(general["Stand_Balance"])
            print(f"Response: {response}")

        elif request.mode in [1, 2]:
            self.path = f"{self.location}/{'LCdown_Parameter.ini' if request.mode == 2 else 'LCstep_Parameter.ini'}"
            config = configparser.ConfigParser()
            config.read(self.path)
            general = config["General"]
            # 讀取參數
            response.x_swing_range = float(general["X_Swing_Range"])
            response.y_swing_range = float(general["Y_Swing_Range"])
            response.z_swing_range = float(general["Z_Swing_Range"])
            response.period_t = int(general["Period_T"])
            response.period_t2 = int(general["Period_T2"])
            response.sample_time = int(general["Sample_Time"])
            response.osc_lockrange = float(general["OSC_LockRange"])
            response.base_default_z = float(general["BASE_Default_Z"])
            response.x_swing_com = float(general["X_Swing_COM"])
            response.com_y_swing = float(general["Y_Swing_Shift"])
            response.base_lift_z = float(general["BASE_LIFT_Z"])
            response.com_y_swing = float(general["com_y_swing"])
            response.now_stand_height = float(general["now_stand_height"])
            response.now_com_height = float(general["now_com_height"])
            response.stand_balance = bool(general["Stand_Balance"])
            print(f"Response: {response}")

        return response

    def SaveWalkingGaitFunction(self, msg):
        print("SaveWalkingGaitFunction")
        print(f"Mode: {msg.mode}")
        if msg.mode == 0:
            if self.back_falg:
                self.path = f"{self.location}/{'Continuous_Back.ini'}"
                config = configparser.ConfigParser()
                config["General"] = {
                    "com_y_swing": msg.com_y_swing,
                    "Y_Swing_Range": msg.y_swing_range,
                    "Period_T": msg.period_t,
                    "OSC_LockRange": msg.osc_lockrange,
                    "BASE_Default_Z": msg.base_default_z,
                    "now_stand_height": msg.now_stand_height,
                    "now_com_height": msg.now_com_height,
                    "Stand_Balance": msg.stand_balance
                }
                with open(self.path, 'w') as f:
                    config.write(f)
            else:
                self.path = f"{self.location}/{'Continuous_Parameter.ini'}"
                config = configparser.ConfigParser()
                config["General"] = {
                    "com_y_swing": msg.com_y_swing,
                    "Y_Swing_Range": msg.y_swing_range,
                    "Period_T": msg.period_t,
                    "OSC_LockRange": msg.osc_lockrange,
                    "BASE_Default_Z": msg.base_default_z,
                    "now_stand_height": msg.now_stand_height,
                    "now_com_height": msg.now_com_height,
                    "Stand_Balance": msg.stand_balance
                }
                with open(self.path, 'w') as f:
                    config.write(f)
        elif msg.mode == 3:
            self.path = f"{self.location}/Single_Parameter.ini"
            config = configparser.ConfigParser()
            config["General"] = {
                "com_y_swing": msg.com_y_swing,
                "Y_Swing_Range": msg.y_swing_range,
                "Period_T": msg.period_t,
                "OSC_LockRange": msg.osc_lockrange,
                "BASE_Default_Z": msg.base_default_z,
                "now_stand_height": msg.now_stand_height,
                "now_com_height": msg.now_com_height,
                "Stand_Balance": msg.stand_balance
            }
            with open(self.path, 'w') as f:
                config.write(f)
        elif msg.mode in [1, 2]:
            self.path = f"{self.location}/{'LCdown_Parameter.ini' if msg.mode == 2 else 'LCstep_Parameter.ini'}"
            config = configparser.ConfigParser()
            config["General"] = {
                "X_Swing_Range": msg.x_swing_range,
                "Y_Swing_Range": msg.y_swing_range,
                "Z_Swing_Range": msg.z_swing_range,
                "Period_T": msg.period_t,
                "Period_T2": msg.period_t2,
                "Sample_Time": msg.sample_time,
                "OSC_LockRange": msg.osc_lockrange,
                "BASE_Default_Z": msg.base_default_z,
                "X_Swing_COM": msg.x_swong_com,
                "Y_Swing_Shift": msg.y_swing_shift,
                "BASE_LIFT_Z": msg.base_lift_z,
                "com_y_swing": msg.com_y_swing,
                "now_stand_height": msg.now_stand_height,
                "now_com_height": msg.now_com_height,
                "Stand_Balance": msg.stand_balance
            }
            with open(self.path, 'w') as f:
                config.write(f)
   
    def ChangeContinuousValueFunction(self, msg):
        print("ChangeContinuousValueFunction")
        # 注意这里是 B3fB，不是 8：
        packet = struct.pack('<B3fB', 0x47, msg.x, msg.y, msg.theta, 0x45)
        print(packet)

        # 丢掉旧 ACK
        self.serial_walk.reset_input_buffer()
        # 发送并 flush
        self.serial_walk.write(packet)
        self.serial_walk.flush()
        line = self.serial_walk.readline().decode('utf-8', errors='ignore').strip()
        print(f"ACK raw: {line}")

    #################################################################
    def ContinousbackFunction(self, msg):
        print("Continuousback")
        self.back_falg = msg.data
    #################################################################

    def GerenteFunction(self, msg):
        print(f"Gerente: {msg.data}")
        packet = bytes([0x49, msg.data & 0xFF, 0x45])

        # 丢掉上一次残留
        self.serial_walk.reset_input_buffer()

        # 写包并 flush
        self.serial_walk.write(packet)
        self.serial_walk.flush()
        print(f"Packet Length: {len(packet)}")
        print(f"Packet: {packet}")

        line = self.serial_walk.readline().decode("utf-8").strip()
        print(f"ACK raw: {line}")

    def SendtoOpenCR(self, msg):
        if not self.gait_params:
            print("尚未載入 gait_params，請先呼叫 LoadingWalkingGaitFunction")
            return
        if not msg.data:
            return

        print("SendtoOpenCR")

        # 組 packet
        p = self.gait_params
        packet = struct.pack(
            '<B7f?B',
            0x48,
            p["com_y_swing"],
            p["y_swing_range"],
            float(p["period_t"]),
            p["osc_lockrange"],
            p["base_default_z"],
            p["now_stand_height"],
            p["now_com_height"],
            p["stand_balance"],
            0x45
        )

        # 1) 丟掉殘留
        self.serial_walk.reset_input_buffer()

        # 2) 寫入並 flush
        self.serial_walk.write(packet)
        self.serial_walk.flush()
        print(f"Packet Length: {len(packet)}")
        print(f"Packet: {packet}")

        # 3) 讀 ACK
        line = self.serial_walk.readline().decode("utf-8").strip()
        print(f"ACK raw: {line}")

    def RobotisListinit(self):
        self.robotislist.clear()
        for i in range(1, 3):
            motor = Motor(ID=i, position=2048, speed=511)
            self.robotislist.append(motor)

    def HeadMotorFunction(self, msg):
        HeadPackage = [0]*7
        motor = self.robotislist[msg.id - 1]
        updated_motor = Motor(ID=motor.ID, position=msg.position, speed=msg.speed)
        self.robotislist[msg.id - 1] = updated_motor
        print(self.robotislist)
        # for i in range(2):
        HeadPackage[0] = 1 #表示動絕對刻度
        for i in range(2):
            HeadPackage[i * 3 + 1] = self.robotislist[i].ID
            HeadPackage[i * 3 + 2] = self.robotislist[i].position
            HeadPackage[i * 3 + 3] = self.robotislist[i].speed
        data_str = ','.join(map(str, HeadPackage))  # 轉成 "1,1430,602,2,2048,511"
        self.serial_head.write(data_str.encode('utf-8'))  # 發送 UTF-8 編碼的字串
        self.serial_head.write(b'\n')  # 可選擇加換行符號
        
        print(len(HeadPackage))

    def serial_init(self):
        self.port_imu  = '/dev/ttyTHS1'
        self.port_walk = '/dev/opencr' #'/dev/ttyACM0'
        self.baudrate  = 115200

        # # IMU
        try:
            self.get_logger().info(f"Opening IMU port: {self.port_imu}")
            self.serial_imu = serial.Serial(self.port_imu, self.baudrate, timeout=1)
            time.sleep(2)
            if self.serial_imu.is_open:
                self.get_logger().info(f"[OK] IMU open on {self.port_imu}")
            else:
                self.get_logger().error("[FAIL] IMU port not open!")
                self.serial_imu = None
        except serial.SerialException as e:
            self.get_logger().error(f"[Serial ERROR] IMU port error: {e}")
            self.serial_imu = None

        # Walk
        try:
            self.get_logger().info(f"Opening WALK port: {self.port_walk}")
            self.serial_walk = serial.Serial(self.port_walk, self.baudrate, timeout=1)
            time.sleep(1)
            if self.serial_walk.is_open:
                self.get_logger().info(f"[OK] WALK open on {self.port_walk}")
            else:
                self.get_logger().error("[FAIL] WALK port not open!")
                self.serial_walk = None
        except serial.SerialException as e:
            self.get_logger().error(f"[Serial ERROR] Walk port error: {e}")
            self.serial_walk = None

    def start_imu_thread(self):
        self.imu_thread = threading.Thread(target=self.imu_port, daemon=True)
        self.imu_thread.start()

    def imu_port(self):
        print("[INFO] imu_port 啟動")
        msg = SensorPackage()
        self.serial_imu.flushInput()  # 清空殘留的資料避免卡住

        while True:
            try:
                if self.serial_imu is None:
                    print("[ERROR] serial_imu 未初始化，無法讀取 IMU 資料")
                    time.sleep(1)
                    continue

                line = self.serial_imu.readline()
                if not line:
                    continue  # timeout 無資料，跳過迴圈

                try:
                    line = line.decode("utf-8").strip()
                except UnicodeDecodeError:
                    print("[解析失敗] 解碼錯誤（非 UTF-8）")
                    continue

                if line.startswith("#YPR="):
                    ypr_data = line[5:]
                    parts = ypr_data.split(',')
                    if len(parts) != 3:
                        print(f"[格式錯誤] 欄位數錯誤，收到: {parts}")
                        continue

                    try:
                        msg.yaw = float(parts[0])
                        msg.pitch = float(parts[1])
                        msg.roll = float(parts[2])
                        self.sensor_data_pub.publish(msg)
                    except ValueError:
                        print(f"[解析失敗] 轉換失敗: {parts}")
                else:
                    print(f"[非預期格式] 收到: {line}")

            except serial.SerialException as e:
                print(f"[Serial 錯誤] 無法讀取 IMU 資料: {e}")
                time.sleep(1)  # 延遲避免瘋狂報錯

    def head_port(self):
        pass

    def SensorSetFunction(self, msg):
        print("SensorSetFunction")
        print("reset:", msg.reset)
        # msg = SensorSet()
        if msg.reset:
            print("Reset")
            try:
                self.serial_imu.write(bytes([0]))   # 傳送數值 0 的 byte
            except Exception as e:
                print(f"Error: {e}")
            # line = self.serial_imu.readline().decode("utf-8").strip()
            # if line.startswith("#YPR="):
            #     print(line)
            
    def packageinit(self):
        self.parameterpackage = [0]*31
        self.parameterpackage[0]    = 0x53
        self.parameterpackage[1]    = 0x54
        self.parameterpackage[2]    = 0xF5
        self.parameterpackage[5]    = 6
        self.parameterpackage[30]   = 0x45
        self.motorpackage = [0]*19
        self.motorpackage[0]    = 0x53
        self.motorpackage[1]    = 0x54
        self.motorpackage[2]    = 0xF5
        self.motorpackage[3]    = 1
        self.motorpackage[5]    = 3
        self.motorpackage[18]   = 0x45

    def standini(self):
        print("Standini")
        path = os.path.join("/home/iclab/Standmotion/sector/29.ini")
        with open(path, 'r') as fin:
            try:
                
                for line in fin:
                    if "PackageCnt" in line:
                        self.packagecnt = int(line.split('=')[1])
                        print(f"PackageCnt: {self.packagecnt}")
                    elif "Package" in line:
                        values = line.split('=')[1].split('||')
                        # 去掉每个值的空格并过滤掉空字符串，然后将其转换为整数
                        values = [int(value.strip()) for value in values if value.strip().isdigit()]
                        print(f"Package: {values}")
                        print(type(values))  # 确认 values 的类型是列表，并且里面的元素是整数
                        break  # 找到后就可以跳出循环（如果只需要第一个匹配）

                self.packageMotorData[0:3] = [0x53, 0x54, 0xF2]
                self.packageMotorData[3:3 + len(values)-1] = values[1:]
                print(f"packageMotorData: {self.packageMotorData}")
                self.serial_motor.write(self.packageMotorData)
            except Exception as e:
                print(f"Error: {e}")
        print("End_LoadSector")
        self.SendSectorPackage.clear()

    def dio(self):
        msg = Dio()
        raw = [GPIO.input(p) for p in self.pins]
        # 前三個腳 (18,19,21) 組成 3-bit mask
        group_mask = (raw[0] << 0) | (raw[1] << 1) | (raw[2] << 2)
        # pin22 單獨存取
        msg.data  = raw[3]

        # # publish group_mask
        # msg_group = Int16()
        # msg_group.data = group_mask
        # self.Dio_pub.publish(msg_group)
        # self.get_logger().info(f'DIO group mask: 0b{group_mask:03b} ({group_mask})')
        # 只有 pin22_val 改變時才 publish
        # if pin22_val != self.prev_pin22_val:
        #     msg.data = pin22_val
        #     # self.pin22_pub.publish(msg)
        #     self.get_logger().info(f'Pin22 changed → {msg.data}')
        #     self.prev_pin22_val = pin22_val
        self.Dio_pub.publish(msg)

    def destroy_node(self):
        # 結束前清理 GPIO
        GPIO.cleanup()
        super().destroy_node()

#########################
# Save motion functions #
#########################
    def InterfaceSaveDataDataFunction(self, msg):
        # 1) 每收到一筆就 append
        self.savemotionvector.append(msg)
        self.get_logger().debug(f"Appended msg: state={msg.motionstate}, id={msg.id}")

        # 2) 不是最後那條含 saveflag 的，就先回
        if not msg.saveflag:
            return

        # 3) 準備路徑
        if msg.savestate == 0:
            base_dir = os.path.join(
                "/home/iclab/Desktop/humanoid/wula",
                self.location
                # "Parameter"
            )
        else:
            base_dir = "/home/iclab/Desktop/Standmotion"
        fname = msg.name
        if not fname.lower().endswith(".toml"):
            fname += ".toml"
        os.makedirs(base_dir, exist_ok=True)
        path = os.path.join(base_dir, fname)

        # 4) 去除重複 (ID, State)
        seen    = set()
        records = []
        for motion in self.savemotionvector[:-1]:
            key = (motion.id, motion.motionstate)
            if key in seen:
                continue
            seen.add(key)

            # 根據 state 決定 M 欄位
            if motion.motionstate == 0:
                m_vals = list(motion.motionlist)
            else:
                m_vals = list(motion.motordata)

            records.append({
                "ID":    motion.id,
                "State": motion.motionstate,
                "M":     m_vals
            })

        # 5) VectorCnt 改成實際的 record 數量
        data = {
            "VectorCnt": len(records),
            "records":   records
        }

        # 6) 寫入 TOML
        try:
            with open(path, 'w') as fout:
                toml.dump(data, fout)
            self.get_logger().info(f"Saved deduped motion to {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save TOML: {e}")

        # 7) 清空暫存
        self.savemotionvector.clear()

#########################
# Save motion functions #
#########################

#########################
# Read motion functions #
#########################

    def InterfaceReadDataFunction(self, request, response):
        if request.readstate == 1:
            base_dir = "/home/iclab/Desktop/Standmotion"
            fname    = request.name
            if not fname.lower().endswith(".toml"):
                fname += ".toml"
            file_path = os.path.join(base_dir, fname)

            # 如果檔案找不到，僅 warn 並回傳空 response
            if not os.path.isfile(file_path):
                response.readcheck = False
                self.get_logger().warn(f"No such motion file: {file_path}")
                return response

            # load toml 並 catch 可能的解析錯誤
            try:
                data = toml.load(file_path)
            except Exception as e:
                response.readcheck = False
                self.get_logger().error(f"Failed to parse TOML '{file_path}': {e}")
                return response

            # 填 response
            response.readcheck = True
            print(response.readcheck)
            response.vectorcnt = data.get('VectorCnt', 0)
            states            = []
            ids               = []
            relativedata_list = []
            absolutedata_list = []
            motion_list       = []

            for rec in data.get('records', []):
                state    = rec.get('State')
                id_      = rec.get('ID')
                m_values = rec.get('M', [])
                states.append(state)
                ids.append(id_)
                if state in (1, 2):
                    relativedata_list.append(m_values)
                elif state in (3, 4):
                    absolutedata_list.append(m_values)
                elif state == 0:
                    motion_list.append(m_values)

            response.motionstate   = states
            response.id            = ids
            response.relativedata  = [
                v for sub in relativedata_list for v in sub
                # if -32768 <= v <= 32767
            ]
            response.absolutedata  = [
                v for sub in absolutedata_list for v in sub
                # if -32768 <= v <= 32767
            ]
            response.motionlist    = [
                v for sub in motion_list for v in sub
                # if -32768 <= v <= 32767
            ]
            self.get_logger().info(f"Loaded motion '{fname}' successfully")
        return response

#########################
# Read motion functions #
#########################

############################
# Execute motion functions #
############################

    def InterfaceCheckSectorFunction(self, request, response):
        # 1) 清 buffer
        self.checkSectorPackage.clear()
        sector = request.data
        self.get_logger().info(f"InterfaceCheckSectorFunction: sector={sector}")

        # 2) 組檔案路徑，副檔名一律用 .toml
        if sector == 29:
            path = "/home/iclab/Desktop/Standmotion/sector/29.toml"
        else:
            path = os.path.join(
                "/home/iclab/Desktop/humanoid/src/wula",
                self.location,              # 你原本的路徑變數
                "Parameter/sector",
                f"{sector}.toml"
            )
        # 3) 試著用 toml.load 讀檔
        try:
            data = toml.load(path)
        except Exception as e:
            self.get_logger().error(f"Failed to load TOML '{path}': {e}")
            response.checkflag = False
            return response

        # 4) 取出兩個欄位
        packagecnt = data.get("PackageCnt")
        package    = data.get("Package", [])

        # 5) 基本驗證：沒有欄位或型別錯誤，就失敗
        if not isinstance(packagecnt, int) or not isinstance(package, list):
            self.get_logger().warn("TOML 格式錯誤：PackageCnt 或 Package 型別不對")
            response.checkflag = False
            return response

        # 6) 任何負值都不合格
        # if any(v < 0 for v in package):
        #     self.get_logger().warn("Package 含負值，視為錯誤")
        #     response.checkflag = False
        #     return response

        # 7) 檢查長度是否一致
        if packagecnt != len(package):
            self.get_logger().warn(f"PackageCnt 不符：{packagecnt} vs len={len(package)}")
            response.checkflag = False
            return response

        # 8) 檢查第一個指令碼必須是 241/242/243 才算合法
        if package and package[0] in (241, 242, 243):
            self.checkSectorPackage = package
            response.checkflag = True
            self.get_logger().info(f"Sector {sector} check OK")
        else:
            self.get_logger().warn(f"第一個指令碼非法：{package[0] if package else None}")
            response.checkflag = False

        return response
############################
# Execute motion functions #
############################

###################################
# Interface Send sector functions #
###################################
    def InterfaceSend2SectorFunction(self, msg):
        # 1) 累積每個 byte
        self.SaveSectorPackage.append(msg.package)
        self.get_logger().debug(f"Got byte: 0x{msg.package:02X}")

        # 2) 還沒收到完整的頭 (0x53,0x54) + 尾 (0x4E,0x45) 就等
        N = len(self.SaveSectorPackage)
        if (
            N < 4
            or self.SaveSectorPackage[0] != 0x53
            or self.SaveSectorPackage[1] != 0x54
            or self.SaveSectorPackage[-2] != 0x4E
            or self.SaveSectorPackage[-1] != 0x45
        ):
            return

        # 3) 切出真正要存的那段
        package_list = self.SaveSectorPackage[2:-1]

        # 4) 計算長度
        pkg_cnt = len(package_list)

        # 5) 準備要 dump 的 dict
        data = {
            "PackageCnt": pkg_cnt,
            "Package":    package_list,
        }

        # —— 新增這一段：32-bit big-endian 拆成 4 byte，speed + pos 各 4 byte —— #
        package32 = []
        # 跳過第一個 opcode，從 speed/pos 序列裡每兩個數對應一顆馬達
        payload = package_list[1:]
        for speed, pos in zip(payload[::2], payload[1::2]):
            # speed → 4 byte big-endian signed
            package32.extend(speed.to_bytes(4, byteorder='big', signed=True))
            # pos   → 4 byte big-endian signed
            package32.extend(pos.to_bytes(4, byteorder='big', signed=True))

        data["Package32"] = package32
        # ——————————————————————————————————————————————— #

        # 6) 寫 TOML
        sector = msg.sectorname
        if sector == "29":
            base_dir = "/home/iclab/Desktop/Standmotion/sector"
        else:
            base_dir = os.path.join(
                "/home/iclab/Desktop/humanoid/src/wula",
                self.location,
                "Parameter/sector"
            )
        os.makedirs(base_dir, exist_ok=True)
        path = os.path.join(base_dir, f"{sector}.toml")

        try:
            with open(path, 'w') as fout:
                toml.dump(data, fout)
            self.get_logger().info(f"Wrote TOML → {path}")
            self.interface_ack.data = True
        except Exception as e:
            self.get_logger().error(f"Failed to write TOML: {e}")
            self.interface_ack.data = False

        self.InterfaceCallback_Publish.publish(self.interface_ack)
        self.SaveSectorPackage.clear()
###################################
# Interface Send sector functions #
###################################

                
############################ 
# execute sector functions #
############################

    def SectorSendOpenCRFunction(self, msg):
        sector = msg.data
        self.get_logger().info(f"[OpenCR] SectorSend2OpenCR: sector={sector}")

        # 1) TOML 路径
        if sector == 29:
            path = "/home/iclab/Desktop/Standmotion/sector/29.toml"
        else:
            path = os.path.join(
                "/home/iclab/Desktop/humanoid/src/wula",
                self.location,
                "Parameter/sector",
                f"{sector}.toml"
            )

        # 2) 读 TOML
        try:
            data = toml.load(path)
        except Exception as e:
            self.get_logger().error(f"Cannot load TOML '{path}': {e}")
            return

        pkg16 = data.get("Package", [])
        if not isinstance(pkg16, list) or len(pkg16) < 3:
            self.get_logger().error("Missing Package or too short")
            return

        # 3) 丢掉 opcode(0) 和 尾标78
        payload_vals = pkg16[1:-1]

        # 4) 組封包
        buf = bytearray([242])
        for val in payload_vals:
            v = val & 0xFFFFFFFF
            buf.extend((v & 0xFFFF).to_bytes(2, 'little'))
            buf.extend(((v >> 16) & 0xFFFF).to_bytes(2, 'little'))
        # buf.append(0x45)

        try:
            # 1) 確保我們在正確的 port（假設 serial_imu 連的是 OpenCR）
            ser = self.serial_walk

            # 2) 丟掉舊資料、設定 timeout
            ser.reset_input_buffer()
            ser.timeout = 0.1  # readline 最多等 100ms

            # 3) 送封包並 flush
            written = ser.write(buf)
            ser.flush()
            self.get_logger().info(f"[OpenCR] Sent {written} bytes: {list(buf)}")

            # 4)
            # line = self.serial_walk.readline().decode('utf-8', errors='ignore').strip()
            # print(f"ACK raw: {line}")
            acks = []
            while True:
                line = ser.readline().decode().strip()
                if not line:
                    break
                acks.append(line)
            print("All ACKs:", acks)

            # 5) 根據 ACK 內容設定 execut_ack
            self.execut_ack.data = True

        except Exception as e:
            self.get_logger().error(f"[OpenCR] Serial error: {e}")
            self.execut_ack.data = False

        # 6) Publish 結果
        self.ExecuteCallBack_Publish.publish(self.execut_ack)
        self.get_logger().info(f"{pkg16[0]} Execute is finish! ACK={self.execut_ack.data}")


############################
# execute sector functions #
############################
                
def main():
    rclpy.init()  # Initialize rclpy
    motion = Motionpackage()
    # motion.RobotisListinit()
    # motion.packageinit()
    # motion.standini()

    rclpy.spin(motion)  # Spin the node so it doesn't exit

if __name__ == "__main__":
    main()