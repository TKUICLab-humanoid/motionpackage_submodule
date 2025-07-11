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
import gpiod
import shutil



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
        # ############################  location  ##############################
        self.declare_parameter('location', 'ar')
        loc = self.get_parameter('location').value
        self.location_callback(loc)
        # self.location_subscription = self.create_subscription(
        #     Location,
        #     '/location',
        #     self.location_callback,
        #     10
        # )
        # self.location_subscription  # prevent unused variable warning
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
        self.standini()
        self.motionsavedata = SaveMotionVector()
        self.savemotionvector = []
        self.serial_head = False
        self.serial_motor = False
        self.dio_tmpstatus = 0
        self.walkdata_receive = False
        self.robotislist = []
        self.SendSectorPackage = []
        self.packageMotorData = []
        self.checkSectorPackage = []
        self.SaveSectorPackage = []
        self.interface_ack = Bool()
        self.execut_ack = Bool()
        GPIO.setmode(GPIO.BOARD)
        # # 2. 要讀的腳位清單
        self.pins = [29, 31, 32, 33]
        self.exponents = [1, 2, 3, 4]
        self.prev_code = None
        for p in self.pins:
            GPIO.setup(p, GPIO.IN)
        self.timer = self.create_timer(0.1, self.dio)
        self.dio_strategy = False
        self.pre_dio_strategy = False
        self.prev_pin22_val = None
        self.lines = []


    ######################         walking      ###############################
    def location_callback(self, loc):
        self.location = f"/home/iclab/Desktop/towen/src/strategy/strategy/{loc}/Parameter"
        self.get_logger().info(f"Location :{self.location}")

    def LoadingWalkingGaitFunction(self, request, response):
        if request.mode == 0:
            if self.back_falg:
                self.path = f"{self.location}/Continuous_Back.ini"
            else:
                self.path = f"{self.location}/{'Continuous_Parameter.ini'}"
            config = configparser.ConfigParser()
            config.read(self.path)
            general = config["General"]
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
            for key, value in self.gait_params.items():
                setattr(response, key, value)
            self.SendtoOpenCR(Bool(data=True))
        elif request.mode == 3:
            self.path = f"{self.location}/Single_Parameter.ini"
            config = configparser.ConfigParser()
            config.read(self.path)
            general = config["General"]
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

        elif request.mode in [1, 2]:
            self.path = f"{self.location}/{'LCdown_Parameter.ini' if request.mode == 2 else 'LCstep_Parameter.ini'}"
            config = configparser.ConfigParser()
            config.read(self.path)
            general = config["General"]
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
        return response
    def SaveWalkingGaitFunction(self, msg):
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
        packet = struct.pack('<B3fB', 0x47, msg.x, msg.y, msg.theta, 0x45)
        self.serial_walk.reset_input_buffer()
        self.serial_walk.write(packet)
        self.serial_walk.flush()
        line = self.serial_walk.readline().decode('utf-8', errors='ignore').strip()

    #################################################################
    def ContinousbackFunction(self, msg):
        self.back_falg = msg.data
    #################################################################

    def GerenteFunction(self, msg):
        packet = bytes([0x49, msg.data & 0xFF, 0x45])
        self.serial_walk.reset_input_buffer()
        self.serial_walk.write(packet)
        self.serial_walk.flush()
        line = self.serial_walk.readline().decode("utf-8").strip()

    def SendtoOpenCR(self, msg):
        if not self.gait_params:
            return
        if not msg.data:
            return
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
        self.serial_walk.reset_input_buffer()
        self.serial_walk.write(packet)
        self.get_logger().info(f"Walking_Package : {packet}")
        self.serial_walk.flush()
        line = self.serial_walk.readline().decode("utf-8").strip()
        self.get_logger().info(f"Walking_ack : {line}")

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
        HeadPackage[0] = 246 #表示動頭部馬達
        for i in range(2):
            HeadPackage[i * 3 + 1] = self.robotislist[i].ID
            HeadPackage[i * 3 + 2] = self.robotislist[i].speed
            HeadPackage[i * 3 + 3] = self.robotislist[i].position
        # data_str = ','.join(map(str, HeadPackage))  # 轉成 "1,1430,602,2,2048,511"
        # self.serial_head.write(data_str.encode('utf-8'))  # 發送 UTF-8 編碼的字串
        # self.serial_head.write(b'\n')  # 可選擇加換行符號
        self.get_logger().debug(f"ahjdhasjkdhjak{HeadPackage}")

    def serial_init(self):
        self.port_imu  = '/dev/ttyTHS1'
        self.port_walk = '/dev/ttyACM0' #'/dev/ttyACM0'
        self.baudrate  = 115200

        # # IMU
        try:
            self.get_logger().debug(f"Opening IMU port: {self.port_imu}")
            self.serial_imu = serial.Serial(self.port_imu, self.baudrate, timeout=1)
            time.sleep(2)
            if self.serial_imu.is_open:
                self.get_logger().debug(f"[OK] IMU open on {self.port_imu}")
            else:
                self.get_logger().error("[FAIL] IMU port not open!")
                self.serial_imu = None
        except serial.SerialException as e:
            self.get_logger().error(f"[Serial ERROR] IMU port error: {e}")
            self.serial_imu = None

        # Walk
        try:
            self.get_logger().debug(f"Opening WALK port: {self.port_walk}")
            self.serial_walk = serial.Serial(self.port_walk, self.baudrate, timeout=0.1,rtscts=True)
            time.sleep(1)
            if self.serial_walk.is_open:
                self.get_logger().debug(f"[OK] WALK open on {self.port_walk}")
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
        msg = SensorPackage()
        self.serial_imu.flushInput()  # 清空殘留的資料避免卡住

        while True:
            try:
                if self.serial_imu is None:
                    time.sleep(1)
                    continue

                line = self.serial_imu.readline()
                if not line:
                    continue  # timeout 無資料，跳過迴圈

                try:
                    line = line.decode("utf-8").strip()
                except UnicodeDecodeError:
                    continue

                if line.startswith("#YPR="):
                    ypr_data = line[5:]
                    parts = ypr_data.split(',')
                    if len(parts) != 3:
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
                time.sleep(1)  # 延遲避免瘋狂報錯

    def head_port(self):
        pass

    def SensorSetFunction(self, msg):
        if msg.reset:
            try:
                self.serial_imu.write(bytes([0]))   # 傳送數值 0 的 byte
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
            # line = self.serial_imu.readline().decode("utf-8").strip()
            # if line.startswith("#YPR="):
            
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
        self.get_logger().info(f"Standini")
        path = "/home/iclab/Desktop/Standmotion/sector/29.toml"
        backup_path = os.path.join(
            os.path.dirname(path),
            "now_motion.toml"
        )
        try:
            shutil.copyfile(path, backup_path)
            self.get_logger().debug(f"Backed up TOML as {backup_path}")
        except Exception as e:
            self.get_logger().error(f"Backup failed: {e}")
        try:
            data = toml.load(path)
        except Exception as e:
            self.get_logger().error(f"Cannot load TOML '{path}': {e}")
            return

        pkg16 = data.get("Package", [])
        if not isinstance(pkg16, list) or len(pkg16) < 3:
            self.get_logger().error("Missing Package or too short")
            return
        payload_vals = pkg16[1:-1]
        buf = bytearray([242])
        for val in payload_vals:
            v = val & 0xFFFFFFFF
            buf.extend((v & 0xFFFF).to_bytes(2, 'little'))
            buf.extend(((v >> 16) & 0xFFFF).to_bytes(2, 'little'))
        try:
            ser = self.serial_walk
            ser.reset_input_buffer()
            ser.timeout = 0.1
            written = ser.write(buf)
            ser.flush()
            self.get_logger().debug(f"[OpenCR] Sent {written} bytes: {list(buf)}")
            acks = []
            while True:
                line = ser.readline().decode().strip()
                if not line:
                    break
                acks.append(line)
            self.get_logger().info(f"Standini_walk_ack : {acks}")
        except EnvironmentError:
            pass
    # def dio(self):
    #         msg = Dio()
    #         raw = [GPIO.input(p) for p in self.pins]
    #         self.get_logger().info(f"all pins status : {raw}")
    #         self.Dio_pub.publish(msg)
    # def destroy_node(self):
    #     # 結束前清理 GPIO
    #     GPIO.cleanup()
    #     super().destroy_node()

    def dio(self):
        raw = [GPIO.input(p) for p in self.pins]
        self.dio_strategy = bool(raw[0])
        code = 0
        for state, exp in zip(raw, self.exponents):
            if state:
                code += (1 << exp)
        # 3) 只在有變化時才印 & 發佈
        if code != self.prev_code:
            msg = Dio()
            msg.data = code        # 確認你的 Dio.msg 有 data 欄位
            msg.strategy = self.dio_strategy
            hexstr = f"0x{code:02X}"
            self.get_logger().info(f"raw={raw} → encoded={hexstr}")
            self.Dio_pub.publish(msg)
            # 更新 prev_code
            self.prev_code = code
            self.pre_dio_strategy = self.dio_strategy
    
    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()
#########################
# Save motion functions #
#########################
    def InterfaceSaveDataDataFunction(self, msg):
        self.get_logger().debug(f"savestate {msg.saveflag}") 
        self.savemotionvector.append(msg)
        self.get_logger().debug(f"Appended msg: state={msg.motionstate}, id={msg.id}")
        if not msg.saveflag:
            return
        if msg.savestate == 0:
            base_dir = os.path.join(
                self.location
            )
        else:
            base_dir = "/home/iclab/Desktop/Standmotion"
        fname = msg.name
        if not fname.lower().endswith(".toml"):
            fname += ".toml"
        os.makedirs(base_dir, exist_ok=True)
        path = os.path.join(base_dir, fname)
        seen    = set()
        records = []
        for motion in self.savemotionvector[:-1]:
            key = (motion.id, motion.motionstate)
            if key in seen:
                continue
            seen.add(key)
            if motion.motionstate == 0:
                m_vals = list(motion.motionlist)
            else:
                m_vals = list(motion.motordata)
            records.append({
                "ID":    motion.id,
                "State": motion.motionstate,
                "M":     m_vals
            })
        data = {
            "VectorCnt": len(records),
            "records":   records
        }
        try:
            with open(path, 'w') as fout:
                toml.dump(data, fout)
            self.get_logger().debug(f"Saved deduped motion to {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save TOML: {e}")
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
        elif request.readstate ==0:
            base_dir = os.path.join(
                self.location
            )
        fname    = request.name
        if not fname.lower().endswith(".toml"):
            fname += ".toml"
        file_path = os.path.join(base_dir, fname)
        if not os.path.isfile(file_path):
            response.readcheck = False
            self.get_logger().warn(f"No such motion file: {file_path}")
            return response
        try:
            data = toml.load(file_path)
        except Exception as e:
            response.readcheck = False
            self.get_logger().error(f"Failed to parse TOML '{file_path}': {e}")
            return response
        response.readcheck = True
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
        self.get_logger().debug(f"Loaded motion '{fname}' successfully")
        return response
#########################
# Read motion functions #
#########################

############################
# Execute motion functions #
############################
    def InterfaceCheckSectorFunction(self, request, response):
        self.checkSectorPackage.clear()
        sector = request.data
        self.get_logger().debug(f"InterfaceCheckSectorFunction: sector={sector}")
        if sector == 29:
            path = "/home/iclab/Desktop/Standmotion/sector/29.toml"
        else:
            path = os.path.join(
                self.location,             
                "sector",
                f"{sector}.toml"
            )
        try:
            data = toml.load(path)
        except Exception as e:
            self.get_logger().error(f"Failed to load TOML '{path}': {e}")
            response.checkflag = False
            return response
        packagecnt = data.get("PackageCnt")
        package    = data.get("Package", [])
        if not isinstance(packagecnt, int) or not isinstance(package, list):
            self.get_logger().warn("TOML 格式錯誤：PackageCnt 或 Package 型別不對")
            response.checkflag = False
            return response
        if packagecnt != len(package):
            self.get_logger().warn(f"PackageCnt 不符：{packagecnt} vs len={len(package)}")
            response.checkflag = False
            return response
        if package and package[0] in (241, 242, 243, 244,245):
            self.checkSectorPackage = package
            response.checkflag = True
            self.get_logger().debug(f"Sector {sector} check OK")
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
        self.SaveSectorPackage.append(msg.package)
        N = len(self.SaveSectorPackage)
        if (
            N < 4
            or self.SaveSectorPackage[0] != 0x53
            or self.SaveSectorPackage[1] != 0x54
            or self.SaveSectorPackage[-2] != 0x4E
            or self.SaveSectorPackage[-1] != 0x45
        ):
            return
        package_list = self.SaveSectorPackage[2:-1]
        pkg_cnt = len(package_list)
        data = {
            "PackageCnt": pkg_cnt,
            "Package":    package_list,
        }
        package32 = []
        payload = package_list[1:]
        for speed, pos in zip(payload[::2], payload[1::2]):
            package32.extend(speed.to_bytes(4, byteorder='big', signed=True))
            package32.extend(pos.to_bytes(4, byteorder='big', signed=True))
        data["Package32"] = package32
        opcode = package_list[0]
        if self.SaveSectorPackage[2] == 243 :
            self.get_logger().debug(f"package = {self.SaveSectorPackage[2]}")
            sector_name = msg.sectorname
            if sector_name == "29":
                base_dir = "/home/iclab/Desktop/Standmotion/sector"
            else:
                base_dir = os.path.join(self.location, "sector")
            os.makedirs(base_dir, exist_ok=True)

            path = os.path.join(base_dir, f"{sector_name}.toml")
            try:
                with open(path, 'w') as fout:
                    toml.dump(data, fout)
                self.get_logger().debug(f"Wrote TOML → {path}")
                self.interface_ack.data = True
            except Exception as e:
                self.get_logger().error(f"Failed to write TOML: {e}")
                self.interface_ack.data = False
        elif self.SaveSectorPackage[2] == 245:
            package_list[0] = 243
            sector_name = msg.sectorname
            self.delay = msg.delay
            self.cnt = msg.cnt
            if sector_name == "29":
                base_dir = "/home/iclab/Desktop/Standmotion/sector"
            else:
                base_dir = os.path.join(self.location, "sector")
            os.makedirs(base_dir, exist_ok=True)
            path = os.path.join(base_dir, f"{sector_name}_id.toml")
            try:
                with open(path, 'w') as fout:
                    toml.dump(data, fout)
                self.get_logger().debug(f"Wrote 245 TOML → {path}")
                self.interface_ack.data = True
            except Exception as e:
                self.get_logger().error(f"Failed to write TOML: {e}")
                self.interface_ack.data = False
        elif self.SaveSectorPackage[2] == 244:
            pkg_cnt = len(package_list)
            data = {
                "PackageCnt": pkg_cnt,
                "Package":    package_list,
            }
            sector_name = msg.sectorname
            filename = f"{sector_name}.toml"
            base_dir = os.path.join(self.location, "sector")
            os.makedirs(base_dir, exist_ok=True)
            path = os.path.join(base_dir, filename)
            try:
                with open(path, 'w') as fout:
                    toml.dump(data, fout)
                self.get_logger().debug(f"Wrote 244 TOML → {path}")
                self.interface_ack.data = True
            except Exception as e:
                self.get_logger().error(f"Failed to write 244 TOML: {e}")
                self.interface_ack.data = False
        elif self.SaveSectorPackage[2] == 242 :
            self.get_logger().debug(f"package = {self.SaveSectorPackage[2]}")
            sector_name = msg.sectorname
            if sector_name == "29":
                base_dir = "/home/iclab/Desktop/Standmotion/sector"
            else:
                base_dir = os.path.join(self.location, "sector")
            os.makedirs(base_dir, exist_ok=True)
            path = os.path.join(base_dir, f"{sector_name}.toml")
            try:
                with open(path, 'w') as fout:
                    toml.dump(data, fout)
                self.get_logger().debug(f"Wrote TOML → {path}")
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
        self.get_logger().debug(f"[OpenCR] SectorSend2OpenCR: sector={sector}")
        if sector == 29:
            path = "/home/iclab/Desktop/Standmotion/sector/29.toml"
        else:
            path = os.path.join(self.location, "sector", f"{sector}.toml")
        try:
            data = toml.load(path)
        except Exception as e:
            self.get_logger().error(f"Cannot load TOML '{path}': {e}")
            return
        pkg      = data.get("Package", [])
        if not isinstance(pkg, list) or len(pkg) < 3:
            self.get_logger().error("Missing Package or too short")
            return
        mode     = pkg[0]
        now_fn   = "/home/iclab/Desktop/Standmotion/sector/now_motion.toml"
        if mode == 242:
            try:
                shutil.copyfile(path, now_fn)
                self.get_logger().debug(f"Backed up TOML as {now_fn}")
                merged_pkg = pkg[:]
            except Exception as e:
                self.get_logger().error(f"Backup failed: {e}")
                return
        elif mode == 243:
            try:
                prev_data   = toml.load(now_fn)
            except Exception as e:
                self.get_logger().error(f"Cannot load existing now_motion: {e}")
                return
            prev_pkg     = prev_data.get("Package", [])
            curr_pkg     = data.get("Package", [])
            prev_pkg32   = prev_data.get("Package32", [])
            curr_pkg32   = data.get("Package32", [])
            if len(prev_pkg) != len(curr_pkg) or len(prev_pkg32) != len(curr_pkg32):
                self.get_logger().error("Package 或 Package32 長度不一致，無法合併")
                return
            merged_pkg   = []
            last_idx     = len(prev_pkg) - 1
            for i, (p, c) in enumerate(zip(prev_pkg, curr_pkg)):
                if i == last_idx:
                    merged_pkg.append(c)
                elif i % 2 == 1:
                    merged_pkg.append(c)
                else:
                    merged_pkg.append(p + c)
            merged_pkg[0] = 242
            merged_pkg32 = []
            last32       = len(prev_pkg32) - 1
            for i, (p32, c32) in enumerate(zip(prev_pkg32, curr_pkg32)):
                if i == last32:
                    merged_pkg32.append(c32)
                elif i % 2 == 1:
                    merged_pkg32.append(c32)
                else:
                    merged_pkg32.append(p32 + c32)
            prev_data["Package"]    = merged_pkg
            prev_data["PackageCnt"] = len(merged_pkg)
            prev_data["Package32"]  = merged_pkg32
            try:
                with open(now_fn, "w") as f:
                    toml.dump(prev_data, f)
                self.get_logger().debug(f"Merged 243 into {now_fn}")
            except Exception as e:
                self.get_logger().error(f"Merge write failed: {e}")
                return
        elif mode == 244:
            now_fn   = "/home/iclab/Desktop/Standmotion/sector/now_motion.toml"
            self.get_logger().debug(f"244 package :{pkg}")
            motionlist = [x for x in pkg[1:-1] if x != 0]
            self.get_logger().debug(f"motionlist package :{motionlist}")
            delaytime_cnt = sum(1 for idx, x in enumerate(motionlist, start=1) if idx % 2 == 0)
            delaytime_package = [x for idx, x in enumerate(motionlist, start=1) if idx % 2 == 0]
            motion_id = [x for idx, x in enumerate(motionlist, start=1) if idx % 2 == 1]
            self.get_logger().debug(f"delaytime_cnt :{delaytime_cnt}")
            self.get_logger().debug(f"motion_id :{motion_id}")
            for a in range(delaytime_cnt):
                path = os.path.join(self.location, "sector", f"{motion_id[a]}_id.toml")
                try:
                    data = toml.load(path)
                except Exception as e:
                    self.get_logger().error(f"Cannot load TOML '{path}': {e}")
                    return
                self.get_logger().debug(f"path :{data}")
                try:
                    prev_data   = toml.load(now_fn)
                except Exception as e:
                    self.get_logger().error(f"Cannot load existing now_motion: {e}")
                    return
                prev_pkg     = prev_data.get("Package", [])
                curr_pkg     = data.get("Package", [])
                prev_pkg32   = prev_data.get("Package32", [])
                curr_pkg32   = data.get("Package32", [])
                self.get_logger().debug(f"prev_pkg{motion_id[a]} :{prev_pkg}")
                if len(prev_pkg) != len(curr_pkg) or len(prev_pkg32) != len(curr_pkg32):
                    self.get_logger().error("Package 或 Package32 長度不一致，無法合併")
                    return
                merged_pkg   = []
                last_idx     = len(prev_pkg) - 1
                for i, (p, c) in enumerate(zip(prev_pkg, curr_pkg)):
                    if i == last_idx:
                        merged_pkg.append(c)
                    elif i % 2 == 1:
                        merged_pkg.append(c)
                    else:
                        merged_pkg.append(p + c)
                merged_pkg[0] = 242
                merged_pkg32 = []
                last32       = len(prev_pkg32) - 1
                for i, (p32, c32) in enumerate(zip(prev_pkg32, curr_pkg32)):
                    if i == last32:
                        merged_pkg32.append(c32)
                    elif i % 2 == 1:
                        merged_pkg32.append(c32)
                    else:
                        merged_pkg32.append(p32 + c32)
                prev_data["Package"]    = merged_pkg
                prev_data["PackageCnt"] = len(merged_pkg)
                prev_data["Package32"]  = merged_pkg32
                try:
                    with open(now_fn, "w") as f:
                        toml.dump(prev_data, f)
                    self.get_logger().debug(f"Merged 243 into {now_fn}")
                except Exception as e:
                    self.get_logger().error(f"Merge write failed: {e}")
                    return
                header       = merged_pkg[0]
                payload_vals = merged_pkg[1:-1]
                buf          = bytearray([header])
                for v in payload_vals:
                    v32 = v & 0xFFFFFFFF
                    buf.extend((v32 & 0xFFFF).to_bytes(2, "little"))
                    buf.extend(((v32 >> 16) & 0xFFFF).to_bytes(2, "little"))

                try:
                    ser = self.serial_walk
                    ser.reset_input_buffer()
                    ser.timeout = 0.1

                    written = ser.write(buf)
                    ser.flush()
                    self.get_logger().debug(f"[OpenCR] Sent {written} bytes: {list(buf)}")
                    acks = []
                    while True:
                        line = ser.readline().decode().strip()
                        if not line:
                            break
                        acks.append(line)
                    self.get_logger().info(f"ID from Motionlist ack :{acks}")
                    self.execut_ack.data = True
                    self.ExecuteCallBack_Publish.publish(self.execut_ack)
                    self.get_logger().debug(f"{header} Execute is finish! ACK={self.execut_ack.data}")
                    time.sleep(delaytime_package[a]/1000)
                except Exception as e:
                    self.get_logger().error(f"[OpenCR] Serial error: {e}")
                    self.execut_ack.data = False
                    self.ExecuteCallBack_Publish.publish(self.execut_ack)
                    self.get_logger().debug(f"{header} Execute is finish! ACK={self.execut_ack.data}")
                    return
            return
        header       = merged_pkg[0]
        payload_vals = merged_pkg[1:-1]
        buf          = bytearray([header])
        for v in payload_vals:
            v32 = v & 0xFFFFFFFF
            buf.extend((v32 & 0xFFFF).to_bytes(2, "little"))
            buf.extend(((v32 >> 16) & 0xFFFF).to_bytes(2, "little"))
        try:
            ser = self.serial_walk
            ser.reset_input_buffer()
            ser.timeout = 0.1
            written = ser.write(buf)
            ser.flush()
            self.get_logger().debug(f"[OpenCR] Sent {written} bytes: {list(buf)}")
            acks = []
            while True:
                line = ser.readline().decode().strip()
                if not line:
                    break
                acks.append(line)
            self.get_logger().info(f"Motion ack : {acks}")
            self.execut_ack.data = True
        except Exception as e:
            self.get_logger().error(f"[OpenCR] Serial error: {e}")
            self.execut_ack.data = False
        self.ExecuteCallBack_Publish.publish(self.execut_ack)
        self.get_logger().debug(f"{header} Execute is finish! ACK={self.execut_ack.data}")
############################
# execute sector functions #
############################
                
def main():
    rclpy.init()
    motion = Motionpackage()
    motion.RobotisListinit()
    rclpy.spin(motion)

if __name__ == "__main__":
    main()