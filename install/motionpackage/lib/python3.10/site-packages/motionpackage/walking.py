import math
import numpy as np
import matplotlib.pyplot as plt
# from motionpackage.motionpackage import Motionpackage
# Walkingpara = Motionpackage
class WalkingGaitByLIPM:
    def __init__(self,period_t_,T_DSP_):
        self.pre_step_ = -1
        
        self.now_step_ = 0
        self.sample_time_ = 30
        self.lift_height_ = 2
        self.board_height = 0
        self.width_size_ = -4.5
        self.rightfoot_shift_z = 0
        self.period_t_ = period_t_
        self.stop = False
        self.T_DSP_ = T_DSP_
        self.pz_ = 24.3
        self.Length_Leg = 23.5
        self.Length_Pelvis = 9
        ###
        self.Tc_ = np.sqrt(27/9.8)/10
        self.TT_ = period_t_ * 0.001
        #我是x
        self.step_length_ = 5
        #我是y
        self.shift_length_ = 0
        self.init()
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        self.e = []
        self.f = []

    def init(self):
        self.walking_state = 0 # StopStep:0 ,StartStep:1 ,FirstStep:2 ,Repeat:3
        self.step_ = 9999    
        self.left_step = 0  
        self.right_step = 0          
        self.footstep_x = 0
        self.footstep_y = 0
        self.sample_point_ = 0
        self.t_ = 0
        #              l  r 
        self.now_x_ = [0 ,0]
        self.now_y_ = [0 ,0]
        #                      now  last
        self.displacement_x = [0   ,0]
        self.displacement_y = [0   ,0]
        self.base_x         = [0   ,0]  
        self.base_y         = [0   ,0]
        self.zmp_x          = [0   ,0]
        self.zmp_y          = [0   ,0]
        self.theta          = [0   ,0]

    def process(self,stop=False):
        #readwalkparameter
        self.sample_point_  += 1
        time_point_ = self.sample_point_ * self.sample_time_
        self.t_ = ((time_point_ - self.sample_time_) % self.period_t_ + self.sample_time_)/1000
        # print("t_"  ,self.t_)
        self.now_step_ = int((self.sample_point_ - 1)/(self.period_t_ / self.sample_time_))
        # print("nself.theta[0]  #ow_step_",self.now_step_)
        
        if self.now_step_ == self.step_:
            self.walking_state = 0
        elif self.now_step_ < 2:
            self.walking_state = 1
        elif self.now_step_ > self.step_:
            pass
        elif self.now_step_ == 2 :
            self.walking_state = 2
        else:
            self.walking_state = 3
        
        if self.pre_step_ != self.now_step_:
            if (self.now_step_ % 2) == 1 and self.now_step_ > 1:
                self.left_step += 1
            else:
                self.right_step += 1
            if self.pre_step_ == -1:
                self.footstep_x = 0
                self.footstep_y = -self.width_size_
                self.now_x_[1] = self.footstep_x
                self.now_y_[1] = -self.footstep_y
                self.now_x_[0] = 0
                self.now_y_[0] = self.width_size_
            elif (self.pre_step_ % 2) == 0:
                self.now_x_[0] = self.footstep_x
                self.now_y_[0] = self.footstep_y
            elif (self.pre_step_ % 2) == 1:
                self.now_x_[1] = self.footstep_x
                self.now_y_[1]= self.footstep_y

            self.zmp_x[1] = self.zmp_x[0]
            self.zmp_y[1] = self.zmp_y[0]
            self.zmp_x[0] = self.footstep_x
            self.zmp_y[0] = self.footstep_y
            self.displacement_x[1] = self.displacement_x[0]  # 上次的跨幅
            self.base_x[1] = self.base_x[0]  # 上次到達的位置
            self.displacement_y[1] = self.displacement_y[0]  # 上次的Y軸位移量
            self.base_y[1] = self.base_y[0]  # 上次的Y軸位移位置
            self.theta[1] = self.theta[0]  # 前一次的Theta變化量

            #readwalkdata

            if self.walking_state == 1:
                self.theta_ = 0
                self.theta[0] = 0

                self.now_width_ = 2 * self.width_size_ * (-pow(-1, self.now_step_ + 1))
                self.width_x = -np.sin(self.theta_) * self.now_width_
                self.width_y = np.cos(self.theta_) * self.now_width_
                self.displacement_x[0] = self.width_x
                self.displacement_y[0] = self.width_y
                self.footstep_x += self.width_x
                self.footstep_y += self.width_y

            elif self.walking_state == 0:
                self.theta_ = self.theta[0]

                self.now_width_ = 2 * self.width_size_ * (-pow(-1, self.now_step_ + 1))
                self.width_x = -np.sin(self.theta_) * self.now_width_
                self.width_y = np.cos(self.theta_) * self.now_width_
                self.displacement_x[0] = self.width_x
                self.displacement_y[0] = self.width_y
                self.footstep_x += self.width_x
                self.footstep_y += self.width_y

            else:
                self.theta_ = self.theta[0]

                self.now_width_ = 2 * self.width_size_ * (-pow(-1, self.now_step_ + 1))
                self.width_x = -np.sin(self.theta_) * self.now_width_
                self.width_y = np.cos(self.theta_) * self.now_width_
                self.displacement_x[0] = (self.step_length_ * np.cos(self.theta_) - self.shift_length_ * np.sin(self.theta_)) + self.width_x
                self.displacement_y[0] = (self.step_length_ * np.sin(self.theta_) + self.shift_length_ * np.cos(self.theta_)) + self.width_y
                self.footstep_x += self.displacement_x[0]
                self.footstep_y += self.displacement_y[0]

            self.base_x[0] = (self.footstep_x + self.zmp_x[0]) / 2
            self.base_y[0] = (self.footstep_y + self.zmp_y[0]) / 2

        self.pre_step_= self.now_step_

        if stop:
            self.step = self.now_step_+1

        px_, py_, lpx_, lpy_, rpx_, rpy_, lpz_, rpz_, lpt_, rpt_ = self.walkinggait(self.walking_state)
        
        py_u = py_
        px_u = px_
        # self.a.append(self.zmp_y[0])
        # self.b.append(self.zmp_y[1])
        # self.c.append(lpy_)
        return self.coordinate_transformation(py_u, px_u, lpx_, lpy_, rpx_, rpy_, lpz_, rpz_, lpt_, rpt_)


    def walkinggait(self,walking_state):
        px_, py_, lpx_, lpy_, rpx_, rpy_, lpz_, rpz_, lpt_, rpt_ = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        if walking_state == 1:
            vx0_ = self.wComVelocityInit(0, 0, self.zmp_x[0], self.TT_, self.Tc_)
            px_ = self.wComPosition(0, vx0_, self.zmp_x[0], self.t_, self.Tc_)
            vy0_ = self.wComVelocityInit(0, 0, self.zmp_y[0], self.TT_, self.Tc_)

            # self.StartHeight_ = self.StartHeight_
            if (self.now_step_ % 2) == 1:
                py_ = self.wComPosition(0, vy0_, self.zmp_y[0], self.t_, self.Tc_)
                lpx_ = self.zmp_x[0]
                rpx_ = self.wFootPositionRepeat(self.now_x_[1], 0, self.t_, self.TT_, self.T_DSP_)
                lpy_ = self.zmp_y[0]
                rpy_ = -self.wFootPositionRepeat(self.now_y_[1], 0, self.t_, self.TT_, self.T_DSP_)
                # print("rpy_1",rpy_)
                lpz_ = 0
                rpz_ = self.wFootPositionZ(self.lift_height_, self.t_, self.TT_, self.T_DSP_)

                lpt_ = 0
                rpt_ = 0
                
            elif (self.now_step_ % 2) == 0:
                py_ = self.wComPosition(0, vy0_, self.zmp_y[0], self.t_, self.Tc_)
                lpx_ = self.wFootPositionRepeat(self.now_x_[0], 0, self.t_, self.TT_, self.T_DSP_)
                rpx_ = self.zmp_x[0]
                lpy_ = self.wFootPositionRepeat(self.now_y_[0], 0, self.t_, self.TT_, self.T_DSP_)
                rpy_ = self.zmp_y[0]
                # print("rpy_0",rpy_)
                lpz_ = self.wFootPositionZ(self.lift_height_, self.t_, self.TT_, self.T_DSP_)
                rpz_ = 0

                lpt_ = 0
                rpt_ = 0

        elif walking_state == 2:
            vx0_ = self.wComVelocityInit(0, self.base_x[0], self.zmp_x[0], self.TT_, self.Tc_)
            px_ = self.wComPosition(0, vx0_, self.zmp_x[0], self.t_, self.Tc_)
            vy0_ = self.wComVelocityInit(0, self.base_y[0], self.zmp_y[0], self.TT_, self.Tc_)
            py_ = self.wComPosition(0, vy0_, self.zmp_y[0], self.t_, self.Tc_)
            lpx_ = self.wFootPosition(self.now_x_[0], self.displacement_x[0], self.t_, self.TT_, self.T_DSP_)
            rpx_ = self.zmp_x[0]
            lpy_ = self.wFootPosition(self.now_y_[0], self.displacement_y[0] - self.now_width_, self.t_, self.TT_, self.T_DSP_)
            rpy_ = self.zmp_y[0]
            lpz_ = self.wFootPositionZ(self.lift_height_, self.t_, self.TT_, self.T_DSP_)
            rpz_ = 0

            self.lpt_ = self.wFootTheta(-self.theta[1], 1, self.t_, self.TT_, self.T_DSP_)
            self.rpt_ = self.wFootTheta(-self.theta[0], 0, self.t_, self.TT_, self.T_DSP_)

        elif walking_state == 0:
            vx0_ = self.wComVelocityInit(self.base_x[1], self.base_x[0], self.zmp_x[0], self.TT_, self.Tc_)
            px_ = self.wComPosition(self.base_x[1], vx0_, self.zmp_x[0], self.t_, self.Tc_)
            vy0_ = self.wComVelocityInit(self.base_y[1], self.base_y, self.zmp_y[0], self.TT_, self.Tc_)
            py_ = self.wComPosition(self.base_y[1], vy0_, self.zmp_y[0], self.t_, self.Tc_)

            if (self.now_step_ % 2) == 1:
                lpx_ = self.zmp_x[0]
                lpy_ = self.zmp_y[0]
                rpx_ = self.wFootPosition(self.now_x_[1], (self.displacement_x[1] + self.displacement_x[0]), self.t_, self.TT_, self.T_DSP_)
                rpy_ = self.wFootPosition(self.now_y_[1], (self.displacement_y[1] + self.displacement_y[0]), self.t_, self.TT_, self.T_DSP_)
                lpz_ = 0
                rpz_ = self.wFootPositionZ(self.lift_height_, self.t_, self.TT_, self.T_DSP_)
                
                lpt_ = 0
                rpt_ = self.wFootTheta(-self.theta[1], 1, self.t_, self.TT_, self.T_DSP_)

            elif (self.now_step_ % 2) == 0:
                lpx_ = self.wFootPosition(self.now_x_[0], (self.displacement_x[1] + self.displacement_x[0]), self.t_, self.TT_, self.T_DSP_)
                rpx_ = self.zmp_x[0]
                lpy_ = self.wFootPosition(self.now_y_[0], (self.displacement_y[1] + self.displacement_y[0]), self.t_, self.TT_, self.T_DSP_)
                rpy_ = self.zmp_y[0]
                lpz_ = self.wFootPositionZ(self.lift_height_, self.t_, self.TT_, self.T_DSP_)
                rpz_ = 0
                
                lpt_ = self.wFootTheta(-self.theta[1], 1, self.t_, self.TT_, self.T_DSP_)
                rpt_ = 0

        elif walking_state == 3:
            vx0_ = self.wComVelocityInit(self.base_x[1], self.base_x[0], self.zmp_x[0], self.TT_, self.Tc_)
            px_ = self.wComPosition(self.base_x[1], vx0_, self.zmp_x[0], self.t_, self.Tc_)
            vy0_ = self.wComVelocityInit(self.base_y[1], self.base_y[0], self.zmp_y[0] ,self.TT_, self.Tc_)
            py_ = self.wComPosition(self.base_y[1], vy0_, self.zmp_y[0], self.t_, self.Tc_)

            if (self.now_step_ % 2) == 1:
                lpx_ = self.zmp_x[0]
                rpx_ = self.wFootPositionRepeat(self.now_x_[1], (self.displacement_x[1] + self.displacement_x[0]) / 2, self.t_, self.TT_, self.T_DSP_)
                lpy_ = self.zmp_y[0]
                rpy_ = self.wFootPositionRepeat(self.now_y_[1], (self.displacement_y[1] + self.displacement_y[0]) / 2, self.t_, self.TT_, self.T_DSP_)
                lpz_ = 0
                rpz_ = self.wFootPositionZ(self.lift_height_, self.t_, self.TT_, self.T_DSP_)
                if self.theta[0] * self.theta[1] >= 0:
                    lpt_ = self.wFootTheta(-self.theta[0], 0, self.t_, self.TT_, self.T_DSP_)
                    rpt_ = self.wFootTheta(-self.theta[1], 1, self.t_, self.TT_, self.T_DSP_)
                else:
                    lpt_ = 0
                    rpt_ = 0

            elif (self.now_step_ % 2) == 0:
                lpx_ = self.wFootPositionRepeat(self.now_x_[0], (self.displacement_x[1] + self.displacement_x[0]) / 2, self.t_, self.TT_, self.T_DSP_)
                rpx_ = self.zmp_x[0]
                lpy_ = self.wFootPositionRepeat(self.now_y_[0], (self.displacement_y[1] + self.displacement_y[0]) / 2, self.t_, self.TT_, self.T_DSP_)
                rpy_ = self.zmp_y[0]
                lpz_ = self.wFootPositionZ(self.lift_height_, self.t_, self.TT_, self.T_DSP_)
                rpz_ = 0
                if self.theta[0] * self.theta[1] >= 0:
                    lpt_ = self.wFootTheta(-self.theta[1], 1, self.t_, self.TT_, self.T_DSP_)
                    rpt_ = self.wFootTheta(-self.theta[0], 0, self.t_, self.TT_, self.T_DSP_)
                else:
                    lpt_ = 0
                    rpt_ = 0

        return px_, py_, lpx_, lpy_, rpx_, rpy_, lpz_, rpz_, lpt_, rpt_
        
    def coordinate_transformation(self,py_u, px_u, lpx_, lpy_, rpx_, rpy_, lpz_, rpz_, lpt_, rpt_):
        """座標平移 W to B"""
        step_point_lx_W_ = lpx_ - px_u
        step_point_rx_W_ = rpx_ - px_u
        step_point_ly_W_ = lpy_ - py_u
        step_point_ry_W_ = rpy_ - py_u
        step_point_lz_ = self.pz_ - lpz_
        step_point_rz_ = self.pz_ - rpz_
        step_point_lthta_ = -lpt_
        step_point_rthta_ = -rpt_

        """座標旋轉 W to B"""
        cos_theta = np.cos(-self.theta_)
        sin_theta = np.sin(-self.theta_)
        
        step_point_lx_ = step_point_lx_W_ * cos_theta - step_point_ly_W_ * sin_theta
        step_point_ly_ = step_point_lx_W_ * sin_theta + step_point_ly_W_ * cos_theta
        step_point_rx_ = step_point_rx_W_ * cos_theta - step_point_ry_W_ * sin_theta
        step_point_ry_ = step_point_rx_W_ * sin_theta + step_point_ry_W_ * cos_theta

        end_point_lx_ = step_point_lx_
        end_point_rx_ = step_point_rx_
        end_point_ly_ = step_point_ly_  + self.Length_Pelvis/2
        end_point_ry_ = -step_point_ry_ + self.Length_Pelvis/2
        end_point_lz_ = step_point_lz_ - (self.pz_ - self.Length_Leg)
        end_point_rz_ = step_point_rz_ - (self.pz_ - self.Length_Leg)
        end_point_lthta_ = step_point_lthta_
        end_point_rthta_ = step_point_rthta_
        print("end_point_rx_",end_point_rx_, "end_point_ry_",end_point_ry_, "end_point_rz_",end_point_rz_, "end_point_lx_",end_point_lx_, "end_point_ly_",end_point_ly_, "end_point_lz_",end_point_lz_, "end_point_rthta_",end_point_rthta_, "end_point_lthta_",end_point_lthta_)
        print(step_point_ry_-step_point_ly_)
        self.a.append(end_point_rx_)
        self.b.append(step_point_ry_)
        self.c.append(end_point_ry_)
        self.d.append(end_point_lx_)
        self.e.append(step_point_ly_)
        self.f.append(end_point_ly_)
        return end_point_rx_, end_point_ry_, end_point_rz_, end_point_lx_, end_point_ly_, end_point_lz_, end_point_rthta_, end_point_lthta_

    def wComVelocityInit(self, x0, xt, px, t, T):
        return (xt - x0 * np.cosh(t / T) + px * (np.cosh(t / T) - 1)) / (T * np.sinh(t / T))

    def wComPosition(self, x0, vx0, px, t, T):
        return x0 * np.cosh(t / T) + T * vx0 * np.sinh(t / T) - px * (np.cosh(t / T) - 1)

    def wFootPosition(self, start, length, t, T, T_DSP):
        new_T = T * (1 - T_DSP)
        new_t = t - T * T_DSP / 2
        omega = 2 * np.pi / new_T

        if t > 0 and t <= T * T_DSP / 2:
            return start
        elif t > T * T_DSP / 2 and t <= T * (1 - T_DSP / 2):
            return length * (omega * new_t - np.sin(omega * new_t)) / (2 * np.pi) + start
        else:
            return length + start

    def wFootPositionRepeat(self, start, length, t, T, T_DSP):
        new_T = T * (1 - T_DSP)
        new_t = t - T * T_DSP / 2
        omega = 2 * np.pi / new_T

        if t > 0 and t <= T * T_DSP / 2:
            return start
        elif t >= T * T_DSP / 2 and t <= T * (1 - T_DSP / 2):
            return 2 * length * (omega * new_t - np.sin(omega * new_t)) / (2 * np.pi) + start
        else:
            return 2 * length + start

    def wFootPositionZ(self, height, t, T, T_DSP):
        new_T = T * (1 - T_DSP)
        new_t = t - T * T_DSP / 2
        omega = 2 * np.pi / new_T

        if T * T_DSP / 2 < t < T * (1 - (T_DSP / 2)):
            return 0.5 * height * (1 - np.cos(omega * new_t))
        else:
            return 0

    def wFootTheta(self, theta, reverse, t, T, T_DSP):
        new_T = T * (1 - T_DSP)
        new_t = t - T * T_DSP / 2
        omega = 2 * np.pi / new_T
        
        if t > 0 and t <= T * T_DSP / 2:
            return theta if reverse else 0
        elif T * T_DSP / 2 < t <= T * (1 - T_DSP / 2):
            if reverse:
                return 0.5 * theta * (1 - np.cos(0.5 * omega * (new_t - new_T)))
            else:
                return 0.5 * theta * (1 - np.cos(0.5 * omega * new_t))
        else:
            return 0 if reverse else theta
        
class InverseKinematic:
    def __init__(self):
        self.Thta = np.zeros(22)
        self.a = []
        self.b = []

    def init(self,end_point_rx_, end_point_ry_, end_point_rz_, end_point_lx_, end_point_ly_, end_point_lz_, end_point_rthta_, end_point_lthta_):
        # print(end_point_rx_, end_point_ry_, end_point_rz_, end_point_lx_, end_point_ly_, end_point_lz_, end_point_rthta_, end_point_lthta_)
        self.l1 = 12.5
        self.l2 = 12.5
        self.RX_2 = end_point_rx_ * end_point_rx_
        self.RY_2 = end_point_ry_ * end_point_ry_
        self.RZ_2 = end_point_rz_ * end_point_rz_
        self.LX_2 = end_point_lx_ * end_point_lx_
        self.LY_2 = end_point_ly_ * end_point_ly_
        self.LZ_2 = end_point_lz_ * end_point_lz_
        self.l1_2 = self.l1 * self.l1
        self.l2_2 = self.l2 * self.l2
        self.l1_l2 = self.l1 + self.l2
        self.R_Lyz = np.sqrt(self.RY_2 + self.RZ_2)
        self.L_Lyz = np.sqrt(self.LY_2 + self.LZ_2)
        self.R_Lxyz = np.sqrt(self.RX_2 + self.RY_2 + self.RZ_2)
        self.L_Lxyz = np.sqrt(self.LX_2 + self.LY_2 + self.LZ_2)
        

    def ik(self,end_point_rx_=0, end_point_ry_=0, end_point_rz_=23.5, end_point_lx_=0, end_point_ly_=0, end_point_lz_=23.5, end_point_rthta_=0, end_point_lthta_=0):
        self.init(end_point_rx_, end_point_ry_, end_point_rz_, end_point_lx_, end_point_ly_, end_point_lz_, end_point_rthta_, end_point_lthta_)
        if self.R_Lxyz > self.l1_l2:
            self.R_Lxyz = self.l1_l2
        if self.L_Lxyz > self.l1_l2:
            self.L_Lxyz = self.l1_l2
        self.RL_2 = self.R_Lxyz * self.R_Lxyz
        self.LL_2 = self.L_Lxyz * self.L_Lxyz
        # print("l1",self.l1, 'l2',self.l2, 'RX_2',self.RX_2, 'RY_2',self.RY_2, 'RZ_2', self.RZ_2, 'LX_2',self.LX_2)
        # print("LY_2",self.LY_2, 'LZ_2',self.LZ_2, 'l1_2',self.l1_2, 'l2_2',self.l2_2, 'l1_l2',self.l1_l2, 'R_Lyz',self.R_Lyz, 'L_Lyz',self.L_Lyz)
        # print("R_Lxyz",self.R_Lxyz, 'L_Lxyz',self.L_Lxyz, 'RL_2',self.RL_2, 'LL_2',self.LL_2)
        self.Thta[10] = 0#end_point_lthta_ + 0
        if end_point_ly_ == 0:
            self.Thta[11] = 0
        else:
            self.Thta[11] = math.atan2(end_point_lz_, end_point_ly_) - np.pi/2

        if end_point_lx_ == 0:
            self.Thta[12] = 0 - math.acos((self.l1_2 + self.LL_2 - self.l2_2) / (2 * self.l1 * self.L_Lxyz))
        elif end_point_lx_ > 0:
            self.Thta[12] = 0 - math.acos((self.l1_2 + self.LL_2 - self.l2_2) / (2 * self.l1 * self.L_Lxyz)) - math.atan2(end_point_lx_, self.L_Lyz)
        else:
            self.Thta[12] = np.pi/2 - math.acos((self.l1_2 + self.LL_2 - self.l2_2) / (2 * self.l1 * self.L_Lxyz)) - math.atan2(self.L_Lyz, -end_point_lx_)

        self.Thta[13] = np.pi - math.acos((self.l1_2 + self.l2_2 - self.LL_2) / (2 * self.l1 * self.l2))
        self.Thta[14] = self.Thta[12] + self.Thta[13]

        self.Thta[15] = -self.Thta[11]

        self.Thta[16] = 0#end_point_rthta_ + 0

        if end_point_ry_ == 0:
            self.Thta[17] = 0
        else:
            self.Thta[17] = math.atan2(end_point_rz_, end_point_ry_) - np.pi/2

        if end_point_rx_ == 0:
            self.Thta[18] = 0 - math.acos((self.l1_2 + self.RL_2 - self.l2_2) / (2 * self.l1 * self.R_Lxyz))
        elif end_point_rx_ > 0:
            self.Thta[18] = 0 - math.acos((self.l1_2 + self.RL_2 - self.l2_2) / (2 * self.l1 * self.R_Lxyz)) - math.atan2(end_point_rx_, self.R_Lyz)
        else:
            self.Thta[18] = np.pi/2 - math.acos((self.l1_2 + self.RL_2 - self.l2_2) / (2 * self.l1 * self.R_Lxyz)) - math.atan2(self.R_Lyz, -end_point_rx_)

        self.Thta[19] = np.pi - math.acos((self.l1_2 + self.l2_2 - self.RL_2) / (2 * self.l1 * self.l2))
        self.Thta[20] = self.Thta[18] + self.Thta[19]

        self.Thta[21] = self.Thta[17]

        self.Thta[12] = -self.Thta[12]
        self.Thta[13] = -self.Thta[13]
        print("LF",self.Thta[10:16])
        print("RF",self.Thta[16:22])
        # self.a.append(self.Thta[11])
        # self.b.append(self.Thta[17])
        