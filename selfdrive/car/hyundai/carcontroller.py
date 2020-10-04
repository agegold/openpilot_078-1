from cereal import car, log
from common.realtime import DT_CTRL
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa, create_mdps12
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

# speed controller
from selfdrive.car.hyundai.spdcontroller  import SpdController
from selfdrive.car.hyundai.spdctrl  import Spdctrl

import common.log as trace1
import common.CTime1000 as tm

VisualAlert = car.CarControl.HUDControl.VisualAlert
LaneChangeState = log.PathPlan.LaneChangeState

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.resume_cnt = 0
    self.last_resume_frame = 0
    self.last_lead_distance = 0
    self.lanechange_manual_timer = 0
    self.emergency_manual_timer = 0
    self.driver_steering_torque_above_timer = 0
    self.mode_change_timer = 0
    self.mode_change_switch = 1

    self.steer_mode = ""
    self.mdps_status = ""
    self.lkas_switch = ""

    self.lkas11_cnt = 0

    self.nBlinker = 0

    self.dRel = 0
    self.yRel = 0
    self.vRel = 0

    self.timer1 = tm.CTime1000("time")
    self.SC = Spdctrl()
    self.model_speed = 0
    self.model_sum = 0
    
    # hud
    self.hud_timer_left = 0
    self.hud_timer_right = 0

    self.traceCC = trace1.Loger("CarController")

    self.cruise_gap = 0
    self.cruise_gap_prev = 0
    self.cruise_gap_set_init = 0
    self.cruise_gap_switch_timer = 0

  def process_hud_alert(self, enabled, CC ):
    visual_alert = CC.hudControl.visualAlert
    left_lane = CC.hudControl.leftLaneVisible
    right_lane = CC.hudControl.rightLaneVisible

    sys_warning = (visual_alert == VisualAlert.steerRequired)

    if left_lane:
      self.hud_timer_left = 100

    if right_lane:
      self.hud_timer_right = 100

    if self.hud_timer_left:
      self.hud_timer_left -= 1
 
    if self.hud_timer_right:
      self.hud_timer_right -= 1


    # initialize to no line visible
    sys_state = 1
    if self.hud_timer_left and self.hud_timer_right or sys_warning:  # HUD alert only display when LKAS status is active
      if enabled or sys_warning:
        sys_state = 3
      else:
        sys_state = 4
    elif self.hud_timer_left:
      sys_state = 5
    elif self.hud_timer_right:
      sys_state = 6

    return sys_warning, sys_state



  def update(self, CC, CS, frame, sm, CP ):

    if self.CP != CP:
      self.CP = CP

    enabled = CC.enabled
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel
    
    self.dRel, self.yRel, self.vRel = SpdController.get_lead( sm )

    self.model_speed, self.model_sum = self.SC.calc_va(  sm, CS.out.vEgo  )

    # Steering Torque
    new_steer = actuators.steer * SteerLimitParams.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and abs(CS.out.steeringAngle) < 90.

    if (( CS.out.leftBlinker and not CS.out.rightBlinker) or ( CS.out.rightBlinker and not CS.out.leftBlinker)) and CS.out.vEgo <= 59 * CV.KPH_TO_MS:
      self.lanechange_manual_timer = 10
    if CS.out.leftBlinker and CS.out.rightBlinker:
      self.emergency_manual_timer = 10
    if abs(CS.out.steeringTorque) > 200:
      self.driver_steering_torque_above_timer = 20
    if self.lanechange_manual_timer or self.driver_steering_torque_above_timer:
      lkas_active = 0
    if self.lanechange_manual_timer > 0:
      self.lanechange_manual_timer -= 1
    if self.emergency_manual_timer > 0:
      self.emergency_manual_timer -= 1
    if self.driver_steering_torque_above_timer > 0:
      self.driver_steering_torque_above_timer -= 1

    if not lkas_active:
      apply_steer = 0

    steer_req = 1 if apply_steer else 0      

    self.apply_steer_last = apply_steer

    sys_warning, sys_state = self.process_hud_alert( lkas_active, CC )

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph  else 55
    if clu11_speed > enabled_speed:
      enabled_speed = clu11_speed

    can_sends = []
    if frame == 0: # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"] + 1
    self.lkas11_cnt %= 0x10

    can_sends.append(create_lkas11(self.packer, self.lkas11_cnt, self.car_fingerprint, apply_steer, steer_req,
                                   CS.lkas11, sys_warning, sys_state, CC, enabled, 0 ))
    if CS.mdps_bus or CS.scc_bus == 1: # send lkas11 bus 1 if mdps is on bus 1                               
      can_sends.append(create_lkas11(self.packer, self.lkas11_cnt, self.car_fingerprint, apply_steer, steer_req,
                                   CS.lkas11, sys_warning, sys_state, CC, enabled, 1 ))

    if CS.mdps_bus: # send clu11 to mdps if it is not on bus 0                                   
    #if frame % 2 and CS.mdps_bus == 1: # send clu11 to mdps if it is not on bus 0                                   
      can_sends.append(create_clu11(self.packer, frame, CS.mdps_bus, CS.clu11, Buttons.NONE, enabled_speed))
    
    #if CS.mdps_bus:
    can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))                                   

    str_log1 = '곡률={:05.1f}'.format(  self.model_speed )
    str_log2 = '프레임율={:03.0f}  TPMS=FL:{:04.1f}/FR:{:04.1f}/RL:{:04.1f}/RR:{:04.1f}'.format( self.timer1.sampleTime(), CS.tpmsPressureFl, CS.tpmsPressureFr, CS.tpmsPressureRl, CS.tpmsPressureRr )
    trace1.printf( '{}  {}'.format( str_log1, str_log2 ) )

    if CS.out.cruiseState.modeSel == 0 and self.mode_change_switch == 4:
      self.mode_change_timer = 50
      self.mode_change_switch = 0
    elif CS.out.cruiseState.modeSel == 1 and self.mode_change_switch == 0:
      self.mode_change_timer = 50
      self.mode_change_switch = 1
    elif CS.out.cruiseState.modeSel == 2 and self.mode_change_switch == 1:
      self.mode_change_timer = 50
      self.mode_change_switch = 2
    elif CS.out.cruiseState.modeSel == 3 and self.mode_change_switch == 2:
      self.mode_change_timer = 50
      self.mode_change_switch = 3
    elif CS.out.cruiseState.modeSel == 4 and self.mode_change_switch == 3:
      self.mode_change_timer = 50
      self.mode_change_switch = 4
    if self.mode_change_timer > 0:
      self.mode_change_timer -= 1

    run_speed_ctrl = CS.acc_active and self.SC != None and (CS.out.cruiseState.modeSel == 1 or CS.out.cruiseState.modeSel == 2 or CS.out.cruiseState.modeSel == 3)
    if not run_speed_ctrl:
      if CS.out.cruiseState.modeSel == 0:
        self.steer_mode = "오파모드"
      elif CS.out.cruiseState.modeSel == 1:
        self.steer_mode = "차간+커브"
      elif CS.out.cruiseState.modeSel == 2:
        self.steer_mode = "차간ONLY"
      elif CS.out.cruiseState.modeSel == 3:
        self.steer_mode = "정체구간"
      elif CS.out.cruiseState.modeSel == 4:
        self.steer_mode = "순정모드"
      if CS.out.steerWarning == 0:
        self.mdps_status = "정상"
      elif CS.out.steerWarning == 1:
        self.mdps_status = "오류"
      if CS.lkas_button_on == 0:
        self.lkas_switch = "OFF"
      elif CS.lkas_button_on == 1:
        self.lkas_switch = "ON"
      else:
        self.lkas_switch = "-"
      
      if CS.out.cruiseState.modeSel == 3:
        str_log2 = '주행모드={:s}  LKAS버튼={:s}  정체구간=(D:{:05.1f}/GPR:{:2.1f}/CG:{:2.1f}/CSG:{:2.1f})'.format( self.steer_mode, self.lkas_switch, self.dRel, self.cruise_gap_prev, self.cruise_gap, CS.cruiseGapSet)
      else:
        str_log2 = '주행모드={:s}  MDPS상태={:s}  LKAS버튼={:s}'.format( self.steer_mode, self.mdps_status, self.lkas_switch )
      trace1.printf2( '{}'.format( str_log2 ) )

    #print( 'st={} cmd={} long={}  steer={} req={}'.format(CS.out.cruiseState.standstill, pcm_cancel_cmd, self.CP.openpilotLongitudinalControl, apply_steer, steer_req ) )


    if pcm_cancel_cmd and self.CP.openpilotLongitudinalControl:
      can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.CANCEL, clu11_speed))
    elif CS.out.cruiseState.standstill:
      # run only first time when the car stopped
      if self.last_lead_distance == 0:
        # get the lead distance from the Radar
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0

      # when lead car starts moving, create 6 RES msgs
      elif CS.lead_distance != self.last_lead_distance and (frame - self.last_resume_frame) > 5:
        can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
        self.resume_cnt += 1
        # interval after 6 msgs
        if self.resume_cnt > 5:
          self.last_resume_frame = frame
          self.resume_cnt = 0

      elif self.cruise_gap_prev == 0: 
        self.cruise_gap_prev = CS.cruiseGapSet
        self.cruise_gap_set_init = 1
      elif self.cruise_gap != 1.0:
        self.cruise_gap_switch_timer += 1
        if self.cruise_gap_switch_timer > 30:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.GAP_DIST, clu11_speed))
          self.cruise_gap = CS.cruiseGapSet
          self.cruise_gap_switch_timer = 0

    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0
      self.cruise_gap = 0
    elif run_speed_ctrl and self.SC != None:
      is_sc_run = self.SC.update( CS, sm, self )
      if is_sc_run:
        can_sends.append(create_clu11(self.packer, self.resume_cnt, CS.scc_bus, CS.clu11, self.SC.btn_type, self.SC.sc_clu_speed ))
        self.resume_cnt += 1
      else:
        self.resume_cnt = 0
      if self.dRel > 15 and self.cruise_gap_prev != CS.cruiseGapSet and self.cruise_gap_set_init == 1 and CS.out.cruiseState.modeSel != 3:
        self.cruise_gap_switch_timer += 1
        if self.cruise_gap_switch_timer > 30:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.GAP_DIST, clu11_speed))
          self.cruise_gap_switch_timer = 0
      elif self.cruise_gap_prev == CS.cruiseGapSet:
        self.cruise_gap_set_init = 0
        self.cruise_gap_prev = 0

    if CS.out.cruiseState.modeSel == 3 and CS.acc_active:
      if 20 > self.dRel > 18 and self.vRel < 0 and CS.cruiseGapSet != 4.0:
        self.cruise_gap_switch_timer += 1
        if self.cruise_gap_switch_timer > 30:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.GAP_DIST, clu11_speed))
          self.cruise_gap_switch_timer = 0
      elif 16 > self.dRel > 14 and self.vRel < 0 and CS.cruiseGapSet != 3.0:
        self.cruise_gap_switch_timer += 1
        if self.cruise_gap_switch_timer > 30:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.GAP_DIST, clu11_speed))
          self.cruise_gap_switch_timer = 0
      elif 12 > self.dRel > 10 and self.vRel < 0 and CS.cruiseGapSet != 2.0:
        self.cruise_gap_switch_timer += 1
        if self.cruise_gap_switch_timer > 30:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.GAP_DIST, clu11_speed))
          self.cruise_gap_switch_timer = 0
      elif 9 > self.dRel > 7 and self.vRel < 0 and CS.cruiseGapSet != 1.0:
        self.cruise_gap_switch_timer += 1
        if self.cruise_gap_switch_timer > 30:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.GAP_DIST, clu11_speed))
          self.cruise_gap_switch_timer = 0
      elif 15 > self.dRel > 4 and self.vRel > 0 and CS.cruiseGapSet != 1.0:
        self.cruise_gap_switch_timer += 1
        if self.cruise_gap_switch_timer > 30:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.GAP_DIST, clu11_speed))
          self.cruise_gap_switch_timer = 0
      elif 25 > self.dRel > 18 and self.vRel >= 0 and CS.cruiseGapSet != 2.0:
        self.cruise_gap_switch_timer += 1
        if self.cruise_gap_switch_timer > 30:
          can_sends.append(create_clu11(self.packer, frame, CS.scc_bus, CS.clu11, Buttons.GAP_DIST, clu11_speed))
          self.cruise_gap_switch_timer = 0

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE, CAR.IONIQ]:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    self.lkas11_cnt += 1
    return can_sends
