#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.values import Ecu, ECU_FINGERPRINT, CAR, FINGERPRINTS, Buttons
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase, MAX_CTRL_SPEED

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState )
    self.cp2 = self.CS.get_can2_parser(CP)
    
  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    ret.carName = "hyundai"
    ret.safetyModel = car.CarParams.SafetyModel.hyundai
    ret.radarOffCan = False

    # Most Hyundai car ports are community features for now
    ret.communityFeature = False

    tire_stiffness_factor = 1.
    ret.steerActuatorDelay = 0.3
    ret.steerRateCost = 0.4
    ret.steerLimitTimer = 0.4

    if candidate == CAR.KIA_OPTIMA_H:
      ret.wheelbase = 2.80
      ret.mass = 1595. + STD_CARGO_KG
      ret.steerRatio = 13.5
      
      #ret.lateralTuning.pid.kf = 0.00005
      #ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      #ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.20], [0.04]]
      ret.lateralTuning.pid.kpBP = [0., 9., 17., 28.]
      ret.lateralTuning.pid.kpV = [0.10, 0.15, 0.20, 0.25]
      ret.lateralTuning.pid.kiBP = [0., 9., 17., 28.]
      ret.lateralTuning.pid.kiV = [0.02, 0.03, 0.04, 0.05]
      ret.lateralTuning.pid.kfBP = [0., 9., 17., 28.]
      ret.lateralTuning.pid.kfV = [0.00003, 0.00004, 0.00005, 0.00006]

      
      #ret.lateralTuning.init('lqr')
      #ret.lateralTuning.lqr.scale = 1750.0
      #ret.lateralTuning.lqr.ki = 0.02
      #ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
      #ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
      #ret.lateralTuning.lqr.c = [1., 0.]
      #ret.lateralTuning.lqr.k = [-100., 450.]
      #ret.lateralTuning.lqr.l = [0.22, 0.318]
      #ret.lateralTuning.lqr.dcGain = 0.0025

      #ret.lateralTuning.init('indi')
      #ret.lateralTuning.indi.innerLoopGain = 3.0
      #ret.lateralTuning.indi.outerLoopGain = 2.0
      #ret.lateralTuning.indi.timeConstant = 1.0
      #ret.lateralTuning.indi.actuatorEffectiveness = 1.0

    # these cars require a special panda safety mode due to missing counters and checksums in the messages
    if candidate == CAR.HYUNDAI_GENESIS:
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiLegacy

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay



    # ignore CAN2 address if L-CAN on the same BUS
    ret.mdpsBus = 1 if 593 in fingerprint[1] and 1296 not in fingerprint[1] else 0    # MDPS12
    ret.sasBus = 1 if 688 in fingerprint[1] and 1296 not in fingerprint[1] else 0     # SAS11
    ret.sccBus = 0 if 1056 in fingerprint[0] else 1 if 1056 in fingerprint[1] and 1296 not in fingerprint[1] else 2 if 1056 in fingerprint[2] else -1  # SCC11
    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid

    # TODO: button presses
    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == Buttons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.GAP_DIST:
        be.type = ButtonType.gapAdjustCruise
      elif but == Buttons.CANCEL:
        be.type = ButtonType.cancel
      buttonEvents.append(be)
    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents
    #ret.buttonEvents = []    

    events = self.create_common_events(ret)
    #TODO: addd abs(self.CS.angle_steers) > 90 to 'steerTempUnavailable' event

    if self.CC.lanechange_manual_timer:
      events.add(EventName.laneChangeManual)
    if self.CC.emergency_manual_timer:
      events.add(EventName.emgButtonManual)
    if self.CC.driver_steering_torque_above_timer:
      events.add(EventName.driverSteering)

    if self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 0:
      events.add(EventName.modeChangeOpenpilot)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 1:
      events.add(EventName.modeChangeDistcurv)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 2:
      events.add(EventName.modeChangeDistance)
    elif self.CC.mode_change_timer and self.CS.out.cruiseState.modeSel == 3:
      events.add(EventName.modeChangeTrafficjam)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c, sm, CP ):
    can_sends = self.CC.update(c, self.CS, self.frame, sm, CP )

    self.frame += 1
    return can_sends
