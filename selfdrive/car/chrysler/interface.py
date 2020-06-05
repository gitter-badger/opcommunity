#!/usr/bin/env python3
from cereal import car, arne182
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.car.chrysler.values import Ecu, ECU_FINGERPRINT, CAR, FINGERPRINTS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.tfcp = CP.create_tf_can_parser()

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)
    ret.carName = "chrysler"
    ret.safetyModel = car.CarParams.SafetyModel.chrysler

    # Chrysler port is a community feature, since we don't own one to test
    ret.communityFeature = True

    # Speed conversion:              20, 45 mph
    ret.wheelbase = 3.089  # in meters for Pacifica Hybrid 2017
    ret.steerRatio = 16.2 # Pacifica Hybrid 2017
    ret.mass = 1964. + STD_CARGO_KG  # kg curb weight Pacifica General
    ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15,0.30], [0.03,0.05]]
    ret.lateralTuning.pid.kf = 0.00006   # full torque for 10 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.7
    ret.steerLimitTimer = 0.4

    if candidate in (CAR.JEEP_CHEROKEE_2017, CAR.JEEP_CHEROKEE_2018, CAR.JEEP_CHEROKEE_2019):
      ret.wheelbase = 2.91  # in meters
      ret.steerRatio = 12.7
      ret.steerActuatorDelay = 0.2  # in seconds

    if candidate in (CAR.CHRYSLER_300_2018, CAR.CHRYSLER_300_2018_TRAFFICFLOW):
      ret.wheelbase = 3.05308 # in meters
      ret.steerRatio = 15.5 # 2013 V-6 (RWD) — 15.5:1 V-6 (AWD) — 16.5:1 V-8 (RWD) — 15.5:1 V-8 (AWD) — 16.5:1
      ret.mass = 1828.0 + STD_CARGO_KG # 2013 V-6 RWD
      ret.lateralTuning.pid.kf = 0.00005584521385   # full torque for 10 deg at 80mph means 0.00007818594

    if candidate in (CAR.CHRYSLER_300_2018_TRAFFICFLOW):
      ret.trafficflow = True

    if ret.trafficflow:
      ret.openpilotLongitudinalControl = True
      ret.longitudinalTuning.kpBP = [0., 5., 35.]
      ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
      ret.longitudinalTuning.kiBP = [0., 35.]
      ret.longitudinalTuning.kiV = [0.54, 0.36]

    ret.centerToFront = ret.wheelbase * 0.44

    ret.minSteerSpeed = 3.8  # m/s
    if candidate in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
      # TODO allow 2019 cars to steer down to 13 m/s if already engaged.
      ret.minSteerSpeed = 17.5  # m/s 17 on the way up, 13 on the way down once engaged.

    # starting with reasonable value for civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)

    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay
    #print("ECU Camera Simulated: {0}".format(ret.enableCamera))

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)
    self.tfcp.update_string(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.tfcp)
    ret_arne182 = arne182.CarStateArne182.new_message()

    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    # speeds
    ret.yawRate = self.VM.yaw_rate(ret.steeringAngle * CV.DEG_TO_RAD, ret.vEgo)
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    ret.buttonEvents = []
      # unknown @0;
      # leftBlinker @1;
      # rightBlinker @2;
      # accelCruise @3;
      # decelCruise @4;
      # cancel @5;
      # altButton1 @6;
      # altButton2 @7;
      # altButton3 @8;
      # setCruise @9;
      # resumeCruise @10;
      # gapAdjustCruise @11;

    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      if not self.CS.cruise_buttons.cancel and self.CS.prev_cruise_buttons.cancel:
        be.type = ButtonType.cancel
      elif not self.CS.cruise_buttons.resume and self.CS.prev_cruise_buttons.resume: 
        be.type = ButtonType.resumeCruise
      elif not self.CS.cruise_buttons.speed_decrease and self.CS.prev_cruise_buttons.speed_decrease: 
        be.type = ButtonType.decelCruise
      elif not self.CS.cruise_buttons.speed_increase and self.CS.prev_cruise_buttons.speed_increase: 
        be.type = ButtonType.accelCruise
      else:
        be.type = ButtonType.unknown

      ret.buttonEvents.append(be)

    # events
    events, ret_arne182.events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low], gas_resume_speed = 2.)

    if ret.vEgo < self.CP.minSteerSpeed:
      events.append(create_event('belowSteerSpeed', [ET.WARNING]))

    ret.events = events

    # copy back carState packet to CS
    self.CS.out = ret.as_reader()

    return self.CS.out, ret_arne182.as_reader()


  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    if (self.CS.frame == -1):
      return [] # if we haven't seen a frame 220, then do not update.

    can_sends = self.CC.update(c.enabled, self.CS, c.actuators, c.cruiseControl.cancel, c.hudControl.visualAlert)

    return can_sends

