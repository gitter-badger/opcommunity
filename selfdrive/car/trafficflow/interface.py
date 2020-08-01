from cereal import car, arne182
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, EventTypes as ET
from selfdrive.car.trafficflow.values import CAR
from common.params import put_nonblocking
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    # VW port is a community feature, since we don't own one to test
    ret.communityFeature = True
    ret.carName = "TrafficFlow Equipped Vehicle"
    ret.radarOffCan = True
    ret.safetyModel = car.CarParams.SafetyModel.allOutput

    ret.wheelbase = 3.05308 # in meters
    ret.steerRatio = 15.5 # 2013 V-6 (RWD) — 15.5:1 V-6 (AWD) — 16.5:1 V-8 (RWD) — 15.5:1 V-8 (AWD) — 16.5:1
    ret.steerRatioRear = 0.
    ret.mass = 1828.0 + STD_CARGO_KG # 2013 V-6 RWD
    ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15,0.30], [0.03,0.05]]
    ret.lateralTuning.pid.kf = 0.00006   # full torque for 10 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 0.7
    ret.steerLimitTimer = 0.4
    ret.centerToFront = ret.wheelbase * 0.44
    ret.minSteerSpeed = 3.8  # m/s
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)
    ret.enableCamera = True
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.steerControlType = car.CarParams.SteerControlType.torque
  
    # ret.enableCruise = True  # Stock ACC still controls acceleration and braking
    # ret.openpilotLongitudinalControl = False

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    canMonoTimes = []

    ret_arne182 = arne182.CarStateArne182.new_message()
    buttonEvents = []

    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.canValid = True
   
    events, eventsArne182 = self.create_common_events(ret)

    ret_arne182.events = eventsArne182
    ret.events = events
    ret.buttonEvents = buttonEvents
    ret.canMonoTimes = canMonoTimes

    self.CS.out = ret.as_reader()
    return self.CS.out, ret_arne182.as_reader()


  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators)

    return can_sends
