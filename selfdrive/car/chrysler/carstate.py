from cereal import car
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["GEAR"]['PRNDL']

  def update(self, cp, cp_cam):

    ret = car.CarState.new_message()

    self.frame = int(cp.vl["EPS_STATUS"]['COUNTER'])

    ret.doorOpen = any([cp.vl["DOORS"]['DOOR_OPEN_FL'],
                        cp.vl["DOORS"]['DOOR_OPEN_FR'],
                        cp.vl["DOORS"]['DOOR_OPEN_RL'],
                        cp.vl["DOORS"]['DOOR_OPEN_RR']])
    ret.seatbeltUnlatched = cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER'] == 1 or cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER'] == 2 # 1 or 2 means unbuckled. 

    ret.brakePressed = cp.vl["BRAKE_2"]['BRAKE_PRESSED_2'] == 5 # human-only
    ret.brake = 0
    ret.brakeLights = ret.brakePressed
    ret.gas = cp.vl["ACCEL_GAS_134"]['ACCEL_134']
    ret.gasPressed = ret.gas > 1e-5

    ret.espDisabled = (cp.vl["TRACTION_BUTTON"]['TRACTION_OFF'] == 1)

    ret.wheelSpeeds.fl = cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FL']
    ret.wheelSpeeds.rr = cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RR']
    ret.wheelSpeeds.rl = cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RL']
    ret.wheelSpeeds.fr = cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FR']
    ret.vEgoRaw = (cp.vl['SPEED_1']['SPEED_LEFT'] + cp.vl['SPEED_1']['SPEED_RIGHT']) / 2.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001

    ret.leftBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 1
    ret.rightBlinker = cp.vl["STEERING_LEVERS"]['TURN_SIGNALS'] == 2
    ret.steeringAngle = cp.vl["STEERING"]['STEER_ANGLE'] + cp.vl["STEERING"]['STEER_ANGLE_HIGH_PRECISION']
    ret.steeringRate = cp.vl["STEERING"]['STEERING_RATE']
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl['GEAR']['PRNDL'], None))

    ret.cruiseState.enabled = cp.vl["ACC_2"]['ACC_STATUS_2'] == 7  # ACC is green.
    ret.cruiseState.available = ret.cruiseState.enabled  # FIXME: for now same as enabled
    ret.cruiseState.speed = cp.vl["DASHBOARD"]['ACC_SPEED_CONFIG_KPH'] * CV.KPH_TO_MS

    ret.steeringTorque = cp.vl["EPS_STATUS"]["TORQUE_DRIVER"]
    ret.steeringTorqueEps = cp.vl["EPS_STATUS"]["TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD
    steer_state = cp.vl["EPS_STATUS"]["LKAS_STATE"]
    ret.steerError = steer_state == 4 or (steer_state == 0 and ret.vEgo > self.CP.minSteerSpeed)

    ret.genericToggle = bool(cp.vl["STEERING_LEVERS"]['HIGH_BEAM_FLASH'])

    self.last_tf_control_id = self.tf_control_id
    self.tf_control_id = cp.vl["TF_CONTROL_ANNOUNCEMENT"]["CTRL_ID"]

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = {
      'cancel': cp.vl["WHEEL_BUTTONS"]['ACC_CANCEL'],
      'speed_increase': cp.vl["WHEEL_BUTTONS"]['ACC_SPEED_INC'],
      'speed_decrease': cp.vl["WHEEL_BUTTONS"]['ACC_SPEED_DEC'],
      'resume': cp.vl["WHEEL_BUTTONS"]['ACC_RESUME'],
      'follow_increase': cp.vl["WHEEL_BUTTONS"]['ACC_FOLLOW_INC'],
      'follow_decrease': cp.vl["WHEEL_BUTTONS"]['ACC_FOLLOW_DEC'],
    }
    self.lkas_counter = cp_cam.vl["LKAS_COMMAND"]['COUNTER']
    self.lkas_car_model = cp_cam.vl["LKAS_HUD"]['CAR_MODEL']
    self.lkas_status_ok = cp_cam.vl["LKAS_HEARTBIT"]['LKAS_STATUS_OK']

    return ret

# BO_ 2221 TF_CONTROL_ANNOUNCEMENT: 8 TRAFFICFLOW
#  SG_ LAT_CTRL_STATUS : 0|4@0+ (1,0) [0|0] "" CONTROL_CLIENT
#  SG_ LONG_CTRL_STATUS : 4|4@0+ (1,0) [0|0] "" CONTROL_CLIENT
#  SG_ LAT_CTRL_SUPPORTED_UNITS : 8|2@0+ (1,0) [0|0] "" CONTROL_CLIENT
#  SG_ LONG_POWER_CTRL_SUPPORTED_UNITS : 10|3@0+ (1,0) [0|0] "" CONTROL_CLIENT
#  SG_ LONG_BRAKE_CTRL_SUPPORTED_UNITS : 13|3@0+ (1,0) [0|0] "" CONTROL_CLIENT
#  SG_ LONG_POWER_CTRL_ACCELERATION_MAX : 16|10@0+ (0.9775171065,0) [0.0|1000.0] "m/s^2" CONTROL_CLIENT
#  SG_ LONG_POWER_CTRL_TORQUE_MAX : 26|10@0+ (0.9775171065,0) [0.0|1000.0] "Nm" CONTROL_CLIENT
#  SG_ LONG_BRAKE_CTRL_ACCELERATION_MAX : 36|10@0+ (0.9775171065,0) [0.0|1000.0] "m/s^2" CONTROL_CLIENT
#  SG_ LONG_BRAKE_CTRL_TORQUE_MAX : 46|10@0+ (0.5865102639,0) [0.0|1000.0] "Nm" CONTROL_CLIENT
#  SG_ CTRL_ID : 58|8@0+ (1,0) [0|255] "" CONTROL_CLIENT
  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      {"TF_CONTROL_ANNOUNCEMENT", "LONG_CTRL_STATUS", 0},
      {"TF_CONTROL_ANNOUNCEMENT", "CTRL_ID", 0},
      {"WHEEL_BUTTONS", "ACC_CANCEL", 0},
      {"WHEEL_BUTTONS", "ACC_SPEED_INC", 0},
      {"WHEEL_BUTTONS", "ACC_SPEED_DEC", 0},
      {"WHEEL_BUTTONS", "ACC_FOLLOW_INC", 0},
      {"WHEEL_BUTTONS", "ACC_RESUME", 0},
      {"WHEEL_BUTTONS", "ACC_FOLLOW_DEC", 0},
      ("PRNDL", "GEAR", 0),
      ("DOOR_OPEN_FL", "DOORS", 0),
      ("DOOR_OPEN_FR", "DOORS", 0),
      ("DOOR_OPEN_RL", "DOORS", 0),
      ("DOOR_OPEN_RR", "DOORS", 0),
      ("BRAKE_PRESSED_2", "BRAKE_2", 0),
      ("ACCEL_134", "ACCEL_GAS_134", 0),
      ("SPEED_LEFT", "SPEED_1", 0),
      ("SPEED_RIGHT", "SPEED_1", 0),
      ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
      ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
      ("STEER_ANGLE", "STEERING", 0),
      ("STEER_ANGLE_HIGH_PRECISION", "STEERING", 0),
      ("STEERING_RATE", "STEERING", 0),
      ("TURN_SIGNALS", "STEERING_LEVERS", 0),
      ("ACC_STATUS_2", "ACC_2", 0),
      ("HIGH_BEAM_FLASH", "STEERING_LEVERS", 0),
      ("ACC_SPEED_CONFIG_KPH", "DASHBOARD", 0),
      ("TORQUE_DRIVER", "EPS_STATUS", 0),
      ("TORQUE_MOTOR", "EPS_STATUS", 0),
      ("LKAS_STATE", "EPS_STATUS", 1),
      ("COUNTER", "EPS_STATUS", -1),
      ("TRACTION_OFF", "TRACTION_BUTTON", 0),
      ("SEATBELT_DRIVER", "SEATBELT_STATUS", 0),
    ]

    checks = [
      # sig_address, frequency
      ("BRAKE_2", 50),
      ("EPS_STATUS", 100),
      ("SPEED_1", 100),
      ("WHEEL_SPEEDS", 50),
      ("STEERING", 100),
      ("ACC_2", 50),
    ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("COUNTER", "LKAS_COMMAND", -1),
      ("CAR_MODEL", "LKAS_HUD", -1),
      ("LKAS_STATUS_OK", "LKAS_HEARTBIT", -1)
    ]
    checks = []

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)
