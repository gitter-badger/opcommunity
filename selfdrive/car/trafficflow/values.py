from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class SteerLimitParams:
  STEER_MAX = 1000         # 262 faults
  STEER_DELTA_UP = 1      # 3 is stock. 100 is fine. 200 is too much it seems
  STEER_DELTA_DOWN = 1    # no faults on the way down it seems
  STEER_ERROR_MAX = 1000

class CAR:
  TRAFFICFLOW_EQUIPPED_CAR = "TRAFFICFLOW EQUIPPED"

FINGERPRINTS = {
  CAR.TRAFFICFLOW_EQUIPPED_CAR: [
    { 2222: 8 }
  ]
}


DBC = {
  CAR.TRAFFICFLOW_EQUIPPED_CAR: dbc_dict('trafficflow', None)
}

STEER_THRESHOLD = 120

ECU_FINGERPRINT = {

}
