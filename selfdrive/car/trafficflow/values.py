from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

class CAR:
  TRAFFICFLOW = "TRAFFICFLOW EQUIPPED VEHICLE"

FINGERPRINTS = {
  CAR.TRAFFICFLOW: [
    {2221: 8}
  ]
}


DBC = {
  CAR.TRAFFICFLOW: dbc_dict('trafficflow', None)
}
