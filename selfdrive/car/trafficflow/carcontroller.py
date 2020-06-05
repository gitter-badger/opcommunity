from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.trafficflow.values import CAR
from opendbc.can.packer import CANPacker

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.packer = CANPacker(dbc_name)


  def update(self, enabled, CS, actuators):
    can_sends = []

    return can_sends
