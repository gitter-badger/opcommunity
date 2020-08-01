import os
import math

import cereal.messaging as messaging
import cereal.messaging_arne as messaging_arne
from selfdrive.swaglog import cloudlog
from common.realtime import sec_since_boot
from selfdrive.controls.lib.radar_helpers import _LEAD_ACCEL_TAU
from selfdrive.controls.lib.longitudinal_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LONG
from common.op_params import opParams
from common.numpy_fast import interp, clip
from common.travis_checker import travis

LOG_MPC = os.environ.get('LOG_MPC', False)


class LongitudinalMpc():
  def __init__(self, mpc_id):
    self.mpc_id = mpc_id
    self.op_params = opParams()

    self.setup_mpc()
    self.v_mpc = 0.0
    self.v_mpc_future = 0.0
    self.a_mpc = 0.0
    self.v_cruise = 0.0
    self.prev_lead_status = False
    self.prev_lead_x = 0.0
    self.new_lead = False
    self.TR_Mod = 0
    self.last_cloudlog_t = 0.0

    if not travis and mpc_id == 1:
      self.pm = messaging_arne.PubMaster(['smiskolData'])
    else:
      self.pm = None
    self.last_cost = 0.0
    self.df_profile = self.op_params.get('dynamic_follow', 'relaxed').strip().lower()
    self.sng = False

  def send_mpc_solution(self, pm, qp_iterations, calculation_time):
    qp_iterations = max(0, qp_iterations)
    dat = messaging.new_message('liveLongitudinalMpc')
    dat.liveLongitudinalMpc.xEgo = list(self.mpc_solution[0].x_ego)
    dat.liveLongitudinalMpc.vEgo = list(self.mpc_solution[0].v_ego)
    dat.liveLongitudinalMpc.aEgo = list(self.mpc_solution[0].a_ego)
    dat.liveLongitudinalMpc.xLead = list(self.mpc_solution[0].x_l)
    dat.liveLongitudinalMpc.vLead = list(self.mpc_solution[0].v_l)
    dat.liveLongitudinalMpc.cost = self.mpc_solution[0].cost
    dat.liveLongitudinalMpc.aLeadTau = self.a_lead_tau
    dat.liveLongitudinalMpc.qpIterations = qp_iterations
    dat.liveLongitudinalMpc.mpcId = self.mpc_id
    dat.liveLongitudinalMpc.calculationTime = calculation_time
    pm.send('liveLongitudinalMpc', dat)

  def setup_mpc(self):
    ffi, self.libmpc = libmpc_py.get_libmpc(self.mpc_id)
    self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                     MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)

    self.mpc_solution = ffi.new("log_t *")
    self.cur_state = ffi.new("state_t *")
    self.cur_state[0].v_ego = 0
    self.cur_state[0].a_ego = 0
    self.a_lead_tau = _LEAD_ACCEL_TAU

  def set_cur_state(self, v, a):
    self.cur_state[0].v_ego = v
    self.cur_state[0].a_ego = a

  def get_TR(self, CS, lead):
    if not lead.status or travis:
      TR = 1.8
    elif CS.vEgo < 5.0:
      TR = 1.8
    else:
      TR = self.dynamic_follow(CS, lead)

    if not travis:
      self.change_cost(TR,CS.vEgo)
      self.send_cur_TR(TR)
    return TR

  def send_cur_TR(self, TR):
    if self.mpc_id == 1 and self.pm is not None:
      dat = messaging_arne.new_message('smiskolData')
      dat.smiskolData.mpcTR = TR
      self.pm.send('smiskolData', dat)

  def change_cost(self, TR, vEgo):
    TRs = [0.9, 1.8, 2.7]
    costs = [1.0, 0.11, 0.05]
    cost = interp(TR, TRs, costs)
    if self.last_cost != cost:
      self.libmpc.change_tr(MPC_COST_LONG.TTC, cost, MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.last_cost = cost

  def dynamic_follow(self, CS, lead):
    self.df_profile = self.op_params.get('dynamic_follow', 'normal').strip().lower()
    x_vel = [5.0, 15.0]  # velocities
    if self.df_profile == 'far':
      y_dist = [1.8, 2.7]  # TRs
    elif self.df_profile == 'close':  # for in congested traffic
      x_vel = [5.0, 15.0]
      y_dist = [1.8, 0.9]
    else:  # default to normal
      y_dist = [1.8, 1.8]
    TR = interp(CS.vEgo, x_vel, y_dist)

    # Dynamic follow modifications (the secret sauce)
    x = [-5.0, 0.0, 5.0]  # relative velocity values
    y = [0.3, 0.0, -0.3]  # modification values

    self.TR_Mod = interp(lead.vRel, x, y)
    TR += self.TR_Mod

    if CS.leftBlinker or CS.rightBlinker:
      x = [9.0, 55.0]  #
      y = [1.0, 0.65]  # reduce TR when changing lanes
      TR *= interp(CS.vEgo, x, y)

    return clip(TR, 0.9, 2.7)

  def update(self, pm, CS, lead, v_cruise_setpoint):
    v_ego = CS.vEgo
    # Setup current mpc state
    self.cur_state[0].x_ego = 0.0

    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = max(0.0, lead.vLead)
      a_lead = lead.aLeadK

      if (v_lead < 0.1 or -a_lead / 2.0 > v_lead):
        v_lead = 0.0
        a_lead = 0.0
      self.a_lead_tau = lead.aLeadTau
      self.new_lead = False
      if not self.prev_lead_status or abs(x_lead - self.prev_lead_x) > 2.5:
        self.libmpc.init_with_simulation(self.v_mpc, x_lead, v_lead, a_lead, self.a_lead_tau)
        self.new_lead = True

      self.prev_lead_status = True
      self.prev_lead_x = x_lead
      self.cur_state[0].x_l = x_lead
      self.cur_state[0].v_l = v_lead
    else:
      self.prev_lead_status = False
      # Fake a fast lead car, so mpc keeps running
      self.cur_state[0].x_l = 50.0
      self.cur_state[0].v_l = v_ego + 10.0
      a_lead = 0.0
      self.a_lead_tau = _LEAD_ACCEL_TAU

    # Calculate mpc
    t = sec_since_boot()
    n_its = self.libmpc.run_mpc(self.cur_state, self.mpc_solution, self.a_lead_tau, a_lead, self.get_TR(CS, lead))
    duration = int((sec_since_boot() - t) * 1e9)

    if LOG_MPC:
      self.send_mpc_solution(pm, n_its, duration)

    # Get solution. MPC timestep is 0.2 s, so interpolation to 0.05 s is needed
    self.v_mpc = self.mpc_solution[0].v_ego[1]
    self.a_mpc = self.mpc_solution[0].a_ego[1]
    self.v_mpc_future = self.mpc_solution[0].v_ego[10]

    # Reset if NaN or goes through lead car
    crashing = any(lead - ego < -50 for (lead, ego) in zip(self.mpc_solution[0].x_l, self.mpc_solution[0].x_ego))
    nans = any(math.isnan(x) for x in self.mpc_solution[0].v_ego)
    backwards = min(self.mpc_solution[0].v_ego) < -0.01

    if ((backwards or crashing) and self.prev_lead_status) or nans:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Longitudinal mpc %d reset - backwards: %s crashing: %s nan: %s" % (
                          self.mpc_id, backwards, crashing, nans))

      self.libmpc.init(MPC_COST_LONG.TTC, MPC_COST_LONG.DISTANCE,
                       MPC_COST_LONG.ACCELERATION, MPC_COST_LONG.JERK)
      self.cur_state[0].v_ego = v_ego
      self.cur_state[0].a_ego = 0.0
      self.v_mpc = v_ego
      self.a_mpc = CS.aEgo
      self.prev_lead_status = False
