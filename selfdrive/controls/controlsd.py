#!/usr/bin/env python3
import os
import gc
from cereal import car, log
from common.android import ANDROID, get_sound_card_online
from common.numpy_fast import clip
from common.realtime import sec_since_boot, set_realtime_priority, set_core_affinity, Ratekeeper, DT_CTRL
from common.profiler import Profiler
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
import cereal.messaging_arne as messaging_arne
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.longcontrol import LongControl, STARTING_TARGET_SPEED
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.events import Events, Events_arne182, ET
from selfdrive.controls.lib.alertmanager import AlertManager
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.planner import LON_MPC_STEP
from selfdrive.locationd.calibration_helpers import Calibration

from selfdrive.car.disable_radar import disable_radar
from common.op_params import opParams
from selfdrive.controls.df_alert_manager import DfAlertManager

op_params = opParams()

traffic_light_alerts = op_params.get('traffic_light_alerts', True)

#LANE_DEPARTURE_THRESHOLD = 0.1
STEER_ANGLE_SATURATION_TIMEOUT = 1.0 / DT_CTRL
STEER_ANGLE_SATURATION_THRESHOLD = 2.75  # Degrees

ThermalStatus = log.ThermalData.ThermalStatus
State = log.ControlsState.OpenpilotState
HwType = log.HealthData.HwType
LongitudinalPlanSource = log.Plan.LongitudinalPlanSource
Desire = log.PathPlan.Desire
LaneChangeState = log.PathPlan.LaneChangeState
LaneChangeDirection = log.PathPlan.LaneChangeDirection
EventName = car.CarEvent.EventName


class Controls:
  def __init__(self, sm=None, pm=None, can_sock=None, arne_sm=None):
    gc.disable()
    set_realtime_priority(53)
    set_core_affinity(3)

    # Setup sockets
    self.pm = pm
    if self.pm is None:
      self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                     'carControl', 'carEvents', 'carParams'])

    self.sm = sm
    if self.sm is None:
      self.sm = messaging.SubMaster(['thermal', 'health', 'frame', 'model', 'liveCalibration',
                                     'dMonitoringState', 'plan', 'pathPlan', 'liveLocationKalman', 'radarState'])
    self.arne_sm = arne_sm
    if self.arne_sm is None:
      self.arne_sm = messaging_arne.SubMaster(['arne182Status', 'dynamicFollowButton', 'trafficModelEvent', 'modelLongButton' ])

    self.op_params = opParams()
    self.df_manager = dfManager(self.op_params)
    self.hide_auto_df_alerts = self.op_params.get('hide_auto_df_alerts', False)
    self.traffic_light_alerts = self.op_params.get('traffic_light_alerts', True)
    self.last_model_long = False

    self.can_sock = can_sock
    if can_sock is None:
      can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
      self.can_sock = messaging.sub_sock('can', timeout=can_timeout)

    # wait for one health and one CAN packet
    hw_type = messaging.recv_one(self.sm.sock['health']).health.hwType
    has_relay = hw_type in [HwType.blackPanda, HwType.uno, HwType.dos]
    print("Waiting for CAN messages...")
    messaging.get_one_can(self.can_sock)

    self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], has_relay)

    # read params
    params = Params()
    self.is_metric = params.get("IsMetric", encoding='utf8') == "1"
    self.is_ldw_enabled = params.get("IsLdwEnabled", encoding='utf8') == "1"
    internet_needed = (params.get("Offroad_ConnectivityNeeded", encoding='utf8') is not None) and (params.get("DisableUpdates") != b"1")
    community_feature_toggle = params.get("CommunityFeaturesToggle", encoding='utf8') == "1"
    openpilot_enabled_toggle = params.get("OpenpilotEnabledToggle", encoding='utf8') == "1"
    passive = params.get("Passive", encoding='utf8') == "1" or \
              internet_needed or not openpilot_enabled_toggle

    # detect sound card presence and ensure successful init
    sounds_available = not ANDROID or get_sound_card_online()

    car_recognized = self.CP.carName != 'mock'
    # If stock camera is disconnected, we loaded car controls and it's not dashcam mode
    controller_available = self.CP.enableCamera and self.CI.CC is not None and not passive
    community_feature_disallowed = self.CP.communityFeature and not community_feature_toggle
    self.read_only = not car_recognized or not controller_available or \
                       self.CP.dashcamOnly or community_feature_disallowed
    if self.read_only:
      self.CP.safetyModel = car.CarParams.SafetyModel.noOutput

    # Write CarParams for radard and boardd safety mode
    cp_bytes = self.CP.to_bytes()
    params.put("CarParams", cp_bytes)
    put_nonblocking("CarParamsCache", cp_bytes)
    put_nonblocking("LongitudinalControl", "1" if self.CP.openpilotLongitudinalControl else "0")
    if self.CP.openpilotLongitudinalControl and self.CP.safetyModel in [car.CarParams.SafetyModel.hondaBoschGiraffe, car.CarParams.SafetyModel.hondaBoschHarness]:
      disable_radar(can_sock, pm.sock['sendcan'], 1 if has_relay else 0, timeout=1, retry=10)

    self.CC = car.CarControl.new_message()
    self.AM = AlertManager()
    self.events = Events()
    self.eventsArne182 = Events_arne182()

    self.LoC = LongControl(self.CP, self.CI.compute_gb)
    self.VM = VehicleModel(self.CP)

    if self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP)
    elif self.CP.lateralTuning.which() == 'indi':
      self.LaC = LatControlINDI(self.CP)
    elif self.CP.lateralTuning.which() == 'lqr':
      self.LaC = LatControlLQR(self.CP)

    self.state = State.disabled
    self.enabled = False
    self.active = False
    self.can_rcv_error = False
    self.soft_disable_timer = 0
    self.v_cruise_kph = 255
    self.v_cruise_kph_last = 0
    self.mismatch_counter = 0
    self.can_error_counter = 0
    self.last_blinker_frame = 0
    self.last_ldw_frame = 0
    self.saturated_count = 0
    self.distance_traveled_now = 0
    if not travis:
      self.distance_traveled = float(params.get("DistanceTraveled", encoding='utf8'))
      self.distance_traveled_engaged = float(params.get("DistanceTraveledEngaged", encoding='utf8'))
      self.distance_traveled_override = float(params.get("DistanceTraveledOverride", encoding='utf8'))
    else:
      self.distance_traveled = 0
      self.distance_traveled_engaged = 0
      self.distance_traveled_override = 0

    self.distance_traveled_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]

    self.sm['liveCalibration'].calStatus = Calibration.INVALID
    self.sm['thermal'].freeSpace = 1.
    self.sm['dMonitoringState'].events = []
    self.sm['dMonitoringState'].awarenessStatus = 1.
    self.sm['dMonitoringState'].faceDetected = False

  overtemp = sm['thermal'].thermalStatus >= ThermalStatus.red
  free_space = sm['thermal'].freeSpace < 0.07  # under 7% of space free no enable allowed
  low_battery = sm['thermal'].batteryPercent < 1 and sm['thermal'].chargingError  # at zero percent battery, while discharging, OP should not allowed
  mem_low = sm['thermal'].memUsedPercent > 90

  # Create events for battery, temperature and disk space
  if low_battery:
    events.append(create_event('lowBattery', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
  if overtemp:
    events.append(create_event('overheat', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
  if free_space:
    events.append(create_event('outOfSpace', [ET.NO_ENTRY]))
  if mem_low:
    events.append(create_event('lowMemory', [ET.NO_ENTRY, ET.SOFT_DISABLE, ET.PERMANENT]))

  if CS.stockAeb:
    events.append(create_event('stockAeb', []))

  # Handle calibration
  cal_status = sm['liveCalibration'].calStatus
  cal_perc = sm['liveCalibration'].calPerc

  if cal_status != Calibration.CALIBRATED:
    if cal_status == Calibration.UNCALIBRATED:
      events.append(create_event('calibrationIncomplete', [ET.NO_ENTRY, ET.SOFT_DISABLE, ET.PERMANENT]))
    else:
      events.append(create_event('calibrationInvalid', [ET.NO_ENTRY, ET.SOFT_DISABLE]))

  if CS.vEgo > 92 * CV.MPH_TO_MS:
    events.append(create_event('speedTooHigh', [ET.NO_ENTRY, ET.SOFT_DISABLE]))

  # When the panda and controlsd do not agree on controls_allowed
  # we want to disengage openpilot. However the status from the panda goes through
  # another socket other than the CAN messages and one can arrive earlier than the other.
  # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
  if not enabled:
    mismatch_counter = 0

  controls_allowed = sm['health'].controlsAllowed
  if not controls_allowed and enabled:
    mismatch_counter += 1
  if mismatch_counter >= 200:
    events.append(create_event('controlsMismatch', [ET.IMMEDIATE_DISABLE]))


  return CS, events, cal_perc, mismatch_counter, can_error_counter, events_arne182


def state_transition(frame, CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM, events_arne182, arne_sm, df_alert_manager):
  """Compute conditional state transitions and execute actions on state transitions"""
  enabled = isEnabled(state)

  v_cruise_kph_last = v_cruise_kph

  # if stock cruise is completely disabled, then we can use our own set speed logic
  if not CP.enableCruise:
    v_cruise_kph = update_v_cruise(v_cruise_kph, CS.buttonEvents, enabled)
  elif CP.enableCruise and CS.cruiseState.enabled:
    v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH

  # decrease the soft disable timer at every step, as it's reset on
  # entrance in SOFT_DISABLING state
  soft_disable_timer = max(0, soft_disable_timer - 1)
  if traffic_light_alerts:
    traffic_status = arne_sm['trafficModelEvent'].status
    traffic_confidence = round(arne_sm['trafficModelEvent'].confidence * 100, 2)
    if traffic_confidence >= 75:
      if traffic_status == 'SLOW':
        AM.add(frame, 'trafficSlow', enabled, extra_text_2=' ({}%)'.format(traffic_confidence))
      elif traffic_status == 'GREEN':
        AM.add(frame, 'trafficGreen', enabled, extra_text_2=' ({}%)'.format(traffic_confidence))
      elif traffic_status == 'DEAD':  # confidence will be 100
        AM.add(frame, 'trafficDead', enabled)

  df_alert = df_alert_manager.update(arne_sm)
  if df_alert is not None:
    AM.add(frame, 'dfButtonAlert', enabled, extra_text_1=df_alert, extra_text_2='Dynamic follow: {} profile active'.format(df_alert))

  # DISABLED
  if state == State.disabled:
    if get_events(events, [ET.ENABLE]):
      if get_events(events, [ET.NO_ENTRY]):
        for e in get_events(events, [ET.NO_ENTRY]):
          AM.add(frame, str(e) + "NoEntry", enabled)

    if not sounds_available:
      self.events.add(EventName.soundsUnavailable, static=True)
    if internet_needed:
      self.events.add(EventName.internetConnectivityNeeded, static=True)
    if community_feature_disallowed:
      self.events.add(EventName.communityFeatureDisallowed, static=True)
    if self.read_only and not passive:
      self.events.add(EventName.carUnrecognized, static=True)
    if hw_type == HwType.whitePanda:
      self.events.add(EventName.whitePandaUnsupported, static=True)

    # controlsd is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)
    self.prof = Profiler(False)  # off by default

  def update_events(self, CS, CS_arne182):
    """Compute carEvents from carState"""

    self.events.clear()
    self.eventsArne182.clear()
    self.events.add_from_msg(CS.events)
    self.eventsArne182.add_from_msg(CS_arne182.events)
    self.events.add_from_msg(self.sm['dMonitoringState'].events)

    # Handle startup event
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    # Create events for battery, temperature, disk space, and memory
    if self.sm['thermal'].batteryPercent < 1 and self.sm['thermal'].chargingError:
      # at zero percent battery, while discharging, OP should not allowed
      self.events.add(EventName.lowBattery)
    if self.sm['thermal'].thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
    if self.sm['thermal'].freeSpace < 0.07:
      # under 7% of space free no enable allowed
      self.events.add(EventName.outOfSpace)
    if self.sm['thermal'].memUsedPercent > 90:
      self.events.add(EventName.lowMemory)

    # Handle calibration status
    cal_status = self.sm['liveCalibration'].calStatus
    if cal_status != Calibration.CALIBRATED:
      if cal_status == Calibration.UNCALIBRATED:
        self.events.add(EventName.calibrationIncomplete)
      else:
        self.events.add(EventName.calibrationInvalid)

    # Handle lane change
    if self.sm['pathPlan'].laneChangeState == LaneChangeState.preLaneChange:
      direction = self.sm['pathPlan'].laneChangeDirection
      if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
         (CS.rightBlindspot and direction == LaneChangeDirection.right):
        self.events.add(EventName.laneChangeBlocked)
      else:
        if direction == LaneChangeDirection.left:
          self.events.add(EventName.preLaneChangeLeft)
        else:
          self.events.add(EventName.preLaneChangeRight)
    elif self.sm['pathPlan'].laneChangeState in [LaneChangeState.laneChangeStarting,
                                        LaneChangeState.laneChangeFinishing]:
      self.events.add(EventName.laneChange)

    if self.can_rcv_error or (not CS.canValid and self.sm.frame > 5 / DT_CTRL):
      self.events.add(EventName.canError)
    if self.mismatch_counter >= 200:
      self.events.add(EventName.controlsMismatch)
    if not self.sm.alive['plan'] and self.sm.alive['pathPlan']:
      # only plan not being received: radar not communicating
      self.events.add(EventName.radarCommIssue)
    elif not self.sm.all_alive_and_valid() and self.sm.frame > 5 / DT_CTRL:
      self.events.add(EventName.commIssue)
    if not self.sm['pathPlan'].mpcSolutionValid and self.sm.frame > 5 / DT_CTRL:
      self.events.add(EventName.plannerError)
    if not self.sm['liveLocationKalman'].sensorsOK and os.getenv("NOSENSOR") is None:
      if self.sm.frame > 5 / DT_CTRL:  # Give locationd some time to receive all the inputs
        self.events.add(EventName.sensorDataInvalid)
    if not self.sm['liveLocationKalman'].gpsOK and (self.distance_traveled_now > 1000) and os.getenv("NOSENSOR") is None:
      # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
      self.events.add(EventName.noGps)
    if not self.sm['pathPlan'].paramsValid:
      self.events.add(EventName.vehicleModelInvalid)
    if not self.sm['liveLocationKalman'].posenetOK:
      self.events.add(EventName.posenetInvalid)
    if not self.sm['frame'].recoverState < 2:
      # counter>=2 is active
      self.events.add(EventName.focusRecoverActive)
    if not self.sm['plan'].radarValid:
      self.events.add(EventName.radarFault)
    if self.sm['plan'].radarCanError:
      self.events.add(EventName.radarCanError)
    if log.HealthData.FaultType.relayMalfunction in self.sm['health'].faults:
      self.events.add(EventName.relayMalfunction)
    if self.sm['plan'].fcw:
      self.events.add(EventName.fcw)
    #if self.sm['model'].frameDropPerc > 1:
    #  self.events.add(EventName.modeldLagging)

    # Only allow engagement with brake pressed when stopped behind another stopped car
    if CS.brakePressed and self.sm['plan'].vTargetFuture >= STARTING_TARGET_SPEED \
       and self.CP.openpilotLongitudinalControl and CS.vEgo < 0.3:
      self.events.add(EventName.noTarget)

  def data_sample(self):
    """Receive data from sockets and update carState"""

    # Update carState from CAN
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    CS, CS_arne182 = self.CI.update(self.CC, can_strs)

    self.sm.update(0)

    # Check for CAN timeout
    if not can_strs:
      self.can_error_counter += 1
      self.can_rcv_error = True
    else:
      self.can_rcv_error = False

    # When the panda and controlsd do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    if not self.sm['health'].controlsAllowed and self.enabled:
      self.mismatch_counter += 1
    self.distance_traveled_now += CS.vEgo * DT_CTRL
    self.distance_traveled += CS.vEgo * DT_CTRL
    if self.enabled:
      self.distance_traveled_engaged += CS.vEgo * DT_CTRL
      if CS.steeringPressed:
        self.distance_traveled_override += CS.vEgo * DT_CTRL
    if (self.sm.frame - self.distance_traveled_frame) * DT_CTRL > 10.0 and not travis:
      y = threading.Thread(target=send_params, args=(str(self.distance_traveled),str(self.distance_traveled_engaged),str(self.distance_traveled_override),))
      y.start()
      self.distance_traveled_frame = self.sm.frame
    return CS, CS_arne182

  def state_transition(self, CS):
    """Compute conditional state transitions and execute actions on state transitions"""

    self.v_cruise_kph_last = self.v_cruise_kph

    # if stock cruise is completely disabled, then we can use our own set speed logic
    if not self.CP.enableCruise:
      self.v_cruise_kph = update_v_cruise(self.v_cruise_kph, CS.buttonEvents, self.enabled)
    elif self.CP.enableCruise and CS.cruiseState.enabled:
      self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH

    # decrease the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

    self.current_alert_types = [ET.PERMANENT]

    # ENABLED, PRE ENABLING, SOFT DISABLING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if self.events.any(ET.USER_DISABLE) or self.eventsArne182.any(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)

      elif self.events.any(ET.IMMEDIATE_DISABLE) or self.eventsArne182.any(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if self.events.any(ET.SOFT_DISABLE) or self.eventsArne182.any(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = 300   # 3s
            self.current_alert_types.append(ET.SOFT_DISABLE)

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if not (self.events.any(ET.SOFT_DISABLE) or self.eventsArne182.any(ET.SOFT_DISABLE)):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif (self.events.any(ET.SOFT_DISABLE) or self.eventsArne182.any(ET.SOFT_DISABLE)) and self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.soft_disable_timer <= 0:
            self.state = State.disabled

        # PRE ENABLING
        elif self.state == State.preEnabled:
          if not (self.events.any(ET.PRE_ENABLE) or self.eventsArne182.any(ET.PRE_ENABLE)):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)

    # DISABLED
    elif self.state == State.disabled:
      if self.events.any(ET.ENABLE) or self.eventsArne182.any(ET.ENABLE):
        if self.events.any(ET.NO_ENTRY) or self.eventsArne182.any(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)

        else:
          if self.events.any(ET.PRE_ENABLE) or self.eventsArne182.any(ET.PRE_ENABLE):
            self.state = State.preEnabled
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)
          self.v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, self.v_cruise_kph_last)

    # Check if actuators are enabled
    self.active = self.state == State.enabled or self.state == State.softDisabling
    if self.active:
      self.current_alert_types.append(ET.WARNING)

    # Check if openpilot is engaged
    self.enabled = self.active or self.state == State.preEnabled

  def state_control(self, CS):
    """Given the state, this function returns an actuators packet"""

    plan = self.sm['plan']
    path_plan = self.sm['pathPlan']

    actuators = car.CarControl.Actuators.new_message()

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame

    # State specific actions

    if not self.active:
      self.LaC.reset()
      self.LoC.reset(v_pid=plan.vTargetFuture)

    plan_age = DT_CTRL * (self.sm.frame - self.sm.rcv_frame['plan'])
    # no greater than dt mpc + dt, to prevent too high extraps
    dt = min(plan_age, LON_MPC_STEP + DT_CTRL) + DT_CTRL

    a_acc_sol = plan.aStart + (dt / LON_MPC_STEP) * (plan.aTarget - plan.aStart)
    v_acc_sol = plan.vStart + dt * (a_acc_sol + plan.aStart) / 2.0

    # Gas/Brake PID loop
    if self.arne_sm.updated['arne182Status']:
      gas_button_status = self.arne_sm['arne182Status'].gasbuttonstatus
    else:
      gas_button_status = 0

    actuators.gas, actuators.brake = self.LoC.update(self.active, CS, v_acc_sol, plan.vTargetFuture, a_acc_sol, self.CP, plan.hasLead, self.sm['radarState'], plan.decelForTurn, plan.longitudinalPlanSource, gas_button_status)
    # Steering PID loop and lateral MPC
    actuators.steer, actuators.steerAngle, lac_log = self.LaC.update(self.active, CS, self.CP, path_plan)

    # Check for difference between desired angle and angle for angle based control
    angle_control_saturated = self.CP.steerControlType == car.CarParams.SteerControlType.angle and \
      abs(actuators.steerAngle - CS.steeringAngle) > STEER_ANGLE_SATURATION_THRESHOLD

  is_metric = params.get("IsMetric", encoding='utf8') == "1"
  is_ldw_enabled = params.get("IsLdwEnabled", encoding='utf8') == "1"
  passive = params.get("Passive", encoding='utf8') == "1"
  openpilot_enabled_toggle = params.get("OpenpilotEnabledToggle", encoding='utf8') == "1"
  community_feature_toggle = params.get("CommunityFeaturesToggle", encoding='utf8') == "1"

  passive = passive or not openpilot_enabled_toggle

  # Passive if internet needed
  internet_needed = params.get("Offroad_ConnectivityNeeded", encoding='utf8') is not None
  passive = passive or internet_needed

  # Pub/Sub Sockets
  if pm is None:
    pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState', 'carControl', 'carEvents', 'carParams'])

  if sm is None:
    sm = messaging.SubMaster(['thermal', 'health', 'liveCalibration', 'dMonitoringState', 'plan', 'pathPlan', \
                              'model', 'gpsLocation', 'radarState'], ignore_alive=['gpsLocation'])

  if arne_sm is None:
    arne_sm = messaging_arne.SubMaster(['arne182Status', 'dynamicFollowButton', 'trafficModelEvent'])

  if can_sock is None:
    can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
    can_sock = messaging.sub_sock('can', timeout=can_timeout)

  # wait for health and CAN packets
  hw_type = messaging.recv_one(sm.sock['health']).health.hwType
  has_relay = hw_type in [HwType.blackPanda, HwType.uno]
  print("Waiting for CAN messages...")
  messaging.get_one_can(can_sock)

  CI, CP = get_car(can_sock, pm.sock['sendcan'], has_relay)

  car_recognized = CP.carName != 'mock'
  # If stock camera is disconnected, we loaded car controls and it's not chffrplus
  controller_available = CP.enableCamera and CI.CC is not None and not passive
  community_feature_disallowed = CP.communityFeature and not community_feature_toggle
  read_only = not car_recognized or not controller_available or CP.dashcamOnly or community_feature_disallowed
  if read_only:
    CP.safetyModel = car.CarParams.SafetyModel.noOutput

  # Write CarParams for radard and boardd safety mode
  cp_bytes = CP.to_bytes()
  params.put("CarParams", cp_bytes)
  put_nonblocking("CarParamsCache", cp_bytes)
  put_nonblocking("LongitudinalControl", "1" if CP.openpilotLongitudinalControl else "0")
  if CP.openpilotLongitudinalControl and CP.safetyModel in [car.CarParams.SafetyModel.hondaBoschGiraffe, car.CarParams.SafetyModel.hondaBoschHarness]:
    disable_radar(can_sock, pm.sock['sendcan'], 1 if has_relay else 0, timeout=1, retry=10)

  CC = car.CarControl.new_message()
  AM = AlertManager()

  startup_alert = get_startup_alert(car_recognized, controller_available)
  AM.add(sm.frame, startup_alert, False)

  LoC = LongControl(CP, CI.compute_gb)
  VM = VehicleModel(CP)

  if CP.lateralTuning.which() == 'pid':
    LaC = LatControlPID(CP)
  elif CP.lateralTuning.which() == 'indi':
    LaC = LatControlINDI(CP)
  elif CP.lateralTuning.which() == 'lqr':
    LaC = LatControlLQR(CP)

  state = State.disabled
  soft_disable_timer = 0
  v_cruise_kph = 255
  v_cruise_kph_last = 0
  mismatch_counter = 0
  can_error_counter = 0
  last_blinker_frame = 0
  saturated_count = 0
  events_prev = []

  sm['liveCalibration'].calStatus = Calibration.INVALID
  sm['pathPlan'].sensorValid = True
  sm['pathPlan'].posenetValid = True
  sm['thermal'].freeSpace = 1.
  sm['dMonitoringState'].events = []
  sm['dMonitoringState'].awarenessStatus = 1.
  sm['dMonitoringState'].faceDetected = False

  # detect sound card presence
  sounds_available = not os.path.isfile('/EON') or (os.path.isdir('/proc/asound/card0') and open('/proc/asound/card0/state').read().strip() == 'ONLINE')

  # controlsd is driven by can recv, expected at 100Hz
  rk = Ratekeeper(100, print_delay_threshold=None)


  prof = Profiler(False)  # off by default
  df_alert_manager = DfAlertManager(op_params)

  while True:
    start_time = sec_since_boot()
    self.prof.checkpoint("Ratekeeper", ignore=True)

    # Sample data from sockets and get a carState
    CS, CS_arne182 = self.data_sample()
    self.prof.checkpoint("Sample")

    self.update_events(CS, CS_arne182)

    if not read_only:
      # update control state
      state, soft_disable_timer, v_cruise_kph, v_cruise_kph_last = \
        state_transition(sm.frame, CS, CP, state, events, soft_disable_timer, v_cruise_kph, AM, events_arne182, arne_sm, df_alert_manager)
      prof.checkpoint("State transition")

    # Compute actuators (runs PID loops and lateral MPC)
    actuators, v_acc, a_acc, lac_log = self.state_control(CS)

    self.prof.checkpoint("State Control")

    # Publish data
    self.publish_logs(CS, start_time, actuators, v_acc, a_acc, lac_log, CS_arne182)
    self.prof.checkpoint("Sent")

  def controlsd_thread(self):
    while True:
      self.step()
      self.rk.monitor_time()
      self.prof.display()
def send_params(a, b, c):
  params = Params()
  params.put("DistanceTraveled", a)
  params.put("DistanceTraveledEngaged", b)
  params.put("DistanceTraveledOverride", c)
      
def main(sm=None, pm=None, logcan=None, arne_sm=None):
  #params = Params()
  #dongle_id = params.get("DongleId").decode('utf-8')
  #cloudlog.bind_global(dongle_id=dongle_id, version=version, dirty=dirty, is_eon=True)
  #crash.bind_user(id=dongle_id)
  #crash.bind_extra(version=version, dirty=dirty, is_eon=True)
  #crash.install()
  controls = Controls(sm, pm, logcan, arne_sm)
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
