#!/usr/bin/env python3
import os
import json
import time
import string
import random
from common.travis_checker import travis


def write_params(params, params_file):
  if not travis:
    with open(params_file, "w") as f:
      json.dump(params, f, indent=2, sort_keys=True)
    os.chmod(params_file, 0o764)


def read_params(params_file, default_params):
  try:
    with open(params_file, "r") as f:
      params = json.load(f)
    return params, True
  except Exception as e:
    print(e)
    params = default_params
    return params, False


class KeyInfo:
  has_allowed_types = False
  live = False
  has_default = False
  has_description = False


class opParams:
  def __init__(self):
    """
      To add your own parameter to opParams in your fork, simply add a new dictionary entry with the name of your parameter and its default value to save to new users' op_params.json file.
      The description, allowed_types, and live keys are no longer required but recommended to help users edit their parameters with opEdit and opTune correctly.
        - The description value will be shown to users when they use opEdit or opTune to change the value of the parameter.
        - The allowed_types key is used to restrict what kinds of values can be entered with opEdit so that users can't reasonably break the fork with unintended behavior.
          Limiting the range of floats or integers is still recommended when `.get`ting the parameter.
          When a None value is allowed, use `type(None)` instead of None, as opEdit checks the type against the values in the key with `isinstance()`.
        - Finally, the live key tells both opParams and opTune that it's a live parameter that will change. Therefore, you must place the `op_params.get()` call in the update function so that it can update.
      Here's an example of the minimum required dictionary:

      self.default_params = {'camera_offset': {'default': 0.06}}
    """

    self.default_params = {'awareness_factor': {'default': 10., 'allowed_types': [float, int], 'description': 'Multiplier for the awareness times', 'live': False},
                           'alca_min_speed': {'default': 20, 'allowed_types': [float, int], 'description': 'Speed limit to start ALC in MPH', 'live': False},
                           'alca_nudge_required': {'default': False, 'allowed_types': [bool], 'description': "Require nudge to start ALC", 'live': False},
                           'camera_offset': {'default': 0.06, 'allowed_types': [float, int], 'description': 'Your camera offset to use in lane_planner.py', 'live': True},
                           'curvature_factor': {'default': 1.2, 'allowed_types': [float, int], 'description': 'Multiplier for the curvature slowdown. Increase for less braking.', 'live': False},
                           'cloak': {'default': True, 'allowed_types': [bool], 'description': "make comma believe you are on their fork", 'live': False},
                           'default_brake_distance': {'default': 250.0, 'allowed_types': [float, int], 'description': 'Distance in m to start braking for mapped speeds.', 'live': False},
                           'dynamic_follow': {'default': 'normal', 'allowed_types': [str],
                                              'description': "Can be: ('close', 'normal', 'far'): Left to right increases in following distance.\n"
                                                             "All profiles support dynamic follow so you'll get your preferred distance while\n"
                                                             "retaining the smoothness and safety of dynamic follow!", 'live': True},
                           'force_pedal': {'default': False, 'allowed_types': [bool], 'description': "If openpilot isn't recognizing your comma pedal, set this to True", 'live': False},
                           'global_df_mod': {'default': None, 'allowed_types': [type(None), float, int], 'description': 'The multiplier for the current distance used by dynamic follow. The range is limited from 0.85 to 1.2\n'
                                                                                                                        'Smaller values will get you closer, larger will get you farther\n'
                                                                                                                        'This is multiplied by any profile that\'s active. Set to None to disable', 'live': True},
                           'hide_auto_df_alerts': {'default': True, 'allowed_types': [bool], 'description': 'Hides the alert that shows what profile the model has chosen'},
                           'keep_openpilot_engaged': {'default': True, 'allowed_types': [bool],
                                                      'description': 'True is stock behavior in this fork. False lets you use the brake and cruise control stalk to disengage as usual', 'live': False},
                           'limit_rsa': {'default': False, 'allowed_types': [bool], 'description': "Switch off RSA above rsa_max_speed", 'live': False},
                           'mpc_offset': {'default': 5.0, 'allowed_types': [float, int], 'description': 'Offset model braking by how many m/s. Lower numbers equals more model braking', 'live': True},
                           'offset_limit': {'default': 0, 'allowed_types': [float, int], 'description': 'Speed at which apk percent offset will work in m/s', 'live': False},
                           'osm': {'default': True, 'allowed_types': [bool], 'description': 'Whether to use OSM for drives', 'live': False},
                           'op_edit_live_mode': {'default': False, 'allowed_types': [bool], 'description': 'This parameter controls which mode opEdit starts in. It should be hidden from the user with the hide key', 'hide': True},
                           'rolling_stop': {'default': False, 'allowed_types': [bool], 'description': 'If you do not want stop signs to go down to 0 kph enable this for 9kph slow down', 'live': False},
                           'rsa_max_speed': {'default': 24.5, 'allowed_types': [float, int], 'description': 'Speed limit to ignore RSA in m/s', 'live': False},
                           'smart_speed': {'default': True, 'allowed_types': [bool], 'description': 'Whether to use Smart Speed for drives above smart_speed_max_vego', 'live': False},
                           'smart_speed_max_vego': {'default': 26.8, 'allowed_types': [float, int], 'description': 'Speed limit to ignore Smartspeed in m/s', 'live': False},
                           'spairrowtuning': {'default': False, 'allowed_types': [bool], 'description': 'Better Tuning for Corolla', 'live': False},
                           'speed_offset': {'default': 0, 'allowed_types': [float, int], 'description': 'Speed limit offset in m/s', 'live': True},
                           'traffic_light_alerts': {'default': False, 'allowed_types': [bool], 'description': "Switch off the traffic light alerts", 'live': False},
                           'traffic_lights': {'default': True, 'allowed_types': [bool], 'description': "Should Openpilot stop for traffic lights", 'live': False},
                           'traffic_lights_without_direction': {'default': False, 'allowed_types': [bool], 'description': "Should Openpilot stop for traffic lights without a direction specified", 'live': False},
                           'use_car_caching': {'default': True, 'allowed_types': [bool], 'description': 'Whether to use fingerprint caching', 'live': False}
                           }

    self.params = {}
    self.params_file = "/data/op_params.json"
    self.kegman_file = "/data/kegman.json"
    self.last_read_time = time.time()
    self.read_frequency = 5.0  # max frequency to read with self.get(...) (sec)
    self.force_update = False  # replaces values with default params if True, not just add add missing key/value pairs
    self.to_delete = ['dynamic_lane_speed', 'longkiV', 'following_distance', 'static_steer_ratio']
    self.run_init()  # restores, reads, and updates params

  def create_id(self):  # creates unique identifier to send with sentry errors. please update uniqueID with op_edit.py to your username!
    need_id = False
    if "uniqueID" not in self.params:
      need_id = True
    if "uniqueID" in self.params and self.params["uniqueID"] is None:
      need_id = True
    if need_id:
      random_id = ''.join([random.choice(string.ascii_lowercase + string.ascii_uppercase + string.digits) for i in range(15)])
      self.params["uniqueID"] = random_id

  def add_default_params(self):
    prev_params = dict(self.params)
    if not travis:
      self.create_id()
      for key in self.default_params:
        if self.force_update:
          self.params[key] = self.default_params[key]['default']
        elif key not in self.params:
          self.params[key] = self.default_params[key]['default']
    return prev_params == self.params

  def format_default_params(self):
    return {key: self.default_params[key]['default'] for key in self.default_params}

  def run_init(self):  # does first time initializing of default params, and/or restoring from kegman.json
    if travis:
      self.params = self.format_default_params()
      return
    self.params = self.format_default_params()  # in case any file is corrupted
    to_write = False
    no_params = False
    if os.path.isfile(self.params_file):
      self.params, read_status = read_params(self.params_file, self.format_default_params())
      if read_status:
        to_write = not self.add_default_params()  # if new default data has been added
        if self.delete_old():  # or if old params have been deleted
          to_write = True
      else:  # don't overwrite corrupted params, just print to screen
        print("ERROR: Can't read op_params.json file")
    elif os.path.isfile(self.kegman_file):
      to_write = True  # write no matter what
      try:
        with open(self.kegman_file, "r") as f:  # restore params from kegman
          self.params = json.load(f)
          self.add_default_params()
      except:
        print("ERROR: Can't read kegman.json file")
    else:
      no_params = True  # user's first time running a fork with kegman_conf or op_params
    if to_write or no_params:
      write_params(self.params, self.params_file)

  def delete_old(self):
    prev_params = dict(self.params)
    for i in self.to_delete:
      if i in self.params:
        del self.params[i]
    return prev_params == self.params

  def put(self, key, value):
    self.params.update({key: value})
    write_params(self.params, self.params_file)

  def get(self, key=None, default=None, force_update=False):  # can specify a default value if key doesn't exist
    self.update_params(key, force_update)
    if key is None:
      return self.params

    if key in self.params:
      key_info = self.get_key_info(key)
      if key_info.has_allowed_types:
        value = self.params[key]
        allowed_types = self.default_params[key]['allowed_types']
        valid_type = type(value) in allowed_types
        if not valid_type:
          if key_info.has_default:  # if value in op_params.json is not correct type, use default
            value = self.default_params[key]['default']
          else:  # else use a standard value based on type (last resort to keep openpilot running)
            value = self.value_from_types(allowed_types)
      else:
        value = self.params[key]
    else:
      value = default

    return value

  def get_key_info(self, key):
    key_info = KeyInfo()
    if key in self.default_params:
      if 'allowed_types' in self.default_params[key]:
        allowed_types = self.default_params[key]['allowed_types']
        if isinstance(allowed_types, list) and len(allowed_types) > 0:
          key_info.has_allowed_types = True
      if 'live' in self.default_params[key] and self.default_params[key]['live'] is True:
        key_info.live = True
      if 'default' in self.default_params[key]:
        key_info.has_default = True
      if 'description' in self.default_params[key]:
        key_info.has_description = True
    return key_info

  def value_from_types(self, allowed_types):
    if list in allowed_types:
      return []
    elif float in allowed_types or int in allowed_types:
      return 0
    elif type(None) in allowed_types:
      return None
    elif str in allowed_types:
      return ''
    return None  # unknown type

  def update_params(self, key, force_update):
    if force_update or self.get_key_info(key).live:  # if is a live param, we want to get updates while openpilot is running
      if not travis and time.time() - self.last_read_time >= self.read_frequency:  # make sure we aren't reading file too often
        self.params, read_status = read_params(self.params_file, self.format_default_params())
        if not read_status:
          time.sleep(1/100.)
          self.params, _ = read_params(self.params_file, self.format_default_params())  # if the file was being written to, retry once
        self.last_read_time = time.time()

  def delete(self, key):
    if key in self.params:
      del self.params[key]
      write_params(self.params, self.params_file)
