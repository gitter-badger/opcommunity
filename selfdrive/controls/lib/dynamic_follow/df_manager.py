import cereal.messaging_arne as messaging_arne
from selfdrive.controls.lib.dynamic_follow.support import dfProfiles
from common.realtime import sec_since_boot


class dfReturn:
  user_profile = None  # stays at user selected profile
  user_profile_text = None  # same as user_profile, but is its text representation
  model_profile = None  # only changes if user selects auto, is model output
  model_profile_text = None  # same as model_profile, but is its text representation
  changed = False  # true if either profile from model or user changes profile
  is_auto = False  # true if auto
  last_is_auto = False


class dfManager:
  def __init__(self, op_params, is_df=False):
    self.op_params = op_params
    self.is_df = is_df
    self.df_profiles = dfProfiles()
    self.sm = messaging_arne.SubMaster(['dynamicFollowButton', 'dynamicFollowData'])

    self.cur_user_profile = self.op_params.get('dynamic_follow', default='auto').strip().lower()
    if not isinstance(self.cur_user_profile, str) or self.cur_user_profile not in self.df_profiles.to_idx:
      self.cur_user_profile = self.df_profiles.default  # relaxed
    else:
      self.cur_user_profile = self.df_profiles.to_idx[self.cur_user_profile]

    self.cur_model_profile = 0
    self.alert_duration = 2.0

    self.offset = self.cur_user_profile
    self.profile_pred = None
    self.change_time = sec_since_boot()
    self.first_run = True
    self.last_is_auto = False

  @property
  def is_auto(self):
    return self.cur_user_profile == self.df_profiles.auto

  @property
  def can_show_alert(self):
    return sec_since_boot() - self.change_time > self.alert_duration

  def update(self):
    self.sm.update(0)
    df_out = dfReturn()
    if self.first_run:
      df_out.changed = True  # to show alert on start
      self.first_run = False

    button_status = self.sm['dynamicFollowButton'].status
    df_out.user_profile = (button_status + self.offset) % len(self.df_profiles.to_profile)
    df_out.user_profile_text = self.df_profiles.to_profile[df_out.user_profile]

    if self.cur_user_profile != df_out.user_profile:
      self.change_time = sec_since_boot()
      self.last_is_auto = False
      df_out.changed = True
      self.op_params.put('dynamic_follow', self.df_profiles.to_profile[df_out.user_profile])  # save current profile for next drive
      self.cur_user_profile = df_out.user_profile

    if self.is_auto:
      df_out.model_profile = self.sm['dynamicFollowData'].profilePred
      df_out.model_profile_text = self.df_profiles.to_profile[df_out.model_profile]
      df_out.is_auto = True
      df_out.last_is_auto = self.last_is_auto
      self.last_is_auto = True
      if self.cur_model_profile != df_out.model_profile and self.can_show_alert:
        df_out.changed = True  # to hide pred alerts until user-selected auto alert has finished
      self.cur_model_profile = df_out.model_profile

    return df_out
