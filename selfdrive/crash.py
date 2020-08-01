"""Install exception handler for process crash."""
import os
import sys
import capnp
import requests
import threading
import traceback
from common.params import Params
from selfdrive.version import version, dirty, origin, branch

from selfdrive.swaglog import cloudlog
from common.android import ANDROID

def save_exception(exc_text):
  i = 0
  log_file = '{}/{}'.format(CRASHES_DIR, datetime.now().strftime('%d-%m-%Y--%I:%M%p.log'))
  if os.path.exists(log_file):
    while os.path.exists(log_file + str(i)):
      i += 1
    log_file += str(i)
  with open(log_file, 'w') as f:
    f.write(exc_text)
  print('Logged current crash to {}'.format(log_file))

if os.getenv("NOLOG") or os.getenv("NOCRASH") or not ANDROID:
  def capture_exception(*args, **kwargs):
    pass

  def bind_user(**kwargs):
    pass

  def bind_extra(**kwargs):
    pass

  def install():
    pass
else:
  from raven import Client
  from raven.transport.http import HTTPTransport
  from common.op_params import opParams
  from datetime import datetime

  COMMUNITY_DIR = '/data/community'
  CRASHES_DIR = '{}/crashes'.format(COMMUNITY_DIR)

  if not os.path.exists(COMMUNITY_DIR):
    os.mkdir(COMMUNITY_DIR)
  if not os.path.exists(CRASHES_DIR):
    os.mkdir(CRASHES_DIR)

  params = Params()
  try:
    dongle_id = params.get("DongleId").decode('utf8')
  except AttributeError:
    dongle_id = "None"
  try:
    ipaddress = requests.get('https://checkip.amazonaws.com/').text.strip()
  except:
    ipaddress = "255.255.255.255"
  error_tags = {'dirty': dirty, 'username': uniqueID, 'dongle_id': dongle_id, 'branch': branch, 'remote': origin}
  
  client = Client('https://21681855cccf4d889a6369637d1721fe@o400203.ingest.sentry.io/5258423',
                  install_sys_hook=False, transport=HTTPTransport, release=version, tags=error_tags)

  def capture_exception(*args, **kwargs):
    save_exception(traceback.format_exc())
    exc_info = sys.exc_info()
    if not exc_info[0] is capnp.lib.capnp.KjException:
      client.captureException(*args, **kwargs)
    cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  def bind_user(**kwargs):
    client.user_context(kwargs)

  def capture_warning(warning_string):
    bind_user(id=dongle_id, ip_address=ipaddress)
    client.captureMessage(warning_string, level='warning')

  def capture_info(info_string):
    bind_user(id=dongle_id, ip_address=ipaddress)
    client.captureMessage(info_string, level='info')

  def bind_extra(**kwargs):
    client.extra_context(kwargs)

  def install():
    """
    Workaround for `sys.excepthook` thread bug from:
    http://bugs.python.org/issue1230540
    Call once from the main thread before creating any threads.
    Source: https://stackoverflow.com/a/31622038
    """
    # installs a sys.excepthook
    __excepthook__ = sys.excepthook

    def handle_exception(*exc_info):
      if exc_info[0] not in (KeyboardInterrupt, SystemExit):
        capture_exception()
      __excepthook__(*exc_info)
    sys.excepthook = handle_exception

    init_original = threading.Thread.__init__

    def init(self, *args, **kwargs):
      init_original(self, *args, **kwargs)
      run_original = self.run

      def run_with_except_hook(*args2, **kwargs2):
        try:
          run_original(*args2, **kwargs2)
        except Exception:
          sys.excepthook(*sys.exc_info())

      self.run = run_with_except_hook

    threading.Thread.__init__ = init
