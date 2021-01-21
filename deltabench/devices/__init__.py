"""Sub-package for delta assembly bench devices."""

import os as _os
import time as _time

from imautils.devices.utils import configure_logging
from imautils.devices import HeidenhainLib as _HeidenhainLib
from imautils.devices import ParkerDriverLib as _ParkerDriverLib
from imautils.devices import Agilent34401ALib as _Agilent34401ALib

_timestamp = _time.strftime('%Y-%m-%d_%H-%M-%S', _time.localtime())

_logs_path = _os.path.join(
    _os.path.dirname(_os.path.dirname(
        _os.path.dirname(
            _os.path.abspath(__file__)))), 'logs')

if not _os.path.isdir(_logs_path):
    _os.mkdir(_logs_path)

logfile = _os.path.join(
    _logs_path, '{0:s}_delta_bench.log'.format(_timestamp))
configure_logging(logfile)

display = _HeidenhainLib.HeidenhainSerial(log=True)
driver = _ParkerDriverLib.ParkerDriverSerial(log=True)
multimeter = _Agilent34401ALib.Agilent34401ASerial(log=True)
