"""Sub-package for delta assembly bench devices."""

import os as _os
import time as _time

from imautils.devices.utils import configure_logging
from imautils.devices import HeidenhainLib as _HeidenhainLib
from imautils.devices import ParkerDriverLib as _ParkerDriverLib
from imautils.devices import Agilent34401ALib as _Agilent34401ALib

class Display(_HeidenhainLib.HeidenhainSerial):
    """ Class with custom functions for Heidenhain display """
#    def configure_display(self):
#        return True

    def read_display(self, display_model, wait=0.1):
        try:
            # remove old data from input buffer
            if self.inst.in_waiting > 0:
                readings = self.inst.read_all()
            # request display readings
            self.inst.write(b'\x1bA0200\r')
            _time.sleep(wait)
            readings = self.inst.read_all().decode('utf-8')
            readings = readings.upper().split(' R\r\n')

            aux1 = readings[0][readings[0].find('X=') + 2:]
            aux1 = aux1.replace(' ', '')

            aux2 = readings[1][readings[1].find('Y=') + 2:]
            aux2 = aux2.replace(' ', '')

            aux3 = readings[2][readings[2].find('Z=') + 2:]
            aux3 = aux3.replace(' ', '')
 
            return [float(aux1), float(aux2), float(aux3)]

        except Exception:
            return [_np.nan, _np.nan, _np.nan]

class Driver(_ParkerDriverLib.ParkerDriverSerial):
    """ Class with custom functions for Parker Driver """
    pass

class Multimeter(_Agilent34401ALib.Agilent34401ASerial):
    """ Class with custom functions for Agilent 34401A multimeter """
#    def configure_voltage(self):
#        return True
#
#    def read(self):
#        return True
    pass

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

display = Display(log=True)
driver = Driver(log=True)
multimeter = Multimeter(log=True)
