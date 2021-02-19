"""Sub-package for delta assembly bench devices."""

import sys as _sys
import os as _os
import time as _time
import numpy as _np
import traceback as _traceback

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
    def input_status(self, address, wait=0.25):
        if self.connected:
            # if pending input, clear buffer
            if self.inst.in_waiting > 0:
                self.inst.read_all()
            # read inputs
            return self.all_input_status(address, wait=wait)
        else:
            return None

class Multimeter(_Agilent34401ALib.Agilent34401ASerial):
    """ Class with custom functions for Agilent 34401A multimeter """
    def single_dc_read(self, wait=0.5):
        """ Do a single fast DC read """
        try:
            if self.inst.in_waiting > 0:
                dummy = self.inst.read_all()
            self.inst.write(b'MEAS:VOLT:DC? 10,0.003\r\n')
            _time.sleep(wait)
            reading = self.inst.read_all().decode('utf-8')
            reading = float(reading.replace('\r\n',''))
            return reading
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            return None

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
