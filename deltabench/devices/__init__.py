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
            self.inst.write(b'*CLS\r\n')
            self.inst.write(b'MEAS:VOLT:DC? 10,0.01\r\n')
            _time.sleep(wait)
            reading = self.inst.read_all().decode('utf-8')
            reading = float(reading.replace('\r\n',''))
            return reading
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            return None

    def configure_fast_dc_volt(self):
        """ Configure device for DC voltage measurements """
        try:
            self.inst.write(b'*RST\r\n')
            _time.sleep(2.0)
            self.inst.write(b'SYSTem:REMote\r\n')
            self.inst.write(b':CONF:VOLT:DC 10,0.01\r\n')
            self.inst.write(b'VOLT:DC:NPLC 0.02\r\n')
            return True
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            return False

    def configure_triggered_read(self, sample_count=1, trigger_count=1):
        """ Configure device to take readings on trigger """
        try:
            if self.inst.in_waiting > 0:
                dummy = self.inst.read_all()
            term = b'\r\n'
            sample_count_bytes = bytes(str(sample_count),'ascii')
            trigger_count_bytes = bytes(str(trigger_count),'ascii')
            self.inst.write(b'*CLS'+term)
            self.inst.write(b'SYSTem:REMote'+term)
            self.inst.write(b'TRIGger:SOURce BUS'+term)
            self.inst.write(
                b'SAMPle:COUNt '+sample_count_bytes+term
            )
            self.inst.write(
                b'TRIGger:COUNt '+trigger_count_bytes+term
            )
            self.inst.write(b'INIT'+term)
            return True
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            return False

    def send_soft_trigger(self):
        """ Send software trigger to device """
        try:
            self.inst.write(b'*TRG\r\n')
            return True
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            return False

    def configure_immediate_read(self, sample_count=1, trigger_count=1):
        """ Configure device for immediate measurements after init """
        try:
            if self.inst.in_waiting > 0:
                dummy = self.inst.read_all()
            term = b'\r\n'
            sample_count_bytes = bytes(str(sample_count),'ascii')
            trigger_count_bytes = bytes(str(trigger_count),'ascii')
            self.inst.write(b'*CLS'+term)
            self.inst.write(b'SYSTem:REMote'+term)
            self.inst.write(b'TRIGger:SOURce IMM'+term)
            self.inst.write(b'SAMPle:COUNt '+sample_count_bytes+term)
            self.inst.write(b'TRIGger:COUNt '+trigger_count_bytes+term)
            return True
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            return False

    def start_immediate_read(self):
        """ Start immediate measurements """
        try:
            self.inst.write(b'INIT\r\n')
            return True
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            return False

    def fetch_readings(self, wait=1.0):
        """ Fetch readings already available in device memory """
        try:
            readings = ''
            # read
            self.inst.write(b'FETCH?\r\n')
            t_start = _time.time()
            diff = _time.time() - t_start
            while diff < wait:
                if wait - diff > 1.0:
                    _time.sleep(1.0)
                else:
                    _time.sleep(wait - diff)
                newdata = self.inst.read_all().decode('utf-8')
                readings += newdata
                diff = _time.time() - t_start
                if '\r\n' in newdata:
                    break

            readings = readings.replace('\r\n','').replace('\x00','')
            # try to convert to float list
            readings_list = [
                float(r) for r in readings.split(',') if r != ''
            ]
            return readings_list
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            return []

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
