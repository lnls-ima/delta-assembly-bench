# -*- coding: utf-8 -*-

"""Manual Measurement widget for the control application."""

import sys as _sys
import numpy as _np
import time as _time
import math
import warnings as _warnings
import traceback as _traceback
from qtpy.QtWidgets import (
    QWidget as _QWidget,
    QMessageBox as _QMessageBox,
    QApplication as _QApplication,
    )
from qtpy.QtCore import (
    Qt as _Qt,
    QTimer as _QTimer
)
import qtpy.uic as _uic

from deltabench.gui import utils as _utils
from deltabench.gui.auxiliarywidgets import (
    ConfigurationWidget as _ConfigurationWidget
    )
import deltabench.data.configuration as _configuration
import deltabench.data.measurement as _measurement
from deltabench.devices import (
    driver as _driver,
    multimeter as _multimeter,
    display as _display,
)
from imautils.db import database as _database
import collections as _collections
import natsort as _natsort

class MeasurementWidget(_ConfigurationWidget):
    """Measurement widget class for the control application."""

    _update_display_interval = _utils.UPDATE_DISPLAY_INTERVAL

    def __init__(self, parent=None):
        """Set up the ui."""
        uifile = _utils.get_ui_file(self)
        config = _configuration.MeasurementConfig()
        super().__init__(uifile, config, parent=parent)
        self.timer = _QTimer()
        self.timer.timeout.connect(self.periodic_display_update)

        # create object to use database functions
        self.access_measurement_data = _database.DatabaseCollection(
            database_name=self.database_name,
            collection_name=_measurement.MeasurementData.collection_name,
            mongo=self.mongo,
            server=self.server
        )

        self.sb_names = [
        ]

        self.le_names = [
            'undulator_name',
            'block_name',
        ]

        self.te_names = [
            'comments',
        ]

        self.sbd_names = [
        ]

        self.cmb_names = [
        ]

        self.chb_names = [
        ]

        self.connect_signal_slots()
        self.load_last_db_entry()

        self.stop_sent = False
        self.current_encoder_position = 0.0
        self.encoder_measurement_index = -1

        self.hall_threshold = None
        self.measurement_data = _measurement.MeasurementData()

        # start to read display periodically
        self.timer.start(self._update_display_interval*1000)

    @property
    def advanced_options(self):
        """Return global advanced options."""
        dialog = _QApplication.instance().advanced_options_dialog
        return dialog.config

    @property
    def global_config(self):
        """Return the global measurement configuration."""
        return _QApplication.instance().measurement_config

    @global_config.setter
    def global_config(self, value):
        _QApplication.instance().measurement_config = value

    def periodic_display_update(self):
        """ Update probe and encoder information periodically. """

        # update probe and encoder readings
        if _display.connected:
            # read probes and encoder
#            readings = _display.read_display(
#                    model=self.advanced_options.display_model
#            )
            try:
                # interval to wait after command
                wait = 0.15

                _display.inst.write(b'\x1bA0200\r')
                _time.sleep(wait)
                readings = _display.inst.read_all().decode('utf-8')
                readings = readings.upper().split(' R\r\n')

                aux1 = readings[0][readings[0].find('X=') + 2:]
                aux1 = aux1.replace(' ', '')
    
                aux2 = readings[1][readings[1].find('Y=') + 2:]
                aux2 = aux2.replace(' ', '')
    
                aux3 = readings[2][readings[2].find('Z=') + 2:]
                aux3 = aux3.replace(' ', '')
    
                readings = [float(aux1), float(aux2), float(aux3)]
    
                # update ui
                self.ui.lcd_curr_position_1.display(readings[0])
                self.ui.lcd_curr_position_2.display(readings[1])
                self.ui.lcd_linear_encoder_position.display(readings[2])
    
                # update global encoder data
                self.current_encoder_position = readings[2]
                self.encoder_measurement_index = (
                    (self.encoder_measurement_index + 1) % 1e6
                )
            except Exception:
                # indicate encoder fault
                self.encoder_measurement_index = -1
                # print fault to stdout
                _traceback.print_exc(file=_sys.stdout)
        else:
            # indicate encoder fault
            self.encoder_measurement_index = -1

        return True

    def disable_invalid_widgets(self):
        if self.ui.rbt_nr_steps.isChecked():
            self.ui.sb_commanded_nr_steps.setEnabled(True)
            self.ui.sbd_commanded_encoder_position.setEnabled(False)
        elif self.ui.rbt_encoder_position.isChecked():
            self.ui.sb_commanded_nr_steps.setEnabled(False)
            self.ui.sbd_commanded_encoder_position.setEnabled(True)
        else:
            self.ui.sb_nr_steps.setEnabled(False)
            self.ui.sb_encoder_position.setEnabled(False)
        return True

    def move_motor(self):
        """ Move motor to commanded position or by the specified
            number of steps. When an encoder target position is
            provided, a max number of retries can also be specified. """
        # clear stop flag
        self.stop_sent = False

        # check motor connection
        if not _driver.connected:
            msg = 'Driver not connected.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

        # whether to use encoder position or step as set point
        use_encoder = self.ui.rbt_encoder_position.isChecked()

        # check display connection
        if use_encoder and not _display.connected:
            msg = 'Display not connected - invalid encoder position.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

        # interval to wait after motion command
        wait = 0.1

        # disable move and homing buttons
        self.ui.pbt_move_motor.setEnabled(False)
        self.ui.pbt_homing.setEnabled(False)
        # process gui events
        _QApplication.processEvents()

        # get retry count
        retry_count = int(self.ui.sb_retry_count.value())

        # get motor info
        driver_address = self.advanced_options.motor_driver_address
        motor_resolution = (
            self.advanced_options.motor_resolution
        )
        rotation_direction = (
            self.advanced_options.motor_rotation_direction
        )
        velocity = self.advanced_options.motor_velocity
        acceleration = self.advanced_options.motor_acceleration
        linear_conversion = self.advanced_options.linear_conversion_factor
        tolerance = self.advanced_options.position_tolerance
        move_timeout = self.advanced_options.move_timeout

        # motion mode is 'preset' (not continuous)
        mode = 0

        # step set point
        steps = 0

        # only used for encoder set point mode
        target_position = 0
        diff = 0
        previous_encoder_index = 0

        # if steps are selected, just assign number
        if self.ui.rbt_nr_steps.isChecked():
            steps = int(
                self.ui.sb_commanded_nr_steps.value()
            )
            if rotation_direction == '-':
                steps = -steps
            # if steps selected, ignore retries
            retry_count = 0
        elif use_encoder:
            # wait until there is a valid encoder reading
            while (self.encoder_measurement_index == -1
                  and not self.stop_sent
                  and (_time.time() - t_start) < move_timeout):
                _time.sleep(wait)
                _QApplication.processEvents()
            # get target position
            target_position = float(
                self.ui.sbd_commanded_encoder_position.value()
            )
            previous_encoder_index = self.encoder_measurement_index
            diff = target_position - self.current_encoder_position
            steps = math.floor(diff / (linear_conversion / motor_resolution))
            if rotation_direction == '-':
                steps = -steps

        # try to reach position at the first move
        if steps != 0 and not self.stop_sent:
            # configure motor
            if not _driver.config_motor(
                   driver_address,
                   mode,
                   rotation_direction,
                   motor_resolution,
                   velocity,
                   acceleration,
                   steps):
                msg = 'Failed to send configuration to motor.'
                _QMessageBox.critical(
                    self, 'Failure', msg, _QMessageBox.Ok)
            else:
                # start motor motion if commanded to
                _driver.move_motor(driver_address)
                # wait for command reception
                _time.sleep(wait)
                # process gui events
                _QApplication.processEvents()

        # if necessary, retry position
        while retry_count > 0:
            # update counter
            retry_count = retry_count - 1
            # wait until motor is idle
            while ((not _driver.ready(driver_address)
                    or previous_encoder_index ==
                       self.encoder_measurement_index
                    or self.encoder_measurement_index == -1)
                  and not self.stop_sent
                  and (_time.time() - t_start) < move_timeout):
                _time.sleep(wait)
                _QApplication.processEvents()
            # break if stop was sent or timeout happened
            if self.stop_sent or (_time.time() - t_start) < move_timeout:
                break
            # update diff
            previous_encoder_index = self.encoder_measurement_index
            diff = target_position - self.current_encoder_position
            # check if diff is small enough
            if abs(diff) > abs(tolerance):
                break
            # update number of steps
            steps = math.floor(diff / (linear_conversion / motor_resolution))
            if rotation_direction == '-':
                steps = -steps
            # configure motor
            if not _driver.config_motor(
                   driver_address,
                   mode,
                   rotation_direction,
                   motor_resolution,
                   velocity,
                   acceleration,
                   steps):
                msg = 'Failed to send configuration to motor.'
                _QMessageBox.critical(
                    self, 'Failure', msg, _QMessageBox.Ok)
            else:
                # start motor motion if commanded to
                _driver.move_motor(driver_address)
                # wait for command reception
                _time.sleep(wait)
                # process gui events
                _QApplication.processEvents()

        # wait until motor is idle
        while (not _driver.ready(driver_address)
              and (_time.time() - t_start) < move_timeout):
            _time.sleep(wait)
            _QApplication.processEvents()

        # update motor moving LED
        self.ui.la_motor_is_moving.setEnabled(False)

        # re-enable move and homing buttons
        self.ui.pbt_move_motor.setEnabled(True)
        self.ui.pbt_homing.setEnabled(True)

        return True

    def stop_motor(self):
        # set flag for move_motor function to see
        self.stop_sent = True

        # check connection
        if not _driver.connected:
            msg = 'Driver not connected - failed to stop motor.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

        # stop motor
        try:
            driver_address = self.advanced_options.motor_driver_address
            _driver.stop_motor(driver_address)
        except Exception:
            msg = 'Failed to stop motor.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)

        return True

    def clear(self):
        """Clear."""
        self.measurement_data.clear()

    def configure_driver(self, steps):
        try:
            _driver.stop_motor(
                self.advanced_options.motor_driver_address)
            _time.sleep(0.1)

            return _driver.config_motor(
                self.advanced_options.motor_driver_address,
                0,
                self.advanced_options.motor_rotation_direction,
                self.advanced_options.motor_resolution,
                self.advanced_options.motor_velocity,
                self.advanced_options.motor_acceleration,
                steps)

        except Exception:
            msg = 'Failed to configure driver.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)
            return False

    def configure_measurement(self):
        self.clear()

        try:

            if not self.update_configuration():
                return False

            if not self.save_db():
                return False

            self.global_config = self.config.copy()

            self.measurement_data.undulator_name = (
                self.global_config.undulator_name
            )
            self.measurement_data.block_name = (
                self.global_config.block_name
            )
            self.measurement_data.comments = (
                self.global_config.comments
            )
            self.measurement_data.advanced_options_id = (
                self.advanced_options.idn
            )
            self.measurement_data.configuration_id = (
                self.global_config.idn
            )
            self.measurement_data.hall_sensor_voltage = (
                float(self.ui.le_hall_sensor_voltage.text())
            )
            self.measurement_data.display_position_1 = (
                float(self.ui.le_display_position_1.text())
            )
            self.measurement_data.display_position_2 = (
                float(self.ui.le_display_position_2.text())
            )
            self.measurement_data.linear_encoder_position = (
                float(self.ui.lcd_linear_encoder_position.value())
            )

            if not self.advanced_options.valid_data():
                msg = 'Invalid advanced options.'
                _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
                return False

            return True

        except Exception:
            msg = 'Measurement configuration failed.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)
            return False

    def store_position_1(self):
        """ Move probe 1 current position to store label """
        # check if heidenhain display is connected
        if not _display.connected:
            msg = 'Heidenhain display not connected.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False
        # transfer current reading
        self.ui.le_display_position_1.setText(
            str(self.ui.lcd_curr_position_1.value())
        )
        return True

    def store_position_2(self):
        """ Move probe 2 current position to store label """
        # check if heidenhain display is connected
        if not _display.connected:
            msg = 'Heidenhain display not connected.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False
        # transfer current reading
        self.ui.le_display_position_2.setText(
            str(self.ui.lcd_curr_position_2.value())
        )
        return True

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        super().connect_signal_slots()
        self.ui.pbt_homing.clicked.connect(self.homing)
        self.ui.rbt_nr_steps.toggled.connect(self.disable_invalid_widgets)
        self.ui.rbt_encoder_position.toggled.connect(
            self.disable_invalid_widgets)
        self.ui.pbt_move_motor.clicked.connect(self.move_motor)
        self.ui.pbt_stop_motor.clicked.connect(self.stop_motor)
        self.ui.tbt_read_hall.clicked.connect(self.read_hall)
        self.ui.tbt_store_position_1.clicked.connect(self.store_position_1)
        self.ui.tbt_store_position_2.clicked.connect(self.store_position_2)
        self.ui.pbt_save_measurement.clicked.connect(self.prepare_measurement)
        self.ui.tbt_update_und_list.clicked.connect(self.update_undulator_list)
        self.ui.cmb_undulator_list.currentTextChanged.connect(self.update_block_list)
        self.ui.cmb_block_list.currentTextChanged.connect(self.update_widget_gb_stored_data)

    @property
    def advanced_options(self):
        """Return global advanced options."""
        dialog = _QApplication.instance().advanced_options_dialog
        return dialog.config

    def homing(self):
        """ Move motor to limit switch at the beginning of range.
            If the motor direction is reversed, than the positive
            limit switch is used as home switch. """
        # clear stop flag
        self.stop_sent = False

        # check motor connection
        if not _driver.connected:
            msg = 'Driver not connected.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

        # disable move and homing buttons
        self.ui.pbt_move_motor.setEnabled(False)
        self.ui.pbt_homing.setEnabled(False)

        try:
            # interval to wat after move
            wait = 0.1

            # motor info
            driver_address = self.advanced_options.motor_driver_address
            resolution = self.advanced_options.motor_resolution
            rotation_direction = self.advanced_options.motor_rotation_direction
            velocity = self.advanced_options.motor_velocity
            acceleration = self.advanced_options.motor_acceleration
            move_timeout = self.advanced_options.move_timeout

            # mode = preset (not continuous)
            mode = 0
            steps = int(int(resolution)*2)

            # register move start
            t_start = _time.time()

            # move start flag
            move_started = False

            # move towards the negative or positive limit switch
            if not self.stop_sent:
                if rotation_direction == '+':
                    if not _driver.move_to_negative_limit(
                            driver_address,
                            velocity,
                            acceleration,
                            wait):
                        msg = 'Failed to send command.'
                        _QMessageBox.critical(
                            self, 'Failure', msg, _QMessageBox.Ok)
                    else:
                        move_started = True
                else:
                    if not _driver.move_to_positive_limit(
                            driver_address,
                            velocity,
                            acceleration,
                            wait):
                        msg = 'Failed to send command.'
                        _QMessageBox.critical(
                            self, 'Failure', msg, _QMessageBox.Ok)
                    else:
                        move_started = True

            # wait motor stop
            if move_started:
                _time.sleep(wait)
                t_curr = _time.time()
                while (not _driver.ready(driver_address)
                       and not self.stop_sent
                       and not (t_curr - t_start) > move_timeout):
                    t_curr = _time.time()
                    _time.sleep(wait)
                    _QApplication.processEvents()

                if (t_curr - t_start) > move_timeout:
                    self.stop_motor()
                    msg = 'Homing timeout - stopping motor.'
                    _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Homing failed.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)

        # re-enable move and homing buttons
        self.ui.pbt_move_motor.setEnabled(True)
        self.ui.pbt_homing.setEnabled(True)

        return

    def load(self):
        """Load configuration to set parameters."""
        try:
#            rbt_example_var = 'rbt_' + self.config.example_var
#            rbt = getattr(self.ui, rbt_example_var)
#            rbt.setChecked(True)
            super().load()

        except Exception:
            _traceback.print_exc(file=_sys.stdout)

    def read_hall(self):
        try:
            if not _multimeter.connected:
                msg = 'Multimeter not connected.'
                _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
                return False

            # interval to wait after command
            wait = 0.1

            # configure multimeter and read measurement
            _multimeter.inst.write(b':MEAS:VOLT:DC? DEF,DEF\r\n')
            _time.sleep(wait)
            reading = _multimeter.inst.readline().decode('utf-8')
            reading = reading.replace('\r\n','')

            self.ui.le_hall_sensor_voltage.setText(reading)

            # check if hall sensor reading is as expected
            is_ok = float(reading) > self.advanced_options.hall_threshold
            self.ui.la_hall_led.setEnabled(is_ok)

        except Exception:
            msg = 'Failed to read hall sensor.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)
            return False

    def save_measurement_data(self):
        try:
            if self.database_name is None:
                msg = 'Invalid database filename.'
                _QMessageBox.critical(
                    self, 'Failure', msg, _QMessageBox.Ok)
                return False

            self.measurement_data.db_update_database(
                self.database_name,
                mongo=self.mongo, server=self.server)
            self.measurement_data.db_save()

            # update undulator list on interface
            self.update_undulator_list()

            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to save measurement to database.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

    def update_widget_gb_stored_data(self):
        # get names from selected undulator and block
        undulator = self.ui.cmb_undulator_list.currentText()
        und_idx = self.ui.cmb_undulator_list.currentIndex()
        block = self.ui.cmb_block_list.currentText()
        block_idx = self.ui.cmb_block_list.currentIndex()
        # if a undulator is selected
        if und_idx != -1:
            # display number of registered blocks for undulator
            entries = self.access_measurement_data.db_search_field(
                'undulator_name', undulator)
            block_count = len(entries)
            self.ui.lcd_block_count.display(block_count)
        else:
            # clear block count display
            self.ui.lcd_block_count.display('')
        # if a unique block is selected, display data stored for it
        if und_idx != -1 and block_idx != -1:
            # search block data
            block_data = self.access_measurement_data.db_search_collection(
                fields=['undulator_name', 'block_name', 'display_position_1',
                        'display_position_2', 'hall_sensor_voltage',
                        'linear_encoder_position'
                ],
                filters=[undulator, block, '', '', '', ''
                ]
            )
            block_data = block_data[0]
            # update displays
            self.ui.lcd_curr_position_1_stored.display(block_data['display_position_1'])
            self.ui.lcd_curr_position_2_stored.display(block_data['display_position_2'])
            self.ui.lcd_hall_reading_stored.display(block_data['hall_sensor_voltage'])
            self.ui.lcd_encoder_stored.display(block_data['linear_encoder_position'])
            # update LEDs
            is_ok = (
                    block_data['hall_sensor_voltage']
                    > self.advanced_options.hall_threshold
            )
            self.ui.la_hall_led_stored.setEnabled(is_ok)
        else:
            # clear displays
            self.ui.lcd_curr_position_1_stored.display('')
            self.ui.lcd_curr_position_2_stored.display('')
            self.ui.lcd_hall_reading_stored.display('')
            self.ui.lcd_encoder_stored.display('')
        return True

    def update_undulator_list(self):
        """ Update the undulator list group box on interface """
        # store selected undulator name
        selected_undulator = self.ui.cmb_undulator_list.currentText()
        # clear undulator list
        self.ui.cmb_undulator_list.clear()
        # get list of undulators in db
        und_list = self.access_measurement_data.db_get_values('undulator_name')
        # remove empty names if any
        und_list = [e for e in und_list if e != '']
        # remove duplicates if any
        und_list = list(_collections.OrderedDict.fromkeys(und_list))
        # apply natural sort order
        und_list = _natsort.natsorted(
            und_list, alg=_natsort.ns.IGNORECASE
        )
        # update undulator list on interface
        self.ui.cmb_undulator_list.addItems(und_list)
        # find index of previously selected undulator
        und_idx = self.ui.cmb_undulator_list.findText(selected_undulator)
        # select corresponding index
        self.ui.cmb_undulator_list.setCurrentIndex(und_idx)

        # update block list in stored data group box
        self.update_block_list()

        return True

    def update_block_list(self):
        """ Update the block list group box on interface """
        # get current undulator list selection
        und_idx = self.ui.cmb_undulator_list.currentIndex()
        undulator = self.ui.cmb_undulator_list.currentText()
        # store current combo box position and clear it
        curr_text = self.ui.cmb_block_list.currentText()
        self.ui.cmb_block_list.clear()
        self.ui.cmb_block_list.setCurrentIndex(-1)
        # if no undulator is selected, return
        if und_idx == -1:
            return True
        # get list of blocks and order it
        row_list = self.access_measurement_data.db_search_field('undulator_name',
            undulator)
        b_list = [i['block_name'] for i in row_list]
        b_list = _natsort.natsorted(
            b_list, alg=_natsort.ns.IGNORECASE
        )
        # add blocks to combo box
        self.ui.cmb_block_list.addItems(b_list)
        # update combo box index
        block_idx = self.ui.cmb_block_list.findText(curr_text)
        self.ui.cmb_block_list.setCurrentIndex(block_idx)
        return True

    def prepare_measurement(self):
        if not self.configure_measurement():
            return False

        # get list of duplicate entries
        undulator = self.global_config.undulator_name
        block = self.global_config.block_name
        elem_list = self.access_measurement_data.db_search_collection(
            fields=['undulator_name', 'block_name'
            ],
            filters=[undulator, block
            ]
        )

        # do not accept duplicate entries
        if len(elem_list) != 0:
            msg = 'Block and undulator name pair already saved in DB.'
            _QMessageBox.information(
                self, 'Measurement', msg, _QMessageBox.Ok)
            return False

        _QApplication.processEvents()

        if not self.save_measurement_data():
            return False

        msg = 'Measurement stored.'
        _QMessageBox.information(
            self, 'Measurement', msg, _QMessageBox.Ok)
        return True

    def update_configuration(self):
        """Update configuration parameters."""
        try:
            return super().update_configuration(clear=False)

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            self.config.clear()
            return False
