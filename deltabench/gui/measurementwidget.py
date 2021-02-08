# -*- coding: utf-8 -*-

"""Manual Measurement widget for the control application."""

import sys as _sys
import os
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
from PyQt5.QtGui import QPixmap        

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

        # create objects to use database functions
#        self.access_measurement_data = _database.DatabaseCollection(
#            database_name=self.database_name,
#            collection_name=_measurement.MeasurementData.collection_name,
#            mongo=self.mongo,
#            server=self.server
#        )
        self.access_block_data = _database.DatabaseCollection(
            database_name=self.database_name,
            collection_name=_measurement.BlockData.collection_name,
            mongo=self.mongo,
            server=self.server
        )
        self.access_hall_data = _database.DatabaseCollection(
            database_name=self.database_name,
            collection_name=_measurement.HallWaveformData.collection_name,
            mongo=self.mongo,
            server=self.server
        )
        self.access_scan_data = _database.DatabaseCollection(
            database_name=self.database_name,
            collection_name=_configuration.ScanConfig.collection_name,
            mongo=self.mongo,
            server=self.server
        )

        self.sb_names = [
        ]

        self.le_names = [
        ]

        self.te_names = [
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
#        self.measurement_data = _measurement.MeasurementData()

        # start to read display periodically
        self.timer.start(self._update_display_interval*1000)

        # create dictionary for magnet directiion images
        self.direction_images = {}
        self.direction_images['up'] = QPixmap(
            os.path.join('deltabench','resources', 'img',
                         'arrow-up-bold-outline.png')
        )
        self.direction_images['down'] = QPixmap(
            os.path.join('deltabench','resources', 'img',
                         'arrow-down-bold-outline.png')
        )
        self.direction_images['left'] = QPixmap(
            os.path.join('deltabench','resources', 'img',
                         'arrow-left-bold-outline.png')
        )
        self.direction_images['right'] = QPixmap(
            os.path.join('deltabench','resources', 'img',
                         'arrow-right-bold-outline.png')
        )
        self.direction_images['none'] = QPixmap()

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

                readings = _display.read_display(
                     display_model=self.advanced_options.display_model,
                     wait=wait
                )
#                _display.inst.write(b'\x1bA0200\r')
#                _time.sleep(wait)
#                readings = _display.inst.read_all().decode('utf-8')
#                readings = readings.upper().split(' R\r\n')
#
#                aux1 = readings[0][readings[0].find('X=') + 2:]
#                aux1 = aux1.replace(' ', '')
#    
#                aux2 = readings[1][readings[1].find('Y=') + 2:]
#                aux2 = aux2.replace(' ', '')
#    
#                aux3 = readings[2][readings[2].find('Z=') + 2:]
#                aux3 = aux3.replace(' ', '')
#    
#                readings = [float(aux1), float(aux2), float(aux3)]
    
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

 #   def clear(self):
 #       """Clear."""
 #       self.measurement_data.clear()

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

    def store_position_1(self):
        """ Move probe 1 current position to store label """
        # check if heidenhain display is connected
        if not _display.connected:
            msg = 'Heidenhain display not connected.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False
        # transfer current reading
        self.ui.le_x_position.setText(
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
        self.ui.le_z_position.setText(
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
        self.ui.tbt_update_measurement_list.clicked.connect(self.update_measurement_list)
        self.ui.cmb_measurement_list.currentTextChanged.connect(self.update_undulator_list)
        self.ui.cmb_undulator_list.currentTextChanged.connect(self.update_cassette_list)
        self.ui.cmb_cassette_list.currentTextChanged.connect(self.update_block_list)
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

        except Exception:
            msg = 'Failed to read hall sensor.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)
            return False

#    def save_measurement_data(self):
#        try:
#            if self.database_name is None:
#                msg = 'Invalid database filename.'
#                _QMessageBox.critical(
#                    self, 'Failure', msg, _QMessageBox.Ok)
#                return False
#
#            self.measurement_data.db_update_database(
#                self.database_name,
#                mongo=self.mongo, server=self.server)
#            self.measurement_data.db_save()
#
#            # update undulator list on interface
#            self.update_measurement_list()
#
#            return True
#
#        except Exception:
#            _traceback.print_exc(file=_sys.stdout)
#            msg = 'Failed to save measurement to database.'
#            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
#            return False

    def update_widget_gb_stored_data(self):
        # get names from selected filter values
        measurement = self.ui.cmb_measurement_list.currentText()
        meas_idx = self.ui.cmb_measurement_list.currentIndex()
        undulator = self.ui.cmb_undulator_list.currentText()
        und_idx = self.ui.cmb_undulator_list.currentIndex()
        cassette = self.ui.cmb_cassette_list.currentText()
        cassette_idx = self.ui.cmb_cassette_list.currentIndex()
        block_str = self.ui.cmb_block_list.currentText()
        block = -1
        if block_str != '':
            block = int(block_str)
        block_idx = self.ui.cmb_block_list.currentIndex()
        # if a measurement, undulator and cassette is selected
        # find associated scan id
        scan_id = -1
        if meas_idx != -1 and und_idx != -1 and cassette_idx != -1:
            # search scan id
            scan_data = self.access_scan_data.db_search_collection(
                fields=['measurement_name', 'undulator_name',
                        'cassette_name', 'id'
                ],
                filters=[measurement, undulator, cassette, '',
                ]
            )
            scan_id = scan_data[0]['id']
        # display number of registered blocks for cassette
        if scan_id != -1:
            entries = self.access_block_data.db_search_field(
                'scan_id', scan_id)
            block_count = len(entries)
            self.ui.lcd_block_count.display(block_count)
        else:
            # clear block count display
            self.ui.lcd_block_count.display('')
        # if a unique block is selected, display data stored for it
        if scan_id != -1 and block_idx != -1:
            # search block data
            block_data = self.access_block_data.db_search_collection(
                fields=['scan_id', 'block_number', 'x_position',
                        'x_position_error', 'z_position',
                        'z_position_error', 'encoder_position',
                        'block_direction'
                ],
                filters=[scan_id, block, '', '', '', '', '', ''
                ]
            )
            block_data = block_data[0]
            # update displays
            self.ui.lcd_stored_x_position.display(block_data['x_position'])
            self.ui.lcd_stored_x_position_error.display(block_data['x_position_error'])
            self.ui.lcd_stored_z_position.display(block_data['z_position'])
            self.ui.lcd_stored_z_position_error.display(block_data['z_position_error'])
#            self.ui.lcd_hall_reading_stored.display(block_data['hall_sensor_voltage'])
            self.ui.lcd_stored_encoder_position.display(block_data['encoder_position'])
            # update block direction image
            block_direction = block_data['block_direction']
            if block_direction == 1:
                self.ui.la_magnet_direction.setPixmap(
                    self.direction_images['up']
                )
            elif block_direction == 2:
                self.ui.la_magnet_direction.setPixmap(
                    self.direction_images['right']
                )
            elif block_direction == 3:
                self.ui.la_magnet_direction.setPixmap(
                    self.direction_images['down']
                )
            elif block_direction == 4:
                self.ui.la_magnet_direction.setPixmap(
                    self.direction_images['left']
                )
            else:
                self.ui.la_magnet_direction.setPixmap(
                    self.direction_images['none']
                )
        else:
            # clear displays
            self.ui.lcd_stored_x_position.display('')
            self.ui.lcd_stored_x_position_error.display('')
            self.ui.lcd_stored_z_position.display('')
            self.ui.lcd_stored_z_position_error.display('')
#            self.ui.lcd_hall_reading_stored.display('')
            self.ui.lcd_stored_encoder_position.display('')
            self.ui.la_magnet_direction.setPixmap(
                self.direction_images['none']
            )
        return True

    def update_measurement_list(self):
        """ Update the measurement list group box on interface """
        # store selected measurement name
        selected_measurement = self.ui.cmb_measurement_list.currentText()
        # clear measurement list
        self.ui.cmb_measurement_list.clear()
        # get list of measurements in db
        meas_list = self.access_scan_data.db_get_values('measurement_name')
        # remove empty names if any
        meas_list = [e for e in meas_list if e != '']
        # remove duplicates if any
        meas_list = list(_collections.OrderedDict.fromkeys(meas_list))
        # apply natural sort order
        meas_list = _natsort.natsorted(
            meas_list, alg=_natsort.ns.IGNORECASE
        )
        # update measurement list on interface
        self.ui.cmb_measurement_list.addItems(meas_list)
        # find index of previously selected measurement
        meas_idx = self.ui.cmb_measurement_list.findText(selected_measurement)
        # select corresponding index
        self.ui.cmb_measurement_list.setCurrentIndex(meas_idx)

        # update undulator list in stored data group box
        self.update_undulator_list()

        return True

    def update_undulator_list(self):
        """ Update the undulator list group box on interface """
        # get current measurement list selection
        meas_idx = self.ui.cmb_measurement_list.currentIndex()
        measurement = self.ui.cmb_measurement_list.currentText()
        # store current combo box position and clear it
        curr_text = self.ui.cmb_undulator_list.currentText()
        self.ui.cmb_undulator_list.clear()
        self.ui.cmb_undulator_list.setCurrentIndex(-1)
        # if no measurement is selected, return
        if meas_idx == -1:
            return True
        # get list of undulators and order it
        row_list = self.access_scan_data.db_search_field('measurement_name',
            measurement)
        u_list = [i['undulator_name'] for i in row_list]
        # remove empty names if any
        u_list = [e for e in u_list if e != '']
        # remove duplicates if any
        u_list = list(_collections.OrderedDict.fromkeys(u_list))
        # apply natural sort order
        u_list = _natsort.natsorted(
            u_list, alg=_natsort.ns.IGNORECASE
        )
        # add undulators to combo box
        self.ui.cmb_undulator_list.addItems(u_list)
        # update combo box index
        undulator_idx = self.ui.cmb_undulator_list.findText(curr_text)
        self.ui.cmb_undulator_list.setCurrentIndex(undulator_idx)
        # update cassette list in stored data group box
        self.update_cassette_list()
        return True

    def update_cassette_list(self):
        """ Update the cassette list group box on interface """
        # get current measurement list selection
        meas_idx = self.ui.cmb_measurement_list.currentIndex()
        measurement = self.ui.cmb_measurement_list.currentText()
        # get current undulator list selection
        und_idx = self.ui.cmb_undulator_list.currentIndex()
        undulator = self.ui.cmb_undulator_list.currentText()
        # store current combo box position and clear it
        curr_text = self.ui.cmb_cassette_list.currentText()
        self.ui.cmb_cassette_list.clear()
        self.ui.cmb_cassette_list.setCurrentIndex(-1)
        # if no measurement or undulator is selected, return
        if meas_idx == -1 or und_idx == -1:
            return True
        # get list of cassettes and order it
        row_list = self.access_scan_data.db_search_collection(
            fields=['measurement_name', 'undulator_name',
                    'cassette_name',
            ],
            filters=[measurement, undulator, '',
            ]
        )
        c_list = [i['cassette_name'] for i in row_list]
        # remove empty names if any
        c_list = [e for e in c_list if e != '']
        # remove duplicates if any
        c_list = list(_collections.OrderedDict.fromkeys(c_list))
        # apply natural sort order
        c_list = _natsort.natsorted(
            c_list, alg=_natsort.ns.IGNORECASE
        )
        # add cassettes to combo box
        self.ui.cmb_cassette_list.addItems(c_list)
        # update combo box index
        cassette_idx = self.ui.cmb_cassette_list.findText(curr_text)
        self.ui.cmb_cassette_list.setCurrentIndex(cassette_idx)
        # update block list in stored data group box
        self.update_block_list()
        return True

    def update_block_list(self):
        """ Update the block list group box on interface """
        # get current measurement list selection
        meas_idx = self.ui.cmb_measurement_list.currentIndex()
        measurement = self.ui.cmb_measurement_list.currentText()
        # get current undulator list selection
        und_idx = self.ui.cmb_undulator_list.currentIndex()
        undulator = self.ui.cmb_undulator_list.currentText()
        # get current cassette list selection
        cassette_idx = self.ui.cmb_cassette_list.currentIndex()
        cassette = self.ui.cmb_cassette_list.currentText()
        # store current combo box position and clear it
        curr_text = self.ui.cmb_block_list.currentText()
        self.ui.cmb_block_list.clear()
        self.ui.cmb_block_list.setCurrentIndex(-1)
        # if no measurement, undulator or cassette is selected, return
        if meas_idx == -1 or und_idx == -1 or cassette_idx == -1:
            return True
        # get scan id associated to cassette, undulator and measurement
        scan_id_list = self.access_scan_data.db_search_collection(
            fields=['measurement_name', 'undulator_name',
                    'cassette_name', 'id'
            ],
            filters=[measurement, undulator, cassette, '']
        )
        scan_id = scan_id_list[0]['id']
        # get list of blocks and order it
        row_list = self.access_block_data.db_search_collection(
            fields=['scan_id', 'block_number'],
            filters=[scan_id, '']
        )
        b_list = [str(i['block_number']) for i in row_list]
        b_list = _natsort.natsorted(
            b_list, alg=_natsort.ns.IGNORECASE
        )
        # add blocks to combo box
        self.ui.cmb_block_list.addItems(b_list)
        # update combo box index
        block_idx = self.ui.cmb_block_list.findText(curr_text)
        self.ui.cmb_block_list.setCurrentIndex(block_idx)
        return True

#    def update_configuration(self):
#        """Update configuration parameters."""
#        try:
#            return super().update_configuration(clear=False)
#
#        except Exception:
#            _traceback.print_exc(file=_sys.stdout)
#            self.config.clear()
#            return False
