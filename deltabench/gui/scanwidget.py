# -*- coding: utf-8 -*-

"""Scan Measurement widget for the control application."""

import sys as _sys
import numpy as _np
import time as _time
import math as _math
import statistics as _statistics
import warnings as _warnings
import traceback as _traceback
import pyqtgraph as _pyqtgraph
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
import re

class ScanWidget(_ConfigurationWidget):
    """Scan widget class for the control application."""

    def __init__(self, parent=None):
        """Set up the ui."""
        uifile = _utils.get_ui_file(self)
        config = _configuration.ScanConfig()
        super().__init__(uifile, config, parent=parent)

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
            'hall_samples_per_block',
        ]

        self.le_names = [
            'measurement_name',
            'undulator_name',
            'cassette_name',
        ]

        self.te_names = [
        ]

        self.sbd_names = [
            'start_reference_position',
            'scan_step_size',
        ]

        self.cmb_names = [
        ]

        self.chb_names = [
        ]

        self.connect_signal_slots()
        self.load_last_db_entry()

        self.stop_sent = False

        # lists of plots
        self.graph_x_probe_plots = []
        self.graph_z_probe_plots = []
        self.graph_hall_plots = []

        # lists of data for plots
        self.x_axis_x_probe = []
        self.y_axis_x_probe = []
        self.y_axis_x_probe_error = []
        self.x_axis_z_probe = []
        self.y_axis_z_probe = []
        self.y_axis_z_probe_error = []
        self.x_axis_hall = []
        self.y_axis_hall = []

        # lists of data to save
        self.hall_sample_list = []
        self.hall_sample_index_list = []
        self.encoder_sample_list_for_hall = []
        self.x_probe_sample_list = []
        self.x_probe_sample_error_list = []
        self.z_probe_sample_list = []
        self.z_probe_sample_error_list = []
        self.block_number_list = []
        self.encoder_sample_list_for_probes = []
#        self.block_direction_list = []

        # object to store DB entries before saving
        self.block_data = _measurement.BlockData()
        self.hall_data = _measurement.HallWaveformData()

        # create legend objects
        self.legend_x_probe = _pyqtgraph.LegendItem(offset=(70, 30))
        self.legend_x_probe.setParentItem(
            self.ui.pw_x_probe.graphicsItem()
        )
        self.legend_x_probe.setAutoFillBackground(1)

        self.legend_z_probe = _pyqtgraph.LegendItem(offset=(70, 30))
        self.legend_z_probe.setParentItem(
            self.ui.pw_z_probe.graphicsItem()
        )
        self.legend_z_probe.setAutoFillBackground(1)

        self.legend_hall = _pyqtgraph.LegendItem(offset=(70, 30))
        self.legend_hall.setParentItem(
            self.ui.pw_hall.graphicsItem()
        )
        self.legend_hall.setAutoFillBackground(1)

        self.colors = (
            (0, 0, 255), (255, 0, 0), (0, 0, 0), (0, 255, 0),
            (204, 0, 204), (153, 153, 0), (204, 102, 0),
            (76, 0, 153)
        )

    @property
    def advanced_options(self):
        """Return global advanced options."""
        dialog = _QApplication.instance().advanced_options_dialog
        return dialog.config

    @property
    def global_config(self):
        """Return the global scan configuration."""
        return _QApplication.instance().scan_config

    @global_config.setter
    def global_config(self, value):
        _QApplication.instance().scan_config = value

    @property
    def encoder_update_enabled(self):
        """Return the global encoder update status."""
        return _QApplication.instance().encoder_update_enabled

    @encoder_update_enabled.setter
    def encoder_update_enabled(self, value):
        """Set the global encoder update enable status."""
        _QApplication.instance().encoder_update_enabled = value

    @property
    def emergency_stop(self):
        """Return the global emergency stop value."""
        return _QApplication.instance().emergency_stop

    @emergency_stop.setter
    def emergency_stop(self, value):
        """Set the global emergency stop value."""
        _QApplication.instance().emergency_stop = value

    def start_hall_scan(self):
        """ Start hall scan along undulator magnets. Motion is done in
            one continuous sweep, with the GUI sending periodic trigger
            commands to the multimeter.

            Return value: True if success, False otherwise. """
        # clear stop flag
        self.stop_sent = False

        # success status
        status = False

        # check motor connection
        if not _driver.connected:
            msg = 'Driver nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # check encoder connection
        if not _display.connected:
            msg = 'Display nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # check multimeter connection
        if not _multimeter.connected:
            msg = 'Multimetro nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # block other tabs from communicating with display
        self.encoder_update_enabled = False

        try:
            # interval to wait after a display read command
            wait_display = _utils.WAIT_DISPLAY
            # interval to wait after pneumatic motion start
            wait_pneumatic_retreat = (
                self.advanced_options.pneumatic_retreat_wait
            )
            # interval to wait after a multimeter command
            wait_multimeter = _utils.WAIT_MULTIMETER
            # interval to wait after a multimeter command
            wait_multimeter_fetch = _utils.WAIT_MULTIMETER_FETCH_READINGS

            # disable start scan buttons
            self.ui.pbt_start_hall_scan.setEnabled(False)
            self.ui.pbt_start_position_scan.setEnabled(False)
            # process gui events
            _QApplication.processEvents()

            # get display info
            display_model = self.advanced_options.display_model

            # get motor info
            driver_address = self.advanced_options.motor_driver_address
            motor_resolution = (
                self.advanced_options.motor_resolution
            )
            rotation_direction = (
                self.advanced_options.motor_rotation_direction
            )
            max_velocity = self.advanced_options.motor_velocity
            hall_scan_velocity = (
                self.advanced_options.motor_velocity_hall_scan
            )
            acceleration = self.advanced_options.motor_acceleration
            linear_conversion = (
                self.advanced_options.linear_conversion_factor
            )
            tolerance = self.advanced_options.position_tolerance
            move_timeout = self.advanced_options.move_timeout

            # move to position (not continuously)
            mode = 0

            # read configuration parameters
            first_block_position = (
                self.ui.sbd_start_reference_position.value()
            )
            start_block_idx = self.ui.sb_scan_start_block.value()
            block_step = self.ui.sbd_scan_step_size.value()
            hall_step_count = self.ui.sb_hall_samples_per_block.value()
            block_center = hall_step_count / 2
            hall_step = float(block_step / hall_step_count)
            block_count = self.ui.sb_scan_nr_blocks.value()

            hall_margin_before = _math.floor(
                self.ui.sbd_margin_hall_before.value() / hall_step
            )
            hall_margin_after = _math.floor(
                self.ui.sbd_margin_hall_after.value() / hall_step
            )
            # sum all hall samples per block plus left and right margins
            total_hall_count = (
                hall_step_count * block_count + hall_margin_before + hall_margin_after
            )

            if total_hall_count > 512:
                msg = (
                       "Multimetro nao pode armazenar mais de 512 "
                       "pontos. Por favor, realize multiplas "
                       "vareduras or reduza o numero de pontos."
                )
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok)
                status = False
                raise ValueError(
                    'Multimetro nao pode armazenar mais de 512 pontos'
                )

            # calculate acceleration distance
            t_acc = hall_scan_velocity / acceleration
            acc_distance = (
                0.5 * acceleration * t_acc * t_acc * linear_conversion
            )
            # add a constant to account for driver reaction time
            t_acc_total = t_acc + 0

            # time between hall points
            t_hall = hall_step / (hall_scan_velocity * linear_conversion)

            if t_hall < _utils.MULTIMETER_MIN_TRIGGER_TIME:
                msg = (
                       "Intervalos de Trigger menores do que "
                       + str(_utils.MULTIMETER_MIN_TRIGGER_TIME)
                       + " segudos podem tornar a aquisicao instavel."
                       + " Por favor, reduza a velocidade do motor ou"
                       + " reduza o numero de pontos por bloco."
                )
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok)
                status = False
                raise ValueError(
                    'Too small Multimetro trigger interval'
                )

            # clear previous data and update scan config
            if not self.configure_scan():
                msg = 'Falha ao configurar a varredura.'
                _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
                raise RuntimeError('Failed to configure database for scan')

            # make sure pneumatic actuator is off
            _driver.set_output_1_low(driver_address)
            # wait pneumatic motion to finish
            _time.sleep(wait_pneumatic_retreat)

            # calculate progress bar step
            pgb_max = self.ui.pgb_status.maximum()
            pgb_min = self.ui.pgb_status.minimum()
            pgb_step = (pgb_max - pgb_min) / total_hall_count

            # clear progress bar
            self.ui.pgb_status.setValue(pgb_min)

            # configure multimeter for triggered measurements
            _multimeter.configure_fast_dc_volt()
            _time.sleep(wait_multimeter)
            _multimeter.configure_triggered_read(
                sample_count=1, trigger_count=total_hall_count
            )
            _time.sleep(wait_multimeter)

            # start position
            scan_beginning = (
                first_block_position
                + (start_block_idx - 1) * block_step
                - hall_margin_before * hall_step
                - acc_distance
            )

            # move to start position
            status_tuple = self.move_and_retry(
                           scan_beginning, tolerance,
                           driver_address, rotation_direction,
                           motor_resolution, max_velocity,
                           acceleration, linear_conversion,
                           timeout=move_timeout)
            status = status_tuple[0]
            last_enc_pos = status_tuple[1]

            if status == True:
                # calculate final position
                target_position = (
                    scan_beginning
                    + block_count*block_step
                    + (hall_margin_before + hall_margin_after)*hall_step
                    + 2 * acc_distance
                )

                # calculate number of steps
                diff = target_position - last_enc_pos
                steps = _math.floor(
                    diff / (linear_conversion / motor_resolution)
                )

                # define direction to move
                curr_dir = rotation_direction
                if steps < 0:
                    if rotation_direction == '-':
                        curr_dir = '+'
                    else:
                        curr_dir = '-'

                # register move start
                t_start = _time.time()

                # command motor to move from start to finish
                # in a single sweep
                if not _driver.config_motor(
                        driver_address,
                        mode,
                        curr_dir,
                        motor_resolution,
                        hall_scan_velocity,
                        acceleration,
                        steps):
                    msg = ('Falha ao tentar enviar configuracao'
                           ' para o driver.'
                    )
                    _QMessageBox.critical(
                        self, 'Falha', msg, _QMessageBox.Ok)
                    status = False
                else:
                    # start motor motion if commanded to
                    _driver.move_motor(driver_address)

                # wait until acceleration finishes
                # update GUI, but try to avoid
                # delaying the readings first trigger 
                n = 0
                if t_acc_total > 0.5:
                    n = _math.floor(t_acc_total / 0.5)
                for i in range(0, n):
                    _time.sleep(0.5)
                    _QApplication.processEvents()
                while True:
                    if _time.time() - t_start >= t_acc_total:
                        break
                    _time.sleep(0.001)

            # take readings while motor moves
            if status == True:
                for i in range(0, total_hall_count):
                    # reset ref time
                    t_old = _time.time()

                    # update progress bar
                    self.ui.pgb_status.setValue(i * pgb_step)

                    # stop if command received
                    if self.stop_sent:
                        status = False
                        msg = 'Comando de Pare recebido.'
                        _QMessageBox.critical(
                            self, 'Falha', msg, _QMessageBox.Ok
                        )
                        break

                    # check for timeout
                    if _time.time() - t_start >= move_timeout:
                        status = False
                        _driver.stop_motor(driver_address)
                        msg = ('Timeout de movimento esgotado'
                               ' - parando motor.'
                        )
                        _QMessageBox.critical(
                            self, 'Falha', msg, _QMessageBox.Ok
                        )
                        break

                    readings = _display.read_display(
                        display_model, wait=wait_display
                    )
                    if _math.isnan(readings[0]):
                        readings = _display.read_display(
                            display_model, wait=wait_display
                        )
                        # if failed again, stop scan
                        if _math.isnan(readings[0]):
                            status = False
                            msg = 'Falha ao tentar ler display.'
                            _QMessageBox.critical(
                                self, 'Falha', msg, _QMessageBox.Ok
                            )
                            break
                    self.encoder_sample_list_for_hall.append(readings[2])
                    # trigger a hall reading
                    _multimeter.send_soft_trigger()
                    # wait next sample time
                    while _time.time() - t_old < t_hall:
                        _time.sleep(0.001)
                        _QApplication.processEvents()

            if status is True:
                # fetch readings from multimeter
                readings = _multimeter.fetch_readings(
                    wait_multimeter_fetch
                )
                # store data to save (encoder data is already stored)
                self.hall_sample_list = self.hall_sample_list + readings
                first_hall = (start_block_idx - 1) * hall_step_count + 1 - hall_margin_before
                self.hall_sample_index_list = (
                    self.hall_sample_index_list
                    + list(
                        range(first_hall, first_hall + total_hall_count)
                    )
                )
                # update plot
                status &= self.update_graph_hall(
                    self.encoder_sample_list_for_hall,
                    self.hall_sample_list
                )

            # re-enable start scan buttons
            self.ui.pbt_start_hall_scan.setEnabled(True)
            self.ui.pbt_start_position_scan.setEnabled(True)

            if status is False:
                self.ui.pgb_status.setValue(pgb_min)
                msg = 'Varredura falhou.'
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok
                )
            else:
                self.ui.pgb_status.setValue(pgb_max)
                msg = 'Varredura finalizada com sucesso.'
                _QMessageBox.information(
                    self, 'Sucesso', msg, _QMessageBox.Ok
                )

            # unlock other tabs communication with display
            self.encoder_update_enabled = True

            return status

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Varredura falhou.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            self.ui.pgb_status.setValue(
                self.ui.pgb_status.minimum()
            )
            # re-enable start scan buttons
            self.ui.pbt_start_hall_scan.setEnabled(True)
            self.ui.pbt_start_position_scan.setEnabled(True)
            # unlock other tabs communication with display
            self.encoder_update_enabled = True
            return False

    def start_position_scan(self):
        """ Start position scan along undulator magnets. Motion is done in
            steps and data is acquired at each temporary stop.

            Return value: True if success, False otherwise. """
        # clear stop flag
        self.stop_sent = False

        # success status
        status = False

        # check motor connection
        if not _driver.connected:
            msg = 'Driver nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # check encoder connection
        if not _display.connected:
            msg = 'Display nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # clear previous data and update scan config
        if not self.configure_scan():
            msg = 'Failed to configure scan.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # block other tabs from communicating with display
        self.encoder_update_enabled = False

        try:
            # interval to wait after a display read command
            wait_display = _utils.WAIT_DISPLAY
            # interval to wait after pneumatic motion start
            wait_pneumatic_advance = (
                self.advanced_options.pneumatic_advance_wait
            )
            wait_pneumatic_retreat = (
                self.advanced_options.pneumatic_retreat_wait
            )

            # disable start scan buttons
            self.ui.pbt_start_hall_scan.setEnabled(False)
            self.ui.pbt_start_position_scan.setEnabled(False)
            # process gui events
            _QApplication.processEvents()

            # get display info
            display_model = self.advanced_options.display_model

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

            # read configuration parameters
            first_block_position = self.ui.sbd_start_reference_position.value()
            start_block_idx = self.ui.sb_scan_start_block.value()
            block_step = self.ui.sbd_scan_step_size.value()
            block_count = self.ui.sb_scan_nr_blocks.value()
            block_list = range(
                start_block_idx, start_block_idx + block_count
            )

            # make sure pneumatic actuator is off
            _driver.set_output_1_low(driver_address)
            # wait pneumatic motion to finish
            _time.sleep(wait_pneumatic_retreat)

            # start position
            scan_beginning = first_block_position

            # calculate progress bar step
            pgb_max = self.ui.pgb_status.maximum()
            pgb_min = self.ui.pgb_status.minimum()
            pgb_step = _math.floor((pgb_max - pgb_min) / block_count)

            # clear progress bar
            self.ui.pgb_status.setValue(pgb_min)

            for block_idx in block_list:
                # if stop sent
                if self.stop_sent:
                    # update status
                    status = False
                    # show msg
                    msg = 'Comando de Pare recebido.'
                    _QMessageBox.critical(
                        self, 'Falha', msg, _QMessageBox.Ok
                    )
                    break
                # update target position
                target_position = (
                    scan_beginning + (block_idx - 1)*block_step + block_step/2
                )

                # move
                status_tuple = self.move_and_retry(
                    target_position, tolerance,
                    driver_address, rotation_direction,
                    motor_resolution, velocity,
                    acceleration, linear_conversion,
                    move_timeout
                )
                status = status_tuple[0]
                last_enc_pos = status_tuple[1]
                if status is False:
                    break
                # measure block x and z positions
                # trigger pneumatic actuator
                _driver.set_output_1_high(driver_address)
                # wait pneumatic motion to finish
                _time.sleep(wait_pneumatic_advance)
                # read position probes
                readings = _display.read_display(display_model, wait=wait_display)
                # if error in reading, wait and try again
                if _math.isnan(readings[0]):
                    _time.sleep(wait_display * 5)
                    readings = _display.read_display(
                        display_model, wait=wait_display
                    )
                    # if failed again, stop scan
                    if _math.isnan(readings[0]):
                        status = False
                        msg = 'Falha ao tentar ler display.'
                        _QMessageBox.critical(
                            self, 'Falha', msg, _QMessageBox.Ok
                        )
                        break
                # store probe measurements
                self.x_probe_sample_list.append(readings[0])
                self.z_probe_sample_list.append(readings[1])
                self.encoder_sample_list_for_probes.append(last_enc_pos)
                # deactivate pneumatic actuator
                _driver.set_output_1_low(driver_address)
                # wait pneumatic motion to finish
                _time.sleep(wait_pneumatic_retreat)
                
                if status is False:
                    break

                # update progress bar
                pgb_value = self.ui.pgb_status.value()
                self.ui.pgb_status.setValue(pgb_value + pgb_step)

            if status is True:
                # store block number list
                self.block_number_list = block_list

                # calculate error for each sample
                x_ref = 0
                z_ref = 0
                for x_sample in self.x_probe_sample_list:
                    self.x_probe_sample_error_list.append(x_sample - x_ref)
                for z_sample in self.z_probe_sample_list:
                    self.z_probe_sample_error_list.append(z_sample - z_ref)

                status &= self.update_graph_x_probe(
                    self.block_number_list,
                    self.x_probe_sample_list,
                    self.x_probe_sample_error_list
                )
                status &= self.update_graph_z_probe(
                    self.block_number_list,
                    self.z_probe_sample_list,
                    self.z_probe_sample_error_list
                )

            # re-enable start scan buttons
            self.ui.pbt_start_hall_scan.setEnabled(True)
            self.ui.pbt_start_position_scan.setEnabled(True)

            if status is False:
                self.ui.pgb_status.setValue(pgb_min)
                msg = 'Varredura falhou.'
                _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            else:
                self.ui.pgb_status.setValue(pgb_max)
                msg = 'Varredura finalizada com sucesso.'
                _QMessageBox.information(self, 'Sucesso', msg, _QMessageBox.Ok)

            # unlock other tabs communication with display
            self.encoder_update_enabled = True

            return status

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Varredura falhou.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            self.ui.pgb_status.setValue(
                self.ui.pgb_status.minimum()
            )
            # re-enable start scan buttons
            self.ui.pbt_start_hall_scan.setEnabled(True)
            self.ui.pbt_start_position_scan.setEnabled(True)
            # unlock other tabs communication with display
            self.encoder_update_enabled = True
            return False

    def start_complete_scan(self):
        """ Start scan along undulator magnets. Motion is done in small
            steps and data is acquired at each temporary stop.

            Return value: True if success, False otherwise. """
        # clear stop flag
        self.stop_sent = False

        # success status
        status = False

        # check motor connection
        if not _driver.connected:
            msg = 'Driver nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # check encoder connection
        if not _display.connected:
            msg = 'Display nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # check multimeter connection
        if not _multimeter.connected:
            msg = 'Multimetro nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # clear previous data and update scan config
        if not self.configure_scan():
            msg = 'Falha ao configurar varredura.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return status

        # block other tabs from communicating with display
        self.encoder_update_enabled = False

        # interval to wait after a display read command
        wait_display = _utils.WAIT_DISPLAY
        # interval to wait after pneumatic motion start
        wait_pneumatic_advance = (
            self.advanced_options.pneumatic_advance_wait
        )
        wait_pneumatic_retreat = (
            self.advanced_options.pneumatic_retreat_wait
        )

        # disable start scan button
        self.ui.pbt_start_scan.setEnabled(False)
        # process gui events
        _QApplication.processEvents()

        # get display info
        display_model = self.advanced_options.display_model

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

        # read configuration parameters
        first_block_position = self.ui.sbd_start_reference_position.value()
        start_block_idx = self.ui.sb_scan_start_block.value()
        block_step = self.ui.sbd_scan_step_size.value()
        hall_step_count = self.ui.sb_hall_samples_per_block.value()
        block_center = hall_step_count / 2
        hall_step = float(block_step / hall_step_count)
        block_count = self.ui.sb_scan_nr_blocks.value()
        block_list = range(
            start_block_idx, start_block_idx + block_count
        )

        # make sure pneumatic actuator is off
        _driver.set_output_1_low(driver_address)
        # wait pneumatic motion to finish
        _time.sleep(wait_pneumatic_retreat)

        # start position
        scan_beginning = first_block_position

        # calculate progress bar step
        pgb_max = self.ui.pgb_status.maximum()
        pgb_min = self.ui.pgb_status.minimum()
        pgb_step = _math.floor((pgb_max - pgb_min) / block_count)

        # clear progress bar
        self.ui.pgb_status.setValue(pgb_min)

        for block_idx in block_list:
            # update block interval being measured
            target_interval = scan_beginning + (block_idx - 1)*block_step

            # stop to get each hall sample and measure position
            # with probes when at the block center
            for pos in range(0, hall_step_count):
                # if stop sent
                if self.stop_sent:
                    # update status
                    status = False
                    # show msg
                    msg = 'Comando de Pare recebido.'
                    _QMessageBox.critical(
                        self, 'Falha', msg, _QMessageBox.Ok
                    )
                    break
                # move
                target_position = target_interval + pos*hall_step
                status_tuple = self.move_and_retry(
                                   target_position, tolerance,
                                   driver_address, rotation_direction,
                                   motor_resolution, velocity,
                                   acceleration, linear_conversion,
                                   move_timeout
                )
                status = status_tuple[0]
                last_enc_pos = status_tuple[1]
                if status is False:
                    break
                # read hall sensor
                hall_volt = _multimeter.single_dc_read(wait=_utils.WAIT_MULTIMETER)
                # store hall and encoder measurements
                self.hall_sample_list.append(hall_volt)
                hall_cnt = 1 + pos + hall_step_count * (block_idx - 1)
                self.hall_sample_index_list.append(hall_cnt)
                self.encoder_sample_list_for_hall.append(last_enc_pos)
                # when all hall samples were taken for a block
#                # calculate direction of block
#                if pos == hall_step_count - 1:
#                    self.block_direction_list.append(
#                        self.calculate_block_direction(
#                            self.hall_sample_list[
#                                (hall_cnt - hall_step_count):hall_cnt
#                            ]
#                        )
#                    )
                # measure position at block center
                if pos == block_center:
                    # trigger pneumatic actuator
                    _driver.set_output_1_high(driver_address)
                    # wait pneumatic motion to finish
                    _time.sleep(wait_pneumatic_advance)
                    # read position probes
                    readings = _display.read_display(display_model, wait=wait_display)
                    # if error in reading, wait and try again
                    if _math.isnan(readings[0]):
                        _time.sleep(wait_display * 5)
                        readings = _display.read_display(
                            display_model, wait=wait_display
                        )
                        # if failed again, stop scan
                        if _math.isnan(readings[0]):
                            status = False
                            msg = 'Falha ao tentar ler display.'
                            _QMessageBox.critical(
                                self, 'Falha', msg, _QMessageBox.Ok
                            )
                            break
                    # store probe measurements
                    self.x_probe_sample_list.append(readings[0])
                    self.z_probe_sample_list.append(readings[1])
                    self.encoder_sample_list_for_probes.append(last_enc_pos)
                    # deactivate pneumatic actuator
                    _driver.set_output_1_low(driver_address)
                    # wait pneumatic motion to finish
                    _time.sleep(wait_pneumatic_retreat)
                
            if status is False:
                break

            # update progress bar
            pgb_value = self.ui.pgb_status.value()
            self.ui.pgb_status.setValue(pgb_value + pgb_step)

        if status is True:
            # store block number list
            self.block_number_list = block_list

            # calculate error for each sample
            x_ref = 0
            z_ref = 0
            for x_sample in self.x_probe_sample_list:
                self.x_probe_sample_error_list.append(x_sample - x_ref)
            for z_sample in self.z_probe_sample_list:
                self.z_probe_sample_error_list.append(z_sample - z_ref)

            self.update_graph_x_probe(
                self.block_number_list,
                self.x_probe_sample_list,
                self.x_probe_sample_error_list
            )
            self.update_graph_z_probe(
                self.block_number_list,
                self.z_probe_sample_list,
                self.z_probe_sample_error_list
            )
            self.update_graph_hall(
                self.encoder_sample_list_for_hall,
                self.hall_sample_list
            )

        # re-enable start scan button
        self.ui.pbt_start_scan.setEnabled(True)

        if status is False:
            self.ui.pgb_status.setValue(pgb_min)
            msg = 'Varredura falhou.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
        else:
            self.ui.pgb_status.setValue(pgb_max)
            msg = 'Varredura finalizada com sucesso.'
            _QMessageBox.information(self, 'Sucesso', msg, _QMessageBox.Ok)

        # unlock other tabs communication with display
        self.encoder_update_enabled = True

        return status

    def stop_scan(self):
        """ Stop scan measurement immediately  """
        # set flag for move_motor function to see
        self.stop_sent = True

        # check connection
        if not _driver.connected:
            msg = 'Driver nao conectado - falha ao tentar parar motor.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

        # stop motor
        try:
            driver_address = self.advanced_options.motor_driver_address
            _driver.stop_motor(driver_address)
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao tentar parar motor.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)

        return True

#    def calculate_block_direction(self, hall_readings):
#        """ Calculate magnet direction based on hall readings.
#
#            Input: list of hall sensor readings for a given magnet.
#            Return value: list of integers, same size as input,
#                          with the following meanings.
#                0: Magnet direction unknow
#                1: Magnet pointing UP
#                2: Magnet pointing to the RIGHT
#                3: Magnet pointing DOWN
#                4: Magnet pointing to the LEFT """
#        magnet_direction = 0
#
#        list_size = len(hall_readings)
#        if list_size < 3:
#            # return 'unknow direction'
#            return magnet_direction
#
#        # calculate vector of first 'differences'
#        diff_1 = []
#        for i in range(1, list_size):
#            diff_1.append(hall_readings[i] - hall_readings[i-1])
#        # calculate vector of second 'differences'
#        diff_2 = []
#        for i in range(1, len(diff_1)):
#            diff_2.append(diff_1[i] - diff_1[i-1])
#
#        pos_diff_1_cnt = sum(1 for i in diff_1 if i > 0)
#        pos_diff_2_cnt = sum(1 for i in diff_2 if i > 0)
#        neg_diff_1_cnt = sum(1 for i in diff_1 if i < 0)
#        neg_diff_2_cnt = sum(1 for i in diff_2 if i < 0)
#
#        # if field waveform is at first quadrant
#        if (pos_diff_1_cnt > neg_diff_1_cnt
#            and pos_diff_2_cnt < neg_diff_2_cnt):
#            magnet_direction = 1
#        # if field waveform is at second quadrant
#        elif (pos_diff_1_cnt < neg_diff_1_cnt
#            and pos_diff_2_cnt < neg_diff_2_cnt):
#            magnet_direction = 2
#        # if field waveform is at third quadrant
#        elif (pos_diff_1_cnt < neg_diff_1_cnt
#            and pos_diff_2_cnt > neg_diff_2_cnt):
#            magnet_direction = 3
#        # if field waveform is at forth quadrant
#        elif (pos_diff_1_cnt > neg_diff_1_cnt
#            and pos_diff_2_cnt > neg_diff_2_cnt):
#            magnet_direction = 4
#
#        return magnet_direction

    def clear_x_probe_graph_only(self):
        """ Clear data from ui probe x graph """
        self.ui.pw_x_probe.plotItem.curves.clear()
        self.legend_x_probe.clear()
        self.ui.pw_x_probe.clear()

        # list of plots
        self.graph_x_probe_plots = []
        return True

    def clear_x_probe_graph_and_data(self):
        """ Clear data from ui probe x graph and global data """
        self.clear_x_probe_graph_only()

        # lists of data for plots
        self.x_axis_x_probe = []
        self.y_axis_x_probe = []
        self.y_axis_x_probe_error = []
        return True

    def clear_z_probe_graph_only(self):
        """ Clear data from ui probe z graph """
        self.ui.pw_z_probe.plotItem.curves.clear()
        self.legend_z_probe.clear()
        self.ui.pw_z_probe.clear()

        # list of plots
        self.graph_z_probe_plots = []
        return True

    def clear_z_probe_graph_and_data(self):
        """ Clear data from ui probe z graph and global data """
        self.clear_z_probe_graph_only()

        # lists of data for plots
        self.x_axis_z_probe = []
        self.y_axis_z_probe = []
        self.y_axis_z_probe_error = []
        return True


    def clear_hall_graph_only(self):
        """ Clear data from ui hall graph """
        self.ui.pw_hall.plotItem.curves.clear()
        self.legend_hall.clear()
        self.ui.pw_hall.clear()

        # list of plots
        self.graph_hall_plots = []
        return True

    def clear_hall_graph_and_data(self):
        """ Clear data from ui hall graph and global data """
        self.clear_hall_graph_only()

        # lists of data for plots
        self.x_axis_hall = []
        self.y_axis_hall = []
        return True

    def clear_graphs(self):
        """ Clear data from ui graphs and their global data """
        # clear data
        self.clear_x_probe_graph_and_data()
        self.clear_z_probe_graph_and_data()
        self.clear_hall_graph_and_data()
        return True

    def clear(self):
        """Clear."""
        self.clear_graphs()
        # clear lists of measurement data
        self.hall_sample_list = []
        self.hall_sample_index_list = []
        self.encoder_sample_list_for_hall = []
        self.x_probe_sample_list = []
        self.x_probe_sample_error_list = []
        self.z_probe_sample_list = []
        self.z_probe_sample_error_list = []
        self.block_number_list = []
#        self.block_direction_list = []
        self.encoder_sample_list_for_probes = []
        # clear database table
        self.block_data.clear()
        self.hall_data.clear()

    def clean_str(self, text):
        """ Remove leading and trailing white spaces from
            input string and remove more than 1 sequential
            spaces between words and numbers """
        return re.sub(' +', ' ', text.strip())

    def configure_scan(self):
        self.clear()

        try:
            # get measurement names that define ID
            meas = self.ui.le_measurement_name.text()
            und = self.ui.le_undulator_name.text()
            cass = self.ui.le_cassette_name.text()

            # clean strings from unnecessary white spaces
            meas = self.clean_str(meas)
            und = self.clean_str(und)
            cass = self.clean_str(cass)

            # correct names on GUI after removing extra spaces
            self.ui.le_measurement_name.setText(meas)
            self.ui.le_undulator_name.setText(und)
            self.ui.le_cassette_name.setText(cass)

            # check that no name field is empty
            if meas == '' or und == '' or cass == '':
                msg = ("Campos nome da medida, ondulador e cassete"
                       " nao podem ser vazios"
                )
                _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
                return False

            # search if scan config already exists
            # if multiple exist, select first one
            scan_id = self.get_scan_id(meas, und, cass)
            if scan_id != -1:
                # check if there is a conflict between
                # the saved config and the parameters for
                # the new one with same id
                scan_data = self.access_scan_data.db_search_field('id', scan_id)
                scan_data = scan_data[0]
                if not (scan_data['start_reference_position']
                            == self.ui.sbd_start_reference_position.value()
                        and scan_data['scan_step_size']
                            == self.ui.sbd_scan_step_size.value()
                        and scan_data['hall_samples_per_block']
                            == self.ui.sb_hall_samples_per_block.value()
                        and scan_data['advanced_options_id']
                            == self.advanced_options.idn):
                    msg = ('Configuracao de varredura ja existe com ID '
                          +str(scan_id)
                          +' mas parametros nao coincidem com'
                          +' valores armazenados.'
                    )
                    _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
                    return False
                # load config
                self.ui.cmb_idn.setCurrentText(str(scan_id))
                self.load_db()

            if not self.update_configuration():
                return False
            if not self.save_db():
                return False

            self.global_config = self.config.copy()
            if not self.advanced_options.valid_data():
                msg = 'Opcoes avancadas invalidas.'
                _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
                return False
            return True

        except Exception:
            msg = 'Falha ao configurar varredura.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)
            return False

    def get_scan_id(self, measurement, undulator, cassette):
        # get scan id from DB
        scan_list = self.access_scan_data.db_search_collection(
            fields=['measurement_name', 'undulator_name', 'cassette_name', 'id'
            ],
            filters=[measurement, undulator, cassette, ''
            ]
        )
        if len(scan_list) == 0:
            return -1
        scan_list = sorted(scan_list, key=lambda k: k['id'])
        scan_data = scan_list[0]
        scan_id = scan_data['id']
        return scan_id

    def update_graphs_lines_visibility(self):
        """ Update graphs without changing data """
        self.update_graph_x_probe([],[],[])
        self.update_graph_z_probe([],[],[])
        self.update_graph_hall([],[])
        return True

    def update_graph_x_probe(self, x_axis_x_probe, y_axis_x_probe,
                              y_axis_x_probe_error):
        """ Update plots with the data provided. If y axis data, besides
            error data, is empty, keep the associated plot as is """
        try:
            # update global plot data
            if len(y_axis_x_probe) > 0:
                # clear graph and data
                self.clear_x_probe_graph_and_data()
                self.x_axis_x_probe = x_axis_x_probe
                self.y_axis_x_probe = y_axis_x_probe
                self.y_axis_x_probe_error = y_axis_x_probe_error
                # update summary
                avg = _statistics.mean(y_axis_x_probe)
                if len(y_axis_x_probe) > 1:
                    std = _statistics.stdev(y_axis_x_probe)
                else:
                    std = 0
                minx = min(y_axis_x_probe)
                maxx = max(y_axis_x_probe)
                summary = (
                    "X Pos: "
                    "avg={0:.3f} std={1:.3f} min={2:.3f} max={3:.3f}"
                )
                summary = summary.format(avg, std, minx, maxx)
                self.ui.la_x_probe_summary.setText(summary)
            else:
                self.clear_x_probe_graph_only()
            # check if should plot data
            if self.ui.chb_show_samples.isChecked():
                self.graph_x_probe_plots.append(
                    self.ui.pw_x_probe.plotItem.plot(
                        _np.array([]),
                        _np.array([]),
                        pen=self.colors[0],
                        symbol='o',
                        symbolPen=self.colors[0],
                        symbolSize=4,
                        symbolBrush=self.colors[0],
                        name='x')
                )
                self.graph_x_probe_plots[-1].setData(
                    self.x_axis_x_probe, self.y_axis_x_probe
                )
                self.legend_x_probe.addItem(
                    self.graph_x_probe_plots[-1], 'x'
                )
            # check if should plot error
            if self.ui.chb_show_error.isChecked():
                self.graph_x_probe_plots.append(
                    self.ui.pw_x_probe.plotItem.plot(
                        _np.array([]),
                        _np.array([]),
                        pen=self.colors[1],
                        symbol='o',
                        symbolPen=self.colors[1],
                        symbolSize=4,
                        symbolBrush=self.colors[1],
                        name='error')
                )
                self.graph_x_probe_plots[-1].setData(
                    self.x_axis_x_probe, self.y_axis_x_probe_error
                )
                self.legend_x_probe.addItem(
                    self.graph_x_probe_plots[-1], 'error'
                )
            self.ui.pw_x_probe.setLabel('bottom', 'Block index')
            self.ui.pw_x_probe.setLabel('left', 'Probe x [mm]')
            self.ui.pw_x_probe.showGrid(x=True, y=True)
            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao atualizar grafico do apalpador X.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def update_graph_z_probe(self, x_axis_z_probe, y_axis_z_probe,
                              y_axis_z_probe_error):
        """ Update plots with the data provided. If y axis data, besides
            error data, is empty, keep the associated plot as is """
        try:
            # update global plot data
            if len(y_axis_z_probe) > 0:
                # clear graph and data
                self.clear_z_probe_graph_and_data()
                self.x_axis_z_probe = x_axis_z_probe
                self.y_axis_z_probe = y_axis_z_probe
                self.y_axis_z_probe_error = y_axis_z_probe_error
                # update summary
                avg = _statistics.mean(y_axis_z_probe)
                if len(y_axis_z_probe) > 1:
                    std = _statistics.stdev(y_axis_z_probe)
                else:
                    std = 0
                minx = min(y_axis_z_probe)
                maxx = max(y_axis_z_probe)
                summary = (
                    "Z Pos: "
                    "avg={0:.3f} std={1:.3f} min={2:.3f} max={3:.3f}"
                )
                summary = summary.format(avg, std, minx, maxx)
                self.ui.la_z_probe_summary.setText(summary)
            else:
                self.clear_z_probe_graph_only()
            # check if should plot data
            if self.ui.chb_show_samples.isChecked():
                self.graph_z_probe_plots.append(
                    self.ui.pw_z_probe.plotItem.plot(
                        _np.array([]),
                        _np.array([]),
                        pen=self.colors[0],
                        symbol='o',
                        symbolPen=self.colors[0],
                        symbolSize=4,
                        symbolBrush=self.colors[0],
                        name='z')
                )
                self.graph_z_probe_plots[-1].setData(
                    self.x_axis_z_probe, self.y_axis_z_probe
                )
                self.legend_z_probe.addItem(
                    self.graph_z_probe_plots[-1], 'z'
                )
            # check if should plot error
            if self.ui.chb_show_error.isChecked():
                self.graph_z_probe_plots.append(
                    self.ui.pw_z_probe.plotItem.plot(
                        _np.array([]),
                        _np.array([]),
                        pen=self.colors[1],
                        symbol='o',
                        symbolPen=self.colors[1],
                        symbolSize=4,
                        symbolBrush=self.colors[1],
                        name='error')
                )
                self.graph_z_probe_plots[-1].setData(
                    self.x_axis_z_probe, self.y_axis_z_probe_error
                )
                self.legend_z_probe.addItem(
                    self.graph_z_probe_plots[-1], 'error'
                )
            self.ui.pw_z_probe.setLabel('bottom', 'Block index')
            self.ui.pw_z_probe.setLabel('left', 'Probe z [mm]')
            self.ui.pw_z_probe.showGrid(x=True, y=True)
            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao atualizar grafico do apalpador Z.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def update_graph_hall(self, x_axis_hall, y_axis_hall):
        """ Update plots with the data provided. If y axis data
            is empty, keep the associated plot as is """
        try:
            # update global plot data
            if len(y_axis_hall) > 0:
                # clear graph and data
                self.clear_hall_graph_and_data()
                self.x_axis_hall = x_axis_hall
                self.y_axis_hall = y_axis_hall
                # update summary
                avg = _statistics.mean(y_axis_hall)
                if len(y_axis_hall) > 1:
                    std = _statistics.stdev(y_axis_hall)
                else:
                    std = 0
                minx = min(y_axis_hall)
                maxx = max(y_axis_hall)
                summary = (
                    "Hall: avg={0:.2f} std={1:.2f} min={2:.2f} max={3:.2f}"
                )
                summary = summary.format(avg, std, minx, maxx)
                self.ui.la_hall_summary.setText(summary)
            else:
                self.clear_hall_graph_only()
            # check if should plot data
            if self.ui.chb_show_samples.isChecked():
                self.graph_hall_plots.append(
                    self.ui.pw_hall.plotItem.plot(
                        _np.array([]),
                        _np.array([]),
                        pen=self.colors[0],
                        symbol='o',
                        symbolPen=self.colors[0],
                        symbolSize=4,
                        symbolBrush=self.colors[0],
                        name='hall')
                )
                self.graph_hall_plots[-1].setData(
                    self.x_axis_hall, self.y_axis_hall
                )
                self.legend_hall.addItem(
                    self.graph_hall_plots[-1], 'hall'
                )
            self.ui.pw_hall.setLabel('bottom', 'Encoder Position')
            self.ui.pw_hall.setLabel('left', 'Hall reading [V]')
            self.ui.pw_hall.showGrid(x=True, y=True)
            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao atualizar grafico do sensor Hall.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def move_and_retry(self, target_position, tolerance, driver_address,
                       rotation_direction, motor_resolution, velocity,
                       acceleration, linear_conversion, timeout=10.0):
        """ Move motor until target position and retry if necessary
            while timeout is not exceeded.

            Return value: (true_if_success, last_encoder_position) """
        # check motor connection
        if not _driver.connected:
            msg = 'Driver nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return (False, 0.0)
        # check encoder connection
        if not _display.connected:
            msg = 'Encoder nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return (False, 0.0)

        # get display info
        display_model = self.advanced_options.display_model

        # interval to wait after display read command
        wait_display = _utils.WAIT_DISPLAY
        # interval to wait after move
        wait = _utils.WAIT_MOTION

        # motion mode is 'preset' (not continuous)
        mode = 0

        # var for encoder position
        encoder_position = 0.0

        # register motion start
        t_start = _time.time()

        try:
            while True:
                if self.stop_sent:
                    msg = 'Comando de Pare recebido.'
                    _QMessageBox.critical(
                        self, 'Falha', msg, _QMessageBox.Ok)
                    return (False, 0.0)
                # check timeout
                if (_time.time() - t_start) > timeout:
                    break
                # read encoder
                readings = _display.read_display(display_model, wait=wait_display)
                encoder_position = readings[2]
                # skip loop if failed to read encoder
                if _math.isnan(encoder_position):
                    _time.sleep(wait_display)
                    continue
                # update difference
                diff = target_position - encoder_position
                steps = _math.floor(
                        diff / (linear_conversion / motor_resolution)
                )
                curr_dir = rotation_direction
                if steps < 0:
                    if rotation_direction == '-':
                        curr_dir = '+'
                    else:
                        curr_dir = '-'
                # check if difference is relevant
                if abs(diff) <= abs(tolerance):
                    break
                # try to reach position
                if steps != 0 and not self.stop_sent:
                    # configure motor
                    if not _driver.config_motor(
                           driver_address,
                           mode,
                           curr_dir,
                           motor_resolution,
                           velocity,
                           acceleration,
                           steps):
                        msg = ('Falha ao tentar enviar configuracao'
                               ' para o driver.'
                        )
                        _QMessageBox.critical(
                            self, 'Falha', msg, _QMessageBox.Ok)
                        return (False, 0.0)
                    else:
                        # start motor motion if commanded to
                        _driver.move_motor(driver_address)
                        # wait for command reception
                        _time.sleep(wait)
                        # process gui events
                        _QApplication.processEvents()
                # wait motion to finish

                while (not _driver.ready(driver_address, wait)
                       and not self.stop_sent
                       and (_time.time() - t_start) < timeout):
                    _time.sleep(wait)
                    _QApplication.processEvents()

            # check if move was successful
            if not abs(diff) <= abs(tolerance):
                msg = 'Timeout de movimentacao esgotado em '+str(timeout)+' segundos.'
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok)
                # stop move
                _driver.stop_motor(driver_address)
                return (False, 0.0)

        except Exception:
            msg = 'Falha ao tentar mover motor.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)
            return (False, 0.0)

        return (True, encoder_position)

    def clear_block_data(self):
        """Clear block measurement data."""
        self.block_data.clear()

    def clear_hall_data(self):
        """Clear hall sensor waveform data."""
        self.hall_data.clear()

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
            msg = 'Falha ao tentar configurar driver.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)
            return False

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        super().connect_signal_slots()
        self.ui.pbt_start_hall_scan.clicked.connect(self.start_hall_scan)
        self.ui.pbt_start_position_scan.clicked.connect(self.start_position_scan)
        self.ui.pbt_stop_scan.clicked.connect(self.stop_scan)
        self.ui.pbt_load_data_db.clicked.connect(self.load_data_from_db)
        self.ui.pbt_save_data_db.clicked.connect(self.save_data_to_db)
        self.ui.sb_hall_samples_per_block.valueChanged.connect(self.hall_samples_even_only)
        self.ui.pbt_clear_all.clicked.connect(self.clear)
        self.ui.chb_show_error.stateChanged.connect(self.update_graphs_lines_visibility)
        self.ui.chb_show_samples.stateChanged.connect(self.update_graphs_lines_visibility)

    @property
    def advanced_options(self):
        """Return global advanced options."""
        dialog = _QApplication.instance().advanced_options_dialog
        return dialog.config

    def load_data_from_db(self):
        # clear current stored data
        self.clear()

        # arrays to hold data
        block_numbers = []
        encoder_data_probe = []
        x_probe_data = []
        x_probe_error_data = []
        z_probe_data = []
        z_probe_error_data = []
        encoder_data_hall = []
        hall_data = []
        hall_data_index = []

        # get element info to filter
        measurement = self.ui.le_measurement_name.text()
        undulator = self.ui.le_undulator_name.text()
        cassette = self.ui.le_cassette_name.text()

        # clean strings from unnecessary white spaces
        measurement = self.clean_str(measurement)
        undulator = self.clean_str(undulator)
        cassette = self.clean_str(cassette)

        # get scan id from DB
        scan_list = self.access_scan_data.db_search_collection(
            fields=['measurement_name', 'undulator_name', 'cassette_name', 'id'
            ],
            filters=[measurement, undulator, cassette, ''
            ]
        )
        if len(scan_list) == 0:
            msg = 'Dados da varredura nao encontrados.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False
        scan_data = scan_list[0]
        scan_id = scan_data['id']

        # read hall data from DB
        elem_list = self.access_hall_data.db_search_collection(
            fields=['scan_id', 'reading_index',
                    'hall_sensor_voltage', 'encoder_position'
            ],
            filters=[scan_id, '', '', ''
            ]
        )
        if len(elem_list) > 0:
            # sort by sample index
            elem_list = sorted(elem_list, key=lambda k: k['reading_index'])
            for elem in elem_list:
                # append to data to show
                encoder_data_hall.append(elem['encoder_position'])
                hall_data.append(elem['hall_sensor_voltage'])
                hall_data_index.append(elem['reading_index'])
            # plot hall data
            self.update_graph_hall(
                               hall_data_index,
                               hall_data
            )
        # read position probe data from DB
        elem_list = self.access_block_data.db_search_collection(
            fields=['scan_id', 'block_number', 'x_position',
                    'x_position_error', 'z_position', 'z_position_error',
                    'encoder_position'
            ],
            filters=[scan_id, '', '', '', '', '', ''
            ]
        )
        if len(elem_list) > 0:
            # sort by sample index
            elem_list = sorted(elem_list, key=lambda k: k['block_number'])
            for elem in elem_list:
                # append to data to show
                block_numbers.append(elem['block_number'])
                encoder_data_probe.append(elem['encoder_position'])
                x_probe_data.append(elem['x_position'])
                x_probe_error_data.append(elem['x_position_error'])
                z_probe_data.append(elem['z_position'])
                z_probe_error_data.append(elem['z_position_error'])
            # plot position probe data
            self.update_graph_x_probe(
                               block_numbers,
                               x_probe_data,
                               x_probe_error_data
            )
            self.update_graph_z_probe(
                               block_numbers,
                               z_probe_data,
                               z_probe_error_data
            )
        return True

    def save_data_to_db(self):
        try:
            if self.database_name is None:
                msg = 'Nome do arquivo de banco de dados invalido.'
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok)
                return False

            hall_samples_per_block = self.global_config.hall_samples_per_block

            # store hall data
            with _pyqtgraph.ProgressDialog(
                    "Savando dados do sensor hall...", 0, 100, cancelText=None) as dlg:

                hall_list_size = len(self.hall_sample_list)
                if hall_list_size > 0:
                    pgb_step = 100/hall_list_size
                    # verify if some point already exists
                    data_list = self.access_hall_data.db_search_collection(
                        fields=['scan_id', 'reading_index'],
                        filters=[self.global_config.idn, '']
                    )
                    if any(e['reading_index'] >= self.hall_sample_index_list[0] 
                           and e['reading_index'] <= self.hall_sample_index_list[-1] for e in data_list):
                        msg = (
                               "As novas medidas do sensor hall"
                               " sobrepoem o indice de medidas ja salvas."
                               " Por favor, apague os pontos antigos"
                               " do banco de dados, se necessario, e"
                               " tente novamente."
                        )
                        _QMessageBox.critical(
                            self, 'Conflito', msg, _QMessageBox.Ok)
                        return False

                # save all new hall data entries
#                j = 0
                for i in range(0, hall_list_size):
#                    if (i % hall_samples_per_block == 0 and i != 0):
#                        j = j + 1
                    self.hall_data.scan_id = self.global_config.idn
#                    self.hall_data.block_number = self.block_number_list[j]
                    self.hall_data.reading_index = self.hall_sample_index_list[i]
                    self.hall_data.hall_sensor_voltage = self.hall_sample_list[i]
                    self.hall_data.encoder_position = self.encoder_sample_list_for_hall[i]
                    self.hall_data.db_update_database(
                        self.database_name, mongo=self.mongo,
                        server=self.server
                    )
                    self.hall_data.db_save()
                    # process gui events
                    _QApplication.processEvents()
                    # update progress bar
                    dlg.setValue(pgb_step * i)

            # store block data
            with _pyqtgraph.ProgressDialog(
                    "Salvando dados de posicao...", 0, 100, cancelText=None) as dlg:

                block_list_size = len(self.block_number_list)
                if block_list_size > 0:
                    pgb_step = 100/block_list_size
                    # verify if some point already exists
                    data_list = self.access_block_data.db_search_collection(
                        fields=['scan_id', 'block_number'],
                        filters=[self.global_config.idn, '']
                    )
                    if any(e['block_number'] >= self.block_number_list[0] 
                           and e['block_number'] <= self.block_number_list[-1] for e in data_list):
                        msg = (
                               "As novas medidas de posicao sobrepoem"
                               " medidas ja salvas. Por favor, apague "
                               " os pontos antigos do banco de dados"
                               ", se necessario, e tente novamente."
                        )
                        _QMessageBox.critical(
                            self, 'Conflito', msg, _QMessageBox.Ok)
                        return False

                # save all new block data entries
                for i in range(0, block_list_size):
                    self.block_data.scan_id = self.global_config.idn
                    self.block_data.block_number = self.block_number_list[i]
#                    self.block_data.block_direction = self.block_direction_list[i]
                    self.block_data.x_position = self.x_probe_sample_list[i]
                    self.block_data.x_position_error = self.x_probe_sample_error_list[i]
                    self.block_data.z_position = self.z_probe_sample_list[i]
                    self.block_data.z_position_error = self.z_probe_sample_error_list[i]
                    self.block_data.encoder_position = self.encoder_sample_list_for_probes[i]
                    self.block_data.db_update_database(
                        self.database_name, mongo=self.mongo,
                        server=self.server
                    )
                    self.block_data.db_save()
                    # process gui events
                    _QApplication.processEvents()
                    # update progress bar
                    dlg.setValue(pgb_step * i)

                return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao salvar dados.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def hall_samples_even_only(self):
        """ Allow only even hall sample counts """
        value = self.ui.sb_hall_samples_per_block.value()
        if value % 2 != 0:
            self.ui.sb_hall_samples_per_block.setValue(value + 1)
        return True

    def homing(self):
        """ Move motor to limit switch at the beginning of range.
            If the motor direction is reversed, than the positive
            limit switch is used as home switch. """
        # clear stop flag
        self.stop_sent = False

        # status flag
        status = True

        # check motor connection
        if not _driver.connected:
            msg = 'Driver nao conectado.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            status = False
            return status

        # disable move and homing buttons
        self.ui.pbt_move_motor.setEnabled(False)
        self.ui.pbt_homing.setEnabled(False)

        try:
            # interval to wait after move
            wait_motion = _utils.WAIT_MOTION
            # interval to wait before command is received
            wait_cmd = _utils.WAIT_DRIVER
            # interval to wait after pneumatic actuator command
            wait_pneumatic_retreat = (
                self.advanced_options.pneumatic_retreat_wait
            )

            # motor info
            driver_address = self.advanced_options.motor_driver_address
            resolution = self.advanced_options.motor_resolution
            rotation_direction = self.advanced_options.motor_rotation_direction
            velocity = self.advanced_options.motor_velocity
            acceleration = self.advanced_options.motor_acceleration
            move_timeout = self.advanced_options.move_timeout

            # mode = preset (not continuous)
            mode = 0

            # register move start
            t_start = _time.time()

            # move start flag
            move_started = False

            # make sure pneumatic actuator is off
            _driver.set_output_1_low(driver_address)
            # wait pneumatic motion to finish
            _time.sleep(wait_pneumatic_retreat)

            # move towards the negative or positive limit switch
            if not self.stop_sent:
                if rotation_direction == '+':
                    if not _driver.move_to_negative_limit(
                            driver_address,
                            velocity,
                            acceleration,
                            wait_cmd):
                        msg = ('Falha ao tentar enviar comando para'
                               ' driver.'
                        )
                        _QMessageBox.critical(
                            self, 'Falha', msg, _QMessageBox.Ok)
                        status = False
                    else:
                        move_started = True
                else:
                    if not _driver.move_to_positive_limit(
                            driver_address,
                            velocity,
                            acceleration,
                            wait_cmd):
                        msg = ('Falha ao tentar enviar comando para'
                               ' driver.'
                        )
                        _QMessageBox.critical(
                            self, 'Falha', msg, _QMessageBox.Ok)
                        status = False
                    else:
                        move_started = True

            # wait motor stop
            if move_started:
                _time.sleep(wait_motion)
                t_curr = _time.time()
                while (not _driver.ready(driver_address)
                       and not self.stop_sent
                       and not (t_curr - t_start) > move_timeout):
                    t_curr = _time.time()
                    _time.sleep(wait_motion)
                    _QApplication.processEvents()

                if (t_curr - t_start) > move_timeout:
                    _driver.stop_motor(driver_address)
                    msg = 'Timeout de Homing - parando motor.'
                    _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
                    status = False
                if self.stop_sent:
                    status = False

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Homing falhou.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            status = False

        # re-enable move and homing buttons
        self.ui.pbt_move_motor.setEnabled(True)
        self.ui.pbt_homing.setEnabled(True)

        return status

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
                msg = 'Multimetro nao conectado.'
                _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
                return False

            # interval to wait after command
            wait = _utils.WAIT_MULTIMETER

            # configure multimeter and read measurement
            _multimeter.inst.write(b':MEAS:VOLT:DC? DEF,DEF\r\n')
            _time.sleep(wait)
            reading = _multimeter.inst.readline().decode('utf-8')
            reading = reading.replace('\r\n','')

            self.ui.le_hall_sensor_voltage.setText(reading)

        except Exception:
            msg = 'Falha ao tentar ler sensor hall.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            _traceback.print_exc(file=_sys.stdout)
            return False

    def update_configuration(self):
        """Update configuration parameters."""
        try:
            self.config.advanced_options_id = self.advanced_options.idn
            return super().update_configuration(clear=False)

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            self.config.clear()
            return False
