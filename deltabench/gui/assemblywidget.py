# -*- coding: utf-8 -*-

"""Assembly widget."""

import os as _os
import sys as _sys
import time as _time
import numpy as _np
import math as _math
import re as _re
import pandas as _pd
import warnings as _warnings
import traceback as _traceback
import qtpy.uic as _uic
from qtpy.QtCore import (
    Qt as _Qt,
    QTimer as _QTimer
)
from qtpy.QtWidgets import (
    QWidget as _QWidget,
    QMessageBox as _QMessageBox,
    QApplication as _QApplication,
    QFileDialog as _QFileDialog,
    )
import qtpy.uic as _uic

from deltabench.gui import utils as _utils
from deltabench.devices import (
    driver as _driver,
)
from imautils.db import database as _database
import deltabench.data.measurement as _measurement

from PyQt5.QtGui import QPixmap as _QPixmap


class AssemblyWidget(_QWidget):
    """Assembly widget class for the control application."""

    # encoder info update time [sec]
    _update_interval = 0.1

    def __init__(self, parent=None):
        """Set up the ui."""
        super().__init__(parent)

        # setup the ui
        uifile = _utils.get_ui_file(self)
        self.ui = _uic.loadUi(uifile, self)
        self.assembly_data = _measurement.AssemblyData()

        self.timer1 = _QTimer()
        self.timer1.timeout.connect(self.periodic_update)

        # create object to use database function
        self.access_assembly_data = _database.DatabaseCollection(
            database_name=self.database_name,
            collection_name=_measurement.AssemblyData.collection_name,
            mongo=self.mongo,
            server=self.server
        )

        # properties
        self.assembly_id = -1
        self.cassette_name = ''
        self.list_position = 0
        self.block_count = 0
        self.block_list = []
        self.cassette_list = [
            _utils.CASSETTE_1_NAME,
            _utils.CASSETTE_2_NAME,
            _utils.CASSETTE_3_NAME,
            _utils.CASSETTE_4_NAME
        ]

        # stylesheets for buttons
        self.button_on_stylesheet = "background-color: rgb(0,255,0);"
        self.button_off_stylesheet = "background-color: rgb(232,232,232);"

        # create dictionary of images for LEDs
        self.led_images = {}
        self.led_images['green'] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                          'green_light.png')
        )
        self.led_images['red'] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                          'red_light.png')
        )

        # create dictionary of images for magnet orientations
        self.block_types = {}
        self.block_types[_utils.PREFIX_BLOCK_TYPE_1] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'Block-BA.png')
        )
        self.block_types[_utils.PREFIX_BLOCK_TYPE_2] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'Block-BB.png')
        )
        self.block_types[_utils.PREFIX_BLOCK_TYPE_3] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'Block-BC.png')
        )
        self.block_types[_utils.PREFIX_BLOCK_TYPE_4] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'Block-TA.png')
        )
        self.block_types[_utils.PREFIX_BLOCK_TYPE_5] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'Block-TB.png')
        )
        self.block_types[_utils.PREFIX_BLOCK_TYPE_6] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'Block-TC.png')
        )
        self.block_types['end'] = _QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'check-circle-outline.png')
        )
        self.block_types['none'] = _QPixmap()

        # initialize sheet name combo box with cassette names
        self.ui.cmb_sheet_name.addItems(self.cassette_list)

        # set push buttons background to off style
        self.set_all_motion_pbt_stylesheet(self.button_off_stylesheet)

        # clear GUI displayed data
        self.reset_all_display_data()

        # connect widgets to functions
        self.connect_signal_slots()

        # start periodic info update
        self.timer1.start(self._update_interval*1000)

    @property
    def database_name(self):
        """Database name."""
        return _QApplication.instance().database_name

    @property
    def mongo(self):
        """MongoDB database."""
        return _QApplication.instance().mongo

    @property
    def server(self):
        """Server for MongoDB database."""
        return _QApplication.instance().server

    @property
    def directory(self):
        """Return the default directory."""
        return _QApplication.instance().directory

    @property
    def advanced_options(self):
        """Return global advanced options."""
        dialog = _QApplication.instance().advanced_options_dialog
        return dialog.config

    @property
    def encoder_update_enabled(self):
        """Return the global encoder update status."""
        return _QApplication.instance().encoder_update_enabled

    @property
    def linear_encoder_position(self):
        """Return the global linear encoder position."""
        return _QApplication.instance().linear_encoder_position

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        self.ui.pbt_open_file.clicked.connect(self.select_file)
        self.ui.pbt_previous.clicked.connect(self.show_previous)
        self.ui.pbt_next.clicked.connect(self.show_next)
        self.ui.pbt_up.pressed.connect(self.move_up)
        self.ui.pbt_up.released.connect(self.stop_all_motors)
        self.ui.pbt_down.pressed.connect(self.move_down)
        self.ui.pbt_down.released.connect(self.stop_all_motors)
        self.ui.pbt_fwd.pressed.connect(self.move_fwd)
        self.ui.pbt_fwd.released.connect(self.stop_all_motors)
        self.ui.pbt_rev.pressed.connect(self.move_rev)
        self.ui.pbt_rev.released.connect(self.stop_all_motors)
        self.ui.chb_block_previous_btn.stateChanged.connect(
            self.update_navigation_buttons_state
        )
        self.ui.cmb_sheet_name.currentIndexChanged.connect(
            self.select_cassette
        )

    def periodic_update(self):
        """ Periodically update relevant info on widget """
        # update LCD with global encoder data
        if self.encoder_update_enabled:
            self.ui.lcd_linear_encoder_position.display(
                self.linear_encoder_position
            )

    def select_cassette(self):
        """ Open already selected file """
        # check if a file is already selected
        filename = self.ui.le_filename.text()
        filename = filename.strip(' ')

        # if no file selected, just silently return false
        if filename == '':
            return False

        # open file and store data
        status = self.open_file(filename)
        return status

    def select_file(self):
        """ Allow user to select file through dialog and open it """
        # let user select file
        filename, filter_choice = _QFileDialog.getOpenFileName(
            self,
            caption="Open File",
            directory=self.directory,
            filter= ("Spreadsheet ("
                     "*.xls *.xlsx *.xlsm *.xlsb *.odf *.ods *.odt)"),
        )

        # if no file selected, just silently return false
        if filename == '':
            return False

        # open file and store data
        status = self.open_file(filename)
        return status

    def open_file(self, filename):
        """ Open file, read it and store file contents into
            properties """
        try:
            # clear GUI displayed data
            self.reset_all_display_data()

            # update filename in GUI
            self.ui.le_filename.setText(filename)

            # select cassette name as first in the list
            cassette = self.ui.cmb_sheet_name.currentText()
            self.cassette_name = cassette

            # check if filename has a DB id
            entry_list = self.access_assembly_data.db_search_collection(
                fields=['filename', 'cassette', 'id', 'last_position'
                ],
                filters=[filename, cassette, '', '']
            )

            # if no id, create entry in DB
            if len(entry_list) == 0:
                # set position to first block
                self.list_position = 0
                # clear object to hold DB entry data
                self.assembly_data.clear()
                # assign filename and cassette to DB entry object
                self.assembly_data.filename = filename
                self.assembly_data.cassette = cassette
                # make sure we are pointing to the right DB
                self.assembly_data.db_update_database(
                    database_name=self.database_name,
                    mongo=self.mongo, server=self.server
                )
                # save DB entry
                self.assembly_id = self.assembly_data.db_save()
            else:
                # store entry id
                self.assembly_id = entry_list[0]['id']
                # set position to last position + 1
                if entry_list[0]['last_position'] is None:
                    self.list_position = 0
                else:
                    self.list_position = (
                        entry_list[0]['last_position'] + 1
                    )

            # read file and store block list
            self.read_file(filename, cassette)

            # check if initial position in within list
            if self.list_position > self.block_count:
                raise RuntimeError(
                    'Initial position is inconsistent'
                    ' with block list size'
                )

            # update file overview on GUI
            self.update_data_overview()

            # update individual block data display
            self.display_block_data(self.list_position)

            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            # reset data
            self.reset_block_list_data()
            # reset display
            self.reset_all_display_data()
            msg = 'Falha ao abrir arquivo.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def update_data_overview(self):
        """ Display block list data as formatted text """
        try:
            # create formatted text with all block data
            formatted_text = ''
            idx = 1
            for block in self.block_list:
                line = '{0:03} | '.format(idx)
                idx += 1
                for k, v in block.items():
                    info = '{0}: {1}; '.format(k,v)
                    line += '{:<20}'.format(info)
                formatted_text += line + '\r\n'
            # show data in ready only text editor
            self.ui.te_file_data.setText(formatted_text)

        except Exception:
            # print error
            _traceback.print_exc(file=_sys.stdout)
            # clear text on GUI
            self.ui.te_file_data.setText('')
            # show error message dialog
            msg = 'Falha ao atualizar visao geral da lista de blocos.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def read_file(self, filename, sheet_name):
        """ Read spreadsheet data """
        try:
            # clear stored block list
            self.block_list = []
            # read spreadsheet
            sheet = _pd.read_excel(
                filename, sheet_name=sheet_name, dtype=str
            )
            sheet.replace({"\n": ""}, inplace=True, regex=True)
            sheet.fillna("", inplace=True)
            for n, row in sheet.iterrows():
                # create empty block dictionary
                block = {}
                # store block name
                name = row[_utils.BLOCK_NAME_COLUMN_TITLE]
                if name.strip(" \t") == "":
                    raise ValueError(
                        'Invalid block name at row {0}'.format(n)
                    )
                block[_utils.BLOCK_NAME_COLUMN_TITLE] = name
                # store block flip boolean state
                flip_or_not = row[_utils.BLOCK_FLIP_BOOL_COLUMN_TITLE]
                block[_utils.BLOCK_FLIP_BOOL_COLUMN_TITLE] = flip_or_not
                # store subcassette name
                subcass = row[_utils.SUBCASSETTE_COLUMN_TITLE]
                block[_utils.SUBCASSETTE_COLUMN_TITLE] = subcass
                # store block type
                block_type = name[:2]
                if ( block_type != _utils.PREFIX_BLOCK_TYPE_1
                     and block_type != _utils.PREFIX_BLOCK_TYPE_2
                     and block_type != _utils.PREFIX_BLOCK_TYPE_3
                     and block_type != _utils.PREFIX_BLOCK_TYPE_4
                     and block_type != _utils.PREFIX_BLOCK_TYPE_5
                     and block_type != _utils.PREFIX_BLOCK_TYPE_6):
                    raise ValueError(
                        'Invalid block type prefix at row {0}'.format(n)
                    )
                block['tipo'] = block_type
                # append block to list
                self.block_list.append(block)
            # update block list size
            self.block_count = len(self.block_list)

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao ler arquivo de entrada.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def update_navigation_buttons_state(self):
        """ Compare current position in block list
            and disable/enable navigation push buttons
            as necessary. """
        # check lower limit
        if self.list_position == 0:
            self.ui.pbt_previous.setEnabled(False)
        else:
            self.ui.pbt_previous.setEnabled(True)

        # check upper limit
        if self.list_position == self.block_count:
            self.ui.pbt_next.setEnabled(False)
        else:
            self.ui.pbt_next.setEnabled(True)

        # if return button is blocked, then disable it
        if self.ui.chb_block_previous_btn.isChecked():
            self.ui.pbt_previous.setEnabled(False)

        return True

    def show_previous(self):
        """ Show previous block in list on GUI """
        # first decrement to display the previous block
        self.list_position -= 1
        # update display
        self.display_block_data(self.list_position)
        # then decrement again since displayed block should
        # still be assembled
        self.list_position -= 1
        # then save
        self.update_db_with_curr_state()
        return True

    def show_next(self):
        """ Show next block in list on GUI """
        # update DB and display
        if (self.block_count > 0
            and self.list_position < self.block_count):
            # first update DB then increment pos
            self.update_db_with_curr_state()
            self.list_position += 1
            # update display
            self.display_block_data(self.list_position)
            return True
        else:
            msg = 'Proxima posicao invalida.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def display_block_data(self, position):
        """ Update individual block data shown to user

            Input arguments:
                position: position of block in block list """
        try:
            # update cassette name
            self.ui.la_cassette_name.setText(self.cassette_name)

            # check if position is equal to block list length
            # meaning that all list is complete
            if position == self.block_count:
                # reset block display data
                self.reset_block_display_data()
                # show end of assembly image
                pixmap = self.block_types['end']
                self.ui.la_block_image.setPixmap(pixmap)
                # update counter
                self.ui.la_block_count.setText(
                    '{0}/{0}'.format(self.block_count)
                )
                # update navigation buttons
                self.update_navigation_buttons_state()
                # show end of assembly message
                msg = 'Montagem do Cassete finalizada.'
                _QMessageBox.information(
                    self, 'Finalizado', msg, _QMessageBox.Ok
                )
                return True

            # get block dictionary
            block = self.block_list[position]

            # update image
            block_type = block['tipo']
            pixmap = self.block_types[block_type]
            self.ui.la_block_image.setPixmap(pixmap)

            # update flip status text and LED
            flip = block[_utils.BLOCK_FLIP_BOOL_COLUMN_TITLE]
            if flip == '0':
                self.ui.la_flip_block_msg.setText('Direcao Normal')
                self.ui.la_flip_block_img.setPixmap(self.led_images['green'])
                self.ui.la_flip_block_img.setEnabled(True)
            if flip == '1':
                self.ui.la_flip_block_msg.setText('Direcao Invertida!')
                self.ui.la_flip_block_img.setPixmap(self.led_images['red'])
                self.ui.la_flip_block_img.setEnabled(True)

            # update name
            name = block[_utils.BLOCK_NAME_COLUMN_TITLE]
            self.ui.la_block_name.setText(name)

            # update subcassette name
            subcass = block[_utils.SUBCASSETTE_COLUMN_TITLE]
            self.ui.la_subcassette_name.setText(subcass)

            # update counter
            self.ui.la_block_count.setText(
                '{0}/{1}'.format(position+1, self.block_count)
            )

            # update navigation buttons
            self.update_navigation_buttons_state()
            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao exibir dados do bloco.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def reset_block_list_data(self):
        """ Reset block list data """
        try:
            self.assembly_id = -1
            self.cassette_name = ''
            self.list_position = 0
            self.block_count = 0
            self.block_list = []
            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao realizar reset dos dados da lista de blocos.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def reset_all_display_data(self):
        """ Reset all display data """
        # clear filename
        self.ui.le_filename.setText('')
        # clear displayed file data
        self.ui.te_file_data.setText('')
        # clear displayed block data
        self.reset_block_display_data()
        return  True

    def reset_block_display_data(self):
        """ Reset block data shown to user """
        try:
            block_type = 'none'
            pixmap = self.block_types[block_type]
            self.ui.la_block_image.setPixmap(pixmap)

            self.ui.la_cassette_name.setText('')
            self.ui.la_subcassette_name.setText('')
            self.ui.la_block_name.setText('')

            self.ui.la_block_count.setText('')

            self.ui.la_flip_block_img.setEnabled(False)
            self.ui.la_flip_block_msg.setText('')

            self.pbt_previous.setEnabled(False)
            self.pbt_next.setEnabled(False)
            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao realizar reset da exibicao de dados do bloco.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def move_up(self):
        """ Start continuous upward move of vertical motor """
        try:
            # check connection
            if not _driver.connected:
                msg = 'Driver nao conectado.'
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok
                )
                return False

            # light button up
            self.ui.pbt_up.setStyleSheet(self.button_on_stylesheet)

            # motion is continuous
            mode = 1

            # get motor info
            driver_address = (
                self.advanced_options.vertical_motor_driver_address
            )
            motor_resolution = (
                self.advanced_options.vertical_motor_resolution
            )
            rotation_direction = (
                self.advanced_options.vertical_motor_rotation_direction
            )
            velocity = self.ui.sbd_vertical_velocity.value()
            acceleration = self.ui.sbd_vertical_acceleration.value()

            # dummy value (ignored in continuous mode)
            steps = 1

            # configure up direction as the opposite of
            # the down direction
            if rotation_direction == '+':
                curr_dir = '-'
            else:
                curr_dir = '+'

            # configure motor
            if not _driver.config_motor(
                    driver_address, mode,
                    curr_dir, motor_resolution,
                    velocity, acceleration,
                    steps):
                # update status
                raise RuntimeError('Failed to config motor.')
            else:
                # start motor motion if commanded to
                _driver.move_motor(driver_address)
                # wait for command reception
                _time.sleep(_utils.WAIT_DRIVER)

            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            # stop motor
            self.stop_all_motors()
#            # light button down
#            self.set_all_motion_pbt_stylesheet(self.button_off_stylesheet)
            # show error message dialog
            msg = 'Falha ao enviar configuracao para motor.'
            _QMessageBox.critical(
                self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def move_down(self):
        """ Start continuous downward move of vertical motor """
        try:
            # check connection
            if not _driver.connected:
                msg = 'Driver nao conectado.'
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok
                )
                return False

            # light button up
            self.ui.pbt_down.setStyleSheet(self.button_on_stylesheet)

            # motion is continuous
            mode = 1

            # get motor info
            driver_address = (
                self.advanced_options.vertical_motor_driver_address
            )
            motor_resolution = (
                self.advanced_options.vertical_motor_resolution
            )
            rotation_direction = (
                self.advanced_options.vertical_motor_rotation_direction
            )
            velocity = self.ui.sbd_vertical_velocity.value()
            acceleration = self.ui.sbd_vertical_acceleration.value()

            # dummy value (ignored in continuous mode)
            steps = 1

            # configure down direction as specified by advanced
            # options
            curr_dir = rotation_direction

            # configure motor
            if not _driver.config_motor(
                    driver_address, mode,
                    curr_dir, motor_resolution,
                    velocity, acceleration,
                    steps):
                # update status
                raise RuntimeError('Failed to config motor.')
            else:
                # start motor motion if commanded to
                _driver.move_motor(driver_address)
                # wait for command reception
                _time.sleep(_utils.WAIT_DRIVER)

            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            # stop motor
            self.stop_all_motors()
#            # light button down
#            self.set_all_motion_pbt_stylesheet(self.button_off_stylesheet)
            # show error message dialog
            msg = 'Falha ao enviar configuracao para motor.'
            _QMessageBox.critical(
                self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def move_fwd(self):
        """ Start continuous forward move of horizontal motor """
        try:
            # check connection
            if not _driver.connected:
                msg = 'Driver nao conectado.'
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok
                )
                return False

            # light button up
            self.ui.pbt_fwd.setStyleSheet(self.button_on_stylesheet)

            # motion is continuous
            mode = 1

            # get motor info
            driver_address = (
                self.advanced_options.motor_driver_address
            )
            motor_resolution = (
                self.advanced_options.motor_resolution
            )
            rotation_direction = (
                self.advanced_options.motor_rotation_direction
            )
            velocity = self.ui.sbd_horizontal_velocity.value()
            acceleration = self.ui.sbd_horizontal_acceleration.value()

            # dummy value (ignored in continuous mode)
            steps = 1

            # configure forward direction as specified by advanced
            # options
            curr_dir = rotation_direction

            # configure motor
            if not _driver.config_motor(
                    driver_address, mode,
                    curr_dir, motor_resolution,
                    velocity, acceleration,
                    steps):
                # update status
                raise RuntimeError('Failed to config motor.')
            else:
                # start motor motion if commanded to
                _driver.move_motor(driver_address)
                # wait for command reception
                _time.sleep(_utils.WAIT_DRIVER)

            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            # stop motor
            self.stop_all_motors()
#            # light button down
#            self.set_all_motion_pbt_stylesheet(self.button_off_stylesheet)
            # show error message dialog
            msg = 'Falha ao enviar configuracao para motor.'
            _QMessageBox.critical(
                self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def move_rev(self):
        """ Start continuous reverse move of horizontal motor """
        try:
            # check connection
            if not _driver.connected:
                msg = 'Driver nao conectado.'
                _QMessageBox.critical(
                    self, 'Falha', msg, _QMessageBox.Ok
                )
                return False

            # light button up
            self.ui.pbt_rev.setStyleSheet(self.button_on_stylesheet)

            # motion is continuous
            mode = 1

            # get motor info
            driver_address = (
                self.advanced_options.motor_driver_address
            )
            motor_resolution = (
                self.advanced_options.motor_resolution
            )
            rotation_direction = (
                self.advanced_options.motor_rotation_direction
            )
            velocity = self.ui.sbd_horizontal_velocity.value()
            acceleration = self.ui.sbd_horizontal_acceleration.value()

            # dummy value (ignored in continuous mode)
            steps = 1

            # configure reverse direction as the opposite of
            # the forward direction
            if rotation_direction == '+':
                curr_dir = '-'
            else:
                curr_dir = '+'

            # configure motor
            if not _driver.config_motor(
                    driver_address, mode,
                    curr_dir, motor_resolution,
                    velocity, acceleration,
                    steps):
                # update status
                raise RuntimeError('Failed to config motor.')
            else:
                # start motor motion if commanded to
                _driver.move_motor(driver_address)
                # wait for command reception
                _time.sleep(_utils.WAIT_DRIVER)

            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            # stop motor
            self.stop_all_motors()
#            # light button down
#            self.set_all_motion_pbt_stylesheet(self.button_off_stylesheet)
            # show error message dialog
            msg = 'Falha ao enviar configuracao para motor.'
            _QMessageBox.critical(
                self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def set_all_motion_pbt_stylesheet(self, style):
        """ Set the stylesheet of all move push buttons in this widget
        """
        self.ui.pbt_up.setStyleSheet(style)
        self.ui.pbt_down.setStyleSheet(style)
        self.ui.pbt_fwd.setStyleSheet(style)
        self.ui.pbt_rev.setStyleSheet(style)

    def stop_all_motors(self):
        """ Stop horizontal and vertical motors """
        address1 = self.advanced_options.motor_driver_address
        address2 = self.advanced_options.vertical_motor_driver_address
        self.stop_motor(address1)
        self.stop_motor(address2)
        # light command buttons down
        self.set_all_motion_pbt_stylesheet(self.button_off_stylesheet)
        return True

    def stop_motor(self, address):
        # check connection
        if not _driver.connected:
            msg = ('Driver nao conectado -'
                  ' falha ao tentar parar motor {0}.'
            )
            msg = msg.format(address)
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

        # send stop command
        try:
            _driver.stop_motor(address)
            return True
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao tentar parar motor {0}.'
            msg = msg.format(address)
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return False

    def update_db_with_curr_state(self):
        """ Update DB entry for the current file with the
            current assemly state """
        # clear object to hold DB entry data
        self.assembly_data.clear()
        # assign data to DB entry object
        self.assembly_data.filename = self.ui.le_filename.text()
        self.assembly_data.cassette = self.cassette_name
        self.assembly_data.last_position = self.list_position
        self.assembly_data.used_blocks = [
            e[_utils.BLOCK_NAME_COLUMN_TITLE]
            for e in self.block_list[:self.list_position+1]
        ]
        # make sure we are pointing to the right DB
        self.assembly_data.db_update_database(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server
        )
        # update DB entry
        self.assembly_data.db_update(self.assembly_id)

