"""Database tables widgets."""

import os as _os
import sys as _sys
import time as _time
import numpy as _np
import math as _math
import re as _re
import warnings as _warnings
import traceback as _traceback
import qtpy.uic as _uic
from qtpy.QtCore import Qt as _Qt
from qtpy.QtWidgets import (
    QWidget as _QWidget,
    QMessageBox as _QMessageBox,
    QApplication as _QApplication,
    QFileDialog as _QFileDialog,
    )
import qtpy.uic as _uic

from deltabench.gui import utils as _utils

from PyQt5.QtGui import QPixmap        



class AssemblyWidget(_QWidget):
    """Assembly widget class for the control application."""

    def __init__(self, parent=None):
        """Set up the ui."""
        super().__init__(parent)

        # setup the ui
        uifile = _utils.get_ui_file(self)
        self.ui = _uic.loadUi(uifile, self)

        # properties
        self.file_data = ''
        self.list_position = 0
        self.block_count = 0
        self.block_list = []

        # create dictionary of images for magnet orientations
        self.block_types = {}
        self.block_types['up'] = QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'arrow-up-bold-outline.png')
        )
        self.block_types['down'] = QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'arrow-down-bold-outline.png')
        )
        self.block_types['left'] = QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'arrow-left-bold-outline.png')
        )
        self.block_types['right'] = QPixmap(
            _os.path.join('deltabench','resources', 'img',
                         'arrow-right-bold-outline.png')
        )
        self.block_types['none'] = QPixmap()

        # connect widgets to functions
        self.connect_signal_slots()

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

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        self.ui.pbt_open_file.clicked.connect(self.open_file)
        self.ui.pbt_previous.clicked.connect(self.show_previous)
        self.ui.pbt_next.clicked.connect(self.show_next)

    def open_file(self):
        """ Allow user to select file through dialog and store
            file contents into properties """
        try:
            # let user select file
            filename, filter_choice = _QFileDialog.getOpenFileName(
                self,
                caption="Open File",
                directory=self.directory,
                filter="Text Files (*.txt *.csv)",
            )
            # if no file selected, just silently return false
            if filename == '':
                return False

            # update filename in GUI
            self.ui.le_filename.setText(filename)

            # read file
            self.file_data = self.read_file(filename)

            # update block data list
            self.block_list = self.process_data(self.file_data)

            # set block count
            self.block_count = len(self.block_list)

            # update info on GUI
            self.ui.te_file_data.setText(self.file_data)

            # reset current block info being displayed
            self.reset_block_data()

            # set position to first block
            self.list_position = 0

            # disable 'previous' button
            self.ui.pbt_previous.setEnabled(False)

            # update display
            self.display_block_data(self.list_position)

            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to open file.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

    def process_data(self, text):
        """ Process file text and return organized data """
        fail_response = []
        try:
            all_blocks = []
            for line in text.replace('\r','').split('\n'):
                block = {}
                # get data
                data = _re.split('\W+', line)
                if len(data) < 2:
                    continue
                block['name'] = data[0]
                block['orientation'] = data[1]
                all_blocks.append(block)
            return all_blocks

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to read file.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return fail_response

    def read_file(self, filename):
        """ Read file and return text for displaying """
        fail_response = ''
        try:
            data = ''
            with open(filename, 'r') as f:
                data = f.read()
            return data
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to read file.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return fail_response

    def show_previous(self):
        """ Show previous block in list on GUI """
        if (self.block_count > 0
            and self.list_position > 0):
            self.list_position -= 1
            self.display_block_data(self.list_position)

        # check lower limit
        if self.list_position == 0:
            self.ui.pbt_previous.setEnabled(False)
        else:
            self.ui.pbt_previous.setEnabled(True)

        # check upper limit
        if self.list_position == self.block_count - 1:
            self.ui.pbt_next.setEnabled(False)
        else:
            self.ui.pbt_next.setEnabled(True)
        return True

    def show_next(self):
        """ Show next block in list on GUI """
        if (self.block_count > 0
            and self.list_position < self.block_count - 1):
            self.list_position += 1
            self.display_block_data(self.list_position)

        # check lower limit
        if self.list_position == 0:
            self.ui.pbt_previous.setEnabled(False)
        else:
            self.ui.pbt_previous.setEnabled(True)

        # check upper limit
        if self.list_position == self.block_count - 1:
            self.ui.pbt_next.setEnabled(False)
        else:
            self.ui.pbt_next.setEnabled(True)
        return True

    def display_block_data(self, position):
        """ Update individual block data shown to user

            Input arguments:
                position: position of block in block list """
        try:
            block = self.block_list[position]

            # update image
            orientation = block['orientation']
            pixmap = self.block_types[orientation]
            self.ui.la_block_image.setPixmap(pixmap)

            # update name
            name = block['name']
            self.ui.la_block_name.setText(name)

            # update counter
            self.ui.la_block_count.setText(
                '{0}/{1}'.format(position+1, self.block_count)
            )
            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to display block data.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

    def reset_block_data(self):
        """ Reset block data shown to user """
        try:
            orientation = 'none'
            pixmap = self.block_types[orientation]
            self.ui.la_block_image.setPixmap(pixmap)

            name = '???'
            self.ui.la_block_name.setText(name)

            self.ui.la_block_count.setText('?/?')
            return True

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to reset displayed block data.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

