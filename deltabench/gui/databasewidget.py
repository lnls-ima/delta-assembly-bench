"""Database tables widgets."""

import os as _os
import sys as _sys
import time as _time
import numpy as _np
import pandas as _pd
import sqlite3 as _sqlite3
import traceback as _traceback
import qtpy.uic as _uic
from qtpy.QtCore import Qt as _Qt
from qtpy.QtWidgets import (
    QWidget as _QWidget,
    QApplication as _QApplication,
    QLabel as _QLabel,
    QTableWidget as _QTableWidget,
    QTableWidgetItem as _QTableWidgetItem,
    QMessageBox as _QMessageBox,
    QVBoxLayout as _QVBoxLayout,
    QHBoxLayout as _QHBoxLayout,
    QSpinBox as _QSpinBox,
    QFileDialog as _QFileDialog,
    QInputDialog as _QInputDialog,
    QAbstractItemView as _QAbstractItemView,
    )

from imautils.gui import databasewidgets as _databasewidgets
from deltabench.gui.utils import get_ui_file as _get_ui_file
import deltabench.data as _data


_ConnectionConfig = _data.configuration.ConnectionConfig
_AdvancedOptions = _data.configuration.AdvancedOptions
_AssemblyData = _data.measurement.AssemblyData
_ControlConfig = _data.configuration.ControlConfig
_ScanConfig = _data.configuration.ScanConfig
#_MeasurementData = _data.measurement.MeasurementData
_BlockData = _data.measurement.BlockData
_HallWaveformData = _data.measurement.HallWaveformData


class DatabaseWidget(_QWidget):
    """Database widget class for the control application."""

    _connection_table_name = _ConnectionConfig.collection_name
    _advanced_options_table_name = _AdvancedOptions.collection_name
    _assembly_table_name = _AssemblyData.collection_name
    _control_configuration_table_name = _ControlConfig.collection_name
    _scan_configuration_table_name = _ScanConfig.collection_name
#    _measurement_table_name = _MeasurementData.collection_name
    _block_table_name = _BlockData.collection_name
    _hall_table_name = _HallWaveformData.collection_name

    _hidden_columns = []

    def __init__(self, parent=None):
        """Set up the ui."""
        super().__init__(parent)

        # setup the ui
        uifile = _get_ui_file(self)
        self.ui = _uic.loadUi(uifile, self)

        self._table_object_dict = {
            self._connection_table_name: _ConnectionConfig,
            self._advanced_options_table_name: _AdvancedOptions,
            self._assembly_table_name: _AssemblyData,
            self._control_configuration_table_name: _ControlConfig,
            self._scan_configuration_table_name: _ScanConfig,
#            self._measurement_table_name: _MeasurementData,
            self._block_table_name: _BlockData,
            self._hall_table_name: _HallWaveformData,
            }

        self._table_page_dict = {}
        for key in self._table_object_dict.keys():
            self._table_page_dict[key] = None
#        self._table_page_dict[
#            self._measurement_table_name] = self.ui.pg_deltabench_measurement

        self.short_version_hidden_tables = []

        self.twg_database = _databasewidgets.DatabaseTabWidget(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server)
        self.ui.lyt_database.addWidget(self.twg_database)

        self.connect_signal_slots()
        self.disable_invalid_buttons()

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

    def clear(self):
        """Clear."""
        try:
            self.twg_database.delete_widgets()
            self.twg_database.clear()
        except Exception:
            _traceback.print_exc(file=_sys.stdout)

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        self.ui.pbt_save.clicked.connect(self.save_files)
        self.ui.pbt_read.clicked.connect(self.read_files)
        self.ui.pbt_delete.clicked.connect(
            self.twg_database.delete_database_documents)

        self.ui.tbt_refresh.clicked.connect(self.update_database_tables)
        self.ui.tbt_clear.clicked.connect(self.clear)
        self.ui.twg_database.currentChanged.connect(
            self.disable_invalid_buttons)

#        self.ui.pbt_save_summary.clicked.connect(self.save_summary)

#    def save_summary(self):
#        try:
#            table_name = self.twg_database.get_current_table_name()
#            if table_name is None:
#                return
#
#            object_class = self._table_object_dict[table_name]
#
#            idns = self.twg_database.get_table_selected_ids(table_name)
#            nr_idns = len(idns)
#            if nr_idns == 0:
#                return
#
#            try:
#                attrs = [
#                    'date', 'hour', 'measurement_name', 'undulator_name',
#                    'cassette_name', 'block_number', 'x_position',
#                    'x_position_error', 'y_position', 'y_position_error',
#                    'encoder_position'
#                    ]
#                df = _pd.DataFrame(columns=attrs)
#                for i in range(nr_idns):
#                    idn = idns[i]
#                    obj = object_class(
#                        database_name=self.database_name,
#                        mongo=self.mongo, server=self.server)
#                    obj.db_read(idn)
#                    for attr in attrs:
#                        value = getattr(obj, attr)
#                        if attr == 'block_volume':
#                            value = value*1e9
#                        df.at[idn, attr] = value
#
#            except Exception:
#                _traceback.print_exc(file=_sys.stdout)
#                msg = 'Failed to read database entries.'
#                _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
#                return
#
#            timestamp = _time.strftime(
#                '%Y-%m-%d_%H-%M-%S', _time.localtime())
#
#            default_filename = timestamp + '_DeltaBench_Measurement_Summary.xlsx'
#
#            filename = _QFileDialog.getSaveFileName(
#                self, caption='Save file',
#                directory=_os.path.join(self.directory, default_filename),
#                filter="Text files (*.txt *.dat)")
#
#            if isinstance(filename, tuple):
#                filename = filename[0]
#
#            if len(filename) == 0:
#                return
#
#            try:
#                df.to_excel(filename)
#
#            except Exception:
#                _traceback.print_exc(file=_sys.stdout)
#                msg = 'Failed to save file.'
#                _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
#        
#        except Exception:
#            _traceback.print_exc(file=_sys.stdout)

    def disable_invalid_buttons(self):
        """Disable invalid buttons."""
        try:
            current_table_name = self.twg_database.get_current_table_name()
            if current_table_name is not None:
                self.ui.stw_buttons.setEnabled(True)

                for table_name, page in self._table_page_dict.items():
                    if page is not None:
                        page.setEnabled(False)
                        _idx = self.ui.stw_buttons.indexOf(page)
                    else:
                        self.ui.stw_buttons.setCurrentIndex(0)

                current_page = self._table_page_dict[current_table_name]
                if current_page is not None:
                    current_page.setEnabled(True)
                    _idx = self.ui.stw_buttons.indexOf(current_page)
                    self.ui.stw_buttons.setCurrentWidget(current_page)
            else:
                self.ui.stw_buttons.setCurrentIndex(0)
                self.ui.stw_buttons.setEnabled(False)

        except Exception:
            _traceback.print_exc(file=_sys.stdout)

    def load_database(self):
        """Load database."""
        try:
            self.twg_database.database_name = self.database_name
            self.twg_database.mongo = self.mongo
            self.twg_database.server = self.server
            if self.ui.chb_short_version.isChecked():
                hidden_tables = self.short_version_hidden_tables
                self.twg_database.hidden_tables = hidden_tables
            else:
                self.twg_database.hidden_tables = []
            self.twg_database.load_database()
            self.disable_invalid_buttons()
        except Exception:
            _traceback.print_exc(file=_sys.stdout)

    def read_files(self):
        """Read file and save in database."""
        table_name = self.twg_database.get_current_table_name()
        if table_name is None:
            return

        object_class = self._table_object_dict[table_name]

        fns = _QFileDialog.getOpenFileNames(
            self, caption='Read files', directory=self.directory,
            filter="Text files (*.txt *.dat)")

        if isinstance(fns, tuple):
            fns = fns[0]

        if len(fns) == 0:
            return

        try:
            idns = []
            for filename in fns:
                obj = object_class(
                    database_name=self.database_name,
                    mongo=self.mongo, server=self.server)
                obj.read_file(filename)
                idn = obj.db_save()
                idns.append(idn)
            msg = 'Adicionado a tabela do banco de dados.\nIDs: ' + str(idns)
            self.update_database_tables()
            _QMessageBox.information(self, 'Informacao', msg, _QMessageBox.Ok)
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Falha ao tentar ler arquivos e salvar valores no banco de dados.'
            _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
            return

    def save_files(self):
        """Save database record to file."""
        try:
            table_name = self.twg_database.get_current_table_name()
            if table_name is None:
                return

            object_class = self._table_object_dict[table_name]

            idns = self.twg_database.get_table_selected_ids(table_name)
            nr_idns = len(idns)
            if nr_idns == 0:
                return

            objs = []
            fns = []
            try:
                for i in range(nr_idns):
                    idn = idns[i]
                    obj = object_class(
                        database_name=self.database_name,
                        mongo=self.mongo, server=self.server)
                    obj.db_read(idn)
                    default_filename = obj.default_filename
                    objs.append(obj)
                    fns.append(default_filename)

            except Exception:
                _traceback.print_exc(file=_sys.stdout)
                msg = 'Falha ao tentar ler entradas no banco de dados.'
                _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
                return

            if nr_idns == 1:
                filename = _QFileDialog.getSaveFileName(
                    self, caption='Save file',
                    directory=_os.path.join(self.directory, fns[0]),
                    filter="Text files (*.txt *.dat)")

                if isinstance(filename, tuple):
                    filename = filename[0]

                if len(filename) == 0:
                    return

                fns[0] = filename
            else:
                directory = _QFileDialog.getExistingDirectory(
                    self, caption='Save files', directory=self.directory)

                if isinstance(directory, tuple):
                    directory = directory[0]

                if len(directory) == 0:
                    return

                for i in range(len(fns)):
                    fns[i] = _os.path.join(directory, fns[i])

            try:
                for i in range(nr_idns):
                    obj = objs[i]
                    idn = idns[i]
                    filename = fns[i]
                    if (not filename.endswith('.txt') and
                       not filename.endswith('.dat')):
                        filename = filename + '.txt'
                    obj.save_file(filename)
            except Exception:
                _traceback.print_exc(file=_sys.stdout)
                msg = 'Falha ao tentar salvar arquivos.'
                _QMessageBox.critical(self, 'Falha', msg, _QMessageBox.Ok)
        except Exception:
            _traceback.print_exc(file=_sys.stdout)

    def update_database_tables(self):
        """Update database tables."""
        if not self.isVisible():
            return

        try:
            self.twg_database.database_name = self.database_name
            self.twg_database.mongo = self.mongo
            self.twg_database.server = self.server
            if self.ui.chb_short_version.isChecked():
                hidden_tables = self.short_version_hidden_tables
                self.twg_database.hidden_tables = hidden_tables
            else:
                self.twg_database.hidden_tables = []
            self.twg_database.update_database_tables()
            self.disable_invalid_buttons()
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
