# -*- coding: utf-8 -*-

"""Main entry point to the control application."""

import sys as _sys
import threading as _threading
from qtpy.QtWidgets import QApplication as _QApplication

from deltabench.gui import utils as _utils
from deltabench.gui.mainwindow import MainWindow as _MainWindow
from deltabench.gui.advancedoptionswidgets import AdvancedOptionsDialog \
    as _AdvancedOptionsDialog
from deltabench.data import configuration as _configuration
from deltabench.data import measurement as _measurement


class MainApp(_QApplication):
    """Main application."""

    def __init__(self, args):
        """Start application."""
        super().__init__(args)
        self.setStyle(_utils.WINDOW_STYLE)

        self.directory = _utils.BASEPATH
        self.database_name = _utils.DATABASE_NAME
        self.mongo = _utils.MONGO
        self.server = _utils.SERVER
        self.create_database()
        self.measurement_config = _configuration.MeasurementConfig()
        self.scan_config = _configuration.ScanConfig()
        self.advanced_options_dialog = None

    def create_dialogs(self):
        """Create dialogs."""
        self.advanced_options_dialog = _AdvancedOptionsDialog()

    def create_database(self):
        """Create database and tables."""
        connection_config = _configuration.ConnectionConfig(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server)
        advanced_options = _configuration.AdvancedOptions(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server)
        measurement_config = _configuration.MeasurementConfig(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server)
        scan_config = _configuration.ScanConfig(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server)
        measurement_data = _measurement.MeasurementData(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server)
        hall_data = _measurement.HallWaveformData(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server)
        block_data = _measurement.BlockData(
            database_name=self.database_name,
            mongo=self.mongo, server=self.server)

        status = []
        status.append(connection_config.db_create_collection())
        status.append(advanced_options.db_create_collection())
        status.append(measurement_config.db_create_collection())
        status.append(scan_config.db_create_collection())
        status.append(measurement_data.db_create_collection())
        status.append(hall_data.db_create_collection())
        status.append(block_data.db_create_collection())
        if not all(status):
            raise Exception("Failed to create database.")


class GUIThread(_threading.Thread):
    """GUI Thread."""

    def __init__(self):
        """Start thread."""
        _threading.Thread.__init__(self)
        self.app = None
        self.window = None
        self.daemon = True
        self.start()

    def run(self):
        """Thread target function."""
        self.app = None
        if not _QApplication.instance():
            self.app = MainApp([])
            translators = _utils.get_translators()
            for translator in translators:
                self.app.installTranslator(translator)
            self.app.create_dialogs()
            self.window = _MainWindow(
                width=_utils.WINDOW_WIDTH, height=_utils.WINDOW_HEIGHT)
            self.app.advanced_options_dialog = (
                self.window.advanced_options_dialog)
            self.window.show()
            self.window.centralize_window()
            _sys.exit(self.app.exec_())


def run():
    """Run application."""
    app = None
    if not _QApplication.instance():
        app = MainApp([])
        translators = _utils.get_translators()
        for translator in translators:
            app.installTranslator(translator)
        app.create_dialogs()
        window = _MainWindow(
            width=_utils.WINDOW_WIDTH, height=_utils.WINDOW_HEIGHT)
        app.advanced_options_dialog = window.advanced_options_dialog
        window.show()
        window.centralize_window()
        _sys.exit(app.exec_())


def run_in_thread():
    """Run application in a thread."""
    return GUIThread()
