# -*- coding: utf-8 -*-

"""Advanced options widget for the control application."""

from qtpy.QtWidgets import (
    QDialog as _QDialog,
    QHBoxLayout as _QHBoxLayout,
    )
from qtpy.QtCore import (
    QCoreApplication as _QCoreApplication,
    )

from deltabench.gui import utils as _utils
from deltabench.gui.auxiliarywidgets import (
    ConfigurationWidget as _ConfigurationWidget
    )
import deltabench.data.configuration as _configuration


class AdvancedOptionsDialog(_QDialog):
    """Advanced options dialog."""

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.main_widget = AdvancedOptionsWidget(parent=parent)
        self.main_layout = _QHBoxLayout()
        self.main_layout.addWidget(self.main_widget)
        self.setLayout(self.main_layout)
        self.setWindowTitle(
            _QCoreApplication.translate('', "Advanced Options"))
        self.setWhatsThis(
            _QCoreApplication.translate(
                '', 'Advanced devices and measurement options.'))

    @property
    def config(self):
        """Return configuration."""
        return self.main_widget.config


class AdvancedOptionsWidget(_ConfigurationWidget):
    """Advanced options widget class for the control application."""

    def __init__(self, parent=None):
        """Set up the ui."""
        uifile = _utils.get_ui_file(self)
        config = _configuration.AdvancedOptions()
        super().__init__(uifile, config, parent=parent)

        self.sb_names = [
            'motor_driver_address',
            'hall_threshold',
        ]

        self.sbd_names = [
            'motor_velocity',
            'motor_acceleration',
            'motor_resolution',
            'position_tolerance',
            'linear_conversion_factor',
            'move_timeout',
            'encoder_resolution',
        ]

        self.cmb_names = [
            'motor_rotation_direction',
            'encoder_direction',
            'display_model',
        ]

        self.connect_signal_slots()
        self.load_last_db_entry()

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        super().connect_signal_slots()
        self.ui.pbt_save.clicked.connect(self.save_db)
        self.ui.pbt_cancel.clicked.connect(self.load)
