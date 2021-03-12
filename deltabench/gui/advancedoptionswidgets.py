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
            'vertical_motor_driver_address',
        ]

        self.sbd_names = [
            'motor_velocity',
            'motor_velocity_hall_scan',
            'motor_acceleration',
            'motor_resolution',
            'position_tolerance',
            'linear_conversion_factor',
            'move_timeout',
            'pneumatic_advance_wait',
            'pneumatic_retreat_wait',
            'vertical_motor_resolution',
        ]

        self.cmb_names = [
            'motor_rotation_direction',
            'vertical_motor_rotation_direction',
            'display_model',
        ]

        self.connect_signal_slots()
        self.load_last_db_entry()

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        super().connect_signal_slots()
        self.ui.pbt_save.clicked.connect(self.save_db)
        self.ui.pbt_cancel.clicked.connect(self.load)
