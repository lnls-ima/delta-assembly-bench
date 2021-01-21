# -*- coding: utf-8 -*-

"""Implementation of classes to handle configuration data."""

import collections as _collections

from imautils.db import database as _database


class ConnectionConfig(_database.DatabaseAndFileDocument):
    """Read, write and stored connection configuration data."""

    label = 'Connection'
    collection_name = 'connection'
    db_dict = _collections.OrderedDict([
        ('idn', {'field': 'id', 'dtype': int, 'not_null': True}),
        ('date', {'dtype': str, 'not_null': True}),
        ('hour', {'dtype': str, 'not_null': True}),
        ('display_enable', {'dtype': int, 'not_null': True}),
        ('display_port', {'dtype': str}),
        ('display_baudrate', {'dtype': int, 'not_null': True}),
        ('display_bytesize', {'dtype': int, 'not_null': True}),
        ('display_parity', {'dtype': str, 'not_null': True}),
        ('display_stopbits', {'dtype': str, 'not_null': True}),
        ('display_timeout', {'dtype': float, 'not_null': True}),
        ('driver_enable', {'dtype': int, 'not_null': True}),
        ('driver_port', {'dtype': str}),
        ('driver_baudrate', {'dtype': int, 'not_null': True}),
        ('driver_bytesize', {'dtype': int, 'not_null': True}),
        ('driver_parity', {'dtype': str, 'not_null': True}),
        ('driver_stopbits', {'dtype': str, 'not_null': True}),
        ('driver_timeout', {'dtype': float, 'not_null': True}),
        ('multimeter_enable', {'dtype': int, 'not_null': True}),
        ('multimeter_port', {'dtype': str}),
        ('multimeter_baudrate', {'dtype': int, 'not_null': True}),
        ('multimeter_bytesize', {'dtype': int, 'not_null': True}),
        ('multimeter_parity', {'dtype': str, 'not_null': True}),
        ('multimeter_stopbits', {'dtype': str, 'not_null': True}),
        ('multimeter_timeout', {'dtype': float, 'not_null': True}),
    ])


class AdvancedOptions(_database.DatabaseAndFileDocument):
    """Read, write and stored advanced options data."""

    label = 'Advanced_Options'
    collection_name = 'advanced_options'
    db_dict = _collections.OrderedDict([
        ('idn', {'field': 'id', 'dtype': int, 'not_null': True}),
        ('date', {'dtype': str, 'not_null': True}),
        ('hour', {'dtype': str, 'not_null': True}),
        ('motor_driver_address', {'dtype': int, 'not_null': True}),
        ('motor_velocity', {'dtype': float, 'not_null': True}),
        ('motor_acceleration', {'dtype': float, 'not_null': True}),
        ('motor_rotation_direction', {'dtype': str, 'not_null': True}),
        ('motor_resolution', {'dtype': int, 'not_null': True}),
        ('hall_threshold', {'dtype': float, 'not_null': True}),
        ('display_model', {'dtype': str, 'not_null': True}),
        ('encoder_direction', {'dtype': str, 'not_null': True}),
        ('encoder_resolution', {'dtype': float, 'not_null': True}),
        ('position_tolerance', {'dtype': float, 'not_null': True}),
        ('linear_conversion_factor', {'dtype': float, 'not_null': True}),
        ('move_timeout', {'dtype': float, 'not_null': True}),
    ])


class MeasurementConfig(_database.DatabaseAndFileDocument):
    """Read, write and stored measurement configuration data."""

    label = 'Configuration'
    collection_name = 'configuration'
    db_dict = _collections.OrderedDict([
        ('idn', {'field': 'id', 'dtype': int, 'not_null': True}),
        ('undulator_name', {'dtype': str, 'not_null': True}),
        ('date', {'dtype': str, 'not_null': True}),
        ('hour', {'dtype': str, 'not_null': True}),
        ('block_name', {'dtype': str, 'not_null': True}),
        ('comments', {'dtype': str}),
    ])
