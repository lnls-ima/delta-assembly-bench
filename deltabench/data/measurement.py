# -*- coding: utf-8 -*-

"""Implementation of classes to handle measurement data."""

import collections as _collections
import numpy as _np

from imautils.db import database as _database


class HallWaveformData(_database.DatabaseAndFileDocument):
    """Read, write and stored hall sensor waveform data."""

#    label = 'DeltaBenchMeasurement'
    collection_name = 'hall'
    db_dict = _collections.OrderedDict([
        ('idn', {'field': 'id', 'dtype': int, 'not_null': True}),
        ('date', {'dtype': str, 'not_null': True}),
        ('hour', {'dtype': str, 'not_null': True}),
        ('scan_id', {'dtype': int, 'not_null': True}),
        ('reading_index', {'dtype': int, 'not_null': True}),
        ('hall_sensor_voltage', {'dtype': float, 'not_null': True}),
        ('encoder_position', {'dtype': float, 'not_null': True}),
    ])

class BlockData(_database.DatabaseAndFileDocument):
    """Read, write and stored block data."""

    collection_name = 'blocks'
    db_dict = _collections.OrderedDict([
        ('idn', {'field': 'id', 'dtype': int, 'not_null': True}),
        ('date', {'dtype': str, 'not_null': True}),
        ('hour', {'dtype': str, 'not_null': True}),
        ('scan_id', {'dtype': int, 'not_null': True}),
        ('block_number', {'dtype': int, 'not_null': True}),
        ('x_position', {'dtype': float, 'not_null': True}),
        ('x_position_error', {'dtype': float, 'not_null': True}),
        ('z_position', {'dtype': float, 'not_null': True}),
        ('z_position_error', {'dtype': float, 'not_null': True}),
        ('encoder_position', {'dtype': float, 'not_null': True}),
    ])

class MeasurementData(_database.DatabaseAndFileDocument):
    """Read, write and stored measurement data."""

    label = 'DeltaBenchMeasurement'
    collection_name = 'measurement'
    db_dict = _collections.OrderedDict([
        ('idn', {'field': 'id', 'dtype': int, 'not_null': True}),
        ('date', {'dtype': str, 'not_null': True}),
        ('hour', {'dtype': str, 'not_null': True}),
        ('undulator_name', {'dtype': str, 'not_null': True}),
        ('block_number', {'dtype': int, 'not_null': True}),
        ('cassette_name', {'dtype': str, 'not_null': True}),
        ('comments', {'dtype': str}),
        ('advanced_options_id', {'dtype': int}),
        ('configuration_id', {'dtype': int}),
        ('hall_sensor_voltage', {'dtype': float, 'not_null': True}),
        ('x_position', {'dtype': float, 'not_null': True}),
        ('x_position_error', {'dtype': float, 'not_null': True}),
        ('z_position', {'dtype': float, 'not_null': True}),
        ('z_position_error', {'dtype': float, 'not_null': True}),
        ('encoder_position', {'dtype': float, 'not_null': True}),
    ])

    @property
    def default_filename(self):
        """Return the default filename."""
        filename = super().default_filename
        
        if (self.block_number is not None
             and (self.cassette_name is not None and len(self.cassette_name) != 0)
             and (self.undulator_name is not None and len(self.undulator_name) != 0)
             and (self.measurement_name is not None and len(self.measurement_name) != 0)):
            filename = filename.replace(self.label, self.measurement_name
                                                    +'_'+self.undulator_name
                                                    +'_'+str(self.block_number)
                                                    +'_'+self.cassette_name)
        
        return filename

    def save_file(self, filename):
        """Save data to file.
        Args:
            filename (str): file fullpath.
        """
        if not self.valid_data():
            message = 'Invalid data.'
            raise ValueError(message)

        columns = []
        return super().save_file(filename, columns=columns)

    def read_file(self, filename):
        """Read from file.

        Args:
        ----
            filename (str): filepath.

        """
        return super().read_file(filename, check_nr_columns=False)

