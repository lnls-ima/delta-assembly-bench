# -*- coding: utf-8 -*-

import os as _os
import sys as _sys
import time as _time
import numpy as _np
import datetime as _datetime
import warnings as _warnings
import pyqtgraph as _pyqtgraph
import traceback as _traceback
from qtpy.QtWidgets import (
    QWidget as _QWidget,
    QDialog as _QDialog,
    QLabel as _QLabel,
    QSpinBox as _QSpinBox,
    QGroupBox as _QGroupBox,
    QComboBox as _QComboBox,
    QCheckBox as _QCheckBox,
    QListView as _QListView,
    QLineEdit as _QLineEdit,
    QTextEdit as _QTextEdit,
    QMessageBox as _QMessageBox,
    QSizePolicy as _QSizePolicy,
    QSpacerItem as _QSpacerItem,
    QPushButton as _QPushButton,
    QToolButton as _QToolButton,
    QHBoxLayout as _QHBoxLayout,
    QVBoxLayout as _QVBoxLayout,
    QGridLayout as _QGridLayout,
    QFormLayout as _QFormLayout,
    QFileDialog as _QFileDialog,
    QTableWidget as _QTableWidget,
    QApplication as _QApplication,
    QDoubleSpinBox as _QDoubleSpinBox,
    QTableWidgetItem as _QTableWidgetItem,
    QAbstractItemView as _QAbstractItemView,
    )
from qtpy.QtGui import (
    QFont as _QFont,
    QColor as _QColor,
    QBrush as _QBrush,
    QStandardItemModel as _QStandardItemModel,
    )
from qtpy.QtCore import (
    Qt as _Qt,
    QSize as _QSize,
    QTimer as _QTimer,
    Signal as _Signal,
    QCoreApplication as _QCoreApplication,
    )
from matplotlib.figure import Figure as _Figure
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as _FigureCanvas,
    NavigationToolbar2QT as _Toolbar
    )
import qtpy.uic as _uic

import deltabench.gui.utils as _utils

_font = _utils.get_default_font()
_font_bold = _utils.get_default_font(bold=True)
_icon_size = _utils.get_default_icon_size()

_autorange_icon_file = "zoom.svg"
_clear_icon_file = "clear.svg"
_copy_icon_file = "copy.svg"
_delete_icon_file = "close.svg"
_move_icon_file = "move.svg"
_save_icon_file = "save.svg"
_stats_icon_file = "stats.svg"
_stop_icon_file = "stop.svg"


class CheckableComboBox(_QComboBox):
    """Combo box with checkable items."""

    def __init__(self, parent=None):
        """Initialize object."""
        super().__init__(parent)
        self.setFont(_font)
        self.setView(_QListView(self))
        self.view().pressed.connect(self.handle_item_pressed)
        self.setModel(_QStandardItemModel(self))

    def handle_item_pressed(self, index):
        """Change item check state."""
        item = self.model().itemFromIndex(index)
        if item.checkState() == _Qt.Checked:
            item.setCheckState(_Qt.Unchecked)
        else:
            item.setCheckState(_Qt.Checked)

    def checked_items(self):
        """Get checked items."""
        items = []
        for index in range(self.count()):
            item = self.model().item(index)
            if item.checkState() == _Qt.Checked:
                items.append(item)
        return items

    def checked_indexes(self):
        """Get checked indexes."""
        indexes = []
        for index in range(self.count()):
            item = self.model().item(index)
            if item.checkState() == _Qt.Checked:
                indexes.append(index)
        return indexes

    def checked_items_text(self):
        """Get checked items text."""
        items_text = []
        for index in range(self.count()):
            item = self.model().item(index)
            if item.checkState() == _Qt.Checked:
                items_text.append(item.text())
        return items_text


class LogDialog(_QDialog):
    """Log dialog."""

    def __init__(self, parent=None):
        """Set up the ui."""
        super().__init__(parent)
        self.setWindowTitle(
            _QCoreApplication.translate('', "Log"))
        self.resize(1000, 460)
        self.setFont(_font)
        self.te_text = _QTextEdit()
        self.te_text.setReadOnly(True)
        self.te_text.setVerticalScrollBarPolicy(_Qt.ScrollBarAlwaysOn)
        main_layout = _QHBoxLayout()
        main_layout.addWidget(self.te_text)
        self.setLayout(main_layout)


class PlotDialog(_QDialog):
    """Matplotlib plot dialog."""

    def __init__(self, parent=None):
        """Add figure canvas to layout."""
        super().__init__(parent)
        self.setFont(_font)
        self.figure = _Figure()
        self.canvas = _FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)

        _layout = _QVBoxLayout()
        _layout.addWidget(self.canvas)
        self.toolbar = _Toolbar(self.canvas, self)
        _layout.addWidget(self.toolbar)
        self.setLayout(_layout)

    def update_plot(self):
        """Update plot."""
        self.canvas.draw()

    def show(self):
        """Show dialog."""
        self.update_plot()
        super().show()


class SelectTabsDialog(_QDialog):
    """Select tabs dialog class."""

    tab_selection_changed = _Signal([dict])

    def __init__(self, chb_names, parent=None):
        """Set up the ui and create connections."""
        super().__init__(parent)
        self.setWindowTitle(
            _QCoreApplication.translate('', "Select Tabs"))
        self.resize(250, 400)
        self.setFont(_font)

        main_layout = _QVBoxLayout()
        vertical_layout = _QVBoxLayout()
        group_box = _QGroupBox(
            _QCoreApplication.translate('', "Select Tabs to Show"))
        group_box.setLayout(vertical_layout)
        group_box.setFont(_font_bold)
        main_layout.addWidget(group_box)
        self.setLayout(main_layout)

        self.chb_names = chb_names
        for name in self.chb_names:
            name_split = name.split('_')
            label = ' '.join([s.capitalize() for s in name_split])
            chb = _QCheckBox(_QCoreApplication.translate('', label))
            setattr(self, 'chb_' + name, chb)
            vertical_layout.addWidget(chb)
            chb.setFont(_font)

        self.pbt_apply = _QPushButton(
            _QCoreApplication.translate('', "Apply Changes"))
        self.pbt_apply.setMinimumSize(_QSize(0, 40))
        self.pbt_apply.setFont(_font_bold)
        vertical_layout.addWidget(self.pbt_apply)

        self.pbt_apply.clicked.connect(self.emit_tab_selection_signal)

    def emit_tab_selection_signal(self):
        """Get tabs checkbox status and emit signal to change tabs."""
        try:
            chb_status = {}
            for chb_name in self.chb_names:
                chb = getattr(self, 'chb_' + chb_name)
                chb_status[chb_name] = chb.isChecked()

            self.tab_selection_changed.emit(chb_status)
        except Exception:
            _traceback.print_exc(file=_sys.stdout)


class TableAnalysisDialog(_QDialog):
    """Table data analysis dialog class."""

    def __init__(self, parent=None):
        """Add table widget and copy button."""
        super().__init__(parent)
        self.setFont(_font)

        self.setWindowTitle(
            _QCoreApplication.translate('', "Statistics"))
        self.tbl_results = _QTableWidget()
        self.tbl_results.setAlternatingRowColors(True)
        self.tbl_results.horizontalHeader().setStretchLastSection(True)
        self.tbl_results.horizontalHeader().setDefaultSectionSize(120)

        self.pbt_copy = _QPushButton(
            _QCoreApplication.translate('', "Copy to clipboard"))
        self.pbt_copy.clicked.connect(self.copy_to_clipboard)
        self.pbt_copy.setFont(_font_bold)

        _layout = _QVBoxLayout()
        _layout.addWidget(self.tbl_results)
        _layout.addWidget(self.pbt_copy)
        self.setLayout(_layout)
        self.table_df = None

        self.resize(500, 200)

    def closeEvent(self, event):
        """Close widget."""
        try:
            self.clear()
            event.accept()
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            event.accept()

    def add_items_to_table(self, text, i, j):
        """Add items to table."""
        item = _QTableWidgetItem(text)
        item.setFlags(_Qt.ItemIsSelectable | _Qt.ItemIsEnabled)
        self.tbl_results.setItem(i, j, item)

    def analyse_and_show_results(self):
        """Analyse data and add results to table."""
        self.tbl_results.clearContents()
        self.tbl_results.setRowCount(0)
        self.tbl_results.setColumnCount(0)

        if self.table_df is None:
            return

        self.tbl_results.setColumnCount(3)

        self.tbl_results.setHorizontalHeaderLabels(
            ['Mean', 'STD', 'Peak-Valey'])

        labels = [
            l for l in self.table_df.columns if l not in ['Date', 'Time']]

        self.tbl_results.setRowCount(len(labels))
        self.tbl_results.setVerticalHeaderLabels(labels)

        for i in range(len(labels)):
            label = labels[i]
            values = self.table_df[label].values
            try:
                values = values.astype(float)
            except Exception:
                values = [_np.nan]*len(values)
            values = _np.array(values)
            values = values[_np.isfinite(values)]
            if len(values) == 0:
                mean = _np.nan
                std = _np.nan
                peak_valey = _np.nan
            else:
                mean = _np.mean(values)
                std = _np.std(values)
                peak_valey = _np.max(values) - _np.min(values)
            self.add_items_to_table('{0:.4f}'.format(mean), i, 0)
            self.add_items_to_table('{0:.4f}'.format(std), i, 1)
            self.add_items_to_table('{0:.4f}'.format(peak_valey), i, 2)

    def accept(self):
        """Close dialog."""
        self.clear()
        super().accept()

    def clear(self):
        """Clear data and table."""
        self.table_df = None
        self.tbl_results.clearContents()
        self.tbl_results.setRowCount(0)
        self.tbl_results.setColumnCount(0)

    def copy_to_clipboard(self):
        """Copy table data to clipboard."""
        df = _utils.table_to_data_frame(self.tbl_results)
        if df is not None:
            df.to_clipboard(excel=True)

    def show(self, table_df):
        """Show dialog."""
        self.table_df = table_df
        self.analyse_and_show_results()
        super().show()

    def update_data(self, table_df):
        """Update table data."""
        self.table_df = table_df
        self.analyse_and_show_results()


class TablePlotWidget(_QWidget):
    """Table and Plot widget."""

    _monitor_name = ''
    _bottom_axis_label = 'Time interval [s]'
    _bottom_axis_format = '{0:.3f}'
    _is_timestamp = True
    
    _left_axis_1_label = ''
    _right_axis_1_label = ''
    _right_axis_2_label = ''

    _left_axis_1_format = '{0:.4f}'
    _right_axis_1_format = '{0:.4f}'
    _right_axis_2_format = '{0:.4f}'

    _left_axis_1_data_labels = []
    _right_axis_1_data_labels = []
    _right_axis_2_data_labels = []

    _left_axis_1_data_colors = []
    _right_axis_1_data_colors = []
    _right_axis_2_data_colors = []

    _show_legend = True

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(
            _QCoreApplication.translate('', "Table and Plot"))
        self.resize(1230, 900)
        self.add_widgets()
        self.setFont(_font)

        # variables initialisation
        self.filename = None
        self._xvalues = []
        self._legend_items = []
        self._graphs = {}
        self._data_labels = (
            self._left_axis_1_data_labels +
            self._right_axis_1_data_labels +
            self._right_axis_2_data_labels)
        self._data_formats = (
            [self._left_axis_1_format]*len(self._left_axis_1_data_labels) +
            [self._right_axis_1_format]*len(self._right_axis_1_data_labels) +
            [self._right_axis_2_format]*len(self._right_axis_2_data_labels))
        self._readings = {}
        for i, label in enumerate(self._data_labels):
            self._readings[label] = []

        # create timer to monitor values
        self.timer = _QTimer(self)
        self.update_monitor_interval()
        self.timer.timeout.connect(lambda: self.read_value(monitor=True))

        # create table analysis dialog
        self.table_analysis_dialog = TableAnalysisDialog()

        # add legend to plot
        if self._show_legend:
            self.legend = _pyqtgraph.LegendItem(offset=(70, 30))
            self.legend.setParentItem(self.pw_plot.graphicsItem())
            self.legend.setAutoFillBackground(1)

        self.right_axis_1 = None
        self.right_axis_2 = None
        self.configure_plot()
        self.configure_table()
        self.connect_signal_slots()

    @property
    def directory(self):
        """Return the default directory."""
        return _QApplication.instance().directory

    @property
    def _timestamp(self):
        """Return the timestamp list."""
        return self._xvalues

    @_timestamp.setter
    def _timestamp(self, value):
        self._xvalues = value

    def closeEvent(self, event):
        """Close widget."""
        try:
            self.timer.stop()
            self.close_dialogs()
            event.accept()
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            event.accept()

    def add_last_value_to_table(self):
        """Add the last value read to table."""
        if len(self._xvalues) == 0:
            return

        n = self.tbl_table.rowCount() + 1
        self.tbl_table.setRowCount(n)

        if self._is_timestamp:
            dt = self._xvalues[-1] - self._xvalues[0]
            self.tbl_table.setItem(
                n-1, 0, _QTableWidgetItem(
                    self._bottom_axis_format.format(dt)))            
        else:
            self.tbl_table.setItem(
                n-1, 0, _QTableWidgetItem(
                    self._bottom_axis_format.format(self._xvalues[-1])))

        for j, label in enumerate(self._data_labels):
            fmt = self._data_formats[j]
            reading = self._readings[label][-1]
            if reading is None:
                reading = _np.nan
            self.tbl_table.setItem(
                n-1, j+1, _QTableWidgetItem(fmt.format(reading)))

        vbar = self.tbl_table.verticalScrollBar()
        vbar.setValue(vbar.maximum())

    def add_last_value_to_file(self):
        """Add the last value read to file."""
        if len(self._xvalues) == 0:
            return

        if self.filename is None:
            return

        line = []
        if self._is_timestamp:
            dt = self._xvalues[-1] - self._xvalues[0]
            line.append(self._bottom_axis_format.format(dt))
        else:
            line.append(self._bottom_axis_format.format(self._xvalues[-1]))

        for j, label in enumerate(self._data_labels):
            fmt = self._data_formats[j]
            reading = self._readings[label][-1]
            if reading is None:
                reading = _np.nan
            line.append(fmt.format(reading))

        with open(self.filename, '+a') as f:
            f.write('\t'.join(line)+'\n')
            
    def add_widgets(self):
        """Add widgets and layouts."""
        # Layouts
        self.vertical_layout_1 = _QVBoxLayout()
        self.vertical_layout_2 = _QVBoxLayout()
        self.vertical_layout_2.setSpacing(20)
        self.vertical_layout_3 = _QVBoxLayout()
        self.horizontal_layout_1 = _QHBoxLayout()
        self.horizontal_layout_2 = _QHBoxLayout()
        self.horizontal_layout_3 = _QHBoxLayout()
        self.horizontal_layout_4 = _QHBoxLayout()

        # Plot Widget
        self.pw_plot = _pyqtgraph.PlotWidget()
        brush = _QBrush(_QColor(255, 255, 255))
        brush.setStyle(_Qt.NoBrush)
        self.pw_plot.setBackgroundBrush(brush)
        self.pw_plot.setForegroundBrush(brush)
        self.horizontal_layout_1.addWidget(self.pw_plot)

        # Read button
        self.pbt_read = _QPushButton(
            _QCoreApplication.translate('', "Read"))
        self.pbt_read.setMinimumSize(_QSize(0, 45))
        self.pbt_read.setFont(_font_bold)
        self.vertical_layout_2.addWidget(self.pbt_read)

        # Monitor button
        self.pbt_monitor = _QPushButton(
            _QCoreApplication.translate('', "Monitor"))
        self.pbt_monitor.setMinimumSize(_QSize(0, 45))
        self.pbt_monitor.setFont(_font_bold)
        self.pbt_monitor.setCheckable(True)
        self.pbt_monitor.setChecked(False)
        self.vertical_layout_2.addWidget(self.pbt_monitor)

        # Monitor frequency
        label = _QLabel(
            _QCoreApplication.translate('', "Frequency [Hz]:"))
        label.setAlignment(
            _Qt.AlignRight | _Qt.AlignTrailing | _Qt.AlignVCenter)
        self.horizontal_layout_3.addWidget(label)

        self.sbd_monitor_freq = _QDoubleSpinBox()
        self.sbd_monitor_freq.setDecimals(2)
        self.sbd_monitor_freq.setMinimum(0.01)
        self.sbd_monitor_freq.setMaximum(100.00)
        self.sbd_monitor_freq.setProperty("value", 1)
        self.horizontal_layout_3.addWidget(self.sbd_monitor_freq)
        self.vertical_layout_2.addLayout(self.horizontal_layout_3)

        # Group box with read and monitor buttons
        self.group_box = _QGroupBox()
        self.group_box.setMinimumSize(_QSize(270, 0))
        self.group_box.setTitle("")
        self.group_box.setLayout(self.vertical_layout_2)

        # Table widget
        self.tbl_table = _QTableWidget()
        sizePolicy = _QSizePolicy(
            _QSizePolicy.Expanding, _QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.tbl_table.sizePolicy().hasHeightForWidth())
        self.tbl_table.setSizePolicy(sizePolicy)
        self.tbl_table.setVerticalScrollBarPolicy(_Qt.ScrollBarAlwaysOn)
        self.tbl_table.setHorizontalScrollBarPolicy(_Qt.ScrollBarAsNeeded)
        self.tbl_table.setEditTriggers(_QAbstractItemView.NoEditTriggers)
        self.tbl_table.setColumnCount(0)
        self.tbl_table.setRowCount(0)
        self.tbl_table.horizontalHeader().setVisible(True)
        self.tbl_table.horizontalHeader().setCascadingSectionResizes(False)
        self.tbl_table.horizontalHeader().setDefaultSectionSize(200)
        self.tbl_table.horizontalHeader().setHighlightSections(True)
        self.tbl_table.horizontalHeader().setMinimumSectionSize(80)
        self.tbl_table.horizontalHeader().setStretchLastSection(True)
        self.tbl_table.verticalHeader().setVisible(False)
        self.horizontal_layout_4.addWidget(self.tbl_table)

        # Tool buttons
        self.tbt_autorange = _QToolButton()
        self.tbt_autorange.setIcon(_utils.get_icon(_autorange_icon_file))
        self.tbt_autorange.setIconSize(_icon_size)
        self.tbt_autorange.setCheckable(True)
        self.tbt_autorange.setToolTip(
            _QCoreApplication.translate('', 'Turn on plot autorange.'))
        self.vertical_layout_3.addWidget(self.tbt_autorange)

        self.tbt_save = _QToolButton()
        self.tbt_save.setIcon(_utils.get_icon(_save_icon_file))
        self.tbt_save.setIconSize(_icon_size)
        self.tbt_save.setToolTip(
            _QCoreApplication.translate('', 'Save table data to file.'))
        self.vertical_layout_3.addWidget(self.tbt_save)

        self.tbt_copy = _QToolButton()
        self.tbt_copy.setIcon(_utils.get_icon(_copy_icon_file))
        self.tbt_copy.setIconSize(_icon_size)
        self.tbt_copy.setToolTip(
            _QCoreApplication.translate('', 'Copy table data.'))
        self.vertical_layout_3.addWidget(self.tbt_copy)

        self.pbt_stats = _QToolButton()
        self.pbt_stats.setIcon(_utils.get_icon(_stats_icon_file))
        self.pbt_stats.setIconSize(_icon_size)
        self.pbt_stats.setToolTip(
            _QCoreApplication.translate('', 'Show data statistics.'))
        self.vertical_layout_3.addWidget(self.pbt_stats)

        self.pbt_remove = _QToolButton()
        self.pbt_remove.setIcon(_utils.get_icon(_delete_icon_file))
        self.pbt_remove.setIconSize(_icon_size)
        self.pbt_remove.setToolTip(
            _QCoreApplication.translate('', 'Remove selected lines from table.'))
        self.vertical_layout_3.addWidget(self.pbt_remove)

        self.tbt_clear = _QToolButton()
        self.tbt_clear.setIcon(_utils.get_icon(_clear_icon_file))
        self.tbt_clear.setIconSize(_icon_size)
        self.tbt_clear.setToolTip(
            _QCoreApplication.translate('', 'Clear table data.'))
        self.vertical_layout_3.addWidget(self.tbt_clear)

        spacer_item = _QSpacerItem(
            20, 100, _QSizePolicy.Minimum, _QSizePolicy.Fixed)
        self.vertical_layout_3.addItem(spacer_item)
        self.horizontal_layout_4.addLayout(self.vertical_layout_3)

        self.horizontal_layout_2.addWidget(self.group_box)
        self.horizontal_layout_2.addLayout(self.horizontal_layout_4)
        self.vertical_layout_1.addLayout(self.horizontal_layout_1)
        self.vertical_layout_1.addLayout(self.horizontal_layout_2)
        self.setLayout(self.vertical_layout_1)

    def add_widgets_next_to_plot(self, widget_list):
        """Add widgets on the side of plot widget."""
        if not isinstance(widget_list, (list, tuple)):
            widget_list = [[widget_list]]

        if not isinstance(widget_list[0], (list, tuple)):
            widget_list = [widget_list]

        for idx, lt in enumerate(widget_list):
            _layout = _QVBoxLayout()
            _layout.setContentsMargins(0, 0, 0, 0)
            for wg in lt:
                if isinstance(wg, _QPushButton):
                    wg.setMinimumHeight(45)
                    wg.setFont(_font_bold)
                _layout.addWidget(wg)
            self.horizontal_layout_1.insertLayout(idx, _layout)

    def add_widgets_next_to_table(self, widget_list):
        """Add widgets on the side of table widget."""
        if not isinstance(widget_list, (list, tuple)):
            widget_list = [[widget_list]]

        if not isinstance(widget_list[0], (list, tuple)):
            widget_list = [widget_list]

        for idx, lt in enumerate(widget_list):
            _layout = _QHBoxLayout()
            for wg in lt:
                if isinstance(wg, _QPushButton):
                    wg.setMinimumHeight(45)
                    wg.setFont(_font_bold)
                _layout.addWidget(wg)
            self.vertical_layout_2.insertLayout(idx, _layout)

    def clear_legend_items(self):
        """Clear plot legend."""
        if self._show_legend:
            for label in self._legend_items:
                self.legend.removeItem(label)

    def clear_button_clicked(self):
        """Clear all values."""
        if len(self._xvalues) == 0:
            return

        msg = 'Clear table data?'
        reply = _QMessageBox.question(
            self, 'Message', msg, buttons=_QMessageBox.No | _QMessageBox.Yes,
            defaultButton=_QMessageBox.No)

        if reply == _QMessageBox.Yes:
            self.clear()

    def clear(self):
        """Clear all values."""
        self._xvalues = []
        for label in self._data_labels:
            self._readings[label] = []
        self.update_table_values()
        self.update_plot()
        self.update_table_analysis_dialog()

    def close_dialogs(self):
        """Close dialogs."""
        try:
            self.table_analysis_dialog.accept()
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            pass

    def configure_plot(self):
        """Configure data plots."""
        self.pw_plot.clear()
        self.pw_plot.setLabel('bottom', self._bottom_axis_label)
        self.pw_plot.showGrid(x=True, y=True)

        # Configure left axis 1
        self.pw_plot.setLabel('left', self._left_axis_1_label)

        colors = self._left_axis_1_data_colors
        data_labels = self._left_axis_1_data_labels
        if len(colors) != len(data_labels):
            colors = [(0, 0, 255)]*len(data_labels)

        for i, label in enumerate(data_labels):
            pen = colors[i]
            graph = self.pw_plot.plotItem.plot(
                _np.array([]), _np.array([]), pen=pen, symbol='o',
                symbolPen=pen, symbolSize=3, symbolBrush=pen)
            self._graphs[label] = graph

        # Configure right axis 1
        data_labels = self._right_axis_1_data_labels
        colors = self._right_axis_1_data_colors
        if len(colors) != len(data_labels):
            colors = [(0, 0, 255)]*len(data_labels)

        if len(data_labels) != 0:
            self.right_axis_1 = _utils.plot_item_add_first_right_axis(
                self.pw_plot.plotItem)
            self.right_axis_1.setLabel(self._right_axis_1_label)
            self.right_axis_1.setStyle(showValues=True)

            for i, label in enumerate(data_labels):
                pen = colors[i]
                graph = _pyqtgraph.PlotDataItem(
                    _np.array([]), _np.array([]), pen=pen, symbol='o',
                    symbolPen=pen, symbolSize=3, symbolBrush=pen)
                self.right_axis_1.linkedView().addItem(graph)
                self._graphs[label] = graph

        # Configure right axis 2
        data_labels = self._right_axis_2_data_labels
        colors = self._right_axis_2_data_colors
        if len(colors) != len(data_labels):
            colors = [(0, 0, 255)]*len(data_labels)

        if len(data_labels) != 0:
            self.right_axis_2 = _utils.plot_item_add_second_right_axis(
                self.pw_plot.plotItem)
            self.right_axis_2.setLabel(self._right_axis_2_label)
            self.right_axis_2.setStyle(showValues=True)

            for i, label in enumerate(data_labels):
                pen = colors[i]
                graph = _pyqtgraph.PlotDataItem(
                    _np.array([]), _np.array([]), pen=pen, symbol='o',
                    symbolPen=pen, symbolSize=3, symbolBrush=pen)
                self.right_axis_2.linkedView().addItem(graph)
                self._graphs[label] = graph

        # Update legend
        self.update_legend_items()

    def configure_table(self):
        """Configure table."""
        col_labels = [self._bottom_axis_label]

        for label in self._data_labels:
            col_labels.append(label)
        self.tbl_table.setColumnCount(len(col_labels))
        self.tbl_table.setHorizontalHeaderLabels(col_labels)
        self.tbl_table.setAlternatingRowColors(True)

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        self.pbt_read.clicked.connect(lambda: self.read_value(monitor=False))
        self.pbt_monitor.toggled.connect(self.monitor_value)
        self.sbd_monitor_freq.valueChanged.connect(
            self.update_monitor_interval)
        self.tbt_autorange.toggled.connect(self.enable_autorange)
        self.tbt_save.clicked.connect(self.save_to_file)
        self.tbt_copy.clicked.connect(self.copy_to_clipboard)
        self.pbt_stats.clicked.connect(self.show_table_analysis_dialog)
        self.pbt_remove.clicked.connect(self.remove_value)
        self.tbt_clear.clicked.connect(self.clear_button_clicked)

    def copy_to_clipboard(self):
        """Copy table data to clipboard."""
        df = _utils.table_to_data_frame(self.tbl_table)
        if df is not None:
            df.to_clipboard(excel=True)

    def enable_autorange(self, checked):
        """Enable or disable plot autorange."""
        if checked:
            if self.right_axis_2 is not None:
                self.right_axis_2.linkedView().enableAutoRange(
                    axis=_pyqtgraph.ViewBox.YAxis)
            if self.right_axis_1 is not None:
                self.right_axis_1.linkedView().enableAutoRange(
                    axis=_pyqtgraph.ViewBox.YAxis)
            self.pw_plot.plotItem.enableAutoRange(
                axis=_pyqtgraph.ViewBox.YAxis)
        else:
            if self.right_axis_2 is not None:
                self.right_axis_2.linkedView().disableAutoRange(
                    axis=_pyqtgraph.ViewBox.YAxis)
            if self.right_axis_1 is not None:
                self.right_axis_1.linkedView().disableAutoRange(
                    axis=_pyqtgraph.ViewBox.YAxis)
            self.pw_plot.plotItem.disableAutoRange(
                axis=_pyqtgraph.ViewBox.YAxis)

    def hide_right_axes(self):
        """Hide right axes."""
        if self.right_axis_1 is not None:
            self.right_axis_1.setStyle(showValues=False)
            self.right_axis_1.setLabel('')
        if self.right_axis_2 is not None:
            self.right_axis_2.setStyle(showValues=False)
            self.right_axis_2.setLabel('')

    def monitor_value(self, checked):
        """Monitor values."""
        if checked:
            try:
                folder = _os.path.join(_utils.BASEPATH, 'monitor')
                timestamp = _time.strftime(
                    '%Y-%m-%d_%H-%M-%S', _time.localtime())
                filename = '{0:s}_{1:s}_{2:s}.txt'.format(
                    timestamp, 'monitor', self._monitor_name)
                filename = _os.path.join(folder, filename)
                
                col_labels = [self._bottom_axis_label]
                for label in self._data_labels:
                    col_labels.append(label)
                
                with open(filename, 'w') as f:
                    f.write('\t'.join(col_labels) + '\n')
                self.filename = filename
            
            except Exception:
                self.filename = None
                _traceback.print_exc(file=_sys.stdout)
                
            self.pbt_read.setEnabled(False)
            self.timer.start()

        else:
            self.filename = None
            self.timer.stop()
            self.pbt_read.setEnabled(True)

    def read_value(self, monitor=False):
        """Read value."""
        pass

    def remove_value(self):
        """Remove value from list."""
        selected = self.tbl_table.selectedItems()
        rows = [s.row() for s in selected]
        n = len(self._xvalues)

        self._xvalues = [
            self._xvalues[i] for i in range(n) if i not in rows]

        for label in self._data_labels:
            readings = self._readings[label]
            self._readings[label] = [
                readings[i] for i in range(n) if i not in rows]

        self.update_table_values()
        self.update_plot()
        self.update_table_analysis_dialog()

    def save_to_file(self):
        """Save table values to file."""
        df = _utils.table_to_data_frame(self.tbl_table)
        if df is None:
            _QMessageBox.critical(
                self, 'Failure', 'Empty table.', _QMessageBox.Ok)
            return

        filename = _QFileDialog.getSaveFileName(
            self, caption='Save measurements file.', directory=self.directory,
            filter="Text files (*.txt *.dat)")

        if isinstance(filename, tuple):
            filename = filename[0]

        if len(filename) == 0:
            return

        try:
            if (not filename.endswith('.txt')
               and not filename.endswith('.dat')):
                filename = filename + '.txt'
            df.to_csv(filename, sep='\t')

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to save data to file.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)

    def set_table_column_size(self, size):
        """Set table horizontal header default section size."""
        self.tbl_table.horizontalHeader().setDefaultSectionSize(size)

    def show_table_analysis_dialog(self):
        """Show table analysis dialog."""
        df = _utils.table_to_data_frame(self.tbl_table)
        self.table_analysis_dialog.accept()
        self.table_analysis_dialog.show(df)

    def update_legend_items(self):
        """Update legend items."""
        if self._show_legend:
            self.clear_legend_items()
            self._legend_items = []
            for label in self._data_labels:
                legend_label = label.split('[')[0]
                self._legend_items.append(legend_label)
                self.legend.addItem(self._graphs[label], legend_label)

    def update_monitor_interval(self):
        """Update monitor interval value."""
        self.timer.setInterval(1000/self.sbd_monitor_freq.value())

    def update_plot(self):
        """Update plot values."""
        if len(self._xvalues) == 0:
            for label in self._data_labels:
                self._graphs[label].setData(
                    _np.array([]), _np.array([]))
            return

        with _warnings.catch_warnings():
            _warnings.simplefilter("ignore")
            if self._is_timestamp:
                timeinterval = _np.array(self._xvalues) - self._xvalues[0]
            
            for label in self._data_labels:
                readings = []
                for r in self._readings[label]:
                    if r is not None:
                        readings.append(r)
                    else:
                        readings.append(_np.nan)
                readings = _np.array(readings)
                
                if self._is_timestamp:
                    x = timeinterval[_np.isfinite(readings)]
                else:
                    x = _np.array(self._xvalues)[_np.isfinite(readings)]
                rd = readings[_np.isfinite(readings)]
                self._graphs[label].setData(x, rd)

        if len(self._xvalues) > 2 and self.tbt_autorange.isChecked():
            if self._is_timestamp:
                xmin = timeinterval[0]
                xmax = timeinterval[-1]
            else:
                xmin = self._xvalues[0]
                xmax = self._xvalues[-1]
            self.pw_plot.plotItem.getViewBox().setRange(xRange=(xmin, xmax))

    def update_table_analysis_dialog(self):
        """Update table analysis dialog."""
        self.table_analysis_dialog.update_data(
            _utils.table_to_data_frame(self.tbl_table))

    def update_table_values(self):
        """Update table values."""
        n = len(self._xvalues)
        self.tbl_table.clearContents()
        self.tbl_table.setRowCount(n)

        for i in range(n):
            if self._is_timestamp:
                dt = self._xvalues[i] - self._xvalues[0]
                self.tbl_table.setItem(
                    i, 0, _QTableWidgetItem(
                        self._bottom_axis_format.format(dt)))
            else:
                self.tbl_table.setItem(
                    i, 0, _QTableWidgetItem(
                        self._bottom_axis_format.format(self._xvalues[i])))

            for j, label in enumerate(self._data_labels):
                fmt = self._data_formats[j]
                reading = self._readings[label][i]
                if reading is None:
                    reading = _np.nan
                self.tbl_table.setItem(
                    i, j+1, _QTableWidgetItem(fmt.format(reading)))

        vbar = self.tbl_table.verticalScrollBar()
        vbar.setValue(vbar.maximum())


class ConfigurationWidget(_QWidget):
    """Configuration widget class for the control application."""

    def __init__(self, uifile, config, parent=None):
        """Set up the ui."""
        super().__init__(parent)
        self.ui = _uic.loadUi(uifile, self)
        self.config = config
        self.update_ids()
        self.sb_names = []
        self.sbd_names = []
        self.cmb_names = []
        self.le_names = []
        self.te_names = []
        self.chb_names = []

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

    def clear_load_options(self):
        """Clear load options."""
        self.ui.cmb_idn.setCurrentIndex(-1)

    def connect_signal_slots(self):
        """Create signal/slot connections."""
        for name in self.sb_names:
            sb = getattr(self.ui, 'sb_' + name)
            sb.valueChanged.connect(self.clear_load_options)

        for name in self.sbd_names:
            sbd = getattr(self.ui, 'sbd_' + name)
            sbd.valueChanged.connect(self.clear_load_options)

        for name in self.cmb_names:
            cmb = getattr(self.ui, 'cmb_' + name)
            cmb.currentIndexChanged.connect(self.clear_load_options)

        for name in self.le_names:
            le = getattr(self.ui, 'le_' + name)
            le.editingFinished.connect(self.clear_load_options)

        for name in self.te_names:
            te = getattr(self.ui, 'te_' + name)
            te.textChanged.connect(self.clear_load_options)

        for name in self.chb_names:
            chb = getattr(self.ui, 'chb_' + name)
            chb.stateChanged.connect(self.clear_load_options)

        self.ui.cmb_idn.currentIndexChanged.connect(self.enable_load_db)
        self.ui.tbt_update_idn.clicked.connect(self.update_ids)
        self.ui.pbt_load_db.clicked.connect(self.load_db)
        self.ui.tbt_save_db.clicked.connect(
            lambda: self.save_db(force=True))

    def enable_load_db(self):
        """Enable button to load configuration from database."""
        if self.ui.cmb_idn.currentIndex() != -1:
            self.ui.pbt_load_db.setEnabled(True)
        else:
            self.ui.pbt_load_db.setEnabled(False)

    def load_db(self):
        """Load configuration from database to set parameters."""
        try:
            idn = int(self.ui.cmb_idn.currentText())
        except Exception:
            _QMessageBox.critical(
                self, 'Failure', 'Invalid database ID.', _QMessageBox.Ok)
            return

        try:
            self.update_ids()
            idx = self.ui.cmb_idn.findText(str(idn))
            if idx == -1:
                self.ui.cmb_idn.setCurrentIndex(-1)
                _QMessageBox.critical(
                    self, 'Failure', 'Invalid database ID.', _QMessageBox.Ok)
                return

            self.config.clear()
            self.config.db_update_database(
                self.database_name, mongo=self.mongo, server=self.server)
            self.config.db_read(idn)
        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to read from database.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return

        self.load()
        self.ui.cmb_idn.setCurrentIndex(self.ui.cmb_idn.findText(str(idn)))
        self.ui.pbt_load_db.setEnabled(False)

    def load_last_db_entry(self):
        """Load configuration from database to set parameters."""
        try:
            self.config.clear()
            self.config.db_update_database(
                self.database_name, mongo=self.mongo, server=self.server)
            self.config.db_read()

            idn = self.config.idn
            self.update_ids()
            idx = self.ui.cmb_idn.findText(str(idn))
            if idx == -1:
                self.ui.cmb_idn.setCurrentIndex(-1)
                return

        except Exception:
            return

        self.load()
        self.ui.cmb_idn.setCurrentIndex(self.ui.cmb_idn.findText(str(idn)))
        self.ui.pbt_load_db.setEnabled(False)

    def load(self):
        """Load configuration to graphic interface."""
        try:
            for name in self.sb_names:
                sb = getattr(self.ui, 'sb_' + name)
                value = getattr(self.config, name)
                sb.setValue(value)

            for name in self.sbd_names:
                sbd = getattr(self.ui, 'sbd_' + name)
                value = getattr(self.config, name)
                sbd.setValue(value)

            for name in self.cmb_names:
                cmb = getattr(self.ui, 'cmb_' + name)
                value = getattr(self.config, name)
                cmb.setCurrentIndex(cmb.findText(str(value)))

            for name in self.le_names:
                le = getattr(self.ui, 'le_' + name)
                value = getattr(self.config, name)
                if value is None:
                    value = ''
                le.setText(str(value))
            
            for name in self.te_names:
                te = getattr(self.ui, 'te_' + name)
                value = getattr(self.config, name)
                if value is None:
                    value = ''
                te.setText(str(value))

            for name in self.chb_names:
                chb = getattr(self.ui, 'chb_' + name)
                value = getattr(self.config, name)
                chb.setChecked(value)

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to load configuration.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)

    def save_db(self, force=False):
        """Save parameters to database."""
        current_text = self.ui.cmb_idn.currentText()
        self.ui.cmb_idn.setCurrentIndex(-1)

        try:
            if self.update_configuration():

                if self.database_name is None:
                    msg = 'Invalid database filename.'
                    _QMessageBox.critical(
                        self, 'Failure', msg, _QMessageBox.Ok)
                    return False

                self.config.db_update_database(
                    self.database_name,
                    mongo=self.mongo, server=self.server)

                selected_config = self.config.copy()
                selected_config.clear()
                       
                if len(current_text) != 0:
                    selected_idn = int(current_text)
                    selected_config.db_read(selected_idn)
                
                if self.config == selected_config and not force:
                    idn = selected_idn
                    self.config.idn = idn
                else:
                    idn = self.config.db_save()
                
                self.ui.cmb_idn.addItem(str(idn))
                self.ui.cmb_idn.setCurrentIndex(self.ui.cmb_idn.count()-1)
                self.ui.pbt_load_db.setEnabled(False)
                return True

            else:
                return False

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            msg = 'Failed to save to database.'
            _QMessageBox.critical(self, 'Failure', msg, _QMessageBox.Ok)
            return False

    def update_ids(self):
        """Update IDs in combo box."""
        current_text = self.ui.cmb_idn.currentText()
        load_enabled = self.ui.pbt_load_db.isEnabled()
        self.ui.cmb_idn.clear()
        try:
            self.config.db_update_database(
                self.database_name,
                mongo=self.mongo, server=self.server)
            idns = self.config.db_get_id_list()
            self.ui.cmb_idn.addItems([str(idn) for idn in idns])
            if len(current_text) == 0:
                self.ui.cmb_idn.setCurrentIndex(self.ui.cmb_idn.count()-1)
                self.ui.pbt_load_db.setEnabled(True)
            else:
                self.ui.cmb_idn.setCurrentText(current_text)
                self.ui.pbt_load_db.setEnabled(load_enabled)
        except Exception:
            _traceback.print_exc(file=_sys.stdout)

    def update_configuration(self, clear=True):
        """Update configuration parameters."""
        try:
            if clear:
                self.config.clear()

            for name in self.sb_names:
                sb = getattr(self.ui, 'sb_' + name)
                setattr(self.config, name, sb.value())

            for name in self.sbd_names:
                sbd = getattr(self.ui, 'sbd_' + name)
                setattr(self.config, name, sbd.value())

            for name in self.cmb_names:
                cmb = getattr(self.ui, 'cmb_' + name)
                setattr(self.config, name, cmb.currentText())

            for name in self.le_names:
                le = getattr(self.ui, 'le_' + name)
                setattr(self.config, name, le.text().strip())

            for name in self.te_names:
                te = getattr(self.ui, 'te_' + name)
                setattr(
                    self.config, name, te.toPlainText().strip())

            for name in self.chb_names:
                chb = getattr(self.ui, 'chb_' + name)
                setattr(self.config, name, chb.isChecked())

            if self.config.valid_data():
                return True

            else:
                msg = 'Invalid configuration.'
                _QMessageBox.critical(
                    self, 'Failure', msg, _QMessageBox.Ok)
                return False

        except Exception:
            _traceback.print_exc(file=_sys.stdout)
            self.config.clear()
            return False
