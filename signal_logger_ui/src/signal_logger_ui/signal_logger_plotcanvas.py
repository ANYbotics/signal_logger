#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
# Copyright 2020 ANYbotics AG
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from PySide2 import QtGui, QtWidgets

from PySide2.QtWidgets import QWidget, QVBoxLayout, QDialog

import math
import numpy
import matplotlib
import matplotlib.pyplot as plt

try:
    matplotlib.use('Qt4Agg')
    matplotlib.rcParams['backend.qt4'] = 'PySide'
    import matplotlib.pyplot
except Exception:
    pass

from collections import OrderedDict
from math import asin, atan2
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
from numpy import array
from signal_logger.config import TOPIC_SEPARATOR as _SEP

from .signal_logger_types import LineStyle


def rpy_from_quat(quat):
    """
    Roll-pitch-yaw angles of a quaternion.

    Parameters
    ----------
    quat : (4,) array
        Quaternion in `[w x y z]` format.

    Returns
    -------
    rpy : (3,) array
        Array of roll-pitch-yaw angles, in [rad].

    Notes
    -----
    Roll-pitch-yaw are Euler angles corresponding to the sequence (1, 2, 3), also known as Euler XYZ angles. Check out
    the Kindr cheat sheet <https://github.com/ANYbotics/kindr/blob/master/doc/cheatsheet/cheatsheet_latest.pdf>.
    """
    roll = atan2(
        2 * quat[2] * quat[3] + 2 * quat[0] * quat[1],
        quat[3] ** 2 - quat[2] ** 2 - quat[1] ** 2 + quat[0] ** 2)
    pitch = -asin(
        2 * quat[1] * quat[3] - 2 * quat[0] * quat[2])
    yaw = atan2(
        2 * quat[1] * quat[2] + 2 * quat[0] * quat[3],
        quat[1] ** 2 + quat[0] ** 2 - quat[3] ** 2 - quat[2] ** 2)
    return array([roll, pitch, yaw])


class PlotFigure(object):
    def __init__(self):
        self.fig = plt.figure(figsize=(5, 4), dpi=100)
        self.axes = self.fig.add_subplot(111)
        self.axes.autoscale(enable=True, axis='both', tight=False)
        self.axes.autoscale_view(False, True, True)
        self.axes.xaxis.set_visible(False)
        self.axes.yaxis.set_visible(False)
        self.axes2 = self.axes.twinx()
        self.axes2.autoscale_view(False, True, True)
        self.axes2.yaxis.set_visible(False)
        self.axes_format_coord = self.axes.format_coord
        self.axes2_format_coord = self.axes2.format_coord
        self.axes.format_coord = self.format_coord
        self.axes2.format_coord = self.format_coord

        self.grid = LineStyle()
        self.grid2 = LineStyle()

        self._x_label_fontsize = 10
        self._y1_label_fontsize = 10
        self._y2_label_fontsize = 10
        self._labelpad = 10
        self._tick_labelsize = 10
        self._legend_fontsize = 10
        self._top_offset = 0.9
        self._bottom_offset = 0.1
        self._y1_legend_ncol = 3
        self._y2_legend_ncol = 3

        self.data = None
        self.computed_data = {}
        self.axes_plots = OrderedDict()
        self.axes2_plots = OrderedDict()

        color_set = matplotlib.cm.Set1
        rgb_from_float = lambda x: color_set(x)
        is_not_yellow = lambda x: abs(x * color_set.N - 6) > 0.5
        self.colors = [
            '#%02x%02x%02x' % tuple((255 * array(rgb_from_float(color_float)[0:3])).astype(int))
            for color_float in numpy.linspace(0, 1, color_set.N)
            if is_not_yellow(color_float)]
        self.color = 0
        self.nb_colors = color_set.N - 1
        self.x_data = 't'

    def _drawGrid(self):
        def draw(axes, style):
            axes.grid(
                color=style.color, linestyle=style.linestyle,
                linewidth=style.linewidth,
                # visible=style.visible,  # disabled when switching to Python3
                which='both')
            axes.set_axisbelow(True)
        if len(self.axes_plots) > 0:
            draw(self.axes, self.grid)
        if len(self.axes2_plots) > 0:
            draw(self.axes2, self.grid2)

    def draw(self, x_limits=None, y1_limits=None, y2_limits=None):
        def fix_axes_limits(axes, axes2):
            p0 = axes2.dataLim.get_points()[0]
            p1 = axes2.dataLim.get_points()[1]
            point = (p1 + p0) / 2
            plt, = axes.plot([point[0]], [point[1]], visible=False)
            plt.remove()
            del plt
            axes.relim()
        if len(self.axes_plots) == 0 and len(self.axes2_plots) != 0:
            fix_axes_limits(self.axes, self.axes2)
        if len(self.axes2_plots) == 0 and len(self.axes_plots) != 0:
            fix_axes_limits(self.axes2, self.axes)

        def set_axes_limits(axes, xlim=None, ylim=None):
            dataLim = axes.dataLim.get_points()
            x_range = (dataLim[1][0] - dataLim[0][0])/2
            y_range = (dataLim[1][1] - dataLim[0][1])/2
            x_min = dataLim[0][0] - x_range*0.05
            x_max = dataLim[1][0] + x_range*0.05
            if xlim is not None:
                x_min = max(x_min, xlim[0])
                x_max = min(x_max, xlim[1])
            axes.set_xlim([x_min, x_max])
            if ylim is None:
                axes.set_ylim([
                    dataLim[0][1] - y_range * 0.05,
                    dataLim[1][1] + y_range * 0.05])
            else:
                axes.set_ylim(ylim)
            return x_min, x_max

        if len(self.axes_plots) > 0:
            x_limits = set_axes_limits(
                self.axes, xlim=x_limits, ylim=y1_limits)
        if len(self.axes2_plots) > 0:
            x_limits = set_axes_limits(
                self.axes2, xlim=x_limits, ylim=y2_limits)
            self.axes.set_xlim(x_limits)
        self._legend_left()
        self._legend_right()
        self._drawGrid()
        top_offset = self._top_offset
        top_legend_rows = math.ceil(len(self.axes_plots.keys()) / 3.)
        if top_legend_rows > 3:
            top_offset = top_offset - 0.015 * (top_legend_rows - 3)
        bottom_offset = self._bottom_offset
        bottom_legend_rows = math.ceil(len(self.axes2_plots.keys()) / 3.)
        if bottom_legend_rows > 3:
            bottom_offset = bottom_offset + 0.015 * (bottom_legend_rows - 3)
        self.fig.subplots_adjust(top=top_offset, bottom=bottom_offset)

    def setData(self, data):
        self.data = data

    def show(self):
        self.fig.show()

    def top_offset(self, off=None):
        if off is None:
            return self._top_offset
        else:
            self._top_offset = off

    def bottom_offset(self, off=None):
        if off is None:
            return self._bottom_offset
        else:
            self._bottom_offset = off

    def title(self, title=None):
        if title is None:
            if self.fig._suptitle is None:
                return ""
            return self.fig._suptitle.get_text()
        else:
            self.fig.suptitle(title)

    def title_fontsize(self, fontsize=None):
        if fontsize is None:
            if self.fig._suptitle is None:
                return 12
            return self.fig._suptitle.get_fontsize()
        else:
            self.fig.suptitle(self.title(), fontsize=fontsize)

    def tick_fontsize(self, size=None):
        if size is None:
            return self._tick_labelsize
        else:
            self._tick_labelsize = size
            self.axes.tick_params(labelsize=self._tick_labelsize)
            self.axes2.tick_params(labelsize=self._tick_labelsize)

    def labelpad(self, pad=None):
        if pad is None:
            return self._labelpad
        else:
            self._labelpad = pad
            self._x_label()
            self._y1_label()
            self._y2_label()

    def _x_label(self, label=None):
        if label is None:
            label = self.x_label()
        self.axes.set_xlabel(
            label, fontsize=self._x_label_fontsize, labelpad=self._labelpad)

    def _y1_label(self, label=None):
        if label is None:
            label = self.y1_label()
        self.axes.set_ylabel(
            label, fontsize=self._y1_label_fontsize, labelpad=self._labelpad)

    def _y2_label(self, label=None):
        if label is None:
            label = self.y2_label()
        self.axes2.set_ylabel(
            label, fontsize=self._y2_label_fontsize, labelpad=self._labelpad)

    def x_label(self, label=None):
        if label is None:
            return self.axes.get_xlabel()
        self._x_label(label)

    def x_label_fontsize(self, fontsize=None):
        if fontsize is None:
            return self._x_label_fontsize
        else:
            self._x_label_fontsize = fontsize
            self._x_label()

    def y1_label(self, label=None):
        if label is None:
            return self.axes.get_ylabel()
        self._y1_label(label)

    def y1_label_fontsize(self, fontsize=None):
        if fontsize is None:
            return self._y1_label_fontsize
        else:
            self._y1_label_fontsize = fontsize
            self._y1_label()

    def y2_label(self, label=None):
        if label is None:
            return self.axes2.get_ylabel()
        self._y2_label(label)

    def y2_label_fontsize(self, fontsize=None):
        if fontsize is None:
            return self._y2_label_fontsize
        else:
            self._y2_label_fontsize = fontsize
            self._y2_label()

    def _next_color(self):
        self.color += 1
        return self.colors[(self.color - 1) % self.nb_colors]

    def legend_fontsize(self, size=None):
        if size is None:
            return self._legend_fontsize
        else:
            self._legend_fontsize = size
            self._legend_left()
            self._legend_right()

    def y1_legend_ncol(self, n=None):
        if n is None:
            return self._y1_legend_ncol
        self._y1_legend_ncol = n
        self._legend_left()

    def y2_legend_ncol(self, n=None):
        if n is None:
            return self._y2_legend_ncol
        self._y2_legend_ncol = n
        self._legend_right()

    def _legend_left(self):
        if len(self.axes_plots) > 0:
            self.axes.legend(
                bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
                ncol=self._y1_legend_ncol, mode="expand", borderaxespad=0.5,
                fontsize=self._legend_fontsize)

    def _legend_right(self):
        top_anchor = -0.125
        if len(self.x_label()):
            top_anchor = -0.175
        if len(self.axes2_plots):
            self.axes2.legend(
                bbox_to_anchor=(0., top_anchor, 1., .102), loc=2,
                ncol=self._y2_legend_ncol, mode="expand", borderaxespad=0.5,
                fontsize=self._legend_fontsize)

    def _show_left_axis(self):
        self.axes.yaxis.set_visible(True)
        if len(self.axes_plots) < 1 and len(self.axes2_plots) < 1:
            self.axes.xaxis.set_visible(True)

    def _show_right_axis(self):
        self.axes2.yaxis.set_visible(True)
        if len(self.axes_plots) < 1 and len(self.axes2_plots) < 1:
            self.axes.xaxis.set_visible(True)

    def _hide_left_axis(self):
        self.axes.yaxis.set_visible(False)
        if len(self.axes_plots) < 1 and len(self.axes2_plots) < 1:
            self.axes.xaxis.set_visible(False)

    def _hide_right_axis(self):
        self.axes2.yaxis.set_visible(False)
        if len(self.axes_plots) < 1 and len(self.axes2_plots) < 1:
            self.axes.xaxis.set_visible(False)

    def _plot(self, axe, update_legend_fn, x, y, y_label, style=None):
        if style is None:
            plt = axe.plot(x, y, label=y_label, color=self._next_color())
        else:
            plt = axe.plot(
                x, y, label=y_label, color=style.color,
                linestyle=style.linestyle, linewidth=style.linewidth)
        update_legend_fn()
        return plt[0]

    def add_plot_left(self, x, y, y_label, style=None):
        self._show_left_axis()
        if y_label in self.axes_plots:
            return False
        self.axes_plots[y] = self._plot(
            self.axes, self._legend_left, self.data[x], self.data[y], y_label,
            style)
        return True

    def add_plot_right(self, x, y, y_label, style=None):
        self._show_right_axis()
        if y_label in self.axes2_plots:
            return False
        self.axes2_plots[y] = self._plot(
            self.axes2, self._legend_right, self.data[x], self.data[y],
            y_label, style)
        return True

    def _add_diff_plot(self, axes, legend, x, y, y_label):
        return self._plot(
            axes, legend, self.data[x][1:], numpy.diff(self.data[y]), y_label)

    def add_diff_plot_left(self, x, y, y_label):
        self._show_left_axis()
        if y_label in self.axes_plots:
            return False
        self.axes_plots[y_label] = self._add_diff_plot(
            self.axes, self._legend_left, x, y, y_label)
        return True

    def add_diff_plot_right(self, x, y, y_label):
        self._show_right_axis()
        if y_label in self.axes2_plots:
            return False
        self.axes2_plots[y_label] = self._add_diff_plot(
            self.axes2, self._legend_right, x, y, y_label)
        return True

    def _add_deriv_plot(self, axes, legend, x, y, y_label):
        times = self.data[x]
        y_vals = self.data[y]
        deriv_vals = []
        dt = times[1] - times[0]
        for i in range(1, len(times)):
            diff = y_vals[i] - y_vals[i - 1]
            new_dt = times[i] - times[i - 1]
            if new_dt > 0.:
                dt = new_dt
            deriv_vals.append(diff / dt)
        return self._plot(axes, legend, times[1:], deriv_vals, y_label)

    def add_deriv_plot_left(self, x, y, y_label):
        self._show_left_axis()
        if y_label in self.axes_plots:
            return False
        self.axes_plots[y_label] = self._add_deriv_plot(self.axes, self._legend_left, x, y, y_label)
        return True

    def add_deriv_plot_right(self, x, y, y_label):
        self._show_right_axis()
        if y_label in self.axes2_plots:
            return False
        self.axes2_plots[y_label] = self._add_deriv_plot(self.axes2, self._legend_right, x, y, y_label)
        return True

    def _add_smooth_deriv_plot(self, axes, legend, x, y, y_label):
        dt = numpy.average(numpy.diff(self.data[x]))
        return self._plot(
            axes, legend, self.data[x][1:], numpy.diff(self.data[y]) / dt, y_label)

    def add_smooth_deriv_plot_left(self, x, y, y_label):
        self._show_left_axis()
        if y_label in self.axes_plots:
            return False
        self.axes_plots[y_label] = self._add_smooth_deriv_plot(self.axes, self._legend_left, x, y, y_label)
        return True

    def add_smooth_deriv_plot_right(self, x, y, y_label):
        self._show_right_axis()
        if y_label in self.axes2_plots:
            return False
        self.axes2_plots[y_label] = self._add_smooth_deriv_plot(self.axes2, self._legend_right, x, y, y_label)
        return True

    def _add_roll_plot(self, axes, legend, x_label, base):
        if (base + _SEP + "qw") in self.data.keys():
            fmt = "q"
        else:
            fmt = ""
        keys = [base + _SEP + fmt + ax for ax in ["w", "x", "y", "z"]]
        qw, qx, qy, qz = [self.data[k] for k in keys]
        rpys = [
            rpy_from_quat([w, x, y, z])
            for w, x, y, z in zip(qw, qx, qy, qz)]
        r = [rpy[0] for rpy in rpys]
        return self._plot(
            axes, legend, self.data[x_label], r, base + _SEP + "roll")

    def add_roll_plot_left(self, x, y):
        self._show_left_axis()
        key = y + _SEP + "roll"
        if key in self.axes_plots:
            return False
        self.axes_plots[key] = self._add_roll_plot(
            self.axes, self._legend_left, x, y)
        return True

    def add_roll_plot_right(self, x, y):
        self._show_right_axis()
        key = y + _SEP + "roll"
        if key in self.axes2_plots:
            return False
        self.axes2_plots[key] = self._add_roll_plot(
            self.axes2, self._legend_right, x, y)
        return True

    def _add_pitch_plot(self, axes, legend, x_label, base):
        if (base + _SEP + "qw") in self.data.keys():
            fmt = "q"
        else:
            fmt = ""
        keys = [base + _SEP + fmt + ax for ax in ["w", "x", "y", "z"]]
        qw, qx, qy, qz = [self.data[k] for k in keys]
        rpys = [
            rpy_from_quat([w, x, y, z])
            for w, x, y, z in zip(qw, qx, qy, qz)]
        p = [rpy[1] for rpy in rpys]
        return self._plot(
            axes, legend, self.data[x_label], p, base + _SEP + "pitch")

    def add_pitch_plot_left(self, x, y):
        self._show_left_axis()
        key = y + _SEP + "pitch"
        if key in self.axes_plots:
            return False
        self.axes_plots[key] = self._add_pitch_plot(
            self.axes, self._legend_left, x, y)
        return True

    def add_pitch_plot_right(self, x, y):
        self._show_right_axis()
        key = y + _SEP + "pitch"
        if key in self.axes2_plots:
            return False
        self.axes2_plots[key] = self._add_pitch_plot(
            self.axes2, self._legend_right, x, y)
        return True

    def _add_yaw_plot(self, axes, legend, x_label, base):
        if (base + _SEP + "qw") in self.data.keys():
            fmt = "q"
        else:
            fmt = ""
        keys = [base + _SEP + fmt + ax for ax in ["w", "x", "y", "z"]]
        qw, qx, qy, qz = [self.data[k] for k in keys]
        rpys = [
            rpy_from_quat([w, x, y, z])
            for w, x, y, z in zip(qw, qx, qy, qz)]
        y = [rpy[2] for rpy in rpys]
        return self._plot(
            axes, legend, self.data[x_label], y, base + _SEP + "yaw")

    def add_yaw_plot_left(self, x, y):
        self._show_left_axis()
        key = y + _SEP + "yaw"
        if key in self.axes_plots:
            return False
        self.axes_plots[key] = self._add_yaw_plot(
            self.axes, self._legend_left, x, y)
        return True

    def add_yaw_plot_right(self, x, y):
        self._show_right_axis()
        key = y + _SEP + "yaw"
        if key in self.axes2_plots:
            return False
        self.axes2_plots[key] = self._add_yaw_plot(
            self.axes2, self._legend_right, x, y)
        return True

    def add_rpy_plot_left(self, x, y):
        self._show_left_axis()
        has_roll = (y + _SEP + "roll") in self.axes2_plots
        has_pitch = (y + _SEP + "pitch") in self.axes2_plots
        has_yaw = (y + _SEP + "yaw") in self.axes2_plots
        if has_roll and has_pitch and has_yaw:
            return False
        if not has_roll:
            self.add_roll_plot_left(x, y)
        if not has_pitch:
            self.add_pitch_plot_left(x, y)
        if not has_yaw:
            self.add_yaw_plot_left(x, y)
        return True

    def add_rpy_plot_right(self, x, y):
        self._show_right_axis()
        has_roll = (y + _SEP + "roll") in self.axes2_plots
        has_pitch = (y + _SEP + "pitch") in self.axes2_plots
        has_yaw = (y + _SEP + "yaw") in self.axes2_plots
        if has_roll and has_pitch and has_yaw:
            return False
        if not has_roll:
            self.add_roll_plot_right(x, y)
        if not has_pitch:
            self.add_pitch_plot_right(x, y)
        if not has_yaw:
            self.add_yaw_plot_right(x, y)
        return True

    def remove_plot_left(self, y_label):
        if y_label not in self.axes_plots:
            return
        self.axes_plots[y_label].remove()
        del self.axes_plots[y_label]
        if len(self.axes_plots) > 0:
            self.axes.relim()
            self._legend_left()
        else:
            self.axes.clear()
            self._hide_left_axis()
        if len(self.axes_plots) == 0 and len(self.axes2_plots) == 0:
            self.color = 0

    def remove_plot_right(self, y_label):
        self.axes2.yaxis.set_visible(True)
        if y_label not in self.axes2_plots:
            return
        self.axes2_plots[y_label].remove()
        del self.axes2_plots[y_label]
        if len(self.axes2_plots):
            self.axes2.relim()
            self._legend_right()
        else:
            self.axes2.clear()
            self._hide_right_axis()
        if len(self.axes_plots) == 0 and len(self.axes2_plots) == 0:
            self.color = 0

    def format_coord(self, x, y):
        display_coord = self.axes2.transData.transform((x, y))
        inv = self.axes.transData.inverted()
        ax_coord = inv.transform(display_coord)
        if len(self.axes2_plots) and len(self.axes_plots):
            return "x: {:.3f}    y1: {:.3f}    y2: {:.3f}".format(
                x, ax_coord[1], y)
        elif len(self.axes_plots):
            return "x: {:.3f}    y1: {:.3f}".format(x, ax_coord[1])
        elif len(self.axes2_plots):
            return "x: {:.3f}    y2: {:.3f}".format(x, y)
        else:
            return "x: {:.3f}".format(x)

    def clear_all(self):
        self.color = 0
        self.axes_plots = {}
        self.axes2_plots = {}
        self.axes.clear()
        self.axes2.clear()

    def _style(self, plots, y, styleIn=None):
        if y not in plots:
            raise KeyError("No plot named {}".format(y))
        plt = plots[y]
        if styleIn is None:
            return LineStyle(
                plt.get_color(), plt.get_linestyle(), plt.get_linewidth(),
                label=plt.get_label())
        else:
            plt.set_color(styleIn.color)
            plt.set_linestyle(styleIn.linestyle)
            plt.set_linewidth(styleIn.linewidth)
            if len(styleIn.label):
                plt.set_label(styleIn.label)

    def style_left(self, y, styleIn=None):
        return self._style(self.axes_plots, y, styleIn)

    def style_right(self, y, styleIn=None):
        return self._style(self.axes2_plots, y, styleIn)


class SimpleAxesDialog(QDialog):

    def __init__(self, parent):
        QtWidgets.QDialog.__init__(self, parent)
        self.setWindowTitle('Edit axes limits')
        self.setModal(True)
        self.layout = QtWidgets.QGridLayout(self)
        self.layout.addWidget(QtWidgets.QLabel("Min"), 0, 1)
        self.layout.addWidget(QtWidgets.QLabel("Max"), 0, 2)

        self.layout.addWidget(QtWidgets.QLabel("X"), 1, 0)
        x_limits = parent.x_limits
        if x_limits is None:
            x_limits = parent.axes.get_xlim()
        self.x_min = QtWidgets.QLineEdit(str(x_limits[0]))
        self.x_min.setValidator(QtGui.QDoubleValidator())
        self.layout.addWidget(self.x_min, 1, 1)
        self.x_max = QtWidgets.QLineEdit(str(x_limits[1]))
        self.x_max.setValidator(QtGui.QDoubleValidator())
        self.x_init = [float(self.x_min.text()), float(self.x_max.text())]
        self.layout.addWidget(self.x_max, 1, 2)

        self.layout.addWidget(QtWidgets.QLabel("Y1"), 2, 0)
        y1_limits = parent.y1_limits
        if y1_limits is None:
            y1_limits = parent.axes.get_ylim()
        self.y1_min = QtWidgets.QLineEdit(str(y1_limits[0]))
        self.y1_min.setValidator(QtGui.QDoubleValidator())
        self.layout.addWidget(self.y1_min, 2, 1)
        self.y1_max = QtWidgets.QLineEdit(str(y1_limits[1]))
        self.y1_max.setValidator(QtGui.QDoubleValidator())
        self.y1_init = [float(self.y1_min.text()), float(self.y1_max.text())]
        self.layout.addWidget(self.y1_max, 2, 2)

        self.layout.addWidget(QtWidgets.QLabel("Y2"), 3, 0)
        y2_limits = parent.y2_limits
        if y2_limits is None:
            y2_limits = parent.axes2.get_ylim()
        self.y2_min = QtWidgets.QLineEdit(str(y2_limits[0]))
        self.y2_min.setValidator(QtGui.QDoubleValidator())
        self.layout.addWidget(self.y2_min, 3, 1)
        self.y2_max = QtWidgets.QLineEdit(str(y2_limits[1]))
        self.y2_max.setValidator(QtGui.QDoubleValidator())
        self.y2_init = [float(self.y2_min.text()), float(self.y2_max.text())]
        self.layout.addWidget(self.y2_max, 3, 2)

        confirmLayout = QtWidgets.QHBoxLayout()
        okButton = QtWidgets.QPushButton("Ok", self)
        confirmLayout.addWidget(okButton)
        okButton.clicked.connect(self.accept)
        applyButton = QtWidgets.QPushButton("Apply", self)
        confirmLayout.addWidget(applyButton)
        applyButton.clicked.connect(self.apply)
        cancelButton = QtWidgets.QPushButton("Cancel", self)
        confirmLayout.addWidget(cancelButton)
        cancelButton.clicked.connect(self.reject)
        self.layout.addLayout(confirmLayout, 4, 0, 1, 3)

    def apply(self):
        changed = False
        x_limits = [float(self.x_min.text()), float(self.x_max.text())]
        if x_limits != self.x_init:
            changed = True
            self.parent().x_locked.setChecked(True)
            self.parent().x_limits = x_limits
        y1_limits = [float(self.y1_min.text()), float(self.y1_max.text())]
        if y1_limits != self.y1_init:
            changed = True
            self.parent().y1_locked.setChecked(True)
            self.parent().y1_limits = y1_limits
        y2_limits = [float(self.y2_min.text()), float(self.y2_max.text())]
        if y2_limits != self.y2_init:
            changed = True
            self.parent().y2_locked.setChecked(True)
            self.parent().y2_limits = y2_limits
        if changed:
            self.parent().draw()

    def accept(self):
        QtWidgets.QDialog.accept(self)
        self.apply()


class PlotCanvasWithToolbar(PlotFigure, QWidget):

    def __init__(self, parent=None):
        PlotFigure.__init__(self)
        QWidget.__init__(self, parent)

        self.canvas = FigureCanvas(self.fig)
        self.canvas.mpl_connect('draw_event', self.on_draw)
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.toolbar.pan()  # Pan by default

        vbox = QVBoxLayout(self)
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.toolbar)
        self.setLayout(vbox)

    def setupLockButtons(self, layout):
        self.x_locked = QtWidgets.QPushButton(u"Lock X", self)
        self.x_locked.setCheckable(True)
        layout.addWidget(self.x_locked)
        self.x_locked.toggled.connect(self.x_locked_changed)
        self.x_limits = None

        self.y1_locked = QtWidgets.QPushButton(u"Lock Y1", self)
        self.y1_locked.setCheckable(True)
        layout.addWidget(self.y1_locked)
        self.y1_locked.toggled.connect(self.y1_locked_changed)
        self.y1_limits = None

        self.y2_locked = QtWidgets.QPushButton(u"Lock Y2", self)
        self.y2_locked.setCheckable(True)
        layout.addWidget(self.y2_locked)
        self.y2_locked.toggled.connect(self.y2_locked_changed)
        self.y2_limits = None

    def axesDialog(self):
        SimpleAxesDialog(self).exec_()

    def on_draw(self, event):
        pass

    def draw(self):
        self.canvas.toolbar.update()  # drop history of Home, Forward and Back buttons in the navigation toolbar
        PlotFigure.draw(self, self.x_limits, self.y1_limits, self.y2_limits)
        self.canvas.draw()

    def _y_lock_changed(self, name, cbox, get_lim):
        if cbox.isChecked():
            cbox.setText(u"Lock {}".format(name))
            return get_lim()
        else:
            cbox.setText(u"Lock {}".format(name))
            return None

    def x_locked_changed(self, status):
        self.x_limits = self._y_lock_changed(
            "X", self.x_locked, self.axes.get_xlim)

    def y1_locked_changed(self, status):
        self.y1_limits = self._y_lock_changed(
            "Y1", self.y1_locked, self.axes.get_ylim)

    def y2_locked_changed(self, status):
        self.y2_limits = self._y_lock_changed(
            "Y2", self.y2_locked, self.axes2.get_ylim)