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

import copy
import re

import signal_logger_ui.ui

from PySide2 import QtCore, QtWidgets
from signal_logger.config import TOPIC_SEPARATOR as _SEP

from signal_logger_ui.config import RPY_LABELS
from signal_logger_ui.signal_logger_types import LineStyle
from signal_logger_ui.signal_logger_plotcanvas import PlotFigure


class SignalLoggerTreeWidgetItem(QtWidgets.QTreeWidgetItem):
    def __init__(self, parent, displayText, actualText, hasData):
        super(SignalLoggerTreeWidgetItem, self).__init__(parent, [displayText])
        self._displayText = displayText
        self.originalText = displayText
        self.actualText = actualText
        self.hasData = hasData

    @property
    def displayText(self):
        return self._displayText

    @displayText.setter
    def displayText(self, value):
        self._displayText = value
        self.setText(0, self._displayText)


class TreeView(object):
    def __init__(self, name=None, parent=None, dataName=None):
        self.name = name
        if dataName is None:
            if parent is not None and parent.dataName is not None:
                self.dataName = parent.dataName + _SEP + self.name
            else:
                self.dataName = self.name
        else:
            self.dataName = dataName
        self.hasData = False
        self.leafs = []
        self.parent = parent
        self.modelIdxs = []
        self.widgets = []

    def leaf(self, name):
        for l in self.leafs:
            if l.name == name:
                return l
        self.leafs.append(TreeView(name, self))
        return self.leafs[-1]

    def add(self, key):
        if len(key) == 0:
            self.hasData = True
            return
        self.leaf(key[0]).add(key[1:])

    def simplify(self):
        while len(self.leafs) == 1 and not(self.hasData):
            self.name = self.name + _SEP + self.leafs[0].name
            self.dataName = self.leafs[0].dataName
            self.hasData = self.leafs[0].hasData
            self.leafs = self.leafs[0].leafs
            for l in self.leafs:
                l.parent = self
            for l in self.leafs:
                l.simplify()

    def update_y_selector(self, ySelector, parent, baseModelIdx=None):
        row = 0
        if all([l.name.isdigit() for l in self.leafs]):
            self.leafs.sort(key=lambda x: x.name)
        for l in self.leafs:
            l.widgets.append(SignalLoggerTreeWidgetItem(
                parent, l.name, l.dataName, l.hasData))
            if baseModelIdx is not None:
                l.modelIdxs.append(ySelector.model().index(
                    row, 0, baseModelIdx))
            else:
                l.modelIdxs.append(ySelector.model().index(row, 0))
            l.update_y_selector(ySelector, l.widgets[-1], l.modelIdxs[-1])
            row += 1

    def select(self, name, ySelector, idx, fullName=""):
        if name == fullName:
            selection = ySelector.selectionModel()
            selection.select(
                self.modelIdxs[idx], QtCore.QItemSelectionModel.Select)
            ySelector.setSelectionModel(selection)
            parent = self.parent
            while parent is not None and idx < len(parent.widgets):
                parent.widgets[idx].setExpanded(True)
                parent = parent.parent
        else:
            for l in self.leafs:
                if len(fullName):
                    fName = fullName + _SEP + l.name
                else:
                    fName = l.name
                if name.startswith(fName):
                    l.select(name, ySelector, idx, fName)

    def __print(self, indent):
        ret = "\n"
        if self.name is not None:
            ret = " "*indent + "| " + self.name + '\n'
        for l in self.leafs:
            ret += l.__print(indent + 1)
        return ret

    def __str__(self):
        return self.__print(-1)


class FilterRightClick(QtCore.QObject):
    def __init__(self, parent):
        super(FilterRightClick, self).__init__(parent)

    def eventFilter(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonPress:
            if event.button() == QtCore.Qt.RightButton:
                return True
        return False


class SpecialPlot(object):
    def __init__(self, name, figure, idx, special_id):
        self.figure = figure
        self.idx = idx
        self.name = name
        self.id = special_id
        self.added = []
        if idx == 0:
            self.remove = self.figure.remove_plot_left
        else:
            self.remove = self.figure.remove_plot_right
        if special_id == "deriv":
            self.__plot = self.__add_deriv
        elif special_id == "diff":
            self.__plot = self.__add_diff
        elif special_id == "p":
            self.__plot = self.__add_pitch
        elif special_id == "r":
            self.__plot = self.__add_roll
        elif special_id == "rpy":
            self.__plot = self.__add_rpy
        elif special_id == "smooth_deriv":
            self.__plot = self.__add_smooth_deriv
        elif special_id == "y":
            self.__plot = self.__add_yaw
        else:
            print("Cannot handle this special plot: {}".format(special_id))
        self.plot()

    def __add_diff(self):
        added = filter(
            lambda x: re.match(
                "{}($|{}.*$)".format(self.name, _SEP), x) is not None,
            self.figure.data.keys())
        if self.idx == 0:
            add_fn = self.figure.add_diff_plot_left
        else:
            add_fn = self.figure.add_diff_plot_right
        for a in added:
            label = a + _SEP + "diff"
            if add_fn(self.figure.x_data, a, label):
                self.added.append(label)

    def __add_deriv(self):
        added = filter(
            lambda x: re.match(
                "{}($|{}.*$)".format(self.name, _SEP), x) is not None,
            self.figure.data.keys())
        if self.idx == 0:
            add_fn = self.figure.add_deriv_plot_left
        else:
            add_fn = self.figure.add_deriv_plot_right
        for a in added:
            label = a + _SEP + "deriv"
            if add_fn(self.figure.x_data, a, label):
                self.added.append(label)

    def __add_smooth_deriv(self):
        added = filter(
            lambda x: re.match(
                "{}($|{}.*$)".format(self.name, _SEP), x) is not None,
            self.figure.data.keys())
        if self.idx == 0:
            add_fn = self.figure.add_smooth_deriv_plot_left
        else:
            add_fn = self.figure.add_smooth_deriv_plot_right
        for a in added:
            label = a + _SEP + "smooth_deriv"
            if add_fn(self.figure.x_data, a, label):
                self.added.append(label)

    def __add_rpy(self):
        if self.idx == 0:
            add_fn = self.figure.add_rpy_plot_left
        else:
            add_fn = self.figure.add_rpy_plot_right
        if add_fn(self.figure.x_data, self.name):
            self.added = [
                self.name + _SEP + s
                for s in ["roll", "pitch", "yaw"]]

    def __add_roll(self):
        if self.idx == 0:
            add_fn = self.figure.add_roll_plot_left
        else:
            add_fn = self.figure.add_roll_plot_right
        if add_fn(self.figure.x_data, self.name):
            self.added = [self.name + _SEP + "roll"]

    def __add_pitch(self):
        if self.idx == 0:
            add_fn = self.figure.add_pitch_plot_left
        else:
            add_fn = self.figure.add_pitch_plot_right
        if add_fn(self.figure.x_data, self.name):
            self.added = [self.name + _SEP + "pitch"]

    def __add_yaw(self):
        if self.idx == 0:
            add_fn = self.figure.add_yaw_plot_left
        else:
            add_fn = self.figure.add_yaw_plot_right
        if add_fn(self.figure.x_data, self.name):
            self.added = [self.name + _SEP + "yaw"]

    def plot(self):
        self.__plot()


class RemoveSpecialPlotButton(SpecialPlot, QtWidgets.QPushButton):
    def __init__(self, name, logtab, idx, special_id):
        self.logtab = logtab
        SpecialPlot.__init__(self, name, logtab.ui.canvas, idx, special_id)
        QtWidgets.QPushButton.__init__(
            self, u"Remove {} {} plot".format(name, special_id), logtab)
        self.clicked.connect(self.on_clicked)
        if idx == 0:
            self.layout = logtab.ui.y1SelectorLayout
        else:
            self.layout = logtab.ui.y2SelectorLayout
        self.layout.addWidget(self)
        if len(self.added) == 0:
            self.deleteLater()
        else:
            self.logtab.specials[name + _SEP + special_id] = self

    def plot(self):
        SpecialPlot.plot(self)
        self.logtab.ui.canvas.draw()

    def on_clicked(self):
        for added in self.added:
            self.remove(added)
        self.logtab.ui.canvas.draw()
        del self.logtab.specials[self.name + _SEP + self.id]
        self.deleteLater()


def set_label(label_fn, label_size_fn, label):
    if len(label.text) > 0:
        label_fn(label.text)
        label_size_fn(label.fontsize)


class SignalLoggerTab(QtWidgets.QWidget):
    canvas_need_update = QtCore.Signal()

    def __init__(self, parent=None):
        super(SignalLoggerTab, self).__init__(parent)
        self.ui = signal_logger_ui.ui.SignalLoggerTab()
        self.ui.setupUi(self)
        self.ui.canvas.setupLockButtons(self.ui.selectorLayout)
        if parent is not None:
            self.ui.canvas.grid = parent.gridStyles['left']
            self.ui.canvas.grid2 = parent.gridStyles['right']

        def setupSelector(ySelector):
            ySelector.setHeaderLabels(["Signal"])
            ySelector.header().setSectionResizeMode(
                QtWidgets.QHeaderView.ResizeMode.Stretch)
            ySelector.viewport().installEventFilter(
                FilterRightClick(ySelector))

        setupSelector(self.ui.y1Selector)
        setupSelector(self.ui.y2Selector)
        self.ui.y1Selector.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.ui.y2Selector.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.y1Selected = []
        self.y2Selected = []

        self.data = None
        self.rm = None
        self.ui.canvas.x_data = 't'
        self.x_data = 't'

        self.specials = {}

    def setData(self, data):
        self.data = data
        self.ui.canvas.setData(data)
        self.update_x_selector()
        self.update_y_selectors()

    def setGridStyles(self, gridStyles):
        self.ui.canvas.grid = copy.deepcopy(gridStyles['left'])
        self.ui.canvas.grid2 = copy.deepcopy(gridStyles['right'])

    @QtCore.Slot(str)
    def on_xSelector_activated(self, k):
        self.x_data = k
        self.ui.canvas.clear_all()
        self.ui.canvas.x_data = k
        self.y1Selected = []
        self.itemSelectionChanged(self.ui.y1Selector, self.y1Selected, 0)
        self.y2Selected = []
        self.itemSelectionChanged(self.ui.y2Selector, self.y2Selected, 1)
        for _, s in self.specials.items():
            s.plot()

    @QtCore.Slot(QtWidgets.QTreeWidgetItem, int)
    def on_y1Selector_itemClicked(self, item, col):
        self.y1Selected = self.itemSelectionChanged(
            self.ui.y1Selector, self.y1Selected, 0)

    @QtCore.Slot(QtWidgets.QTreeWidgetItem, int)
    def on_y2Selector_itemClicked(self, item, col):
        self.y2Selected = self.itemSelectionChanged(
            self.ui.y2Selector, self.y2Selected, 1)

    @QtCore.Slot(QtCore.QPoint)
    def on_y1Selector_customContextMenuRequested(self, point):
        self.showCustomMenu(self.ui.y1Selector, point, 0)

    @QtCore.Slot(QtCore.QPoint)
    def on_y2Selector_customContextMenuRequested(self, point):
        self.showCustomMenu(self.ui.y2Selector, point, 1)

    def itemSelectionChanged(self, ySelector, prevSelected, idx):
        if idx == 0:
            add_fn = self.ui.canvas.add_plot_left
        else:
            add_fn = self.ui.canvas.add_plot_right
        if idx == 0:
            remove_fn = self.ui.canvas.remove_plot_left
        else:
            remove_fn = self.ui.canvas.remove_plot_right
        selected_items = [
            (i.actualText, i.hasData) for i in ySelector.selectedItems()]

        def is_selected(s, x):
            if s[1]:
                return x == s[0]
            return re.match("{}($|{}.*$)".format(s[0], _SEP), x) is not None
        selected = sorted(filter(
            lambda x: any([is_selected(s, x) for s in selected_items]),
            self.data.keys()))

        max_signals_per_axis = 30
        if len(selected) > max_signals_per_axis:
            axis_name = "left" if idx == 0 else "right"
            msg = "Trying to plot {} > {} signals on the {} axis.\n\nUnselect some signals on the {}-hand side to proceed."
            msg_box = QtWidgets.QMessageBox()
            msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
            msg_box.setText(msg.format(len(selected), max_signals_per_axis, axis_name, axis_name))
            msg_box.setWindowTitle("Signal Overflow")
            msg_box.exec_()
            selected = prevSelected

        def find_item(s):
            iterator = QtWidgets.QTreeWidgetItemIterator(ySelector)
            for itm in [it.value() for it in iterator]:
                if itm.actualText == s:
                    return itm
            return None
        legends = [
            itm.actualText.replace(itm.originalText, itm.displayText)
            for itm in [find_item(s) for s in selected]]
        for s, l in zip(selected, legends):
            if s not in prevSelected:
                add_fn(self.x_data, s, l)
        for s in prevSelected:
            if s not in selected:
                remove_fn(s)
        self.ui.canvas.draw()
        return selected

    def update_x_selector(self):
        self.ui.xSelector.clear()
        self.ui.xSelector.addItems(sorted(self.data.keys()))
        idx = self.ui.xSelector.findText(self.x_data)
        if idx != -1:
            self.ui.xSelector.setCurrentIndex(idx)

    def update_y_selectors(self):
        self.ui.y1Selector.clear()
        self.ui.y2Selector.clear()
        self.tree_view = TreeView()
        for k in sorted(self.data.keys()):
            self.tree_view.add(k.split(_SEP))
        self.tree_view.simplify()

        def update_y_selector(ySelector):
            self.tree_view.update_y_selector(ySelector, ySelector)
            ySelector.resizeColumnToContents(0)

        update_y_selector(self.ui.y1Selector)
        update_y_selector(self.ui.y2Selector)

    def showCustomMenu(self, ySelector, point, idx):
        item = ySelector.itemAt(point)
        if item is None:
            return
        menu = QtWidgets.QMenu(ySelector)

        # Plot diff button
        action = QtWidgets.QAction(u"Plot diff".format(item.actualText), menu)
        action.triggered.connect(lambda: RemoveSpecialPlotButton(
            item.actualText, self, idx, "diff"))
        menu.addAction(action)

        # Plot deriv button
        action = QtWidgets.QAction(u"Plot deriv".format(item.actualText), menu)
        action.triggered.connect(lambda: RemoveSpecialPlotButton(item.actualText, self, idx, "deriv"))
        menu.addAction(action)

        # Plot smooth deriv button
        action = QtWidgets.QAction(u"Plot smooth deriv".format(item.actualText), menu)
        action.triggered.connect(lambda: RemoveSpecialPlotButton(item.actualText, self, idx, "smooth_deriv"))
        menu.addAction(action)

        # Plot roll/pitch/yaw buttons
        s = re.match("^(.*){}q?[wxyz]$".format(_SEP), item.actualText)
        if len(item.actualText.split('/')[-1]) < 2:
            pass  # single coordinate like 'x' or 'y'
        elif s is not None:
            for item_label, axis_label in RPY_LABELS:
                label = u"Plot {}".format(item_label, item.actualText)
                action = QtWidgets.QAction(label, menu)
                action.triggered.connect(
                    lambda label=axis_label:
                    RemoveSpecialPlotButton(s.group(1), self, idx, label))
                menu.addAction(action)
        else:  # s is None
            quat_childs = filter(
                lambda x: x is not None,
                [re.match(
                    '{}(({}.+)*){}q?w$'.format(item.actualText, _SEP, _SEP), x)
                 for x in self.data.keys()])
            for qc in quat_childs:
                for item_label, axis_label in RPY_LABELS:
                    if len(qc.group(1)):
                        action_text = u"Plot {} {}".format(
                            qc.group(1)[1:], item_label)
                    else:
                        action_text = u"Plot {}".format(item_label)
                    action = QtWidgets.QAction(action_text, menu)
                    plot_name = item.actualText + qc.group(1)
                    action.triggered.connect(
                        lambda name=plot_name, label=axis_label:
                        RemoveSpecialPlotButton(name, self, idx, label))
                    menu.addAction(action)
        menu.exec_(ySelector.viewport().mapToGlobal(point))

    @staticmethod
    def MakeFigure(data, x, y1, y2, y1_label=None, y2_label=None, figure=None):
        if y1_label is None:
            return SignalLoggerTab.MakeFigure(
                data, x, y1, y2, y1, y2_label, figure)
        if y2_label is None:
            return SignalLoggerTab.MakeFigure(
                data, x, y1, y2, y1_label, y2, figure)
        if figure is None:
            return SignalLoggerTab.MakeFigure(
                data, x, y1, y2, y1_label, y2_label, PlotFigure())
        figure.setData(data)
        for y, yl in zip(y1, y1_label):
            figure.add_plot_left(x, y, yl)
        for y, yl in zip(y2, y2_label):
            figure.add_plot_right(x, y, yl)
        return figure

    @staticmethod
    def MakePlot(parent, x_data, y1, y2, y1_label=None, y2_label=None):
        if y1_label is None:
            return SignalLoggerTab.MakePlot(
                parent, x_data, y1, y2, y1, y2_label)
        if y2_label is None:
            return SignalLoggerTab.MakePlot(
                parent, x_data, y1, y2, y1_label, y2)
        tab = SignalLoggerTab(parent)
        tab.x_data = x_data
        tab.setData(parent.data)
        for y, yl in zip(y1, y1_label):
            tab.tree_view.select(y, tab.ui.y1Selector, 0)
        for y, yl in zip(y2, y2_label):
            tab.tree_view.select(y, tab.ui.y2Selector, 1)
        tab.y1Selected = y1
        tab.y2Selected = y2
        SignalLoggerTab.MakeFigure(
            parent.data, x_data, y1, y2, y1_label, y2_label, tab.ui.canvas)
        tab.ui.canvas.x_data = x_data
        return tab

    @staticmethod
    def UserFigure(data, p, figure=None, special=None):
        if figure is None:
            return SignalLoggerTab.UserFigure(
                data, p,
                SignalLoggerTab.MakeFigure(data, p.x, p.y1, p.y2), special)
        if special is None:
            return SignalLoggerTab.UserFigure(
                data, p, figure,
                lambda y, idx, id_:
                SignalLoggerTab.UserPlot(y, figure, idx, id_))

        set_label(
            figure.title, figure.title_fontsize, p.graph_labels.title)
        set_label(
            figure.x_label, figure.x_label_fontsize, p.graph_labels.x_label)
        set_label(
            figure.y1_label, figure.y1_label_fontsize, p.graph_labels.y1_label)
        set_label(
            figure.y2_label, figure.y2_label_fontsize, p.graph_labels.y2_label)

        def handle_yd(yds, idx):
            for yd in yds:
                match = re.match("(.*){}(.*)$".format(_SEP), yd)
                if match is None:
                    special(yd, idx, "diff")
                elif match.group(2) in ["deriv", "rpy", "r", "p", "y"]:
                    special(match.group(1), idx, match.group(2))
                else:
                    special(match.group(1), idx, "diff")

        handle_yd(p.y1d, 0)
        handle_yd(p.y2d, 1)
        if not isinstance(p.grid1, LineStyle):
            figure.grid = LineStyle(**p.grid1)
        else:
            figure.grid = p.grid1
        if not isinstance(p.grid2, LineStyle):
            figure.grid2 = LineStyle(**p.grid2)
        else:
            figure.grid2 = p.grid2
        for y, s in p.style.items():
            figure.style_left(y, s)
        for y, s in p.style2.items():
            figure.style_right(y, s)
        for param, value in p.extra.items():
            getattr(figure, param)(value)
        return figure

    @staticmethod
    def UserPlot(parent, p):
        tab = SignalLoggerTab.MakePlot(parent, p.x, p.y1, p.y2)
        SignalLoggerTab.UserFigure(
            parent.data, p, tab.ui.canvas,
            lambda y, idx, id_:
            RemoveSpecialPlotButton(y, tab, idx, id_))
        tab.ui.canvas.draw()
        return tab
