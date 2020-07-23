#!/usr/bin/env python
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

import collections
import json
import os
import re

from PySide import QtCore, QtGui
from signal_logger import Silo
from signal_logger import find_log

import ui

from signal_logger_tab import SignalLoggerTab
from signal_logger_types import LineStyle, TextWithFontSize, GraphLabels


UserPlot = collections.namedtuple('UserPlot', [
    'title', 'x', 'y1', 'y1d', 'y2', 'y2d', 'grid1', 'grid2', 'style',
    'style2', 'graph_labels', 'extra'])


def load_user_plots(fpath):
    if not os.path.exists(fpath):
        return []
    with open(fpath) as f:
        user_plot_list = [UserPlot(*x) for x in json.load(f)]
        for i, plt in enumerate(user_plot_list):
            for y in plt.style:
                plt.style[y] = LineStyle(**plt.style[y])
            for y in plt.style2:
                plt.style2[y] = LineStyle(**plt.style2[y])
            if not isinstance(plt.graph_labels, GraphLabels):
                for key, value in plt.graph_labels.items():
                    plt.graph_labels[key] = TextWithFontSize(
                        **plt.graph_labels[key])
                user_plot_list[i] = plt._replace(graph_labels=GraphLabels(
                    **plt.graph_labels))
    return user_plot_list


class RobotAction(QtGui.QAction):

    def __init__(self, display, parent):
        super(RobotAction, self).__init__(display, parent)
        self._actual = display

    def actual(self, n=None):
        if n is None:
            return self._actual
        else:
            self._actual = n


class CommonStyleDialog(QtGui.QDialog):
    def __init__(self, parent, name, canvas, style):
        super(CommonStyleDialog, self).__init__(parent)

        self.name = name
        self.canvas = canvas
        self.style = style

        self.setWindowTitle('Edit {} grid style'.format(name))
        self.setModal(True)

        self.layout = QtGui.QFormLayout(self)

        self.linestyle = QtGui.QComboBox()
        styles = ['-', ':', '--', '-.']
        for s in styles:
            self.linestyle.addItem(s)
        self.linestyle.setCurrentIndex(styles.index(style.linestyle))
        self.layout.addRow("Style", self.linestyle)

        self.linewidth = QtGui.QLineEdit(str(style.linewidth))
        self.linewidth.setValidator(QtGui.QDoubleValidator(0.01, 1e6, 2))
        self.layout.addRow("Width", self.linewidth)

        self.color = QtGui.QColor(style.color)
        self.colorButton = QtGui.QPushButton("#")
        self.colorButton.setStyleSheet(
            "background-color: {}; color: {}".format(
                self.style.color, self.style.color))
        self.colorButton.released.connect(self.selectColor)
        self.layout.addRow("Color", self.colorButton)

        confirmLayout = QtGui.QHBoxLayout()
        okButton = QtGui.QPushButton("Ok", self)
        confirmLayout.addWidget(okButton)
        okButton.clicked.connect(self.accept)
        applyButton = QtGui.QPushButton("Apply", self)
        confirmLayout.addWidget(applyButton)
        applyButton.clicked.connect(self.apply)
        cancelButton = QtGui.QPushButton("Cancel", self)
        confirmLayout.addWidget(cancelButton)
        cancelButton.clicked.connect(self.reject)
        self.layout.addRow(confirmLayout)

    def selectColor(self):
        color = QtGui.QColorDialog.getColor(self.color)
        if color.isValid():
            self.color = color
            self.colorButton.setStyleSheet(
                "background-color: {}; color: {}".format(
                    self.color.name(), self.color.name()))

    def apply(self):
        self.style.linestyle = self.linestyle.currentText()
        self.style.linewidth = float(self.linewidth.text())
        self.style.color = self.color.name()

    def accept(self):
        super(CommonStyleDialog, self).accept()
        self.apply()


class GridStyleDialog(CommonStyleDialog):

    def __init__(self, parent, name, canvas, style):
        super(GridStyleDialog, self).__init__(parent, name, canvas, style)

        self.enabled = QtGui.QCheckBox()
        self.enabled.setChecked(style.visible)
        self.layout.insertRow(0, "Visible", self.enabled)

        self.save = QtGui.QCheckBox()
        self.layout.insertRow(
            self.layout.rowCount() - 2, "Save as default", self.save)

    def apply(self):
        super(GridStyleDialog, self).apply()
        self.style.visible = self.enabled.isChecked()
        self.canvas.draw()
        if self.save.isChecked():
            self.parent().gridStyles[self.name] = self.style
            with open(self.parent().gridStyleFile, 'w') as f:
                json.dump(
                    self.parent().gridStyles, f, default=lambda o: o.__dict__)


class LineStyleDialog(CommonStyleDialog):

    def __init__(self, parent, name, canvas, style, set_style_fn):
        super(LineStyleDialog, self).__init__(parent, name, canvas, style)
        self.set_style = set_style_fn
        self.labelInput = QtGui.QLineEdit(style.label)
        self.layout.insertRow(0, "Label", self.labelInput)

    def apply(self):
        super(LineStyleDialog, self).apply()
        self.style.label = self.labelInput.text()
        self.set_style(self.name, self.style)
        self.canvas.draw()


class AllLineStyleDialog(QtGui.QDialog):

    def __init__(self, parent, name, canvas, plots, style_fn):
        super(AllLineStyleDialog, self).__init__(parent)

        self.name = name
        self.canvas = canvas
        self.plots = plots
        self.style = style_fn

        self.setWindowTitle('Edit {} graph line style'.format(name))
        self.setModal(True)

        self.layout = QtGui.QGridLayout(self)

        row = 0
        [
            self.layout.addWidget(QtGui.QLabel(txt), row, i)
            for i, txt in enumerate(["Label", "Style", "Width", "Color"])
        ]
        row += 1

        self.plotWidgets = {}

        def makeLineStyleComboBox(style):
            ret = QtGui.QComboBox()
            [ret.addItem(s) for s in ['-', ':', '--', '-.']]
            ret.setCurrentIndex(['-', ':', '--', '-.'].index(style.linestyle))
            return ret

        def makeLineWidthEdit(style):
            ret = QtGui.QLineEdit(str(style.linewidth))
            ret.setValidator(QtGui.QDoubleValidator(0.01, 1e6, 2))
            return ret

        def makeColorButton(self, style):
            ret = QtGui.QPushButton("#")
            ret.color = QtGui.QColor(style.color)
            ret.setStyleSheet(
                "background-color: {color}; color: {color}".format(
                    color=style.color))
            ret.released.connect(lambda bt=ret: self.selectColor(bt))
            return ret

        def add_plot(self, plot, style):
            self.plotWidgets[plot] = [
                QtGui.QLineEdit(style.label),
                makeLineStyleComboBox(style),
                makeLineWidthEdit(style),
                makeColorButton(self, style)
            ]
            [
                self.layout.addWidget(w, row, i)
                for i, w in enumerate(self.plotWidgets[plot])
            ]

        for p in self.plots:
            add_plot(self, p, self.style(p))
            row += 1

        hlayout = QtGui.QHBoxLayout()
        okButton = QtGui.QPushButton("Ok", self)
        okButton.clicked.connect(self.accept)
        cancelButton = QtGui.QPushButton("Cancel", self)
        cancelButton.clicked.connect(self.reject)
        applyButton = QtGui.QPushButton("Apply", self)
        applyButton.clicked.connect(self.apply)
        hlayout.addWidget(okButton)
        hlayout.addWidget(cancelButton)
        hlayout.addWidget(applyButton)
        self.layout.addLayout(hlayout, row, 1, 1, 3)

    def selectColor(self, button):
        color = QtGui.QColorDialog.getColor(button.color)
        if color.isValid():
            button.color = color
            button.setStyleSheet(
                "background-color: {color}; color: {color}".format(
                    color=color.name()))

    def apply(self):
        for y, widgets in self.plotWidgets.iteritems():
            label = widgets[0].text()
            linestyle = widgets[1].currentText()
            linewidth = float(widgets[2].text())
            color = widgets[3].color.name()
            st = LineStyle(
                label=label, linestyle=linestyle, linewidth=linewidth,
                color=color)
            self.style(y, st)
        self.canvas.draw()

    def accept(self):
        super(AllLineStyleDialog, self).accept()
        self.apply()


class LabelsTitleEditDialog(QtGui.QDialog):

    def __init__(self, parent, canvas):
        super(LabelsTitleEditDialog, self).__init__(parent)

        self.canvas = canvas

        self.setWindowTitle('Edit graph title')
        self.setModal(True)

        self.layout = QtGui.QGridLayout(self)

        row = 0

        self.titleEdit = QtGui.QLineEdit(canvas.title())
        self.titleFontsizeEdit = QtGui.QLineEdit(str(canvas.title_fontsize()))
        self.titleFontsizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
        self.layout.addWidget(QtGui.QLabel("Title"), row, 0)
        self.layout.addWidget(self.titleEdit, row, 1)
        self.layout.addWidget(self.titleFontsizeEdit, row, 2)
        row += 1

        self.xLabelEdit = QtGui.QLineEdit(canvas.x_label())
        self.xLabelFontsizeEdit = QtGui.QLineEdit(
            str(canvas.x_label_fontsize()))
        self.xLabelFontsizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
        self.layout.addWidget(QtGui.QLabel("X label"), row, 0)
        self.layout.addWidget(self.xLabelEdit, row, 1)
        self.layout.addWidget(self.xLabelFontsizeEdit, row, 2)
        row += 1

        self.y1LabelEdit = QtGui.QLineEdit(canvas.y1_label())
        self.y1LabelFontsizeEdit = QtGui.QLineEdit(
            str(canvas.y1_label_fontsize()))
        self.y1LabelFontsizeEdit.setValidator(
            QtGui.QDoubleValidator(1, 1e6, 1))
        self.layout.addWidget(QtGui.QLabel("Y1 label"), row, 0)
        self.layout.addWidget(self.y1LabelEdit, row, 1)
        self.layout.addWidget(self.y1LabelFontsizeEdit, row, 2)
        row += 1

        self.y2LabelEdit = QtGui.QLineEdit(canvas.y2_label())
        self.y2LabelFontsizeEdit = QtGui.QLineEdit(
            str(canvas.y2_label_fontsize()))
        self.y2LabelFontsizeEdit.setValidator(
            QtGui.QDoubleValidator(1, 1e6, 1))
        self.layout.addWidget(QtGui.QLabel("Y2 label"), row, 0)
        self.layout.addWidget(self.y2LabelEdit, row, 1)
        self.layout.addWidget(self.y2LabelFontsizeEdit, row, 2)
        row += 1

        self.extraLayout = QtGui.QGridLayout()
        extraRow = 0

        self.extraLayout.addWidget(QtGui.QLabel("Tick size"), extraRow, 0)
        self.extraLayout.addWidget(QtGui.QLabel("Label padding"), extraRow, 1)
        self.extraLayout.addWidget(QtGui.QLabel("Top offset"), extraRow, 2)
        self.extraLayout.addWidget(QtGui.QLabel("Bottom offset"), extraRow, 3)
        extraRow += 1

        self.tickSizeEdit = QtGui.QLineEdit(str(canvas.tick_fontsize()))
        self.tickSizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
        self.labelPaddingEdit = QtGui.QLineEdit(str(canvas.labelpad()))
        self.labelPaddingEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
        self.topOffsetEdit = QtGui.QLineEdit(str(canvas.top_offset()))
        self.topOffsetEdit.setValidator(QtGui.QDoubleValidator(0, 1, 3))
        self.bottomOffsetEdit = QtGui.QLineEdit(str(canvas.bottom_offset()))
        self.bottomOffsetEdit.setValidator(QtGui.QDoubleValidator(0, 1, 3))
        self.extraLayout.addWidget(self.tickSizeEdit, extraRow, 0)
        self.extraLayout.addWidget(self.labelPaddingEdit, extraRow, 1)
        self.extraLayout.addWidget(self.topOffsetEdit, extraRow, 2)
        self.extraLayout.addWidget(self.bottomOffsetEdit, extraRow, 3)
        extraRow += 1

        self.extraLayout.addWidget(
            QtGui.QLabel("Legend size"), extraRow, 0)
        self.extraLayout.addWidget(
            QtGui.QLabel("Legend Y1 columns"), extraRow, 1, 1, 2)
        self.extraLayout.addWidget(
            QtGui.QLabel("Legend Y2 columns"), extraRow, 3, 1, 2)
        extraRow += 1

        self.legendSizeEdit = QtGui.QLineEdit(str(canvas.legend_fontsize()))
        self.legendSizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
        self.y1LegendNColEdit = QtGui.QLineEdit(str(canvas.y1_legend_ncol()))
        self.y1LegendNColEdit.setValidator(QtGui.QIntValidator(1, 100))
        self.y2LegendNColEdit = QtGui.QLineEdit(str(canvas.y2_legend_ncol()))
        self.y2LegendNColEdit.setValidator(QtGui.QIntValidator(1, 100))
        self.extraLayout.addWidget(self.legendSizeEdit, extraRow, 0)
        self.extraLayout.addWidget(self.y1LegendNColEdit, extraRow, 1, 1, 2)
        self.extraLayout.addWidget(self.y2LegendNColEdit, extraRow, 3, 1, 2)
        extraRow += 1

        self.layout.addLayout(self.extraLayout, row, 0, extraRow, 3)
        row += extraRow

        hlayout = QtGui.QHBoxLayout()
        Ok = QtGui.QPushButton("Ok")
        Ok.clicked.connect(self.accept)
        hlayout.addWidget(Ok)
        Cancel = QtGui.QPushButton("Cancel")
        Cancel.clicked.connect(self.reject)
        hlayout.addWidget(Cancel)
        Apply = QtGui.QPushButton("Apply")
        Apply.clicked.connect(self.apply)
        hlayout.addWidget(Apply)
        self.layout.addLayout(hlayout, row, 0, 1, 3)

    def apply(self):
        self.canvas.title(self.titleEdit.text())
        self.canvas.title_fontsize(float(self.titleFontsizeEdit.text()))
        self.canvas.x_label(self.xLabelEdit.text())
        self.canvas.x_label_fontsize(self.xLabelFontsizeEdit.text())
        self.canvas.y1_label(self.y1LabelEdit.text())
        self.canvas.y1_label_fontsize(self.y1LabelFontsizeEdit.text())
        self.canvas.y2_label(self.y2LabelEdit.text())
        self.canvas.y2_label_fontsize(self.y2LabelFontsizeEdit.text())
        self.canvas.tick_fontsize(float(self.tickSizeEdit.text()))
        self.canvas.legend_fontsize(float(self.legendSizeEdit.text()))
        self.canvas.labelpad(float(self.labelPaddingEdit.text()))
        self.canvas.top_offset(float(self.topOffsetEdit.text()))
        self.canvas.bottom_offset(float(self.bottomOffsetEdit.text()))
        self.canvas.y1_legend_ncol(int(self.y1LegendNColEdit.text()))
        self.canvas.y2_legend_ncol(int(self.y2LegendNColEdit.text()))
        self.canvas.draw()

    def accept(self):
        super(LabelsTitleEditDialog, self).accept()
        self.apply()


class SignalLoggerUI(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(SignalLoggerUI, self).__init__(parent)
        self.__init__ui = ui.MainWindow()
        self.ui = ui.MainWindow()

        self.ui.setupUi(self)

        self.tab_re = re.compile('^Plot [0-9]+$')

        self.data = {}

        self.gridStyles = {
            'left': LineStyle(),
            'right': LineStyle(linestyle=':')}
        self.gridStyleFile = \
            os.path.expanduser("~") + "/.config/signal_logger/grid_style.json"
        if os.path.exists(self.gridStyleFile):
            with open(self.gridStyleFile) as f:
                data = json.load(f)
                for k in self.gridStyles.keys():
                    if k in data:
                        self.gridStyles[k] = LineStyle(**data[k])
        UserPlot.__new__.__defaults__ = (
            self.gridStyles['left'], self.gridStyles['right'], {}, {},
            GraphLabels(), {})

        self.robotFile = os.path.expanduser("~") + "/.config/signal_logger/robot"
        self.userPlotFile = os.path.expanduser("~") + "/.config/signal_logger/custom_plots.json"
        self.reload_user_plots()

        self.activeRobotAction = None
        self.styleMenu = QtGui.QMenu("Style", self.ui.menubar)

        # Line style menu
        self.lineStyleMenu = QtGui.QMenu("Lines", self.styleMenu)

        def fillLineStyleMenu(self):
            self.lineStyleMenu.clear()
            canvas = self.getCanvas()

            def makePlotMenu(self, name, plots, style_fn):
                if len(plots) < 1:
                    return
                menu = QtGui.QMenu(name, self.lineStyleMenu)
                group = QtGui.QActionGroup(menu)
                action = QtGui.QAction("All", group)
                action.triggered.connect(
                    lambda: AllLineStyleDialog(
                        self, name, self.getCanvas(), plots, style_fn).exec_())
                group.addAction(action)
                sep = QtGui.QAction(group)
                sep.setSeparator(True)
                group.addAction(sep)
                for y in plots:
                    style = style_fn(y)
                    action = QtGui.QAction(style.label, group)
                    action.triggered.connect(
                        lambda yin=y, stylein=style:
                        LineStyleDialog(
                            self, yin, self.getCanvas(), stylein,
                            style_fn).exec_())
                    group.addAction(action)
                menu.addActions(group.actions())
                self.lineStyleMenu.addMenu(menu)
            makePlotMenu(
                self, "Left", canvas.axes_plots.keys(), canvas.style_left)
            makePlotMenu(
                self, "Right", canvas.axes2_plots.keys(), canvas.style_right)

        self.lineStyleMenu.aboutToShow.connect(lambda: fillLineStyleMenu(self))
        self.styleMenu.addMenu(self.lineStyleMenu)

        # Grid style menu
        self.gridStyleMenu = QtGui.QMenu("Grids", self.styleMenu)
        self.gridDisplayActionGroup = QtGui.QActionGroup(self.gridStyleMenu)
        self.gridDisplayActionGroup.setExclusive(True)
        self.leftGridAction = QtGui.QAction(
            "Left", self.gridDisplayActionGroup)
        self.leftGridAction.triggered.connect(
            lambda: GridStyleDialog(
                self, "left", self.getCanvas(), self.getCanvas().grid).exec_())
        self.gridDisplayActionGroup.addAction(self.leftGridAction)
        self.rightGridAction = QtGui.QAction(
            "Right", self.gridDisplayActionGroup)
        self.rightGridAction.triggered.connect(
            lambda: GridStyleDialog(
                self, "right", self.getCanvas(),
                self.getCanvas().grid2).exec_())
        self.gridDisplayActionGroup.addAction(self.rightGridAction)
        self.gridStyleMenu.addActions(self.gridDisplayActionGroup.actions())
        self.styleMenu.addMenu(self.gridStyleMenu)

        # Labels
        self.titleAction = QtGui.QAction(
            "Title, labels and fonts", self.styleMenu)
        self.titleAction.triggered.connect(
            lambda: LabelsTitleEditDialog(self, self.getCanvas()).exec_())
        self.styleMenu.addAction(self.titleAction)

        self.ui.menubar.addMenu(self.styleMenu)

        self.addApplicationShortcut(
            QtCore.Qt.CTRL + QtCore.Qt.Key_O, self.shortcutOpenFile)
        self.addApplicationShortcut(
            QtCore.Qt.CTRL + QtCore.Qt.Key_W, self.shortcutCloseTab)
        self.addApplicationShortcut(
            QtCore.Qt.CTRL + QtCore.Qt.Key_PageDown, self.shortcutNextTab)
        self.addApplicationShortcut(
            QtCore.Qt.CTRL + QtCore.Qt.Key_PageUp, self.shortcutPreviousTab)
        self.addApplicationShortcut(
            QtCore.Qt.CTRL + QtCore.Qt.Key_T, self.shortcutNewTab)
        self.addApplicationShortcut(
            QtCore.Qt.CTRL + QtCore.Qt.Key_S, self.save_userplot)
        self.addApplicationShortcut(
            QtCore.Qt.CTRL + QtCore.Qt.Key_A, self.shortcutAxesDialog)

    def reload_user_plots(self):
        self.userPlotList = load_user_plots(self.userPlotFile)
        self.update_userplot_menu()

    def saveUserPlots(self):
        confDir = os.path.dirname(self.userPlotFile)
        if not os.path.exists(confDir):
            os.makedirs(confDir)
        with open(self.userPlotFile, 'w') as f:
            json.dump(
                self.userPlotList, f, default=lambda o: o.__dict__, indent=2,
                separators=(',', ': '))
        self.update_userplot_menu()

    def saveDefaultRobot(self, name):
        confDir = os.path.dirname(self.robotFile)
        if not os.path.exists(confDir):
            os.makedirs(confDir)
        with open(self.robotFile, 'w') as f:
            f.write("{}".format(name))

    def getDefaultRobot(self):
        if os.path.exists(self.robotFile):
            return open(self.robotFile).read().strip()
        else:
            return u""

    def addApplicationShortcut(self, key, callback):
        shortcut = QtGui.QShortcut(self)
        shortcut.setKey(key)
        shortcut.setContext(QtCore.Qt.ShortcutContext.ApplicationShortcut)
        shortcut.activated.connect(lambda: callback())

    def update_userplot_menu(self):
        self.ui.menuUserPlots.clear()
        for p in self.userPlotList:
            act = QtGui.QAction(p.title, self.ui.menuUserPlots)
            act.triggered.connect(lambda plot=p: self.plot_userplot(plot))
            self.ui.menuUserPlots.addAction(act)
        self.ui.menuUserPlots.addSeparator()
        act = QtGui.QAction("Reload custom plots", self.ui.menuUserPlots)
        act.triggered.connect(self.reload_user_plots)
        self.ui.menuUserPlots.addAction(act)
        if len(self.userPlotList) > 0:
            rmUserPlotMenu = QtGui.QMenu(
                "Remove plot", self.ui.menuUserPlots)
            for p in self.userPlotList:
                act = QtGui.QAction(p.title, self.ui.menuUserPlots)
                act.triggered.connect(
                    lambda plot=p: self.remove_userplot(plot))
                rmUserPlotMenu.addAction(act)
            self.ui.menuUserPlots.addMenu(rmUserPlotMenu)
        act = QtGui.QAction("Save current plot", self.ui.menuUserPlots)
        act.triggered.connect(self.save_userplot)
        self.ui.menuUserPlots.addAction(act)

    def save_userplot(self):
        tab = self.ui.tabWidget.currentWidget()
        canvas = tab.ui.canvas
        valid = len(canvas.axes_plots) != 0 or len(canvas.axes2_plots) != 0
        if not valid:
            err_diag = QtGui.QMessageBox(self)
            err_diag.setModal(True)
            err_diag.setText("Cannot save custom plot if nothing is shown")
            err_diag.exec_()
            return
        defaultTitle = self.ui.tabWidget.tabText(
            self.ui.tabWidget.currentIndex())
        if defaultTitle.startswith("Plot"):
            defaultTitle = ""
        title, ok = QtGui.QInputDialog.getText(
            self, "Custom plot", "Title:", text=defaultTitle)
        if ok:
            y1 = filter(
                lambda k: k in self.data.keys(),
                canvas.axes_plots.keys())
            y2 = filter(
                lambda k: k in self.data.keys(),
                canvas.axes2_plots.keys())
            y1d = map(
                lambda sp: "{}_{}".format(sp.name, sp.id),
                filter(lambda sp: sp.idx == 0, tab.specials.values()))
            y2d = map(
                lambda sp: "{}_{}".format(sp.name, sp.id),
                filter(lambda sp: sp.idx == 1, tab.specials.values()))
            style = {
                y: canvas.style_left(y)
                for y in canvas.axes_plots.keys()}
            style2 = {
                y: canvas.style_right(y)
                for y in canvas.axes2_plots.keys()}
            found = False
            extra = {
                p: getattr(self.getCanvas(), p)()
                for p in [
                    "tick_fontsize", "legend_fontsize", "labelpad",
                    "top_offset", "bottom_offset", "y1_legend_ncol",
                    "y2_legend_ncol"]}
            up = UserPlot(
                title, tab.x_data, y1, y1d, y2, y2d, self.getCanvas().grid,
                self.getCanvas().grid2, style, style2, GraphLabels(
                    title=TextWithFontSize(
                        canvas.title(), canvas.title_fontsize()),
                    x_label=TextWithFontSize(
                        canvas.x_label(), canvas.x_label_fontsize()),
                    y1_label=TextWithFontSize(
                        canvas.y1_label(), canvas.y1_label_fontsize()),
                    y2_label=TextWithFontSize(
                        canvas.y2_label(), canvas.y2_label_fontsize())),
                extra)
            for i in range(len(self.userPlotList)):
                if self.userPlotList[i].title == title:
                    self.userPlotList[i] = up
                    found = True
                    break
            if not found:
                self.userPlotList.append(up)
            self.saveUserPlots()

    def plot_userplot(self, p):
        valid = p.x in self.data.keys() and all([
            y in self.data.keys() for x in [p.y1, p.y2] for y in x])
        if not valid:
            missing_entries = ""
            if p.x not in self.data.keys():
                missing_entries += "- {}\n".format(p.x)
            for x in [p.y1, p.y1d, p.y2, p.y2d]:
                for y in x:
                    if y not in self.data.keys():
                        missing_entries += "- {}\n".format(y)
            missing_entries = missing_entries[:-1]
            err_diag = QtGui.QMessageBox(self)
            err_diag.setModal(True)
            err_diag.setText(
                "Plot {} is not valid for this log file, some data is "
                "missing\nMissing entries:\n{}".format(
                    p.title, missing_entries))
            err_diag.exec_()
            return
        plotW = SignalLoggerTab.UserPlot(self, p)
        self.ui.tabWidget.insertTab(
            self.ui.tabWidget.count() - 1, plotW, p.title)
        self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
        self.updateClosable()

    def remove_userplot(self, p_in):
        for p in self.userPlotList:
            if p.title == p_in.title:
                self.userPlotList.remove(p)
                break
        self.saveUserPlots()

    def setRobot(self, action):
        action.setChecked(False)
        self.activeRobotAction.setChecked(True)

    def getCanvas(self):
        return self.ui.tabWidget.currentWidget().ui.canvas

    @QtCore.Slot()
    def on_actionLoad_triggered(self):
        fpath = QtGui.QFileDialog.getOpenFileName(self, "Log file")[0]
        if len(fpath):
            self.load_log(fpath)

    @QtCore.Slot()
    def on_actionExit_triggered(self):
        QtGui.QApplication.quit()

    @QtCore.Slot(int)
    def on_tabWidget_currentChanged(self, idx):
        if idx == self.ui.tabWidget.count() - 1:
            plotW = SignalLoggerTab(self)
            plotW.setData(self.data)
            plotW.setGridStyles(self.gridStyles)
            j = 1
            for i in range(self.ui.tabWidget.count() - 1):
                if self.tab_re.match(self.ui.tabWidget.tabText(i)):
                    j += 1
            self.ui.tabWidget.insertTab(
                self.ui.tabWidget.count() - 1, plotW, "Plot {}".format(j))
            self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
            self.updateClosable()

    @QtCore.Slot(int)
    def on_tabWidget_tabCloseRequested(self, idx):
        if self.ui.tabWidget.currentIndex() == idx:
            self.ui.tabWidget.setCurrentIndex(abs(idx - 1))
        self.ui.tabWidget.removeTab(idx)
        j = 1
        for i in range(self.ui.tabWidget.count() - 1):
            if self.tab_re.match(self.ui.tabWidget.tabText(i)):
                self.ui.tabWidget.setTabText(i, "Plot {}".format(j))
                j += 1
        self.updateClosable()

    def updateClosable(self):
        has_closable = self.ui.tabWidget.count() > 2
        self.ui.tabWidget.setTabsClosable(has_closable)
        if has_closable:
            self.ui.tabWidget.tabBar().tabButton(
                self.ui.tabWidget.count() - 1, QtGui.QTabBar.RightSide).hide()

    def shortcutOpenFile(self):
        self.ui.actionLoad.triggered.emit()

    def shortcutCloseTab(self):
        if self.ui.tabWidget.tabsClosable():
            self.ui.tabWidget.tabCloseRequested.emit(
                self.ui.tabWidget.currentIndex())

    def shortcutPreviousTab(self):
        if self.ui.tabWidget.currentIndex() > 0:
            self.ui.tabWidget.setCurrentIndex(
                self.ui.tabWidget.currentIndex() - 1)

    def shortcutNextTab(self):
        if self.ui.tabWidget.currentIndex() < self.ui.tabWidget.count() - 2:
            self.ui.tabWidget.setCurrentIndex(
                self.ui.tabWidget.currentIndex() + 1)

    def shortcutNewTab(self):
        self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 1)

    def shortcutAxesDialog(self):
        self.ui.tabWidget.currentWidget().ui.canvas.axesDialog()

    def load_log(self, fpath, logdir=None):
        fpath = find_log(fpath, logdir=logdir)
        self.silo = Silo(fpath, logdir, print_log_file_path=True)
        self.data = self.silo.data
        self.ui.consoleWidget.start(self.silo, self.ui)
        for i in range(self.ui.tabWidget.count() - 1):
            tab = self.ui.tabWidget.widget(i)
            assert(isinstance(tab, SignalLoggerTab))
            tab.setData(self.data)
            tab.setGridStyles(self.gridStyles)
        self.setWindowTitle("Signal Logger - {}".format(
            os.path.basename(fpath)))
