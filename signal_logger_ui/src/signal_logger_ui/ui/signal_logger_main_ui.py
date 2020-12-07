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

from PySide2 import QtCore, QtWidgets


class Ui_MainWindow(object):

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1024, 768)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName("gridLayout")
        self.tabWidget = SignalLoggerTabWidget(self.centralwidget)
        self.tabWidget.setObjectName("tabWidget")
        self.tab = SignalLoggerTab()
        self.tab.setObjectName("tab")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        self.consoleWidget = ConsoleWidget(self.centralwidget)
        self.verticalSplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.verticalSplitter.addWidget(self.tabWidget)
        self.verticalSplitter.addWidget(self.consoleWidget)
        self.verticalSplitter.setStretchFactor(0, 0.5)
        self.verticalSplitter.setStretchFactor(1, 1.0)
        self.verticalSplitter.setSizes([2, 0])  # hide console by default
        self.gridLayout.addWidget(self.verticalSplitter, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 40))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuUserPlots = QtWidgets.QMenu(self.menubar)
        self.menuUserPlots.setObjectName("menuUserPlots")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionLoad = QtWidgets.QAction(MainWindow)
        self.actionLoad.setObjectName("actionLoad")
        self.actionExit = QtWidgets.QAction(MainWindow)
        self.actionExit.setObjectName("actionExit")
        self.menuFile.addAction(self.actionLoad)
        self.menuFile.addAction(self.actionExit)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuUserPlots.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(
            QtWidgets.QApplication.translate(
                "MainWindow", "Signal Logger Plots", None))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.tab),
            QtWidgets.QApplication.translate(
                "MainWindow", "Plot 1", None))
        self.tabWidget.setTabText(
            self.tabWidget.indexOf(self.tab_2),
            QtWidgets.QApplication.translate(
                "MainWindow", "+", None))
        self.menuFile.setTitle(
            QtWidgets.QApplication.translate(
                "MainWindow", "File", None))
        self.menuUserPlots.setTitle(
            QtWidgets.QApplication.translate(
                "MainWindow", "Custom plots", None))
        self.actionLoad.setText(
            QtWidgets.QApplication.translate(
                "MainWindow", "Load...", None))
        self.actionExit.setText(
            QtWidgets.QApplication.translate(
                "MainWindow", "Exit", None))


try:
    from signal_logger_ui.console_widget import ConsoleWidget
    from signal_logger_ui.signal_logger_tab import SignalLoggerTab
    from signal_logger_ui.signal_logger_tab_widget import SignalLoggerTabWidget
except Exception:
    raise
