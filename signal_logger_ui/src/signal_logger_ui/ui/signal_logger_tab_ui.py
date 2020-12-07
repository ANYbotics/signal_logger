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


class Ui_SignalLoggerTab(object):
    def setupUi(self, SignalLoggerTab):
        SignalLoggerTab.setObjectName("SignalLoggerTab")
        SignalLoggerTab.resize(803, 538)
        self.horizontalLayout = QtWidgets.QHBoxLayout(SignalLoggerTab)
        self.horizontalSplitter = QtWidgets.QSplitter(SignalLoggerTab)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.y1SelectorLayout = QtWidgets.QVBoxLayout()
        self.y1SelectorLayout.setObjectName("y1SelectorLayout")
        self.y1Selector = QtWidgets.QTreeWidget(SignalLoggerTab)
        self.y1Selector.setSelectionMode(
            QtWidgets.QAbstractItemView.MultiSelection)
        self.y1Selector.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectItems)
        self.y1Selector.setColumnCount(1)
        self.y1Selector.setObjectName("y1Selector")
        self.y1Selector.headerItem().setText(0, "1")
        self.y1Selector.header().setVisible(True)
        self.y1SelectorLayout.addWidget(self.y1Selector)
        self.y1SelectorLayoutWrapper = QtWidgets.QWidget()
        self.y1SelectorLayoutWrapper.setLayout(self.y1SelectorLayout)
        self.horizontalSplitter.addWidget(self.y1SelectorLayoutWrapper)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.canvas = PlotCanvasWithToolbar(SignalLoggerTab)
        self.canvas.setObjectName("canvas")
        self.verticalLayout.addWidget(self.canvas)
        self.selectorLayout = QtWidgets.QHBoxLayout()
        self.selectorLayout.setObjectName("selectorLayout")
        self.xSelector = QtWidgets.QComboBox(SignalLoggerTab)
        sizePolicy = QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.xSelector.sizePolicy().hasHeightForWidth())
        self.xSelector.setSizePolicy(sizePolicy)
        self.xSelector.setObjectName("xSelector")
        self.selectorLayout.addWidget(self.xSelector)
        self.verticalLayout.addLayout(self.selectorLayout)
        self.verticalLayoutWrapper = QtWidgets.QWidget()
        self.verticalLayoutWrapper.setLayout(self.verticalLayout)
        self.horizontalSplitter.addWidget(self.verticalLayoutWrapper)
        self.y2SelectorLayout = QtWidgets.QVBoxLayout()
        self.y2SelectorLayout.setObjectName("y2SelectorLayout")
        self.y2Selector = QtWidgets.QTreeWidget(SignalLoggerTab)
        self.y2Selector.setSelectionMode(
            QtWidgets.QAbstractItemView.MultiSelection)
        self.y2Selector.setColumnCount(1)
        self.y2Selector.setObjectName("y2Selector")
        self.y2Selector.headerItem().setText(0, "1")
        self.y2Selector.header().setVisible(True)
        self.y2SelectorLayout.addWidget(self.y2Selector)
        self.y2SelectorLayoutWrapper = QtWidgets.QWidget()
        self.y2SelectorLayoutWrapper.setLayout(self.y2SelectorLayout)
        self.horizontalSplitter.addWidget(self.y2SelectorLayoutWrapper)
        self.horizontalSplitter.setStretchFactor(0, 0.8)
        self.horizontalSplitter.setStretchFactor(1, 1.0)
        self.horizontalSplitter.setStretchFactor(2, 0.8)
        self.horizontalLayout.addWidget(self.horizontalSplitter)

        self.retranslateUi(SignalLoggerTab)
        QtCore.QMetaObject.connectSlotsByName(SignalLoggerTab)

    def retranslateUi(self, SignalLoggerTab):
        SignalLoggerTab.setWindowTitle(
            QtWidgets.QApplication.translate(
                "SignalLoggerTab", "SignalLoggerTab", None))


try:
    from signal_logger_ui.signal_logger_plotcanvas import PlotCanvasWithToolbar
except Exception:
    raise
