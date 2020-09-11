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


class SignalLoggerTabBar(QtWidgets.QTabBar):

    def __init__(self, parent=None):
        super(SignalLoggerTabBar, self).__init__(parent)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MiddleButton \
                and self.parent().tabsClosable():
            pos = event.pos()
            for i in range(self.count() - 1):
                if self.tabRect(i).contains(pos):
                    self.parent().tabCloseRequested.emit(i)
                    return
        super(SignalLoggerTabBar, self).mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            name = QtWidgets.QInputDialog.getText(
                self, "New name for current graph", "New name:",
                text=self.tabText(self.currentIndex()))
            if name[1]:
                self.setTabText(self.currentIndex(), name[0])
        super(SignalLoggerTabBar, self).mouseDoubleClickEvent(event)


class SignalLoggerTabWidget(QtWidgets.QTabWidget):

    def __init__(self, parent=None):
        super(SignalLoggerTabWidget, self).__init__(parent)
        self.setTabBar(SignalLoggerTabBar(self))
