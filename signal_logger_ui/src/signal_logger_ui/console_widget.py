#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
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

import numpy

from PySide2 import QtWidgets
from qtconsole.inprocess import QtInProcessKernelManager
from qtconsole.rich_jupyter_widget import RichJupyterWidget


class ConsoleWidget(RichJupyterWidget):

    def __init__(self, *args, **kwargs):
        super(ConsoleWidget, self).__init__(*args, **kwargs)
        self.kernel_client = None
        self.kernel_manager = None

    def start(self, silo, ui):
        self.start_kernel_manager(silo, ui)
        self.start_kernel_client()
        self.exit_requested.connect(QtWidgets.QApplication.quit)
        self.clear()

    def start_kernel_manager(self, silo, ui):
        kernel_manager = QtInProcessKernelManager()
        kernel_manager.start_kernel(show_banner=False)
        kernel_manager.kernel.gui = 'qt'

        def EulerAngles(name, roll, pitch, yaw, *args, **kwargs):
            from signal_logger.rotation_signals import EulerAnglesZyxSignal
            return EulerAnglesZyxSignal.from_constant(
                name, silo.times, roll, pitch, yaw, *args, **kwargs)

        def clf():
            import pylab
            pylab.clf()
            ui.tab.ui.canvas.canvas.draw()

        def plot(signal, *args, **kwargs):
            # TODO(scaron): currently plots to the right axis, give choice
            signal.plot(*args, **kwargs)
            ui.tab.ui.canvas.canvas.draw()

        kernel_manager.kernel.shell.push({
            'EulerAngles': EulerAngles,
            'clf': clf,
            'plot': plot,
            'pi': numpy.pi,
            'silo': silo,
            'ui': ui,
        })
        self.kernel_manager = kernel_manager

    def start_kernel_client(self):
        kernel_client = self._kernel_manager.client()
        kernel_client.start_channels()
        self.kernel_client = kernel_client

    def clear(self):
        """Clear the IPython console."""
        self._control.clear()

    def execute_command(self, command):
        """Execute a command in the frame of the console widget."""
        self._execute(command, False)

    def print_text(self, text):
        """Print some plain text to the IPython console."""
        self._append_plain_text(text)
