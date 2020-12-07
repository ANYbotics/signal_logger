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

from numpy.random import random
from signal_logger_ui import PlotFigure


def test_plot_deriv(data_len=10):
    """
    Test "deriv" plots implemented in PlotFigure.

    :param data_len: Size of synthetic dataset to test on.
    """
    figure = PlotFigure()
    t = [0.]
    y = random(data_len)
    dt = random(data_len)
    for i in range(1, data_len):
        t.append(t[-1] + dt[i])

    def hijack_plot(axes, legend, times, deriv_vals, y_label):
        """Hijack call to _plot to receive results from the tested function."""
        assert len(times) == len(deriv_vals) == data_len - 1
        for i in range(data_len - 1):
            assert abs(deriv_vals[i] - (y[i + 1] - y[i]) / (t[i + 1] - t[i])) < 1e-10

    figure.data = { 't': t, 'y': y}
    figure._plot = hijack_plot
    figure._add_deriv_plot(None, None, 't', 'y', None)


if __name__ == "__main__":
    test_plot_deriv()
