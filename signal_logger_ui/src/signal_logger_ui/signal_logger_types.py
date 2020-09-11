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


class LineStyle(object):
    def __init__(self, color='black', linestyle='--', linewidth=0.5,
                 visible=False, label=""):
        self.color = color
        self.linestyle = linestyle
        self.linewidth = linewidth
        self.visible = visible
        self.label = label

    def __repr__(self):
        fmt = "color: {}, linestyle: {}, linewidth: {}, visible: {}, label: {}"
        return fmt.format(
            self.color, self.linestyle, self.linewidth, self.visible,
            self.label)


class TextWithFontSize(object):
    def __init__(self, text="", fontsize=10):
        self.text = text
        self.fontsize = fontsize


class GraphLabels(object):
    def __init__(self, title=TextWithFontSize(fontsize=12),
                 x_label=TextWithFontSize(), y1_label=TextWithFontSize(),
                 y2_label=TextWithFontSize()):
        self.title = title
        self.x_label = x_label
        self.y1_label = y1_label
        self.y2_label = y2_label
