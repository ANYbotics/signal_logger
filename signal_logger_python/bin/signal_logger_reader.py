#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Adapted from mc_log_ui.py (license: BSD)
# Source: https://github.com/jrl-umi3218/mc_rtc/tree/master/utils/mc_log_gui

import signal_logger
import sys


if __name__ == '__main__':
    silo = None
    if len(sys.argv) > 1:
        silo = signal_logger.Silo(sys.argv[1])
        if len(sys.argv) > 2:
            signal_logger.warn("Other arguments '{}' have been ignored".format(
                ' '.join(sys.argv[2:])))
        print("Read silo with {} keys.\n".format(len(silo.data)))
    else:
        print("Usage: {} path_to_file.silo")
