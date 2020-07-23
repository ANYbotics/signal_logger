#!/usr/bin/env python
#
# Author:       Stephane Caron
# Affiliation:  ANYbotics
# Date:         17.01.2020

import glob
import os
import warnings


def custom_formatwarning(msg, *args, **kwargs):
    return 'Warning: ' + str(msg) + '\n'


warnings.formatwarning = custom_formatwarning


def find_log(fpath, logdir=None):
    """
    Find log file.

    :param: fpath: Initial file name, or path, or identifier.
    :param: logdir: Directory in which log files are looked up (default: ~/.ros).
    :return: Log file path.
    """
    if os.path.isfile(fpath):
        return fpath

    # Look for log file in ~/.ros
    if logdir is None:
        logdir = os.path.expanduser("~") + "/.ros/"
    if logdir[-1] != '/':
        logdir = logdir + "/"
    if os.path.isfile(logdir + fpath):
        return logdir + fpath

    # If argument is a prefix, return most recent log
    prefix_files = glob.glob(logdir + fpath + '*[0-9][0-9][0-9].silo')
    if prefix_files:
        return max(prefix_files, key=os.path.getctime)

    # If argument is a suffix, warn and ask the user
    candidates = [logdir + f for f in os.listdir(logdir) if f.endswith(fpath)]
    if not candidates:
        warnings.warn("Found no log file for '{}'".format(fpath))
        return fpath
    if len(candidates) > 1:
        warnings.warn(
            "Several log file matches in {}".format(candidates) + "\n"
            "Picking '{}'".format(fpath))
    return candidates[0]
