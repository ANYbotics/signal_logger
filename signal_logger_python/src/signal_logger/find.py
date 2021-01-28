#!/usr/bin/env python3
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

def pick_last_modified(files):
    """
    Pick last modified file in a list.

    :param files: List of files.
    :return: Last modified among them.
    """
    last_modified = max(files, key=os.path.getmtime)
    if len(files) > 1:
        warnings.warn(
            "Several candidate log files: {}".format(files) + "\n"
            "Picking last modified: '{}'".format(last_modified))
    return last_modified


def find_log(fpath, logdir=None):
    """
    Find log file.

    :param: fpath: Initial file name, or path, or identifier.
    :param: logdir: Directory where log files are looked up (default: ~/.ros).
    :return: Log file path.
    """
    if os.path.isfile(fpath):
        return fpath
    if os.path.isdir(fpath):
        logdir = fpath
        fpath = ''

    # Look for log file in logdir or else in ~/.ros
    if logdir is None:
        logdir = os.path.expanduser("~") + "/.ros/"
    if logdir[-1] != '/':
        logdir = logdir + "/"
    if os.path.isfile(logdir + fpath):
        return logdir + fpath

    # Try fpath as a prefix
    if len(fpath) > 0:
        prefix_files = glob.glob(logdir + fpath + '*[0-9][0-9][0-9].silo')
        if prefix_files:
            return pick_last_modified(prefix_files)

    # Try fpath as a suffix
    suffix = fpath + ".silo"
    suffix_files = [logdir + f for f in os.listdir(logdir) if f.endswith(suffix)]
    if suffix_files:
        return pick_last_modified(suffix_files)

    # Well, we tried all we could ;)
    warnings.warn("Found no log file for '{}'".format(fpath))
    return fpath