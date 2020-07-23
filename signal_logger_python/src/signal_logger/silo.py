#!/usr/bin/env python
#
# Author:       Stephane Caron
# Affiliation:  ANYbotics
# Date:         07.04.2020

from numpy import array

from .cartesian_signal import CartesianSignal
from .find import find_log
from .read import LogReader
from .rotation_signals import EulerAnglesSignal
from .rotation_signals import QuaternionSignal
from .signal import Signal


class Silo(object):

    """
    Set of signal (time series) data.
    """

    def __init__(self, fpath, logdir=None, print_log_file_path=False):
        """
        Read silo from file.

        :param fpath: Path to file, or hint to it.
        :param logdir: Directory in which to look for silos (default: ~/.ros)
        :param print_log_file_path: If true, print full log file path.
        """
        full_path = find_log(fpath, logdir)
        if print_log_file_path:
            print("Opening log file '{}'".format(full_path))
        self.data = LogReader().read_from_file(full_path).get_data()
        self.path = full_path
        self.times = array(self.data['t'])

    def get_signal(self, full_name):
        """
        Get signal from the silo.

        :param name: Signal name.
        :return: signal
        """
        if full_name[0] == '/':
            full_name = full_name[1:]
        name = full_name.split('/')[-1]
        if full_name in self.data:
            values = self.data[full_name]
            return Signal(name, self.times, values)
        matches = {key for key in self.data.keys() if key.startswith(full_name)}
        suffixes = {key[len(full_name):] for key in matches}
        values = {suffix[1:]: self.data[full_name + suffix] for suffix in suffixes}
        if suffixes == {'/x', '/y', '/z'}:
            if "EulerAngles" in name:
                return EulerAnglesSignal(name, self.times, values['x'], values['y'], values['z'])
            else:  # not Euler angles
                return CartesianSignal(name, self.times, values['x'], values['y'], values['z'])
        elif suffixes == {'/w', '/x', '/y', '/z'}:
            return QuaternionSignal(name, self.times, values['w'], values['x'], values['y'], values['z'])
        raise ValueError("Unrecognized signal type with fields {}".format({suffix[1:] for suffix in suffixes}))
