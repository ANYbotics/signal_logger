#!/usr/bin/env python3
#
# Author:       Stephane Caron
# Affiliation:  ANYbotics
# Date:         16.01.2020

import struct

from os.path import commonprefix
from warnings import warn

from .config import TOPIC_SEPARATOR


common_suffixes = []
common_suffixes.extend(['_angular_{}'.format(coord) for coord in ['x', 'y', 'z']])
common_suffixes.extend(['_linear_{}'.format(coord) for coord in ['x', 'y', 'z']])
common_suffixes.extend(['_{}'.format(coord) for coord in ['w', 'x', 'y', 'z']])


class Descriptor(object):

    """
    Signal descriptor from the log file header.

    See <https://docs.leggedrobotics.com/signal_logger_doc/page_log_file.html>.
    """

    def __init__(self, line):
        split = line.split(' ')
        self.name = self.rename_from_ros(ros_name=split[0])
        self.size = int(split[1])
        self.nb_points = int(split[2])
        self.divider = int(split[3])
        self.looping = (split[4] == '1')
        self.data_type = split[5].strip()

    def __repr__(self):
        return "{} ({} points of type {})".format(
            self.name, self.nb_points, self.data_type)

    def get_format(self):
        """
        Convert format from descriptor data type to C binary format.

        :return: C binary format.
        """
        if self.data_type == "double":
            return 'd'
        elif self.data_type == "int8":
            return 'b'
        elif self.data_type == "int16":
            return 'h'
        elif self.data_type == "int32":
            return 'i'
        elif self.data_type == "int64":
            return 'q'
        elif self.data_type == "uint8":
            return 'B'
        elif self.data_type == "uint16":
            return 'H'
        elif self.data_type == "uint32":
            return 'I'
        elif self.data_type == "uint64":
            return 'Q'
        warn("Unknown data type '{}'".format(self.data_type))
        return 'd'

    def rename_from_ros(self, ros_name):
        """
        Convert descriptor name from ROS to UI format.

        :return: Name in UI format.
        """
        def update_suffix(name):
            if "ulerAnglesZyx" in name:
                suffix_map = {'_x': 'yaw', '_y': 'pitch', '_z': 'roll'}
                for suffix, replacement in suffix_map.items():
                    if name.endswith(suffix):
                      return name[:-len(suffix)] + TOPIC_SEPARATOR + replacement
            else:
                for suffix in common_suffixes:
                    if name.endswith(suffix):
                        return name[:-len(suffix)] + TOPIC_SEPARATOR + suffix[1:]
            return name

        name = update_suffix(ros_name)
        name = name.replace('/', TOPIC_SEPARATOR)
        return name


class LogReader(object):

    def __init__(self):
        self._data = {}
        self.descriptors = []
        self.nb_log_elems = None
        self.time_sync_offset = None

    def read_from_file(self, fpath):
        """
        Read data from file.

        :param fpath: Path to file.
        """
        with open(fpath, 'rb') as fd:
            self._read_header(fd)
            self._read_data(fd)
        self._trim_common_prefix()
        self._compute_time()
        self._pad_data()
        return self

    def get_data(self):
        """
        Return data dictionary.

        :return: Data dictionary.
        """
        return self._data

    def _read_header(self, fd):
        """
        Read log file header.

        :param fd: File descriptor.
        """
        going_binary = False
        while not going_binary:
            line = str(fd.readline(), encoding='latin-1')
            if line.startswith('#'):
                if "Binary Data" in line:
                    going_binary = True
                continue
            if self.time_sync_offset is None:
                self.time_sync_offset = int(line)
            elif self.nb_log_elems is None:
                self.nb_log_elems = int(line)
            else:  # new descriptor
                self.descriptors.append(Descriptor(line))

    def _read_data(self, fd):
        """
        Read data dictionary.

        :param fd: File descriptor.
        """
        for desc in self.descriptors:
            self._data[desc.name] = []
            format = desc.get_format()
            for _ in range(desc.nb_points):
                bytes = fd.read(desc.size)
                if len(bytes) != desc.size:
                    warn("End of file reached earlier than expected.")
                    return
                try:
                    values = struct.unpack(format, bytes)
                    self._data[desc.name].append(values[0])
                except struct.error as e:
                    warn("Reading error for '{}': {}".format(
                        desc.name, str(e)))

    def _trim_common_prefix(self):
        """
        Trim longest common prefix to all signal names.
        """
        common_prefix = commonprefix(list(self._data.keys()))
        print("Removing prefix {} which is common to all signals".format(common_prefix))
        prefix_len = len(common_prefix)
        self._data = {key[prefix_len:]: value for key, value in self._data.items()}

    def _compute_time(self):
        """
        Compute standard time axis.
        """
        assert ("time_s" in self._data and "time_ns" in self._data)
        assert (len(self._data["time_s"]) == len(self._data["time_ns"]))
        epoch_times = [
            self._data["time_s"][i] + 1e-9 * self._data["time_ns"][i]
            for i in range(len(self._data["time_s"]))]
        start_time = epoch_times[0]
        silo_times = [epoch_times[i] - start_time for i in range(len(self._data["time_s"]))]
        self._data["t"] = silo_times
        self._data["time/epoch"] = epoch_times
        self._data["time/nanoseconds"] = self._data["time_ns"]
        self._data["time/seconds"] = self._data["time_s"]
        self._data["time/silo"] = silo_times
        del self._data["time_ns"]
        del self._data["time_s"]

    def _pad_data(self):
        """
        Pad data for items whose size is less than that of data["t"].
        """
        time = self._data["t"]
        size_differences = {
            key: len(time) - len(data)
            for key, data in self._data.items()}
        for key, size_diff in size_differences.items():
            if size_diff > 0:
                self._data[key] = [None] * size_diff + self._data[key]
