#!/usr/bin/env python3
#
# Author:       Stephane Caron
# Affiliation:  ANYbotics
# Date:         07.04.2020

from bisect import bisect_right
from numpy import array, sqrt


class Signal(object):

    """
    Signal from associated with a given silo.
    """

    def __init__(self, name, times, values,):
        """
        Create a new signal.

        :param name: Name of the series.
        :param times: List of time values.
        :param values: List of signal values.
        """
        values = array(values)
        assert(len(times) == values.shape[0])
        self.name = name
        self.times = times
        self.values = values

    @property
    def shape(self):
        return self.values[0].shape

    def check_shape_compatibility(self, other):
        if self.shape != other.shape:
            raise ValueError("Signal shapes for {} {} and {} {} don't match".format(
                self.name, self.shape, other.name, other.shape))

    def index_from_time(self, t):
        """
        Find leftmost index such that self.values[index] >= t, or the last time
        in the series otherwise.

        Parameters
        ----------
        t : scalar
            Time in [s].
        """
        index = bisect_right(self.times, t)
        return index - 1

    def value(self, t):
        """
        Value of signal at time t.

        Parameters
        ----------
        t : scalar
            Time in [s].
        """
        index = self.index_from_time(t)
        return self.values[index]

    def average(self, start_time, end_time, absvals=False):
        """
        Time average of signal between ``start_time`` and ``end_time``.

        Parameters
        ----------
        start_time : scalar
            Start time in [s].
        end_time : scalar
            End time in [s].
        absvals : boolean
            If true, average the absolute value rather than the signed value of the signal.
        """
        duration = end_time - start_time
        assert (duration > 0.), "Duration should be positive"
        start_index = self.index_from_time(start_time)
        end_index = self.index_from_time(end_time)
        if end_index == len(self.times) - 1:
            end_index -= 1
        signal_sum = 0.
        total_time = 0.
        for index in range(start_index, end_index + 1):
            dt = self.times[index + 1] - self.times[index]
            incr = self.values[index] * dt
            signal_sum += abs(incr) if absvals else incr
            total_time += dt
        return signal_sum / total_time

    def std_dev(self, start_time, end_time):
        """
        Standard deviation of signal between ``start_time`` and ``end_time``.

        Parameters
        ----------
        start_time : scalar
            Start time in [s].
        end_time : scalar
            End time in [s].
        """
        average = self.average(start_time, end_time)
        squared_signal = self * self
        return sqrt(squared_signal.average(start_time, end_time) - average * average)

    def __add__(self, other):
        self.check_shape_compatibility(other)
        name = "{} + {}".format(self.name, other.name)
        values = self.values + other.values
        return Signal(name, self.times, values)

    def __sub__(self, other):
        self.check_shape_compatibility(other)
        name = "{} - {}".format(self.name, other.name)
        values = self.values - other.values
        return Signal(name, self.times, values)

    def __mul__(self, other):
        self.check_shape_compatibility(other)
        name = "{} * {}".format(self.name, other.name)
        values = self.values * other.values
        return Signal(name, self.times, values)

    def __div__(self, other):
        self.check_shape_compatibility(other)
        name = "{} / {}".format(self.name, other.name)
        values = self.values / other.values
        return Signal(name, self.times, values)

    def __repr__(self):
        return '{}("{}")'.format(type(self).__name__, self.name)

    def plot(self, *args, **kwargs):
        from pylab import plot
        plot(self.times, self.values, *args, **kwargs)
