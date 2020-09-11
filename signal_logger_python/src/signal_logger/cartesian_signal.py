#!/usr/bin/env python3
#
# Author:       Stephane Caron
# Affiliation:  ANYbotics
# Date:         27.04.2020

import numpy

from .signal import Signal


class CartesianSignal(Signal):

    """
    Signal for 3D cartesian data associated with a given silo.
    """

    def __init__(self, name, times, x, y, z):
        """
        Create a new point signal.

        :param name: Name of the series.
        :param times: List of time values.
        :param x: Signal for x coordinates.
        :param y: Signal for y coordinates.
        :param z: Signal for z coordinates.
        """
        x = numpy.array(x)
        y = numpy.array(y)
        z = numpy.array(z)
        assert len(x.shape) == 1
        assert len(y.shape) == 1
        assert len(z.shape) == 1
        values = numpy.vstack([x, y, z]).T
        super(CartesianSignal, self).__init__(name, times, values)

    @property
    def x(self):
        return Signal("{}.x".format(self.name), self.times, self.values[:, 0])

    @property
    def y(self):
        return Signal("{}.y".format(self.name), self.times, self.values[:, 1])

    @property
    def z(self):
        return Signal("{}.z".format(self.name), self.times, self.values[:, 2])

    def rotate(self, rotation_matrix):
        new_values = numpy.dot(rotation_matrix, self.values.T).T
        self.values = new_values
