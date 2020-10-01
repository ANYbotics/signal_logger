#!/usr/bin/env python3
#
# Author:       Stephane Caron
# Affiliation:  ANYbotics
# Date:         27.04.2020

import numpy

from .cartesian_signal import CartesianSignal
from .signal import Signal


class QuaternionSignal(Signal):

    """
    Signal for quaternion data associated with a given silo.
    """

    def __init__(self, name, times, w, x, y, z):
        """
        Create a new quaternion signal.

        :param name: Name of the series.
        :param times: List of time values.
        :param w: Signal for (scalar) w coordinates.
        :param x: Signal for (vector) x coordinates.
        :param y: Signal for (vector) y coordinates.
        :param z: Signal for (vector) z coordinates.
        """
        w = numpy.array(w)
        x = numpy.array(x)
        y = numpy.array(y)
        z = numpy.array(z)
        assert len(w.shape) == 1
        assert len(x.shape) == 1
        assert len(y.shape) == 1
        assert len(z.shape) == 1
        values = numpy.vstack([w, x, y, z]).T
        super(QuaternionSignal, self).__init__(name, times, values)

    @staticmethod
    def from_cartesian(cartesian_signal):
        """
        Promote a point signal to quaternion.

        :param cartesian_signal: Point signal.
        """
        name = "quat({})".format(cartesian_signal.name)
        times = cartesian_signal.times
        w = numpy.zeros(len(times))
        x = cartesian_signal.values[:, 0]
        y = cartesian_signal.values[:, 1]
        z = cartesian_signal.values[:, 2]
        return QuaternionSignal(name, times, w, x, y, z)

    def _deserialize(self):
        """
        Deserialize quaternion values from NumPy array.

        :return: Tuple (w, x, y, z) of time-series arrays.
        """
        w = self.values[:, 0]
        x = self.values[:, 1]
        y = self.values[:, 2]
        z = self.values[:, 3]
        return (w, x, y, z)

    def conjugate(self):
        """
        Conjugate quaternion.

        :return: Conjugate of this quaternion.
        """
        name = "~{}".format( self.name)
        w, x, y, z = self._deserialize()
        return QuaternionSignal(name, self.times, w, -x, -y, -z)

    def inv(self):
        """
        Inverse rotation, which would be the transpose if the internal representation was a rotation matrix.

        :return: Conjugate quaternion signal.
        """
        return self.conjugate()

    def __mul__(self, other):
        if self.shape == other.shape:
            return self.multiply(other)
        elif other.shape == (3,):
            quat_prod = self.multiply(QuaternionSignal.from_cartesian(other)).multiply(self.conjugate())
            w, x, y, z = quat_prod._deserialize()
            assert numpy.linalg.norm(w) < 1e-5, "Precision error in quaternion product"
            name = "{} * {}".format(self.name, other.name)
            return CartesianSignal(name, self.times, x, y, z)
        raise ValueError("No know way to multiply quaternion {} to signal {} of shape {}".format(
            self.name, other.name, other.shape))

    def multiply(self, other):
        """
        Multiply with another quaternion.

        :param other: Other quaternion.
        :return: Product of the two quaternions.
        """
        w1, x1, y1, z1 = self._deserialize()
        w2, x2, y2, z2 = other._deserialize()
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        name = "{} * {}".format(self.name, other.name)
        return QuaternionSignal(name, self.times, w, x, y, z)

    def transpose(self):
        """
        Inverse rotation, which would be the transpose if the internal representation was a rotation matrix.

        :return: Conjugate quaternion signal.
        """
        return self.conjugate()


class EulerAnglesZyxSignal(QuaternionSignal):

    """
    Signal for Euler angles (z-y'-z'' intrinsic, x-y-z extrinsic convention) associated with a given silo.

    See https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    """

    def __init__(self, name, times, x, y, z):
        """
        Create a new Euler angles signal.

        :param name: Name of the series.
        :param times: List of time values.
        :param x: Signal for (vector) yaw coordinates, saved as "*_x" by signal_logger.
        :param y: Signal for (vector) pitch coordinates, saved as "*_y" by signal_logger.
        :param z: Signal for (vector) roll coordinates, saved as "*_z" by signal_logger.
        """
        x = numpy.array(x)
        y = numpy.array(y)
        z = numpy.array(z)
        assert len(x.shape) == 1
        assert len(y.shape) == 1
        assert len(z.shape) == 1
        roll, pitch, yaw = z, y, x  # first (resp. last) coordinate in EulerAnglesZyx is yaw (resp. roll)
        cr, cp, cy = numpy.cos(roll / 2.), numpy.cos(pitch / 2.), numpy.cos(yaw / 2.)
        sr, sp, sy = numpy.sin(roll / 2.), numpy.sin(pitch / 2.), numpy.sin(yaw / 2.)
        q_w = cr * cp * cy + sr * sp * sy
        q_x = sr * cp * cy - cr * sp * sy
        q_y = cr * sp * cy + sr * cp * sy
        q_z = cr * cp * sy - sr * sp * cy
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        super(EulerAnglesZyxSignal, self).__init__(name, times, q_w, q_x, q_y, q_z)

    @staticmethod
    def from_constant(name, times, roll, pitch, yaw, degrees=False):
        """
        Create constant signal with pre-defined values.

        :param name: Name of signal.
        :param times: Time values.
        :param roll: Constant roll angle in [rad] (or [deg] if ``degrees=True``).
        :param pitch: Constant pitch angle in [rad] (or [deg] if ``degrees=True``).
        :param yaw: Constant yaw angle in [rad] (or [deg] if ``degrees=True``).
        :param degrees: If true, angles are considered in [deg] rather than [rad].
        :return: Euler-angles signal with constant values.
        """
        if degrees:
            roll *= numpy.pi / 180.
            pitch *= numpy.pi / 180.
            yaw *= numpy.pi / 180.
        return EulerAnglesZyxSignal(name, times, [yaw] * len(times), [pitch] * len(times), [roll] * len(times))
