# Signal Logger for Python

## Usage

Run the reader script by:

```bash
rosrun signal_logger_python signal_logger_reader mySiloPrefix
```

This will load the latest silo in your ``~/.ros`` folder whose filename starts with ``mySiloPrefix``. For example, if you have motion control manager silos in your ``~/.ros`` folder, replace ``mySiloFilePrefix`` by e.g. ``high`` or ``highLevel``.

Once loaded, you can read data using e.g.:

```python
vel = silo.get_signal('loco/leftFore/linearVelocityDesiredEEOriginInWorldFrame/x')
vel.value(9.1)  # value of the signal at t=9.1 [s]
vel.average(8.95, 9.4)  # time average between 8.95 [s] and 9.4 [s]
```

Everything is stored in data, and you can use the Signal wrapper so that things
are indexed by time rather than by the dictionary index.

### Example

### Changing frames

Translations and rotations are read as typed signals and can be manipulated as we expect:

```python
import signal_logger

silo = signal_logger.Silo("my_file.silo")
accelInImuFrame = silo.get_signal('/state/imu/linAccImuInImuFrame')
rotationImuToBase = EulerAnglesZyxSignal.from_constant('rotationImuToBase', silo.times, 0., 0., 90, degrees=True)
rotationBaseToWorld = silo.get_signal('loco/torso/measured/orientationEulerAnglesZyxBaseToWorld')
accelInWorldFrame = rotationBaseToWorld * rotationImuToBase * accelInImuFrame
```

#### Creating new signals

The following script computes the norm of the acceleration vector from IMU readings:

```python
import signal_logger

silo = signal_logger.Silo("my_file.silo")
acc_x = silo.get_signal('state/imu/linAccImuInImuFrame/x')
acc_y = silo.get_signal('state/imu/linAccImuInImuFrame/y')
acc_z = silo.get_signal('state/imu/linAccImuInImuFrame/z')

def acc_norm(t):
    from pylab import norm  # don't do that in general, this is just for the example
    return norm([acc_x.value(t), acc_y.value(t), acc_z.value(t)])

times = acc_x.times
acc_norm_signal = signal_logger.Signal(times, [acc_norm(t) for t in times], 'acc_norm')
acc_norm_signal.plot()
```
We can also manipulate signals directly and do things like:
```
(acc_x + acc_y).plot()
```
