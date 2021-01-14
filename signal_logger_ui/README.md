# Signal Logger UI

## Dependencies

Install ``sudo apt install python-pyside`` (Ubuntu 18.04).

## Usage

Run the script from ROS, giving it as argument any of:

- file name
- prefix (e.g. "motionControlManager"): it will then open the most recent log
- suffix (e.g. 200): it will then open the log corresponding to this id
- full file path

For instance:

```bash
rosrun signal_logger_ui signal_logger_ui 199
```

The call above looks for a log file with extension "199" in your ``~/.ros``
directory. You can alias the rosrun for faster access:

```bash
alias silo='rosrun signal_logger_ui signal_logger_ui'
```

So that ``silo motion`` opens the most recent log file recorder with the
motion control manager

## Embedded IPython shell

There is an embedded IPython shell (hidden by default, you need to slide it up)
at the bottom of every window. You can read signals there via the `silo`
dictionary that is loaded by default there:

```python
In [1]: lf_haa_current = silo['state/current/LF_HAA']

In [2]: plot(lf_haa_current)
```

Plotting is currently in a hackish state: it is done on the right axis, and not
appended to the legend properly. Plot something else on the right axis using
the UI to initialize it, so that you can then use the axis customization button
to set its bottom and top boundaries. When you clear all UI signals from the
right axis, it will be removed and all signals plotted from the IPython shell
will be removed as well.

## Convenience

### Opening logs from the file manager

Create ``~/.local/share/applications/signal_logger_ui.desktop`` with the
following:

```
[Desktop Entry]
Name=Signal Logger UI
Comment=Plotting UI for Signal Logger
Exec=bash -c "source /opt/ros/melodic/setup.bash && /opt/ros/melodic/bin/signal_logger_ui %f"
Icon=org.gnome.PowerStats
Type=Application
Categories=Development
Terminal=false
```

Then right-click on a log file (making sure it has the proper extension, for
instance ``.log``), "Properties", go to "Open With", select "Signal Logger Ui"
and click on "Set as default".

### Opening the last log directly

Create ``~/.local/share/applications/signal_logger_mcm.desktop`` with the
following:

```
[Desktop Entry]
Name=Signal Logger motionControlManager
Comment=Open most recent log from motion control manager
Exec=bash -c "source /opt/ros/noetic/setup.bash && /opt/ros/noetic/bin/signal_logger_ui motionControl"
Icon=org.gnome.PowerStats
Type=Application
Categories=Development
```

Press the Meta key on your keyboard and type "MCM". Right-click on the Signal
Logger icon that appears, then "Add to Favorites". You can then move it in your
dock so that it is accessible by a fast keyboard shortcut such as Meta + 1.

## Changelog

This package was adapted from [mc\_log\_gui](https://github.com/jrl-umi3218/mc_rtc/tree/v1.1.0/utils/mc_log_gui). The following main changes were made:

- Adapt to Signal Logger binary format
- Add an embedded IPython shell
- Integrate with [signal\_logger\_python](https://github.com/ANYbotics/signal_logger/tree/master/signal_logger_python)

The original tool was developed by Pierre Gergondet and licensed under the BSD-2-Clause.
