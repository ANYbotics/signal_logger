# Signal Logger UI

## Dependencies

Install ``sudo apt install python-pyside`` (Ubuntu 18.04).

## Usage

Run the script from ROS, giving it as argument any of:

- file name
- prefix (e.g. "highLevelController"): it will then open the most recent log
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

So that ``silo high`` opens the most recent log file recorder with the
high-level controller

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

Create ``~/.local/share/applications/signal_logger_hlc.desktop`` with the
following:

```
[Desktop Entry]
Name=Signal Logger highLevelController
Comment=Open most recent log from high-level controller
Exec=bash -c "source /opt/ros/melodic/setup.bash && /opt/ros/melodic/bin/signal_logger_ui high"
Icon=org.gnome.PowerStats
Type=Application
Categories=Development
```

Press the Meta key on your keyboard and type "HLC". Right-click on the Signal
Logger icon that appears, then "Add to Favorites". You can then move it in your
dock so that it is accessible by a fast keyboard shortcut such as Meta + 1.

## Changelog

This package was adapted from [mc\_log\_gui](https://github.com/jrl-umi3218/mc_rtc/tree/v1.1.0/utils/mc_log_gui). The following main changes were made:

- Adapt to Signal Logger binary format
- Add an embedded IPython shell
- Integrate with [signal\_logger\_python](https://github.com/ANYbotics/signal_logger/tree/master/signal_logger_python)

The original tool was developed by Pierre Gergondet and licensed under the BSD-2-Clause.
