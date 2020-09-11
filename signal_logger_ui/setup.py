## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['bin/signal_logger_ui'],
    packages=['signal_logger_ui', 'signal_logger_ui.ui'],
    package_dir={'': 'src'},
)

setup(**setup_args)
