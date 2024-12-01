from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['safety_module'],
    package_dir={'safety_module': 'safety_module'})

setup(**setup_args)
