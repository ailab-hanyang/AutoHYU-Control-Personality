from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['opti_sn_planning','opti_sn_planning.utils'],
    package_dir={'': 'src'}
)

setup(**d)