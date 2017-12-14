from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['appcast_view'],
    package_dir={'': 'src'},
    scripts=['src/br24_radar.py']
)

setup(**d)
