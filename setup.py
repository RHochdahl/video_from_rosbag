
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["video_from_rosbag"],
    package_dir={"": "src"},
)

setup(**setup_args)
