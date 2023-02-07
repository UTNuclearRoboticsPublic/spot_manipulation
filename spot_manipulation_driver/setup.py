#!/usr/bin/env python

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["spot_manipulation_driver"], package_dir={"": "include"}
)

setup(**setup_args)
