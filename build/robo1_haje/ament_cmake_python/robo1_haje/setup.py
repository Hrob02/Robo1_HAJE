from setuptools import find_packages
from setuptools import setup

setup(
    name='robo1_haje',
    version='1.0.3',
    packages=find_packages(
        include=('robo1_haje', 'robo1_haje.*')),
)
