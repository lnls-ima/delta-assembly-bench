
import os
from setuptools import setup


basedir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(basedir, 'VERSION'), 'r') as _f:
    __version__ = _f.read().strip()


setup(
    name='bench-control',
    version=__version__,
    description='Delta Undulator assembly bench control application',
    url='https://github.com/lnls-ima/delta-assembly-bench',
    author='lnls-ima',
    license='MIT License',
    packages=['deltabench'],
    install_requires=[
        'pyvisa',
        'numpy',
        'scipy',
        'pandas',
        'pyqtgraph',
        'pyserial',
        'qtpy',
        'natsort',
    ],
    test_suite='nose.collector',
    tests_require=['nose'],
    zip_safe=False)
