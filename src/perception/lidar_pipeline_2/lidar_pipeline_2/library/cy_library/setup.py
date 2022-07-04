from distutils.core import setup
from Cython.Build import cythonize

import numpy

import pathlib

working_dir = str(pathlib.Path(__file__).parent.resolve())
setup (
    ext_modules=cythonize(working_dir + '/*.pyx', language_level=3, annotate=True),
    include_dirs=[numpy.get_include()]
)
