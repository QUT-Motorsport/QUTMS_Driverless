from distutils.core import setup
from Cython.Build import cythonize

import pathlib
working_dir = str(pathlib.Path(__file__).parent.resolve())
setup (
    ext_modules=cythonize(working_dir + '/total_least_squares.pyx', annotate=True)
)