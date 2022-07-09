# Run the following command to compile required files
# python setup.py build_ext --inplace

from distutils.core import setup
import pathlib

from Cython.Build import cythonize
import numpy

working_dir = str(pathlib.Path(__file__).parent.resolve())
setup(
    ext_modules=cythonize(working_dir + "/*.pyx", language_level=3, annotate=True), include_dirs=[numpy.get_include()]
)
