from distutils.core import setup, Extension
from Cython.Build import cythonize

setup(ext_modules = cythonize(Extension(
           "cpp_cbs",                                
           sources=["cbs_grid_planning.pyx"], 
           extra_compile_args=["-std=c++11"]
      )))
