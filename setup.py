from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy

sources = [
    "src/ads1256_quad_api/_core.pyx",
    "src/ads1256_core/ads1256_error.c",
    "src/ads1256_core/ads1256_system.c",
    "src/ads1256_core/ads1256_spi.c",
    "src/ads1256_core/ads1256_ring.c",
]

ext = Extension(
    name="ads1256_quad_api._core",
    sources=sources,
    include_dirs=["src/ads1256_core/include", numpy.get_include()],
    language="c",
)

setup(
    ext_modules=cythonize([ext], language_level=3, compiler_directives={"boundscheck": False, "wraparound": False}),
)
