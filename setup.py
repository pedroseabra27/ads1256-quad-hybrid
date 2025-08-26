from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy, os, sys, subprocess

def detect_libgpiod():
    """Attempt to detect libgpiod (v1 or v2) presence to enable edge-driven DRDY.
    Returns (have, libs, define_macros, include_dirs)."""
    # Allow user override
    if os.environ.get("ADS1256_FORCE_NO_GPIOD"):
        return False, [], [], []
    candidates = ["gpiod"]  # simple
    for lib in candidates:
        try:
            # Prefer pkg-config if available
            pc = subprocess.run(["pkg-config", "--cflags", lib], capture_output=True, text=True)
            if pc.returncode == 0:
                cflags = pc.stdout.strip().split()
                libs_out = subprocess.run(["pkg-config", "--libs", lib], capture_output=True, text=True)
                if libs_out.returncode == 0:
                    libs_tokens = libs_out.stdout.strip().split()
                    libs = []
                    extra_link_args = []
                    include_dirs = []
                    for tok in libs_tokens:
                        if tok.startswith("-l"):
                            libs.append(tok[2:])
                        else:
                            extra_link_args.append(tok)
                    for cf in cflags:
                        if cf.startswith("-I"):
                            include_dirs.append(cf[2:])
                    return True, libs, [("ADS1256_HAVE_GPIOD", "1")], include_dirs
        except FileNotFoundError:
            pass
    # Fallback: try simple compile test if gcc/cc present
    test_code = """#include <gpiod.h>\nint main(){return 0;}"""
    import tempfile
    import shutil
    cc = os.environ.get("CC", "cc")
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, "t.c")
        with open(src, "w") as f:
            f.write(test_code)
        exe = os.path.join(td, "t")
        try:
            r = subprocess.run([cc, src, "-lgpiod", "-o", exe], capture_output=True)
            if r.returncode == 0:
                return True, ["gpiod"], [("ADS1256_HAVE_GPIOD", "1")], []
        except FileNotFoundError:
            return False, [], [], []
    return False, [], [], []

have_gpiod, gpiod_libs, gpiod_macros, gpiod_includes = detect_libgpiod()
if have_gpiod:
    print("[setup] libgpiod detected: enabling edge-driven DRDY", file=sys.stderr)
else:
    print("[setup] libgpiod not found (edge-driven DRDY disabled)", file=sys.stderr)

sources = [
    "src/ads1256_quad_api/_core.pyx",
    "src/ads1256_core/ads1256_error.c",
    "src/ads1256_core/ads1256_system.c",
    "src/ads1256_core/ads1256_spi.c",
    "src/ads1256_core/ads1256_ring.c",
    "src/ads1256_core/ads1256_gpio.c",
    "src/ads1256_core/ads1256_log.c",
]

include_dirs = ["src/ads1256_core/include", numpy.get_include()] + gpiod_includes
ext = Extension(
    name="ads1256_quad_api._core",
    sources=sources,
    include_dirs=include_dirs,
    libraries=gpiod_libs if have_gpiod else [],
    define_macros=gpiod_macros,
    language="c",
)

setup(
    ext_modules=cythonize([ext], language_level=3, compiler_directives={"boundscheck": False, "wraparound": False}),
)
