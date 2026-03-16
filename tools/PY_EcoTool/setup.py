# setup.py
from setuptools import setup, Extension
import os

# Define build directory
build_dir = os.path.join(os.path.dirname(__file__), 'build')

aebf_module = Extension(
    'aebf',  # This will produce aebf.so
    sources=['./C_port/aebf_module.c', './C_port/aebfStream.c'],  # Add your C source files
    extra_compile_args=['-Wall', '-Wextra', '-O2'],  # Optional compiler flags
    # If your C code uses special flags or libraries, add them here
)

setup(
    name='aebf',
    version='1.0',
    description='AEBF Protocol Python Bindings',
    ext_modules=[aebf_module],
    # If you need to install with pip install -e .
    options={
        'build': {
            'build_base': build_dir,     # Temporary build files
            'build_lib': build_dir,      # Final .so file location
            'build_temp': os.path.join(build_dir, 'temp')
        }
    }
)