from __future__ import division, absolute_import, with_statement, print_function
from setuptools import setup, find_packages
from torch.utils.cpp_extension import BuildExtension, CUDAExtension
import glob
import os

try:
    import builtins
except:
    import __builtin__ as builtins

builtins.__POINTNET2_SETUP__ = True
#import pointnet2

_ext_src_root = "_ext-src"
_ext_sources = glob.glob("{}/src/*.cpp".format(_ext_src_root)) + glob.glob(
    "{}/src/*.cu".format(_ext_src_root)
)
include_dir = os.path.abspath("{}/include".format(_ext_src_root))
_ext_headers = glob.glob("{}/include/*".format(_ext_src_root))

requirements = ["h5py", "pprint", "enum34", "future"]

setup(
    name="PointNet2",
    version=0.1,
    author="Jules Morel",
    packages=find_packages(),
    install_requires=requirements,
    ext_modules=[
        CUDAExtension(
            name="_ext",
            sources=_ext_sources,
            extra_compile_args={
                "cxx": ["-O2", "-I{}".format(include_dir)],
                "nvcc": ["-O2", "-I{}".format(include_dir)],
            },
        )
    ],
    cmdclass={"build_ext": BuildExtension},
)
