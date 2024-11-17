#!/bin/bash

# ccache is optional, but recommended

# Create build directory
mkdir -p build
pushd build
AR="riscv64-linux-gnu-ar-12" CXX="riscv64-linux-gnu-g++-12" CC="riscv64-linux-gnu-gcc-12" cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain.cmake
cmake --build .
popd
