#!/usr/bin/bash

mkdir -p cmake-build-release
cd cmake-build-release
cmake .. -DCMAKE_BUILD_TYPE=Release && cmake --build . --target All --config Release -j
