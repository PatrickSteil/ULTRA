#!/usr/bin/bash

rm -rf cmake-build-release
mkdir -p cmake-build-release
cd cmake-build-release
cmake .. -DCMAKE_BUILD_TYPE=Release && cmake --build . --target All --config Release -j

cd ..

rm -rf cmake-build-debug
mkdir -p cmake-build-debug
cd cmake-build-debug
cmake .. -DCMAKE_BUILD_TYPE=Debug && cmake --build . --target All --config Debug -j
