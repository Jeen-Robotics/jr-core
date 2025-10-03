#!/bin/bash

scripts/install_deps.sh
source build/conanbuild.sh
protoc --cpp_out=build-android/core/bag --proto_path=core/bag core/bag/bag.proto
conan install . --build=missing -of build-android -s build_type=Release -pr:h android -pr:b default