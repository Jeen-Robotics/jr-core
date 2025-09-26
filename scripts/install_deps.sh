#!/bin/bash

conan install . --build=missing -of build -s build_type=Release
conan install . --build=missing -of build -s build_type=Release -s "&:build_type=Debug"