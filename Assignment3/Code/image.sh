#!/bin/bash
cd build
make
./Rasterizer output.png displacement
cd ..
