#!/bin/bash
kill -9 $(ps | grep Rasterizer | awk '{print $1}')


