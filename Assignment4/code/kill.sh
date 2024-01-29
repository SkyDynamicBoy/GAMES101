#!/bin/bash
kill -9 $(ps | grep BezierCurve | awk '{print $1}')
