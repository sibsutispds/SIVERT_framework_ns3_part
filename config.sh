#!/usr/bin/env zsh

cd $1
CXXFLAGS="-Wtautological-constant-out-of-range-compare" ./waf configure --enable-examples
