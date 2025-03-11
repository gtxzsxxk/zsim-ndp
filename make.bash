#!/bin/bash

export LIBCONFIGPATH=$HOME/.local
export HDF5PATH=$HOME/.local
bear -- scons -j64
