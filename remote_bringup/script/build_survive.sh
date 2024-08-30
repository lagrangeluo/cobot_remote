#!/bin/bash

cd ../lib/libsurvive/build &&
sudo rm -r ./* &&
cmake .. &&
make