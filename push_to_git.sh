#!/bin/sh

cd exampels/PD956_eks/pd956_sensors_low_power
make TARGET=PR956 clean
cd ../../..
git add .
git status
echo -n "Enter the Description for the Change: "
echo ""
read CHANGE_MSG
git commit -m "${CHANGE_MSG}"
git push
