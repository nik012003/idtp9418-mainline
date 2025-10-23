#!/bin/bash
set -xe
TABLET="vio@192.168.77.238"

make -C ../sm8150-mainline M=$(pwd) modules
scp idtp9418.ko $TABLET:
ssh -t $TABLET "sudo rmmod idtp9418.ko; sudo modprobe ./idtp9418.ko"
