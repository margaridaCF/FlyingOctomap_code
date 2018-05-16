#!/bin/sh

sudo ip addr flush dev enp1s0
sudo ip addr add 192.168.0.15/24 dev enp1s0

