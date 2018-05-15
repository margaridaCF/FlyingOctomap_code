#!/bin/sh
echo ''Mem:          total        used        free      shared  buff available'' >> ../data/ram.csv
while true
do
free -m | grep Mem >> ram.csv
sleep 60
done