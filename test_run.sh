#!/usr/bin/zsh
SPAWN=1

# Start the CARLA Simulator
~/opt/carla-simulator/CarlaUE4.sh &
PID1=$!
sleep 10
# Run the Python script to generate traffic with n vehicles
python3 ./generate_traffic_mod.py -n 50 -w 0 -z $SPAWN &
PID2=$!
sleep 1
# can set spawn point here w/ -s 1
python3 ./main.py -s $SPAWN &
PID3=$!
sleep 300
kill $PID1
kill $PID2
kill $PID3

