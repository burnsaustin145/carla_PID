#!/usr/bin/env python
"""
Project: Carla PID Controller
Author: Austin Burns
Date: 4-23-2024
Description: Route planner and PID controller for use with Carla. 
"""
import glob
import os
import sys
import time
import random
import numpy as np
import pprint as pp

from data_collection import DataCollector
from route_planning import Planner

from sim_utility import *
from control import *

# get system version for sys path (from documentation)
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import argparse
import carla as c


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--spawn',
        default=1,
        type=int
    )

    args = argparser.parse_args()

    client, world, actors, main_car, camera, obstacle_sensor, world_map, control = simulation_setup(args)

    # TUNING PARAMETERS FOR ACCELERATION PID **************************************************************
    Kp = 0.005
    Ki = 0.02
    Kd = 0.05
    set_speed = 5
    # *****************************************************************************************************

    # SIMULATION PARAMETERS *******************************************************************************
    waypoint_next_dist = 8  # value of 10 here gives a nice smooth ride
    simulation_time = 100000
    navigation_mode = "planner"  # set to "next" or "planner" for different navigation modes
    # *****************************************************************************************************
    data_collector_velocity = DataCollector(Kp, Ki, Kd, "PID_Tuning_Velocity_")  # collect data for tuning
    data_collector_U = DataCollector(Kp, Ki, Kd, "PID_Tuning_U-value_")
    data_collector_throttle = DataCollector(Kp, Ki, Kd, "PID_Tuning_Throttle_")

    planner = Planner(world_map, main_car, waypoint_next_dist)
    path_index = 0
    path_len = len(planner.path)
    print(f"path len{path_len}")

    PID = PIDController(Kp, Ki, Kd, set_speed, control)
    obstacle_sensor.listen(PID.get_brake_detect)

    run_flag = True
    time_elapsed = 0
    prev_time = time.time()
    while run_flag:
        if navigation_mode == "next":
            # get the next waypoint and use our controller to get there
            waypoint = (world_map.get_waypoint(main_car.get_location(),
                                               project_to_road=True,
                                               lane_type=c.LaneType.Driving)).next(waypoint_next_dist)[0]

        else:
            waypoint = planner.path[path_index].waypoint
            dist = ((main_car.get_location().x - waypoint.transform.location.x) ** 2 +
                   (main_car.get_location().y - waypoint.transform.location.y) ** 2)
            if dist < 1:
                waypoint = planner.path[path_index].waypoint
                path_index += 1
            elif path_index >= path_len - 1:
                path_index = 0
            else:
                pass  # pass the waypoint update if we're not close enough to the new one
        PID.apply_controller(main_car, waypoint)
        main_car.apply_control(control)
        # collect data
        speed = np.sqrt(main_car.get_velocity().x ** 2 + main_car.get_velocity().y ** 2 + main_car.get_velocity().z ** 2)
        data_collector_velocity.collect_data(x=time.time(), y=speed)
        data_collector_U.collect_data(x=time.time(), y=PID.u)
        data_collector_throttle.collect_data(x=time.time(), y=control.throttle)
        # update the spectator location to the car camera
        spectator = world.get_spectator()
        spectator.set_transform(camera.get_transform())
        curr_time = time.time()
        time_elapsed += curr_time - prev_time

        if time_elapsed > simulation_time or path_index > path_len:
            print(path_index > path_len)
            print(time_elapsed > simulation_time)
            run_flag = False

    data_collector_velocity.show_data(set_speed)
    data_collector_U.show_data(0)
    data_collector_throttle.show_data(0.5)

    print('\ndestroying %d actors' % len(actors))
    client.apply_batch([c.command.DestroyActor(x) for x in actors])


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')