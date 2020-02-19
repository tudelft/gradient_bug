# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time
import sys
import os

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
import math
import numpy as np
from scipy.signal import medfilt



import pygame, sys
from pygame.locals import *

class CrazyFlieOffBoard:

    #variables for logging
   

    # Call back
    def crazyFlieLoop(self, radio_id, drone_id):

	# All uneven drones are assigned the channel below
        #if drone_id % 2 == 0:
	channel = drone_id*10
	#else:
 	 #   channel = (drone_id-1)*10

	# The complete uri of the crazyfly
        URI = 'radio://'+str(radio_id)+'/'+str(channel)+'/2M/E7E7E7E70'+str(drone_id)
        print("Connect to ", URI)

        # Only output errors from the logging framework
        logging.basicConfig(level=logging.ERROR)

        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)



        reconnect =True

	while reconnect:
		try:
                        cf = Crazyflie(rw_cache='./cache')

			with SyncCrazyflie(URI, cf=cf) as scf:

			    while 1:
				    param_name = "gbug.keep_flying"
				    param_value = "0"
				    cf.param.set_value(param_name, param_value)


			
		except Exception as ex:
	    		print ex
                        reconnect = True
	                time.sleep(2)
	    		#raw_input()



if __name__ == '__main__':

    CrazyFlieOffBoard = CrazyFlieOffBoard()
    arguments = sys.argv[1:]
    count = len(arguments)

    if count == 2:
        CrazyFlieOffBoard.crazyFlieLoop(int(sys.argv[1]),int(sys.argv[2]))
    else:
        print("NOT ENOUGH ARGUMENTS!")
     
