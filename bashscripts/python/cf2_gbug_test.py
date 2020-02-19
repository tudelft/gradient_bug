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
import cflib.drivers.crazyradio as crazyradio


import pygame, sys
from pygame.locals import *

class CrazyFlieOffBoard:

    #variables for logging
    kalman_states_ox = 0
    kalman_states_oy = 0
    radio_rssi = 0
    stabilizer_yaw=0
    gradientbug_rssi_angle = 0
    gradientbug_state = 0
    gradientbug_state_wf = 0
    gradientbug_rssi_beacon = 0
    gradientbug_rssi_i = 0
    gradientbug_rssi_angle_i =0
    height = 0
    radio_rssi_inter = 0
    pygame.init()
    pygame.display.set_mode((100,100))
    keep_flying = 0
   

    # data received cal back
    def data_received(self,timestamp, data, logconf):

        self.kalman_states_ox = float(data["kalman_states.ox"])
        self.kalman_states_oy = float(data["kalman_states.oy"])
        self.radio_rssi =  float(data["radio.rssi"])
        self.radio_rssi_inter =  float(data["radio.rssi_inter"])
        self.stabilizer_yaw =  float(data["stabilizer.yaw"])
        self.gradientbug_rssi_angle=float(data["gradientbug.rssi_angle"]);
        self.gradientbug_state=int(data["gradientbug.state"]);
	self.gradientbug_state_wf = int(data["gradientbug.state_wf"])
	self.gradientbug_rssi_beacon = int(data["gradientbug.rssi_beacon"])
	self.gradientbug_rssi_i= int(data["gradientbug.rssi_i"])
	self.gradientbug_rssi_angle_i= float(data["gradientbug.rssi_angle_i"])
        #self.height=float(data["stateEstimate.z"]);

    def param_keepflying_callback(self,name, value):
        self.keep_flying = int(value)
        print(name, value)

    # Call back
    def crazyFlieLoop(self, radio_id, drone_id):

	# All uneven drones are assigned the channel below
        #if drone_id % 2 == 0:
	channel = drone_id*10
	#else:
 	#    channel = (drone_id-1)*10

	# The complete uri of the crazyfly
        if drone_id < 10:
        	URI = 'radio://'+str(radio_id)+'/'+str(channel)+'/2M/E7E7E7E70'+str(drone_id)
        else:
        	URI = 'radio://'+str(radio_id)+'/'+str(channel)+'/2M/E7E7E7E7'+str(drone_id)
        
        print("Connect to ", URI)

        # Only output errors from the logging framework
        logging.basicConfig(level=logging.ERROR)

        # Initialize the low-level drivers (don't list the debug drivers)
        cflib.crtp.init_drivers(enable_debug_driver=False)

        # Open up the logging file, seperated by files per date
        datestr = "../../experiments/"+time.strftime("%y%m%d")
        if os.path.exists(datestr)==False:
            os.mkdir(datestr)

        timestr = time.strftime("%Y%m%d_%H%M%S")
        fh = open(datestr+"/log_"+timestr+"_"+str(drone_id)+".txt", "w")

        print("change crazyradio power")
        cr = crazyradio.Crazyradio(None,radio_id)
        cr.set_power(3)

	#in
        first_run = True
        pos_x_int = 0
        pos_y_int = 0
        rssi_array = []
        rssi_heading_array = []

	stop_looping  = False

        start_time = 0;
	outbound = 1;
        reconnect =True

	while reconnect:
		try:
                        cf = Crazyflie(rw_cache='./cache')
                        cf.param.add_update_callback(group="gbug", name="keep_flying",cb=self.param_keepflying_callback)
			lg_states = LogConfig(name='kalman_states', period_in_ms=100)

			lg_states.add_variable('kalman_states.ox')
			lg_states.add_variable('kalman_states.oy')
			lg_states.add_variable('radio.rssi')
			lg_states.add_variable('radio.rssi_inter')
			lg_states.add_variable('stabilizer.yaw')
			lg_states.add_variable('gradientbug.rssi_angle')
			lg_states.add_variable('gradientbug.state')
			lg_states.add_variable('gradientbug.state_wf')
			lg_states.add_variable('gradientbug.rssi_beacon')
			lg_states.add_variable('gradientbug.rssi_angle_i')
			lg_states.add_variable('gradientbug.rssi_i')
		       # lg_states.add_variable('stateEstimate.z')

			lg_states.data_received_cb.add_callback(self.data_received)
			time.sleep(drone_id)
			with SyncCrazyflie(URI, cf=cf) as scf:
			    with SyncLogger(scf, lg_states) as logger_states:
                                    reconnect = False

				    time.sleep(2)
				    start_time = time.time()

	

				    while 1:
				        #print self.gradientbug_state
					#print self.gradientbug_rssi_angle
					#print("RSSI BEACON", self.gradientbug_rssi_beacon)
					#print("rssi angle",self.gradientbug_rssi_angle)
                                        time.sleep(0.03)
					print("time",time.time())
					print("RSSI", self.radio_rssi)
					print("RSSI _inter", self.radio_rssi_inter)
				        
				        #param_name = "gbug.keep_flying"
				        #param_value = "0"
				        #cf.param.set_value(param_name, param_value)

				        if first_run:
				            pos_x_int = self.kalman_states_ox
				            pos_y_int = self.kalman_states_oy
				            first_run = False
				        self.kalman_states_ox = self.kalman_states_ox-pos_x_int
				        self.kalman_states_oy = self.kalman_states_oy-pos_y_int

				        if time.time()-start_time>(0)*10 and self.keep_flying==0:
				            param_name = "gbug.keep_flying"
				            param_value = "0"
				            cf.param.set_value(param_name, param_value)
                                            cf.param.request_param_update("gbug.keep_flying")
					    print "don't fly!"

				        

				     
				        for event in pygame.event.get():
				            if event.type == KEYDOWN:
				                stop_looping = True
				                break

				        pygame.event.pump()
				        if stop_looping:
				            break
		 




			    param_name = "gbug.keep_flying"
			    param_value = "0"
			    cf.param.set_value(param_name, param_value)

			    param_name = "gbug.keep_flying"
			    param_value = "0"
			    cf.param.set_value(param_name, param_value)
			    param_name = "gbug.keep_flying"
			    param_value = "0"
			    cf.param.set_value(param_name, param_value)
			    print "demo over"
			
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
     
