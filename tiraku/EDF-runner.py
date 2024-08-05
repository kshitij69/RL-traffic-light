#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2020 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

from __future__ import absolute_import
from __future__ import print_function

import numpy as np
import os
import sys
import optparse
import random
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

# needs work
def generate_routefile(seedNo):
    random.seed(int(seedNo))  # make tests reproducible
    
    # demand per second from different directions
    # TODO: refactor all routes into 1 route matrix (noOfRoutes x noOfTimesteps) using numpy 
    routesList = [ # route name, route edges, vehicle density at interval i, i+1, i+2...
        ("se4235", "619523290#0 619523290#1 619523288 61645695 628341438#0 628341438#1", 32., 23.),
        ("nw4235", "-628341438#1 -628341438#0 gneE6 619523287#0.17 619523287#1", 19., 19.),
        ("north4219", "620377515  122089525 122089525.34 620377496 620377502#0 620377502#1 620377501#0", 64., 79.),
        ("south4219", "620377506 620377506.61 28597169 620377494 547625725", 50., 41.),
        ("south4220", "gneE10.441 gneE73 gneE79 gneE41", 33., 37.),
        ("north4221", "-620463733 -620377511 -112543664 28573008 547793367 620377512 28573048 669665014 669665015#1 28594060#0 28594060#1 28594060#2 28594060#3 28594060#4", 84., 64.),
        ("south4221", "-28594060#4 -28594060#3 -28594060#2 -28594060#1 -28594060#0 669665013 669665011 7635630#0 7635630#1 207149704#0 207149704#1 112543664 620377510 620463733", 31., 17.),
        ("ne4221", "-620463733 -620377511 -112543664 28573008 547793367 620377512 7637388#0 7637388#1 28573015#0 28573015#1 28573015#2 28573015#3 28573015#4 28573015#5 28573015#6", 5., 4.),
        ("se4221", "-28594060#4 -28594060#3 -28594060#2 -28594060#1 -28594060#0 669665013 669665011 7635630#0 28572986 7637388#1 28573015#0 28573015#1 28573015#2 28573015#3 28573015#4 28573015#5 28573015#6", 2., 1.),
        ("nw4221", "-28573015#6 -28573015#5 -28573015#4 -28573015#3 -28573015#2 -28573015#1 -28573015#0 28573025 28573025.46 28573025.65 28573048 669665014 669665015#1 28594060#0 28594060#1 28594060#2 28594060#3 28594060#4", 0, 4.),
        ("4235e4219n", "619523290#0 619523290#1 619523288 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 gneE1 gneE1.31 620377502#0 620377502#1 620377501#0", 99., 90.),
        ("4235n4219n", "-628341438#1 -628341438#0 -61645695 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 gneE1 gneE1.31 620377502#0 620377502#1 620377501#0", 6., 5.),
        ("4235e4219s", "619523290#0 619523290#1 619523288 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 28597169 620377494 547625725", 30., 24.),
        ("4235n4219s", "-628341438#1 -628341438#0 -61645695 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 28597169 620377494 547625725", 2., 1.),
        
        ("4235e4220n", "619523290#0 619523290#1 619523288 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE3 gneE44", 4., 19.),
        ("4235n4220n", "-628341438#1 -628341438#0 -61645695 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE3 gneE44", 0, 1.),
        ("4235e4220s", "619523290#0 619523290#1 619523288 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE74 gneE79 gneE41", 4., 3.),
        ("4235n4220s", "-628341438#1 -628341438#0 -61645695 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE74 gneE79 gneE41", 0., 0.),
        ("4219s4220n", "620377506 gneE2 620377517 gneE3 gneE44", 1., 4.),
        ("4219s4220s", "620377506 gneE2 620377517 gneE74 gneE79 gneE41", 0., 1.),
        ("4219n4220n", "620377515  122089525 122089525.34 122089528#1 620377517 gneE3 gneE44", 0., 3.),
        ("4219n4220s", "620377515  122089525 122089525.34 122089528#1 620377517 gneE74 gneE79 gneE41", 0., 0.),
        
        ("4235e4221n", "619523290#0 619523290#1 619523288 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE83 669665014 669665015#1 28594060#0 28594060#1 28594060#2 28594060#3 28594060#4", 30., 33.),
        ("4235n4221n", "-628341438#1 -628341438#0 -61645695 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE83 669665014 669665015#1 28594060#0 28594060#1 28594060#2 28594060#3 28594060#4", 2., 2.),
        ("4235e4221s", "619523290#0 619523290#1 619523288 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE82 207149704#0 207149704#1 112543664 620377510 620463733", 7., 19.),
        ("4235n4221s", "-628341438#1 -628341438#0 -61645695 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE82 207149704#0 207149704#1 112543664 620377510 620463733", 0., 1.),
        ("4219s4221n", "620377506 gneE2 620377517 gneE74 gneE72 gneE80 gneE81 gneE83 669665014 669665015#1 28594060#0 28594060#1 28594060#2 28594060#3 28594060#4", 6., 6.),
        ("4219s4221s", "620377506 gneE2 620377517 gneE74 gneE72 gneE80 gneE81 gneE82 207149704#0 207149704#1 112543664 620377510 620463733", 1., 4.),
        ("4219n4221n", "620377515  122089525 122089525.34 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE83 669665014 669665015#1 28594060#0 28594060#1 28594060#2 28594060#3 28594060#4", 2., 5.),
        ("4219n4221s", "620377515  122089525 122089525.34 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE82 207149704#0 207149704#1 112543664 620377510 620463733", 0., 3.),
        ("4220s4221n", "gneE10.441 28641508 gneE80 gneE81 gneE83 669665014 669665015#1 28594060#0 28594060#1 28594060#2 28594060#3 28594060#4", 4., 5.),
        ("4220s4221s", "gneE10.441 28641508 gneE80 gneE81 gneE82 207149704#0 207149704#1 112543664 620377510 620463733", 1., 3.),
        
        ("4235e4221e", "619523290#0 619523290#1 619523288 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE82 7637388#0 7637388#1 28573015#0 28573015#1 28573015#2 28573015#3 28573015#4 28573015#5 28573015#6", 47., 18.),
        ("4235n4221e", "-628341438#1 -628341438#0 -61645695 619526570#0 619526570#1 620377514#1 619526569#1 619526568 619526567 619526566 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE82 7637388#0 7637388#1 28573015#0 28573015#1 28573015#2 28573015#3 28573015#4 28573015#5 28573015#6", 3., 1.),
        ("4219s4221e", "620377506 gneE2 620377517  gneE74 gneE72 gneE80 gneE81 gneE82 7637388#0 7637388#1 28573015#0 28573015#1 28573015#2 28573015#3 28573015#4 28573015#5 28573015#6", 9., 4.),
        ("4219n4221e", "620377515  122089525 122089525.34 122089528#1 620377517 gneE74 gneE72 gneE80 gneE81 gneE82 7637388#0 7637388#1 28573015#0 28573015#1 28573015#2 28573015#3 28573015#4 28573015#5 28573015#6", 3., 3.),
        ("4220s4221e", "gneE10.441 28641508 gneE80 gneE81 gneE82 7637388#0 7637388#1 28573015#0 28573015#1 28573015#2 28573015#3 28573015#4 28573015#5 28573015#6", 7., 3.),
        
        ("4221s4220n", "-28594060#4 -28594060#3 -28594060#2 -28594060#1 -28594060#0 669665013 669665011 7635630#0 7635630#1 624439627 547793365 158617319 gneE77 gneE76 gneE44", 1., 2.),
        ("4221n4220n", "-620463733 -620377511 -112543664 28573008 28573013 547793365 158617319 gneE77 gneE76 gneE44", 3., 4.),
        ("4221w4220n", "-28573015#6 -28573015#5 -28573015#4 -28573015#3 -28573015#2 -28573015#1 -28573015#0 28573025 28573025.46 28573025.65 624439627 547793365 158617319 gneE77 gneE76 gneE44", 1., 1.),
        ("4221s4220s", "-28594060#4 -28594060#3 -28594060#2 -28594060#1 -28594060#0 669665013 669665011 7635630#0 7635630#1 624439627 547793365 158617319 620377513 gneE41", 3., 2.),
        ("4221n4220s", "-620463733 -620377511 -112543664 28573008 28573013 547793365 158617319 620377513 gneE41", 5., 6.),
        ("4221w4220s", "-28573015#6 -28573015#5 -28573015#4 -28573015#3 -28573015#2 -28573015#1 -28573015#0 28573025 28573025.46 28573025.65 624439627 547793365 158617319 620377513 gneE41", 2., 1.),
        
        ("4220s4219n", "gneE10.441 gneE73 gneE75 619526565 547625727 620377496 620377502#0 620377502#1 620377501#0", 22., 13.),
        ("4220n4219n", "gneE47 gneE47.434 gneE78 gneE75 619526565 547625727 620377496 620377502#0 620377502#1 620377501#0", 8., 9.), 
        ("4221s4219n", "-28594060#4 -28594060#3 -28594060#2 -28594060#1 -28594060#0 669665013 669665011 7635630#0 7635630#1 624439627 547793365 158617319 gneE77 gneE75 619526565 547625727 620377496 620377502#0 620377502#1 620377501#0", 6., 4.),
        ("4221n4219n", "-620463733 -620377511 -112543664 28573008 28573013 547793365 158617319 gneE77 gneE75 619526565 547625727 620377496 620377502#0 620377502#1 620377501#0", 11., 10.),
        ("4221w4219n", "-28573015#6 -28573015#5 -28573015#4 -28573015#3 -28573015#2 -28573015#1 -28573015#0 28573025 28573025.46 28573025.65 624439627 547793365 158617319 gneE77 gneE75 619526565 547625727 620377496 620377502#0 620377502#1 620377501#0", 4., 2.),
        
        ("4220s4219s", "gneE10.441 gneE73 gneE75 619526565 28592678 620377494 547625725", 3., 4.),
        ("4220n4219s", "gneE47 gneE47.434 gneE78 gneE75 619526565 28592678 620377494 547625725", 1., 3.),  
        ("4221s4219s", "-28594060#4 -28594060#3 -28594060#2 -28594060#1 -28594060#0 669665013 669665011 7635630#0 7635630#1 624439627 547793365 158617319 gneE77 gneE75 619526565 28592678 620377494 547625725", 1., 1.),
        ("4221n4219s", "-620463733 -620377511 -112543664 28573008 28573013 547793365 158617319 gneE77 gneE75 619526565 28592678 620377494 547625725", 2., 3.),
        ("4221w4219s", "-28573015#6 -28573015#5 -28573015#4 -28573015#3 -28573015#2 -28573015#1 -28573015#0 28573025 28573025.46 28573025.65 624439627 547793365 158617319 gneE77 gneE75 619526565 28592678 620377494 547625725", 1., 1.),
        
        ("4219s4235s", "620377506 620377506.61 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 620377490 628341438#0 628341438#1", 1., 0.),
        ("4219n4235s", "620377515 28592666 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 620377490 628341438#0 628341438#1", 1., 0.),
        ("4221s4235s", "-28594060#4 -28594060#3 -28594060#2 -28594060#1 -28594060#0 669665013 669665011 7635630#0 7635630#1 624439627 547793365 158617319 gneE77 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 620377490 628341438#0 628341438#1", 0., 0.),
        ("4221n4235s", "-620463733 -620377511 -112543664 28573008 28573013 547793365 158617319 gneE77 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 620377490 628341438#0 628341438#1", 0., 0.),
        ("4221w4235s", "-28573015#6 -28573015#5 -28573015#4 -28573015#3 -28573015#2 -28573015#1 -28573015#0 28573025 28573025.46 28573025.65 624439627 547793365 158617319 gneE77 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 620377490 628341438#0 628341438#1", 0., 0.),
        ("4220s4235s", "gneE10.441 gneE73 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 620377490 628341438#0 628341438#1", 1., 0.),
        ("4220n4235s", "gneE47 gneE47.434 gneE78 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 620377490 628341438#0 628341438#1", 0., 0.),
        
        ("4219s4235w", "620377506 620377506.61 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 619523286 619523287#0 619523287#0.17 619523287#1", 13., 16.),
        ("4219n4235w", "620377515 28592666 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 619523286 619523287#0 619523287#0.17 619523287#1", 5., 3.),
        ("4221s4235w", "-28594060#4 -28594060#3 -28594060#2 -28594060#1 -28594060#0 669665013 669665011 7635630#0 7635630#1 624439627 547793365 158617319 gneE77 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 619523286 619523287#0 619523287#0.17 619523287#1", 3., 3.),
        ("4221n4235w", "-620463733 -620377511 -112543664 28573008 28573013 547793365 158617319 gneE77 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 619523286 619523287#0 619523287#0.17 619523287#1", 6., 8.),
        ("4221w4235w", "-28573015#6 -28573015#5 -28573015#4 -28573015#3 -28573015#2 -28573015#1 -28573015#0 28573025 28573025.46 28573025.65 624439627 547793365 158617319 gneE77 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 619523286 619523287#0 619523287#0.17 619523287#1", 2., 2.),
        ("4220s4235w", "gneE10.441 gneE73 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 619523286 619523287#0 619523287#0.17 619523287#1", 12., 10.),
        ("4220n4235w", "gneE47 gneE47.434 gneE78 gneE75 619526565 547625727 122089526 122089583#0 122089583#1 122089583#1-AddedOffRampEdge 619523286 619523287#0 619523287#0.17 619523287#1", 4., 7.)
    
    ]
    
    
    with open("simulation/osm.rou.xml", "w") as routes: 
        print("""<routes>
        <vType id="type1" lcStrategic="100" lcKeepRight="100" lcSpeedGain="100" lcCooperative="1" lcSublane="100" />""", file=routes)

        for route in routesList:                                                                # print route list       
                print('    <route id="r%s" edges="%s" />' % (
                    route[0], route[1]), file=routes)

        vehNr = 0
        N = 300  # number of time steps per interval (5 min)
        for i in range(2):
            for j in range(N):
                for route in routesList:                                                        #print vehicle density for each interval for each route                   
                    if random.uniform(0, 1) < (route[i+2]/300):#type currently does not work
                        print('    <vehicle id="%s_%d" type="type1" route="r%s" depart="%d" />' % (
                            route[0], vehNr, route[0], (i*N)+j), file=routes)
                        vehNr += 1
                        
                    
        print("</routes>", file=routes)
                

# needs work
def run():
    """execute the TraCI control loop"""
    step = 0
    YELLOW_PHASE = 4
    RED_PHASE = 2
    
    site4235_detector_delay = []     
    site4235_LOOP_COUNT = 11 
    for i in range(0, site4235_LOOP_COUNT+1):                                                   # initiliaze array of 0s
        site4235_detector_delay.append(0)
    lanesForPhases_s4235 = {                                                                    # key = phase, values = lanes with green lights
        0 : [1, 2, 3, 9, 10, 11],                                                               #A      
        1 : [1, 2, 3, 4],                                                                       #B
        2 : [6, 7]                                                                              #C
    }   
    
    detector_delay_s4219 = []     
    site4219_LOOP_COUNT = 21
    ignored_phases_s4219 = [20, 21, 22]# could automate this I guess? if not in dict TODO
    for i in range(0, site4219_LOOP_COUNT+1): # initiliaze array of 0s
        detector_delay_s4219.append(0)
    lanesForGroupsForPhases_s4219 = {        # key = phase, values = lanes with green lights
        0 : [0, 1],                     #A    
        1 : [1, 6, 7],                  #B
        2 : [0, 2],                     #C
        3 : [3, 8],                     #D
        4 : [3, 4, 7],                  #E
        5 : [4, 5, 7],                  #F
        6 : [6, 2, 7]                   #G
    }   

    lanesForGroups_s4219 = {                # which lanes are grouped together, this is useful for finding when a lane group is no longer active
        0 : [11, 12],                   # such that, a right turn lane can be replaced by oncoming traffic 
        1 : [1, 2],         
        2 : [13, 14],           
        3 : [15, 16, 17],       
        4 : [7, 8],   
        5 : [9, 10],        
        6 : [3, 4],
        7 : [5, 6],
        8 : [18, 19]        
    }
    conflictingGroups_s4219 = {# TODO automate this from lanesForGroupsForPhases_s4219 (necessary?)
        0 : [3, 4, 5, 6, 7, 8],
        1 : [2, 3, 4, 5, 8],
        2 : [1, 3, 4, 5, 8],
        3 : [0, 1, 2, 3, 5, 6],
        4 : [0, 1, 2, 6, 8],
        5 : [0, 1, 2, 3, 6, 8],
        6 : [0, 3, 4, 5, 8],
        7 : [1, 8],
        8 : [1, 2, 4, 5, 6, 7]
    }
    earliestDeadline_s4219 = 0
    earliestDeadlineGroup_s4219 = 0
    waitTime_s4219 = 0
    activeTraffic_s4219 = []
    currentActivePhase_s4219 = 0
    for i in range(0, len(lanesForGroups_s4219)): # initiliaze array of 99s
        activeTraffic_s4219.append(99)
        
    detector_delay_s4220 = []     
    site4220_LOOP_COUNT = 15
    ignored_phases_s4220 = [3, 8, 11]# could automate this I guess? if not in dict TODO
    for i in range(0, site4220_LOOP_COUNT+1): # initiliaze array of -1s
        detector_delay_s4220.append(0)
    lanesForGroupsForPhases_s4220 = {        # key = phase, values = lanes with green lights
        0 : [1, 5],                     #A    
        1 : [5, 6, 0],                  #B
        2 : [1, 2],                     #C
        3 : [3, 4],                     #D
        4 : [2, 6]                      #E
    }   

    lanesForGroups_s4220 = {                # which lanes are grouped together, this is useful for finding when a lane group is no longer active
        0 : [1, 2],
        1 : [4, 5, 6],
        2 : [7],
        3 : [9],
        4 : [10],
        5 : [12, 13, 14],
        6 : [15]
    }
    conflictingGroups_s4220 = {# TODO automate this from lanesForGroupsForPhases_s4220 (necessary?)
        0 : [1, 4],
        1 : [0, 6, 4, 3],
        2 : [5, 4, 3],
        3 : [0, 1, 2, 5, 6],
        4 : [0, 1, 2, 5, 6],
        5 : [2, 3, 4],
        6 : [2, 3, 4, 5]
    }
    earliestDeadline_s4220 = 0
    earliestDeadlineGroup_s4220 = 0
    waitTime_s4220 = 0
    activeTraffic_s4220 = []
    for i in range(0, len(lanesForGroups_s4220)): # initiliaze array of 99s
        activeTraffic_s4220.append(99)
    next_phase_s4220 = 0
    
    detector_delay_s4221 = []     
    site4221_LOOP_COUNT = 18
    ignored_phases_s4221 = [6, 10, 14]# could automate this I guess? if not in dict TODO
    for i in range(0, site4221_LOOP_COUNT+1): # initiliaze array of -1s
        detector_delay_s4221.append(0)
    lanesForGroupsForPhases_s4221 = {        # key = phase, values = lanes with green lights
        0 : [3, 7],                     #A    
        1 : [3, 4],                     #B
        2 : [7, 8, 0],                     #C
        3 : [1, 2, 0],                     #D
        4 : [5, 6],                      #E
        5 : [4, 8, 0]                   #f
    }   

    lanesForGroups_s4221 = {                # which lanes are grouped together, this is useful for finding when a lane group is no longer active
        0 : [1, 2],
        1 : [3],
        2 : [4, 5],
        3 : [7, 8],
        4 : [9],
        5 : [11, 12],
        6 : [13],
        7 : [15, 16],
        8 : [17, 18]
    }
    conflictingGroups_s4221 = {# TODO automate this from lanesForGroupsForPhases_s4221 (necessary?)
        0 : [3, 6],
        1 : [3, 4, 6, 7, 8],
        2 : [3, 4, 5, 7, 8],
        3 : [0, 1, 2, 5, 6, 8],
        4 : [1, 2, 5, 6, 7],
        5 : [2, 3, 4, 7, 8],
        6 : [0, 1, 2, 3, 4, 7, 8],
        7 : [1, 2, 4, 5, 6],
        8 : [1, 2, 3, 5, 6]
    }
    earliestDeadline_s4221 = 0
    earliestDeadlineGroup_s4221 = 0
    waitTime_s4221 = 0
    activeTraffic_s4221 = []
    for i in range(0, len(lanesForGroups_s4221)): # initiliaze array of 99s
        activeTraffic_s4221.append(99)
    next_phase_s4221 = 0
    
    MINIMUM_TRAFFIC_ACTIVITY = 3
    traci.trafficlight.setPhase("5861321343", 0)
    traci.trafficlight.setPhase("313863797", 0) 
    
    while step <= 600:
    #while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # add in other traffic lights
        #static edf algo
        # ----------------------------------------------------------------------SITE 4221--------------------------------------------------------------------------------------------------
        phase_s4221 = traci.trafficlight.getPhase("cluster_25953432_313863435_313863521_314053282")     # phase indexing starts at 0 

        if (phase_s4221 in lanesForGroupsForPhases_s4221.keys()):                                           # if not a transition phase
            waitTime_s4221 += 1                                                                             # current phase wait time
            transitionCounter_s4221 = 0                                                                     # purely for transition phases
                        
            
            for i in range(0, len(lanesForGroups_s4221)):
                if i in lanesForGroupsForPhases_s4221[phase_s4221]: 
                    groupActivity = []
                    for lane in lanesForGroups_s4221[i]:
                        groupActivity.append(int(traci.inductionloop.getTimeSinceDetection("site4221_" + str(lane))))
                        detector_delay_s4221[lane] = 0
                        activeTraffic_s4221[i] = min(groupActivity)
                else:
                    for lane in lanesForGroups_s4221[i]:
                        if int(traci.inductionloop.getTimeSinceDetection("site4221_" + str(lane))) == 0:      # if getLastStepVehicleNumber>0, 
                            detector_delay_s4221[lane] = detector_delay_s4221[lane] + 1                     # increment loopDetectorDelay
                        
         
            earliestDeadline_s4221 = detector_delay_s4221.index(max(detector_delay_s4221))
            
            switchPhase_s4221 = True 
            for group in lanesForGroups_s4221:                                                            # find which group the EDF lane belongs to
                if earliestDeadline_s4221 in lanesForGroups_s4221[group]:
                    earliestDeadlineGroup_s4221 = group
                    
            
            for group in lanesForGroupsForPhases_s4221[phase_s4221]:                            # do not switch phase if any conflicting lane has traffic, all inactive groups automatically are 99
                if group != 0:
                    if activeTraffic_s4221[group] < MINIMUM_TRAFFIC_ACTIVITY:
                        switchPhase_s4221 = False
                   
            
            #if earliestDeadlineGroup_s4221 in lanesForGroupsForPhases_s4221[phase_s4221]:
            #    switchPhase_s4221 = False
            if earliestDeadline_s4221 == 0:
                switchPhase_s4221 = False 
                
            if waitTime_s4221 == 180:#TODO: revise waiting time
                switchPhase_s4221 = True    
                
            if switchPhase_s4221 == True:
                transitionCounter = 0
                # build new phase
                secondDeadline = -1
                secondaryDeadlineGroup_s4221 = earliestDeadlineGroup_s4221
                for i in range(0,len(lanesForGroupsForPhases_s4221)):                                                 # find the secondary group to create new phase
                    if earliestDeadlineGroup_s4221 in lanesForGroupsForPhases_s4221[i]:
                        for group in lanesForGroupsForPhases_s4221[i]:
                            if group != earliestDeadlineGroup_s4221:
                                
                                for lane in lanesForGroups_s4221[group]:
                                    if detector_delay_s4221[lane] >= secondDeadline:
                                        secondDeadline = detector_delay_s4221[lane]
                                        secondaryDeadlineGroup_s4221 = group
                
                for i in range(0,len(lanesForGroupsForPhases_s4221)):                                                 # find the [hase containing both groups
                    if earliestDeadlineGroup_s4221 in lanesForGroupsForPhases_s4221[i]:
                        if secondaryDeadlineGroup_s4221 in lanesForGroupsForPhases_s4221[i]:
                            next_phase_s4221 = i
                            
                prev_phase_s4221 = phase_s4221
                transition = chr(65+phase_s4221) + "-" + chr(65+next_phase_s4221) + "-1"                         # eg. A-C-1
                if phase_s4221 != next_phase_s4221:
                    
                    
                    logics = traci.trafficlight.getAllProgramLogics("cluster_25953432_313863435_313863521_314053282") 
                    names = str(logics[0].getPhases()).split("name='")
                    for i in range(1,len(names)):
                        
                        if str(names[i].split("'")[0]) == str(transition):
                            transitionID = i-1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", transitionID)              #change to transition
                    phase_s4221 = traci.trafficlight.getPhase("cluster_25953432_313863435_313863521_314053282")              
                    
                    
                    leftTurn_phase = traci.trafficlight.getPhase("313863797")
                    if (next_phase_s4221 == 0 or next_phase_s4221 == 1 or next_phase_s4221 == 4) and leftTurn_phase == 0: # if traffic is turning left and should not in next phase, switch to red light
                        traci.trafficlight.setPhase("313863797", 1) 
                    elif (next_phase_s4221 == 2 or next_phase_s4221 == 3 or next_phase_s4221 == 5) and leftTurn_phase == 2: # if traffic can turn left, switch to green light
                        traci.trafficlight.setPhase("313863797", 3)
                         
        else:                                                                                                   #   if active phase is transition phase
            switchPhase_s4221 = False                                                                               
            waitTime_s4221 = 0
            for i in range(0, len(lanesForGroups_s4221)):
                for lane in lanesForGroups_s4221[i]:
                    if int(traci.inductionloop.getTimeSinceDetection("site4221_" + str(lane))) == 0:      # if getLastStepVehicleNumber>0, 
                        detector_delay_s4221[lane] = detector_delay_s4221[lane] + 1                     # increment loopDetectorDelay
                        
            
            if transitionCounter_s4221 == YELLOW_PHASE + RED_PHASE - 1:
                traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", next_phase_s4221) #change to next phase
            transitionCounter_s4221 = transitionCounter_s4221 + 1
            
        # ----------------------------------------------------------------------SITE 4220--------------------------------------------------------------------------------------------------
        phase_s4220 = traci.trafficlight.getPhase("gneJ41")     # phase indexing starts at 0 

        if (phase_s4220 in lanesForGroupsForPhases_s4220.keys()):                                    # if not a transition phase
            waitTime_s4220 += 1                                                                             # current phase wait time
            transitionCounter_s4220 = 0
                        
            for i in range(0, len(lanesForGroups_s4220)):
                if i in lanesForGroupsForPhases_s4220[phase_s4220]: 
                    groupActivity = []
                    for lane in lanesForGroups_s4220[i]:
                        groupActivity.append(int(traci.inductionloop.getTimeSinceDetection("site4220_" + str(lane))))
                        detector_delay_s4220[lane] = 0
                    activeTraffic_s4220[i] = min(groupActivity)
                else:
                    for lane in lanesForGroups_s4220[i]:
                        if int(traci.inductionloop.getTimeSinceDetection("site4220_" + str(lane))) == 0:      # if getLastStepVehicleNumber>0, 
                            detector_delay_s4220[lane] = detector_delay_s4220[lane] + 1                     # increment loopDetectorDelay
         
            earliestDeadline_s4220 = detector_delay_s4220.index(max(detector_delay_s4220))
            
            switchPhase_s4220 = True 
            for group in lanesForGroups_s4220:                                                            # find which group the EDF lane belongs to
                if earliestDeadline_s4220 in lanesForGroups_s4220[group]:
                    earliestDeadlineGroup_s4220 = group
                    
            for group in lanesForGroupsForPhases_s4220[phase_s4220]:                            # do not switch phase if any conflicting lane has traffic, all inactive groups automatically are 99
                if activeTraffic_s4220[group] < MINIMUM_TRAFFIC_ACTIVITY:
                    switchPhase_s4220 = False
            
            #if earliestDeadlineGroup_s4220 in lanesForGroupsForPhases_s4220[phase_s4220]:
            #    switchPhase_s4220 = False
            if earliestDeadline_s4220 == 0:
                switchPhase_s4220 = False 
                
            if waitTime_s4220 == 180:
                switchPhase_s4220 = True    
                
            if switchPhase_s4220 == True:
                transitionCounter = 0
                # build new phase
                secondDeadline = -1
                secondaryDeadlineGroup_s4220 = earliestDeadlineGroup_s4220
                for i in range(0,len(lanesForGroupsForPhases_s4220)):                                                 # find the secondary group to create new phase
                    if earliestDeadlineGroup_s4220 in lanesForGroupsForPhases_s4220[i]:
                        for group in lanesForGroupsForPhases_s4220[i]:
                            if group != earliestDeadlineGroup_s4220:
                                for lane in lanesForGroups_s4220[group]:
                                    if detector_delay_s4220[lane] >= secondDeadline:
                                        secondDeadline = detector_delay_s4220[lane]
                                        secondaryDeadlineGroup_s4220 = group
                                        
                for i in range(0,len(lanesForGroupsForPhases_s4220)):                                                 # find the [hase containing both groups
                    if earliestDeadlineGroup_s4220 in lanesForGroupsForPhases_s4220[i]:
                        if secondaryDeadlineGroup_s4220 in lanesForGroupsForPhases_s4220[i]:
                            next_phase_s4220 = i
                            
                prev_phase_s4220 = phase_s4220
                transition = chr(65+phase_s4220) + "-" + chr(65+next_phase_s4220) + "-1"                         # eg. A-C-1
                if phase_s4220 != next_phase_s4220:
                    
                    
                    logics = traci.trafficlight.getAllProgramLogics("gneJ41") 
                    names = str(logics[0].getPhases()).split("name='")
                    for i in range(1,len(names)):
                        
                        if str(names[i].split("'")[0]) == str(transition):
                            transitionID = i-1
                    traci.trafficlight.setPhase("gneJ41", transitionID)              #change to transition
                    
                    for group in lanesForGroups_s4220:                                                            # find which group the EDF lane belongs to
                        if next_phase_s4220 in lanesForGroups_s4220[group]:
                            next_group_s4220 = group
                         
        else:                                                                                                   #   if active phase is transition phase
            switchPhase_s4220 = False                                                                               
            waitTime_s4220 = 0
            for group in lanesForGroupsForPhases_s4220[prev_phase_s4220]:                                 # reset the traffic activity level of each active phase group
                    activeTraffic_s4220[group] = 99
            if transitionCounter_s4220 < YELLOW_PHASE:           # while lights are still yellow
                switchPhase_s4220 = False
                #for lane in lanesForGroups_s4220[group]:   
                 #   detector_delay_s4220[lane] = 0  
            elif transitionCounter_s4220 == YELLOW_PHASE + RED_PHASE - 1:
                traci.trafficlight.setPhase("gneJ41", next_phase_s4220) #change to next phase
            transitionCounter_s4220 = transitionCounter_s4220 + 1
            
        # ----------------------------------------------------------------------SITE 4219--------------------------------------------------------------------------------------------------
        phase_s4219 = traci.trafficlight.getPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525")     # phase indexing starts at 0 
        
        if (phase_s4219 in lanesForGroupsForPhases_s4219.keys()):                                           # if not a transition phase
            waitTime_s4219 += 1                                                                             # current phase wait time
            transitionCounter_s4219 = 0                                                                     # purely for transition phases
                        
            
            for i in range(0, len(lanesForGroups_s4219)):
                if i in lanesForGroupsForPhases_s4219[currentActivePhase_s4219]: 
                    groupActivity = []
                    for lane in lanesForGroups_s4219[i]:
                        groupActivity.append(int(traci.inductionloop.getTimeSinceDetection("site4219_" + str(lane))))
                        detector_delay_s4219[lane] = 0
                    activeTraffic_s4219[i] = min(groupActivity)
                else:
                    for lane in lanesForGroups_s4219[i]:
                        if int(traci.inductionloop.getTimeSinceDetection("site4219_" + str(lane))) == 0:      # if getLastStepVehicleNumber>0, 
                            detector_delay_s4219[lane] = detector_delay_s4219[lane] + 1                     # increment loopDetectorDelay
                        
            
            
            earliestDeadline_s4219 = detector_delay_s4219.index(max(detector_delay_s4219))
            
                    
            switchPhase_s4219 = True 
            for group in lanesForGroups_s4219:                                                            # find which group the EDF lane belongs to
                if earliestDeadline_s4219 in lanesForGroups_s4219[group]:
                    earliestDeadlineGroup_s4219 = group
                    
            
            for group in lanesForGroupsForPhases_s4219[phase_s4219]:                            # do not switch phase if any conflicting lane has traffic, all inactive groups automatically are 99
                if group != 7:
                    if activeTraffic_s4219[group] < MINIMUM_TRAFFIC_ACTIVITY:
                        switchPhase_s4219 = False
                   
            
            #if earliestDeadlineGroup_s4219 in lanesForGroupsForPhases_s4219[phase_s4219]:
            #    switchPhase_s4219 = False
            if earliestDeadline_s4219 == 0:
                switchPhase_s4219 = False 
                
            if waitTime_s4219 == 180:#TODO: revise waiting time
                switchPhase_s4219 = True    
                
            if switchPhase_s4219 == True:
                transitionCounter = 0
                # build new phase
                secondDeadline = -1
                secondaryDeadlineGroup_s4219 = earliestDeadlineGroup_s4219
                for i in range(0,len(lanesForGroupsForPhases_s4219)):                                                 # find the secondary group to create new phase
                    if earliestDeadlineGroup_s4219 in lanesForGroupsForPhases_s4219[i]:
                        for group in lanesForGroupsForPhases_s4219[i]:
                            if group != earliestDeadlineGroup_s4219:
                                
                                for lane in lanesForGroups_s4219[group]:
                                    if detector_delay_s4219[lane] >= secondDeadline:
                                        secondDeadline = detector_delay_s4219[lane]
                                        secondaryDeadlineGroup_s4219 = group
                
                for i in range(0,len(lanesForGroupsForPhases_s4219)):                                                 # find the [hase containing both groups
                    if earliestDeadlineGroup_s4219 in lanesForGroupsForPhases_s4219[i]:
                        if secondaryDeadlineGroup_s4219 in lanesForGroupsForPhases_s4219[i]:
                            next_phase_s4219 = i
                            
                prev_phase_s4219 = phase_s4219
                transition = chr(65+phase_s4219) + "-" + chr(65+next_phase_s4219) + "-1"                         # eg. A-C-1
                if phase_s4219 != next_phase_s4219:
                    
                    
                    logics = traci.trafficlight.getAllProgramLogics("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525") 
                    names = str(logics[0].getPhases()).split("name='")
                    for i in range(1,len(names)):
                        
                        if str(names[i].split("'")[0]) == str(transition):
                            transitionID = i-1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", transitionID)              #change to transition
                    phase_s4219 = traci.trafficlight.getPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525")              
                    
                    
                    leftTurn_phase = traci.trafficlight.getPhase("5861321343")
                    if (next_phase_s4219 == 0 or next_phase_s4219 == 2 or next_phase_s4219 == 3) and leftTurn_phase == 0: # if traffic is turning left and should not in next phase, switch to red light
                        traci.trafficlight.setPhase("5861321343", 1) 
                    elif (next_phase_s4219 == 1 or next_phase_s4219 == 4 or next_phase_s4219 == 5 or next_phase_s4219 == 6) and leftTurn_phase == 2: # if traffic can turn left, switch to green light
                        traci.trafficlight.setPhase("5861321343", 3)
                         
        else:                                                                                                   #   if active phase is transition phase
            switchPhase_s4219 = False                                                                               
            waitTime_s4219 = 0
            for i in range(0, len(lanesForGroups_s4219)):
                for lane in lanesForGroups_s4219[i]:
                    if int(traci.inductionloop.getTimeSinceDetection("site4219_" + str(lane))) == 0:      # if getLastStepVehicleNumber>0, 
                        detector_delay_s4219[lane] = detector_delay_s4219[lane] + 1                     # increment loopDetectorDelay
                        
            
            if transitionCounter_s4219 == YELLOW_PHASE + RED_PHASE - 1:
                currentActivePhase_s4219 = next_phase_s4219
                traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", next_phase_s4219) #change to next phase
            transitionCounter_s4219 = transitionCounter_s4219 + 1
        
        
        
        
        # ----------------------------------------------------------------------SITE 4235--------------------------------------------------------------------------------------------------
        site4235_phase = traci.trafficlight.getPhase("cluster_1707799581_314056954_5931861577") # phase indexing starts at 0 
        for i in range(1,site4235_LOOP_COUNT):                                                  # for each loop detector
            if (i != 5) and (i != 8):                                                           # ignore lanes 5 and 8
                if int(traci.inductionloop.getLastStepVehicleNumber("site4235_" + str(i))) > 0 or site4235_detector_delay[i] > 0:      # if getLastStepVehicleNumber>0, 
                    site4235_detector_delay[i] = site4235_detector_delay[i] + 1                 # increment loopDetectorDelay
                
        if (site4235_phase in lanesForPhases_s4235.keys()):                 # if not a transition phase
            activeTraffic = 99 
            
            for i in lanesForPhases_s4235[site4235_phase]:
                site4235_detector_delay[i] = 0                                                  # reset loopDetectorDelay for loops in current phase
                if i != 1 and i != 2 and i != 3:                                                # ignore non conflicting traffic
                    if int(traci.inductionloop.getTimeSinceDetection("site4235_" + str(i))) < activeTraffic:
                        activeTraffic = int(traci.inductionloop.getTimeSinceDetection("site4235_" + str(i)))
                             
            if activeTraffic > MINIMUM_TRAFFIC_ACTIVITY:                                                              # if no traffic through active lanes, switch to transition phase for max(loopDetectorDelay)
                activeTraffic = 0 
                transitionCounter = 0
                site4235_prev_phase = site4235_phase
                earliestDeadline = site4235_detector_delay.index(max(site4235_detector_delay))
                # build next phase
                if earliestDeadline == 9 or earliestDeadline == 10 or earliestDeadline == 11:#this can be changed with lanesForGroups_s4235 (TODO)
                    site4235_next_phase = 0                                                     #A
                    if site4235_phase == 1:
                        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 7)   #change to B-A
                    elif site4235_phase == 2:
                        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 11)   #change to C-A
                
                elif earliestDeadline == 4:          # loop detector no. 4
                    site4235_next_phase = 1                                                     #B
                    if site4235_phase == 0:
                        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 3)   #change to A-B
                    elif site4235_phase == 2:
                        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 13)   #change to C-B
                        
                elif earliestDeadline == 6 or earliestDeadline == 7:
                    site4235_next_phase = 2                                                     #C
                    if site4235_phase == 0:
                        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 5)   #change to A-C
                    elif site4235_phase == 1:
                        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 9)   #change to B-C
                        
                elif earliestDeadline == 1 or earliestDeadline == 2 or earliestDeadline == 3:
                    if site4235_detector_delay[4] > max(site4235_detector_delay[9:11]):         # compare conflicting lanes
                        site4235_next_phase = 1                                                     #B
                        if site4235_phase == 0:
                            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 3)   #change to A-B
                        elif site4235_phase == 2:
                            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 13)   #change to C-B
                        
                    else:
                        site4235_next_phase = 0                                                     #A
                        if site4235_phase == 1:
                            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 7)   #change to B-A
                        elif site4235_phase == 2:
                            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 11)   #change to C-A
                            
        else:                                                                                       #   if active phase is transition phase
            
            if transitionCounter < YELLOW_PHASE:           # while lights are still yellow
                for i in lanesForPhases_s4235[site4235_prev_phase]:
                    site4235_detector_delay[i] = 0  
            elif transitionCounter == YELLOW_PHASE + RED_PHASE - 1:
                traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", site4235_next_phase) #change to next phase
            transitionCounter = transitionCounter + 1
                #add for case where lights change with no traffic?
        step += 1

    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=True, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    seedNo = sys.argv[1]
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile(seedNo)

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "simulation/osm.sumocfg",
                             "--tripinfo-output", "output/tripinfo.xml", "--no-internal-links", "--summary", "output/summary.xml"])
    run()
