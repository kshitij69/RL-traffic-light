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
def generate_routefile():
    random.seed(42)  # make tests reproducible
    
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
    o2r = 4 #orange to red delay time
    r2g = 2 #red to green delay time
    A_4235 = 0
    B_4235 = 1
    C_4235 = 2
    AB1_4235 = 3
    AB2_4235 = 4
    AC1_4235 = 5
    AC2_4235 = 6
    BA1_4235 = 7
    BA2_4235 = 8
    BC1_4235 = 9
    BC2_4235 = 10
    CA1_4235 = 11
    CA2_4235 = 12
    CB1_4235 = 13
    CB2_4235 = 14
    A_4219 = 0
    B_4219 = 1
    C_4219 = 2
    D_4219 = 3
    E_4219 = 4
    F_4219 = 5
    G_4219 = 6
    AB1_4219 = 7
    AB2_4219 = 8
    AC1_4219 = 9
    AC2_4219 = 10
    AD1_4219 = 11
    AD2_4219 = 12
    AE1_4219 = 13
    AE2_4219 = 14
    AF1_4219 = 16
    AF2_4219 = 17
    AG1_4219 = 18
    AG2_4219 = 19
    BA1_4219 = 20
    BA2_4219 = 21
    BC1_4219 = 22
    BC2_4219 = 23
    BD1_4219 = 24
    BD2_4219 = 25
    BE1_4219 = 26
    BE2_4219 = 27
    BF1_4219 = 28
    BF2_4219 = 29
    BG1_4219 = 30
    BG2_4219 = 31
    CA1_4219 = 32
    CA2_4219 = 33
    CB1_4219 = 34
    CB2_4219 = 35
    CD1_4219 = 36
    CD2_4219 = 37
    CE1_4219 = 38
    CE2_4219 = 39
    CF1_4219 = 40
    CF2_4219 = 41
    CG1_4219 = 42
    CG2_4219 = 43
    DA1_4219 = 44
    DA2_4219 = 45
    DB1_4219 = 46
    DB2_4219 = 47
    DC1_4219 = 48
    DC2_4219 = 49
    DE1_4219 = 50
    DE2_4219 = 51
    DF1_4219 = 52
    DF2_4219 = 53
    DG1_4219 = 54
    DG2_4219 = 55
    EA1_4219 = 56
    EA2_4219 = 57
    EB1_4219 = 58
    EB2_4219 = 59
    EC1_4219 = 60
    EC2_4219 = 61
    ED1_4219 = 62
    ED2_4219 = 63
    EF1_4219 = 64
    EF2_4219 = 65
    EG1_4219 = 66
    EG2_4219 = 67
    FA1_4219 = 68
    FA2_4219 = 69
    FB1_4219 = 70
    FB2_4219 = 71
    FC1_4219 = 72
    FC2_4219 = 73
    FD1_4219 = 74
    FD2_4219 = 75
    FE1_4219 = 76
    FE2_4219 = 77
    FG1_4219 = 78
    FG2_4219 = 79
    GA1_4219 = 80
    GA2_4219 = 81
    GB1_4219 = 82
    GB2_4219 = 83
    GC1_4219 = 84
    GC2_4219 = 85
    GD1_4219 = 86
    GD2_4219 = 87
    GE1_4219 = 88
    GE2_4219 = 89
    GF1_4219 = 90
    GF2_4219 = 91
    A_4220 = 0
    B_4220 = 1
    C_4220 = 2
    D_4220 = 3
    E_4220 = 4
    AB1_4220 = 5
    AB2_4220 = 6
    AC1_4220 = 7
    AC2_4220 = 8
    AD1_4220 = 9
    AD2_4220 = 10
    AE1_4220 = 11
    AE2_4220 = 12
    BA1_4220 = 13
    BA2_4220 = 14
    BC1_4220 = 15
    BC2_4220 = 16
    BD1_4220 = 17
    BD2_4220 = 18
    BE1_4220 = 19
    BE2_4220 = 20
    CA1_4220 = 21
    CA2_4220 = 22
    CB1_4220 = 23
    CB2_4220 = 24
    CD1_4220 = 25
    CD2_4220 = 26
    CE1_4220 = 27
    CE2_4220 = 28
    DA1_4220 = 29
    DA2_4220 = 30
    DB1_4220 = 31
    DB2_4220 = 32
    DC1_4220 = 33
    DC2_4220 = 34
    DE1_4220 = 35
    DE2_4220 = 36
    EA1_4220 = 37
    EA2_4220 = 38
    EB1_4220 = 39
    EB2_4220 = 40
    EC1_4220 = 41
    EC2_4220 = 42
    ED1_4220 = 43
    ED2_4220 = 44
    A_4221 = 0
    B_4221 = 1
    C_4221 = 2
    D_4221 = 3
    E_4221 = 4
    F_4221 = 5
    AB1_4221 = 6
    AB2_4221 = 7
    AC1_4221 = 8
    AC2_4221 = 9
    AD1_4221 = 10
    AD2_4221 = 11
    AE1_4221 = 12
    AE2_4221 = 13
    AF1_4221 = 14
    AF2_4221 = 15
    BA1_4221 = 16
    BA2_4221 = 17
    BC1_4221 = 18
    BC2_4221 = 19
    BD1_4221 = 20
    BD2_4221 = 21
    BE1_4221 = 22
    BE2_4221 = 23
    BF1_4221 = 24
    BF2_4221 = 25
    CA1_4221 = 26
    CA2_4221 = 27
    CB1_4221 = 28
    CB2_4221 = 29
    CD1_4221 = 30
    CD2_4221 = 31
    CE1_4221 = 32
    CE2_4221 = 33
    CF1_4221 = 34
    CF2_4221 = 35
    DA1_4221 = 36
    DA2_4221 = 37
    DB1_4221 = 38
    DB2_4221 = 39
    DC1_4221 = 40
    DC2_4221 = 41
    DE1_4221 = 42
    DE2_4221 = 43
    DF1_4221 = 44
    DF2_4221 = 45
    EA1_4221 = 46
    EA2_4221 = 47
    EB1_4221 = 48
    EB2_4221 = 49
    EC1_4221 = 50
    EC2_4221 = 51
    ED1_4221 = 52
    ED2_4221 = 53
    EF1_4221 = 54
    EF2_4221 = 55
    FA1_4221 = 56
    FA2_4221 = 57
    FB1_4221 = 58
    FB2_4221 = 59
    FC1_4221 = 60
    FC2_4221 = 61
    FD1_4221 = 62
    FD2_4221 = 63
    FE1_4221 = 64
    FE2_4221 = 65
    
    #while traci.simulation.getMinExpectedNumber() > 0:
    while step < 600:
        traci.simulationStep()
        if step == 0:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",6)
        if step == 6:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 10:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 12:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", A_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",75)
        if step == 87:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", AB1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 91:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", AB2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 93:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",15)
        if step == 108:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BC1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 112:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BC2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 114:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", C_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",12)
        if step == 126:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", CB1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 130:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", CB2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 132:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",32)
        if step == 164:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 168:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 170:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", A_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",50)
        if step == 220:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", AB1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 224:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", AB2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 226:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",15)
        if step == 241:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BC1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 245:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BC2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 247:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", C_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",14)
        if step == 261:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", CB1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 265:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", CB2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 267:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",13)
        if step == 280:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 284:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 286:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", A_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",74)
        if step == 360:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", AB1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 364:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", AB2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 366:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",15)
        if step == 381:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BC1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 385:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BC2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 387:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", C_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",10)
        if step == 397:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", CB1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 401:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", CB2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 403:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",16)
        if step == 419:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 423:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 425:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", A_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",74)
        if step == 499:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", AB1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 503:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", AB2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 505:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",15)
        if step == 520:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BC1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 524:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BC2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 526:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", C_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",8)
        if step == 534:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", CB1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 538:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", CB2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 540:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", B_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",16)
        if step == 556:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA1_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",o2r)
        if step == 560:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", BA2_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",r2g)
        if step == 562:
            traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", A_4235)
            traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577",38)
        if step == 0:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 6)
        if step == 6:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 10:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 12:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 20)
        if step == 32:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 36:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 38:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
        if step == 43:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 47:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 49:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 31)
        if step == 80:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", FG1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 84:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", FG2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 86:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 12)
        if step == 98:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", GA1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 102:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", GA2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 104:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 29)
        if step == 133:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 137:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 139:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 20)
        if step == 159:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 163:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 165:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
        if step == 170:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 174:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 176:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 38)
        if step == 214:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", FG1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 218:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", FG2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 220:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 13)
        if step == 233:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", GA1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 237:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", GA2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 239:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 34)
        if step == 273:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 277:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 279:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 22)
        if step == 301:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 305:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 307:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
        if step == 312:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 316:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 318:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 31)
        if step == 349:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", FG1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 353:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", FG2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 355:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 18)
        if step == 373:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", GA1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 377:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", GA2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 379:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 36)
        if step == 415:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 419:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 421:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 19)
        if step == 440:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 444:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 446:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
        if step == 451:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 455:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 457:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 29)
        if step == 486:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", FG1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 490:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", FG2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 492:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 18)
        if step == 510:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", GA1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 514:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", GA2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 516:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 41)
        if step == 557:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 561:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", AD2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 563:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 19)
        if step == 582:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 586:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", DE2_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", r2g)
        if step == 588:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 8)
        if step == 596:
            traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", EF1_4219)
            traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", o2r)
        if step == 0:
            traci.trafficlight.setPhase("gneJ41", D_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 17)
        if step == 17:
            traci.trafficlight.setPhase("gneJ41", DE1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 21:
            traci.trafficlight.setPhase("gneJ41", DE2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 23:
            traci.trafficlight.setPhase("gneJ41", E_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 9)
        if step == 32:
            traci.trafficlight.setPhase("gneJ41", EA1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 36:
            traci.trafficlight.setPhase("gneJ41", EA2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 38:
            traci.trafficlight.setPhase("gneJ41", A_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 67)
        if step == 105:
            traci.trafficlight.setPhase("gneJ41", AB1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 109:
            traci.trafficlight.setPhase("gneJ41", AB2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 111:
            traci.trafficlight.setPhase("gneJ41", B_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 19)
        if step == 130:
            traci.trafficlight.setPhase("gneJ41", BD1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 134:
            traci.trafficlight.setPhase("gneJ41", BD2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 136:
            traci.trafficlight.setPhase("gneJ41", D_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 16)
        if step == 152:
            traci.trafficlight.setPhase("gneJ41", DE1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 156:
            traci.trafficlight.setPhase("gneJ41", DE2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 158:
            traci.trafficlight.setPhase("gneJ41", E_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 11)
        if step == 169:
            traci.trafficlight.setPhase("gneJ41", EA1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 173:
            traci.trafficlight.setPhase("gneJ41", EA2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 175:
            traci.trafficlight.setPhase("gneJ41", A_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 63)
        if step == 238:
            traci.trafficlight.setPhase("gneJ41", AD1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 242:
            traci.trafficlight.setPhase("gneJ41", AD2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 244:
            traci.trafficlight.setPhase("gneJ41", D_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 13)
        if step == 257:
            traci.trafficlight.setPhase("gneJ41", DE1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 261:
            traci.trafficlight.setPhase("gneJ41", DE2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 263:
            traci.trafficlight.setPhase("gneJ41", E_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 9)
        if step == 272:
            traci.trafficlight.setPhase("gneJ41", EA1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 276:
            traci.trafficlight.setPhase("gneJ41", EA2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 278:
            traci.trafficlight.setPhase("gneJ41", A_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 95)
        if step == 373:
            traci.trafficlight.setPhase("gneJ41", AB1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 377:
            traci.trafficlight.setPhase("gneJ41", AB2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 379:
            traci.trafficlight.setPhase("gneJ41", B_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 19)
        if step == 398:
            traci.trafficlight.setPhase("gneJ41", BD1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 402:
            traci.trafficlight.setPhase("gneJ41", BD2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 404:
            traci.trafficlight.setPhase("gneJ41", D_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 24)
        if step == 428:
            traci.trafficlight.setPhase("gneJ41", DE1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 432:
            traci.trafficlight.setPhase("gneJ41", DE2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 434:
            traci.trafficlight.setPhase("gneJ41", E_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 13)
        if step == 447:
            traci.trafficlight.setPhase("gneJ41", EA1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 451:
            traci.trafficlight.setPhase("gneJ41", EA2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 453:
            traci.trafficlight.setPhase("gneJ41", A_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 56)
        if step == 509:
            traci.trafficlight.setPhase("gneJ41", AB1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 513:
            traci.trafficlight.setPhase("gneJ41", AB2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 515:
            traci.trafficlight.setPhase("gneJ41", B_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 19)
        if step == 534:
            traci.trafficlight.setPhase("gneJ41", BD1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 538:
            traci.trafficlight.setPhase("gneJ41", BD2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 540:
            traci.trafficlight.setPhase("gneJ41", D_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 22)
        if step == 562:
            traci.trafficlight.setPhase("gneJ41", DE1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 566:
            traci.trafficlight.setPhase("gneJ41", DE2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 568:
            traci.trafficlight.setPhase("gneJ41", E_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 20)
        if step == 588:
            traci.trafficlight.setPhase("gneJ41", EA1_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", o2r)
        if step == 592:
            traci.trafficlight.setPhase("gneJ41", EA2_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", r2g)
        if step == 594:
            traci.trafficlight.setPhase("gneJ41", A_4220)
            traci.trafficlight.setPhaseDuration("gneJ41", 6)
        if step == 0:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 9)
        if step == 9:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 13:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 15:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 19)
        if step == 34:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 38:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 40:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 20)
        if step == 60:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 64:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 66:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 24)
        if step == 90:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 94:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 96:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 9)
        if step == 105:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 109:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 111:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 19)
        if step == 130:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 134:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 136:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 30)
        if step == 166:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 170:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 172:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 28)
        if step == 200:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 204:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 206:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 19)
        if step == 225:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 229:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 231:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 8)
        if step == 239:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 243:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 245:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 32)
        if step == 277:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 281:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 283:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 27)
        if step == 310:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 314:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 316:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 15)
        if step == 331:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 335:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 337:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 14)
        if step == 351:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 355:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 357:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 22)
        if step == 379:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 383:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 385:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 24)
        if step == 409:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 413:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 415:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 11)
        if step == 426:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 430:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 432:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 14)
        if step == 446:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 450:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 452:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 30)
        if step == 482:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 486:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 488:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 26)
        if step == 514:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 518:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 520:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 8)
        if step == 528:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 532:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", EF2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 534:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 18)
        if step == 552:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 556:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", FA2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 558:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 19)
        if step == 577:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 581:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", AD2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 583:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 9)
        if step == 592:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE1_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", o2r)
        if step == 596:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", DE2_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", r2g)
        if step == 598:
            traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
            traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 2)

        step += 1

    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "simulation/osm.sumocfg",
                             "--tripinfo-output", "output/tripinfo.xml"])
    run()
