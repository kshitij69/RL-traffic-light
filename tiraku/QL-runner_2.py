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
    
    
    with open("data/osm.rou.xml", "w") as routes: 
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

    step = 0
    cycle_4235 = 0
    Duration_4235 = 0
    
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
    
    cycle_4219 = 0
    Duration_4219 = 0
    
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
    AF1_4219 = 15
    AF2_4219 = 16
    AG1_4219 = 17
    AG2_4219 = 18
    BA1_4219 = 19
    BA2_4219 = 20
    BC1_4219 = 21
    BC2_4219 = 22
    BD1_4219 = 23
    BD2_4219 = 24
    BE1_4219 = 25
    BE2_4219 = 26
    BF1_4219 = 27
    BF2_4219 = 28
    BG1_4219 = 29
    BG2_4219 = 30
    CA1_4219 = 31
    CA2_4219 = 32
    CB1_4219 = 33
    CB2_4219 = 34
    CD1_4219 = 35
    CD2_4219 = 36
    CE1_4219 = 37
    CE2_4219 = 38
    CF1_4219 = 39
    CF2_4219 = 40
    CG1_4219 = 41
    CG2_4219 = 42
    DA1_4219 = 43
    DA2_4219 = 44
    DB1_4219 = 45
    DB2_4219 = 46
    DC1_4219 = 47
    DC2_4219 = 48
    DE1_4219 = 49
    DE2_4219 = 50
    DF1_4219 = 51
    DF2_4219 = 52
    DG1_4219 = 53
    DG2_4219 = 54
    EA1_4219 = 55
    EA2_4219 = 56
    EB1_4219 = 57
    EB2_4219 = 58
    EC1_4219 = 59
    EC2_4219 = 60
    ED1_4219 = 61
    ED2_4219 = 62
    EF1_4219 = 63
    EF2_4219 = 64
    EG1_4219 = 65
    EG2_4219 = 66
    FA1_4219 = 67
    FA2_4219 = 68
    FB1_4219 = 69
    FB2_4219 = 70
    FC1_4219 = 71
    FC2_4219 = 72
    FD1_4219 = 73
    FD2_4219 = 74
    FE1_4219 = 75
    FE2_4219 = 76
    FG1_4219 = 77
    FG2_4219 = 78
    GA1_4219 = 79
    GA2_4219 = 80
    GB1_4219 = 81
    GB2_4219 = 82
    GC1_4219 = 83
    GC2_4219 = 84
    GD1_4219 = 85
    GD2_4219 = 86
    GE1_4219 = 87
    GE2_4219 = 88
    GF1_4219 = 89
    GF2_4219 = 90
    
    cycle_4220 = 0
    Duration_4220 = 0
    
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
    
    cycle_4221 = 0
    Duration_4221 = 0
    
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
    
    transition_dictionary4235 = {
        'A-A' : 0,
        'A-B' : 3,
        'A-C' : 5,
        'B-A' : 7,
        'B-B' : 1,
        'B-C' : 9,
        'C-A' : 11,
        'C-B' : 13,
        'C-C' : 2
    }
    
    transition_dictionary4219 = {
        'A-A' : 0,
        'A-B' : 7,
        'A-C' : 9,
        'A-D' : 11,
        'A-E' : 13,
        'A-F' : 15,
        'A-G' : 17,
        'B-A' : 19,
        'B-B' : 1,
        'B-C' : 21,
        'B-D' : 23,
        'B-E' : 25,
        'B-F' : 27,
        'B-G' : 29,
        'C-A' : 31,
        'C-B' : 33,
        'C-C' : 2,
        'C-D' : 35,
        'C-E' : 37,
        'C-F' : 39,
        'C-G' : 41,
        'D-A' : 43,
        'D-B' : 45,
        'D-C' : 47,
        'D-D' : 3,
        'D-E' : 49,
        'D-F' : 51,
        'D-G' : 53,
        'E-A' : 55,
        'E-B' : 57,
        'E-C' : 59,
        'E-D' : 61,
        'E-E' : 4,
        'E-F' : 63,
        'E-G' : 65,
        'F-A' : 67,
        'F-B' : 69,
        'F-C' : 71,
        'F-D' : 73,
        'F-E' : 75,
        'F-F' : 5,
        'F-G' : 77,
        'G-A' : 79,
        'G-B' : 81,
        'G-C' : 83,
        'G-D' : 85,
        'G-E' : 87,
        'G-F' : 89,
        'G-G' : 6
    }
    
    transition_dictionary4220 = {
        'A-A' : 0,
        'A-B' : 5,
        'A-C' : 7,
        'A-D' : 9,
        'A-E' : 11,
        'B-A' : 13,
        'B-B' : 1,
        'B-C' : 15,
        'B-D' : 17,
        'B-E' : 19,
        'C-A' : 21,
        'C-B' : 23,
        'C-C' : 2,
        'C-D' : 25,
        'C-E' : 27,
        'D-A' : 29,
        'D-B' : 31,
        'D-C' : 33,
        'D-D' : 3,
        'D-E' : 35,
        'E-A' : 37,
        'E-B' : 39,
        'E-C' : 41,
        'E-D' : 43,
        'E-E' : 4
    }
    
    transition_dictionary4221 = {
        'A-A' : 0,
        'A-B' : 6,
        'A-C' : 8,
        'A-D' : 10,
        'A-E' : 12,
        'A-F' : 14,
        'B-A' : 16,
        'B-B' : 1,
        'B-C' : 18,
        'B-D' : 20,
        'B-E' : 22,
        'B-F' : 24,
        'C-A' : 26,
        'C-B' : 28,
        'C-C' : 2,
        'C-D' : 30,
        'C-E' : 32,
        'C-F' : 34,
        'D-A' : 36,
        'D-B' : 38,
        'D-C' : 40,
        'D-D' : 3,
        'D-E' : 42,
        'D-F' : 44,
        'E-A' : 46,
        'E-B' : 48,
        'E-C' : 50,
        'E-D' : 52,
        'E-E' : 4,
        'E-F' : 54,
        'F-A' : 56,
        'F-B' : 58,
        'F-C' : 60,
        'F-D' : 62,
        'F-E' : 64,
        'F-F' : 5
    }
    
    transition_complete_4235 = True
    transition_counter_4235 = 0
    end_phase_4235 = 'A'
    scale_4235 = 1
    offset_4235 = 2
    
    transition_complete_4219 = True
    transition_counter_4219 = 0
    end_phase_4219 = 'A'
    scale_4219 = 1
    offset_4219 = 2
    
    transition_complete_4220 = True
    transition_counter_4220 = 0
    end_phase_4220 = 'A'
    scale_4220 = 1
    offset_4220 = 2
    
    transition_complete_4221 = True
    transition_counter_4221 = 0
    end_phase_4221 = 'A'
    scale_4221 = 1
    offset_4221 = 2
    
    while step < 600:
        traci.simulationStep()
        
     
        # ----------------------------------------------------------------------SITE 4219--------------------------------------------------------------------------------------------------
        # Demand Scheduling Algorithm using queue lengths
        # Obtain queue length data from SUMO
        #
        # At intersection iterative through competing traffic flows 
        # - selecting largest first until they are all 0
        # - have maximum time period to switch phase
        #
        # At points where there are secondary traffic flows
        # - select phase with the most density 
        #
        # If else statement selecting Phase with maximum density
        # - Exit loop when all cars pass or time limit is reached
        
        #Phase_A_4235_Density = traci.lane.getLastStepVehicleNumber("619523288_0") + traci.lane.getLastStepVehicleNumber("619523288_1") + traci.lane.getLastStepVehicleNumber("619523288_2") + traci.lane.getLastStepVehicleNumber("619523290#1_0") + traci.lane.getLastStepVehicleNumber("619523290#1_1") + traci.lane.getLastStepVehicleNumber("619523290#1_2") + traci.lane.getLastStepVehicleNumber("619523286_0") + traci.lane.getLastStepVehicleNumber("619523286_1") + traci.lane.getLastStepVehicleNumber("619523286_2") + traci.lane.getLastStepVehicleNumber("122089583#1-AddedOffRampEdge_0") + traci.lane.getLastStepVehicleNumber("122089583#1-AddedOffRampEdge_1") + traci.lane.getLastStepVehicleNumber("122089583#1-AddedOffRampEdge_2")
        #Phase_B_4235_Density = traci.lane.getLastStepVehicleNumber("619523288_0") + traci.lane.getLastStepVehicleNumber("619523288_1") + traci.lane.getLastStepVehicleNumber("619523288_2") + traci.lane.getLastStepVehicleNumber("619523288_3") + traci.lane.getLastStepVehicleNumber("619523290#1_0") + traci.lane.getLastStepVehicleNumber("619523290#1_1") + traci.lane.getLastStepVehicleNumber("619523290#1_2") + traci.lane.getLastStepVehicleNumber("619523290#1_3")
        #Phase_C_4235_Density = traci.lane.getLastStepVehicleNumber("122089583#1-AddedOffRampEdge_0") + traci.lane.getLastStepVehicleNumber("619523286_0") + traci.lane.getLastStepVehicleNumber("gneE6_0") + traci.lane.getLastStepVehicleNumber("-61645695_1") + traci.lane.getLastStepVehicleNumber("-628341438#0_0") + traci.lane.getLastStepVehicleNumber("-628341438#1_0")
        #Density_List_4235 = [Phase_A_4235_Density,Phase_B_4235_Density,Phase_C_4235_Density]
        #print('Phase_A_4235_Density is: ' + str(Phase_A_4235_Density))
        
        #if cycle_4235 == 0:
        
            #if Phase_A_4235_Density == max(Density_List_4235):
            #    cycle_1_4235 = 'A'
            #elif Phase_B_4235_Density == max(Density_List_4235):
            #    cycle_1_4235 = 'B'
            #else:
            #    cycle_1_4235 = 'C'
            #
            #if transition_complete_4235 == True:
            #    cycle_4235 = 1
                
            #    if cycle_1_4235 == 'A':
            #        Duration_4235 += (Phase_A_4235_Density + 5)
            #        transition_complete_4235 = False
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 0)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                 
            #        if Phase_B_4235_Density > Phase_C_4235_Density:
            #            cycle_2_4235 = 'B'
            #            cycle_3_4235 = 'C'
            #            transition_phase_1_2 = AB1_4235
            #            transition_phase_2_3 = BC1_4235
            #        else:
            #            cycle_2_4235 = 'C'
            #            cycle_3_4235 = 'B'
            #            transition_phase_1_2 = AC1_4235
            #            transition_phase_2_3 = CB1_4235
            #            
            #    elif cycle_1_4235 == 'B':
            #        Duration_4235 += (Phase_B_4235_Density + 5)
            #        transition_complete_4235 = False
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 1)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
            #        
            #        if Phase_A_4235_Density > Phase_C_4235_Density:
            #            cycle_2_4235 = 'A'
            #            cycle_3_4235 = 'C'
            #            transition_phase_1_2 = BA1_4235
            #            transition_phase_2_3 = AC1_4235
            #        else:
            #            cycle_2_4235 = 'C'
            #            cycle_3_4235 = 'A'
            #            transition_phase_1_2 = BC1_4235
            #            transition_phase_2_3 = CA1_4235
            #    else:
            #       cycle_1_4235 = 'C'
            #        Duration_4235 += (Phase_C_4235_Density + 5)
            #        transition_complete_4235 = False
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 2)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                   
            #        if Phase_A_4235_Density > Phase_B_4235_Density:
            #            cycle_2_4235 = 'A'
            #            cycle_3_4235 = 'B'
            #            transition_phase_1_2 = CA1_4235
            #            transition_phase_2_3 = AB1_4235
            #        else:
            #            cycle_2_4235 = 'B'
            #            cycle_3_4235 = 'A'
            #            transition_phase_1_2 = CB1_4235
            #            transition_phase_2_3 = BA1_4235
            #else:
            #    if transition_counter_4235 >= 6:
            #        transition_counter_4235 = 0
            #        transition_complete_4235 = True
            #        Duration_4235 += 1
            #    elif transition_counter_4235 == 0:
            #        transition_counter_4235 += 1
            #        Duration_4235 += 1
            #        if cycle_1_4235 == 'A':
            #            if end_cycle_4235 == 'B':
            #                transition_phase_3_1_1 = BA1_4235
            #                transition_phase_3_1_2 = BA2_4235
            #            elif end_cycle_4235 == 'C':
            #                transition_cycle_3_1_1 = CA1_4235
            #                transition_cycle_3_1_2 = CA2_4235
            #            else:
            #                transition_cycle_3_1_1 = A_4235
            #                transition_cycle_3_1_2 = A_4235
            #        elif cycle_1_4235 == 'B':
            #            if end_cycle_4235 == 'A':
            #                transition_phase_3_1_1 = AB1_4235
            #                transition_phase_3_1_2 = AB2_4235
            #            elif end_cycle_4235 == 'C':
            #                transition_cycle_3_1_1 = CB1_4235
            #                transition_cycle_3_1_2 = CB2_4235
            #            else:
            #                transition_cycle_3_1_1 = B_4235
            #                transition_cycle_3_1_2 = B_4235
            #        else:
            #            if end_cycle_4235 == 'A':
            #                transition_phase_3_1_1 = AC1_4235
            #                transition_phase_3_1_2 = AC2_4235
            #            elif end_cycle_4235 == 'B':
            #                transition_cycle_3_1_1 = BC1_4235
            #                transition_cycle_3_1_2 = BC2_4235
            #            else:
            #                transition_cycle_3_1_1 = C_4235
            #                transition_cycle_3_1_2 = C_4235
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", transition_phase_3_1_1)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 10)
            #    elif transition_counter_4235 == 4:
            #        transition_counter_4235 += 1
            #        Duration_4235 += 1
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", transition_phase_3_1_2)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 5)
            #    else:
            #        transition_counter_4235 += 1
            #        Duration_4235 += 1
            #            
       # elif cycle_4235 == 1 and step == Duration_4235:
            #if transition_complete_4235 == True:
            #    cycle_4235 = 2
            #    if cycle_2_4235 == 'A':
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 0)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
            #        Duration_4235 += (Phase_A_4235_Density + 5)
            #        transition_complete_4235 = False
            #    elif cycle_2_4235 == 'B':
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 1)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
            #        Duration_4235 += (Phase_B_4235_Density + 5)
            #        transition_complete_4235 = False
            #    else:
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 2)
           #         traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
            #        Duration_4235 += (Phase_C_4235_Density + 5)
            #        transition_complete_4235 = False
            #else:
            #    if transition_counter_4235 >= 6:
            #        transition_counter_4235 = 0
            #        transition_complete_4235 = True
            #        Duration_4235 += 1
            #    elif transition_counter_4235 == 0:
            #        transition_counter_4235 += 1
            #        Duration_4235 += 1
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", transition_phase_1_2)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 6)   
            #    else:
            #        transition_counter_4235 += 1
            #        Duration_4235 += 1
        #elif cycle_4235 == 2 and step == Duration_4235:
            #if transition_complete_4235 == True:
            #    cycle_4235 = 3
            #    if cycle_3_4235 == 'A':
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 0)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
            #        Duration_4235 += (Phase_A_4235_Density + 5)
            #        transition_complete_4235 = False
            #        end_cycle_4235 = 'A'
            #    elif cycle_3_4235 == 'B':
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 1)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
            #        Duration_4235 += (Phase_B_4235_Density + 5)
            #        transition_complete_4235 = False
            #        end_cycle_4235 = 'B'
            #    else:
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 2)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
            #        Duration_4235 += (Phase_C_4235_Density + 5)
            #        transition_complete_4235 = False
            #        end_cycle_4235 = 'C'
            #else:
            #    if transition_counter_4235 >= 6:
            #        transition_counter_4235 = 0
            #        transition_complete_4235 = True
            #        Duration_4235 += 1
            #    elif transition_counter_4235 == 0:
            #        transition_counter_4235 += 1
            #        Duration_4235 += 1
            #        traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", transition_phase_2_3)
            #        traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 6)   
            #    else:
            #        transition_counter_4235 += 1
            #        Duration_4235 += 1
        #elif cycle_4235 == 3 and step == Duration_4235:
            #cycle_4235 = 0
        
        Phase_A_4235_Density = traci.lane.getLastStepVehicleNumber("619523288_0") + traci.lane.getLastStepVehicleNumber("619523288_1") + traci.lane.getLastStepVehicleNumber("619523288_2") + traci.lane.getLastStepVehicleNumber("619523290#1_0") + traci.lane.getLastStepVehicleNumber("619523290#1_1") + traci.lane.getLastStepVehicleNumber("619523290#1_2") + traci.lane.getLastStepVehicleNumber("619523286_0") + traci.lane.getLastStepVehicleNumber("619523286_1") + traci.lane.getLastStepVehicleNumber("619523286_2") + traci.lane.getLastStepVehicleNumber("122089583#1-AddedOffRampEdge_0") + traci.lane.getLastStepVehicleNumber("122089583#1-AddedOffRampEdge_1") + traci.lane.getLastStepVehicleNumber("122089583#1-AddedOffRampEdge_2")
        Phase_B_4235_Density = traci.lane.getLastStepVehicleNumber("619523288_0") + traci.lane.getLastStepVehicleNumber("619523288_1") + traci.lane.getLastStepVehicleNumber("619523288_2") + traci.lane.getLastStepVehicleNumber("619523288_3") + traci.lane.getLastStepVehicleNumber("619523290#1_0") + traci.lane.getLastStepVehicleNumber("619523290#1_1") + traci.lane.getLastStepVehicleNumber("619523290#1_2") + traci.lane.getLastStepVehicleNumber("619523290#1_3")
        Phase_C_4235_Density = traci.lane.getLastStepVehicleNumber("122089583#1-AddedOffRampEdge_0") + traci.lane.getLastStepVehicleNumber("619523286_0") + traci.lane.getLastStepVehicleNumber("gneE6_0") + traci.lane.getLastStepVehicleNumber("-61645695_1") + traci.lane.getLastStepVehicleNumber("-628341438#0_0") + traci.lane.getLastStepVehicleNumber("-628341438#1_0")
        Density_4235 = {
            'A' : Phase_A_4235_Density,
            'B' : Phase_B_4235_Density,
            'C' : Phase_C_4235_Density
        }
        
        sorted_density_4235 = sorted(Density_4235.items(),key=lambda x:x[1], reverse=True)
        
        
        print(transition_counter_4235)
        print(cycle_4235)
        
        if cycle_4235 == 0:
        
            cycle_4235 = 1
            
            cycle_1_4235 = sorted_density_4235[0][0]
            cycle_2_4235 = sorted_density_4235[1][0]
            cycle_3_4235 = sorted_density_4235[2][0]
            transition_cycle_4235_0 = transition_dictionary4235[end_phase_4235 + '-' + cycle_1_4235]
            transition_cycle_4235_1 = transition_dictionary4235[cycle_1_4235 + '-' + cycle_2_4235]
            transition_cycle_4235_2 = transition_dictionary4235[cycle_2_4235 + '-' + cycle_3_4235]
            
            
            
               
            
            if transition_complete_4235 == True:

        
                
                if cycle_1_4235 == 'A':
                    Duration_4235 += round(Phase_A_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 0)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    print('Cycle 1: A')
                elif cycle_1_4235 == 'B':
                    Duration_4235 += round(Phase_B_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 1)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    print('Cycle 1: B')
                else:
                    cycle_1_4235 = 'C'
                    Duration_4235 += round(Phase_C_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 2)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    print('Cycle 1: C')
                   
            else:
                if transition_counter_4235 >= 6:
                    transition_counter_4235 = 0
                    transition_complete_4235 = True
                    Duration_4235 += 1
                elif transition_counter_4235 == 0:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", transition_cycle_4235_0)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 10)
                    
                elif transition_counter_4235 == 4:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", (transition_cycle_4235_0+1))
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 5)
                else:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
                        
        elif cycle_4235 == 1 and step == Duration_4235:
            if transition_complete_4235 == True:
                cycle_4235 = 2
                if cycle_2_4235 == 'A':
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 0)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    Duration_4235 += round(Phase_A_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    print('Cycle 2: A')
                elif cycle_2_4235 == 'B':
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 1)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    Duration_4235 += round(Phase_B_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    print('Cycle 2: B')
                else:
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 2)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    Duration_4235 += round(Phase_C_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    print('Cycle 2: C')
            else:
                if transition_counter_4235 >= 6:
                    transition_counter_4235 = 0
                    transition_complete_4235 = True
                    Duration_4235 += 1
                elif transition_counter_4235 == 0:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", transition_cycle_4235_1)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 10)
                    
                elif transition_counter_4235 == 4:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", (transition_cycle_4235_1+1))
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 5)
                else:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
        elif cycle_4235 == 2 and step == Duration_4235:
            if transition_complete_4235 == True:
                cycle_4235 = 3
                if cycle_3_4235 == 'A':
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 0)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    Duration_4235 += round(Phase_A_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    end_phase_4235 = 'A'
                    print('Cycle 3: A')
                elif cycle_3_4235 == 'B':
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 1)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    Duration_4235 += round(Phase_B_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    end_phase_4235 = 'B'
                    print('Cycle 3: B')
                else:
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", 2)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 60)
                    Duration_4235 += round(Phase_C_4235_Density * scale_4235 + offset_4235)
                    transition_complete_4235 = False
                    end_phase_4235 = 'C'
                    print('Cycle 3: C')
            else:
                if transition_counter_4235 >= 6:
                    transition_counter_4235 = 0
                    transition_complete_4235 = True
                    Duration_4235 += 1
                elif transition_counter_4235 == 0:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", transition_cycle_4235_2)
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 10)
                    
                elif transition_counter_4235 == 4:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
                    traci.trafficlight.setPhase("cluster_1707799581_314056954_5931861577", (transition_cycle_4235_2+1))
                    traci.trafficlight.setPhaseDuration("cluster_1707799581_314056954_5931861577", 5)
                else:
                    transition_counter_4235 += 1
                    Duration_4235 += 1
        elif cycle_4235 == 3 and step == Duration_4235:
            cycle_4235 = 0
            
        Phase_A_4219_Density = traci.lane.getLastStepVehicleNumber("122089525.34_0") + traci.lane.getLastStepVehicleNumber("122089525.34_1") + traci.lane.getLastStepVehicleNumber("620377515_0") + traci.lane.getLastStepVehicleNumber("620377515_1") + traci.lane.getLastStepVehicleNumber("620377506.61_0") + traci.lane.getLastStepVehicleNumber("620377506.61_1") + traci.lane.getLastStepVehicleNumber("620377506_0") + traci.lane.getLastStepVehicleNumber("620377506_1")
        Phase_B_4219_Density = traci.lane.getLastStepVehicleNumber("620377506.61_0") + traci.lane.getLastStepVehicleNumber("620377506.61_1") + traci.lane.getLastStepVehicleNumber("620377506.61_2") + traci.lane.getLastStepVehicleNumber("620377506.61_3") + traci.lane.getLastStepVehicleNumber("620377506_0") + traci.lane.getLastStepVehicleNumber("620377506_1") + traci.lane.getLastStepVehicleNumber("620377506_2")
        Phase_C_4219_Density = traci.lane.getLastStepVehicleNumber("122089525.34_0") + traci.lane.getLastStepVehicleNumber("122089525.34_1") + traci.lane.getLastStepVehicleNumber("122089525.34_2") + traci.lane.getLastStepVehicleNumber("122089525.34_3") + traci.lane.getLastStepVehicleNumber("620377515_0") + traci.lane.getLastStepVehicleNumber("620377515_1") + traci.lane.getLastStepVehicleNumber("620377515_2") + traci.lane.getLastStepVehicleNumber("620377515_3")
        Phase_D_4219_Density = traci.lane.getLastStepVehicleNumber("547625727_0") + traci.lane.getLastStepVehicleNumber("547625727_1") + traci.lane.getLastStepVehicleNumber("547625727_2") + traci.lane.getLastStepVehicleNumber("547625727_3") + traci.lane.getLastStepVehicleNumber("547625727_4") + traci.lane.getLastStepVehicleNumber("619526565_1") + traci.lane.getLastStepVehicleNumber("619526565_2") + traci.lane.getLastStepVehicleNumber("619526565_3") + traci.lane.getLastStepVehicleNumber("619526565_4") + traci.lane.getLastStepVehicleNumber("619526565_5") + traci.lane.getLastStepVehicleNumber("gneE75_0") + traci.lane.getLastStepVehicleNumber("gneE75_1") + traci.lane.getLastStepVehicleNumber("gneE75_2") + traci.lane.getLastStepVehicleNumber("gneE75_3")
        Phase_E_4219_Density = traci.lane.getLastStepVehicleNumber("547625727_0") + traci.lane.getLastStepVehicleNumber("547625727_1") + traci.lane.getLastStepVehicleNumber("547625727_2") + traci.lane.getLastStepVehicleNumber("619526565_1") + traci.lane.getLastStepVehicleNumber("619526565_2") + traci.lane.getLastStepVehicleNumber("619526565_3") + traci.lane.getLastStepVehicleNumber("gneE75_0") + traci.lane.getLastStepVehicleNumber("gneE75_1") + traci.lane.getLastStepVehicleNumber("619526566_0") + traci.lane.getLastStepVehicleNumber("619526566_1") + traci.lane.getLastStepVehicleNumber("619526567_1") + traci.lane.getLastStepVehicleNumber("619526567_2") + traci.lane.getLastStepVehicleNumber("619526568_1")
        Phase_F_4219_Density = traci.lane.getLastStepVehicleNumber("619526566_0") + traci.lane.getLastStepVehicleNumber("619526566_1") + traci.lane.getLastStepVehicleNumber("619526566_2") + traci.lane.getLastStepVehicleNumber("619526566_3") + traci.lane.getLastStepVehicleNumber("619526567_1") + traci.lane.getLastStepVehicleNumber("619526567_2") + traci.lane.getLastStepVehicleNumber("619526567_3") + traci.lane.getLastStepVehicleNumber("619526568_0") + traci.lane.getLastStepVehicleNumber("619526568_1") + traci.lane.getLastStepVehicleNumber("619526568_2")
        Phase_G_4219_Density = traci.lane.getLastStepVehicleNumber("122089525.34_2") + traci.lane.getLastStepVehicleNumber("122089525.34_3") + traci.lane.getLastStepVehicleNumber("620377515_2") + traci.lane.getLastStepVehicleNumber("620377515_3") + traci.lane.getLastStepVehicleNumber("620377506.61_2") + traci.lane.getLastStepVehicleNumber("620377506.61_3") + traci.lane.getLastStepVehicleNumber("620377506_1") + traci.lane.getLastStepVehicleNumber("620377506_2")
        
        Density_4219 = {
            'A' : Phase_A_4219_Density,
            'B' : Phase_B_4219_Density,
            'C' : Phase_C_4219_Density,
            'D' : Phase_D_4219_Density,
            'E' : Phase_E_4219_Density,
            'F' : Phase_F_4219_Density,
            'G' : Phase_G_4219_Density
        }

        sorted_density_4219 = sorted(Density_4219.items(),key=lambda x:x[1], reverse=True)
        
        
        if cycle_4219 == 0:
            cycle_1_4219 = sorted_density_4219[0][0]
            cycle_2_4219 = sorted_density_4219[1][0]
            cycle_3_4219 = sorted_density_4219[2][0]
            cycle_4_4219 = sorted_density_4219[3][0]
            cycle_5_4219 = sorted_density_4219[4][0]
            cycle_6_4219 = sorted_density_4219[5][0]
            cycle_7_4219 = sorted_density_4219[6][0]
            transition_cycle_4219_0 = transition_dictionary4219[end_phase_4219 + '-' + cycle_1_4219]
            transition_cycle_4219_1 = transition_dictionary4219[cycle_1_4219 + '-' + cycle_2_4219]
            transition_cycle_4219_2 = transition_dictionary4219[cycle_2_4219 + '-' + cycle_3_4219]
            transition_cycle_4219_3 = transition_dictionary4219[cycle_3_4219 + '-' + cycle_4_4219]
            transition_cycle_4219_4 = transition_dictionary4219[cycle_4_4219 + '-' + cycle_5_4219]
            transition_cycle_4219_5 = transition_dictionary4219[cycle_5_4219 + '-' + cycle_6_4219]
            transition_cycle_4219_6 = transition_dictionary4219[cycle_6_4219 + '-' + cycle_7_4219]
            
            if transition_complete_4219 == True:
                cycle_4219 = 1
                if cycle_1_4219 == 'A':
                    Duration_4219 += round(Phase_A_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_1_4219 == 'B':
                    Duration_4219 += round(Phase_B_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", B_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_1_4219 == 'C':
                    Duration_4219 += round(Phase_C_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", C_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_1_4219 == 'D':
                    Duration_4219 += round(Phase_D_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_1_4219 == 'E':
                    Duration_4219 += round(Phase_E_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_1_4219 == 'F':
                    Duration_4219 += round(Phase_F_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                else:
                    Duration_4219 += round(Phase_G_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
            else:
                if transition_counter_4219 >= 6:
                    transition_counter_4219 = 0
                    transition_complete_4219 = True
                    Duration_4219 += 1
                elif transition_counter_4219 == 0:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", transition_cycle_4219_0)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 10)
                    
                elif transition_counter_4219 == 4:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", (transition_cycle_4219_0+1))
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
                else:
                    transition_counter_4219 += 1
                    Duration_4219 += 1

        
        elif cycle_4219 == 1 and step == Duration_4219:
            if transition_complete_4219 == True:
                cycle_4219 = 2
                if cycle_2_4219 == 'A':
                    Duration_4219 += round(Phase_A_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_2_4219 == 'B':
                    Duration_4219 += round(Phase_B_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", B_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_2_4219 == 'C':
                    Duration_4219 += round(Phase_C_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", C_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_2_4219 == 'D':
                    Duration_4219 += round(Phase_D_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_2_4219 == 'E':
                    Duration_4219 += round(Phase_E_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_2_4219 == 'F':
                    Duration_4219 += round(Phase_F_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                else:
                    Duration_4219 += round(Phase_G_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
            else:
                if transition_counter_4219 >= 6:
                    transition_counter_4219 = 0
                    transition_complete_4219 = True
                    Duration_4219 += 1
                elif transition_counter_4219 == 0:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", transition_cycle_4219_1)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 10)
                elif transition_counter_4219 == 4:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", (transition_cycle_4219_1+1))
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
                else:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
        elif cycle_4219 == 2 and step == Duration_4219:
            if transition_complete_4219 == True:
                cycle_4219 = 3
                if cycle_3_4219 == 'A':
                    Duration_4219 += round(Phase_A_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_3_4219 == 'B':
                    Duration_4219 += round(Phase_B_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", B_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_3_4219 == 'C':
                    Duration_4219 += round(Phase_C_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", C_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_3_4219 == 'D':
                    Duration_4219 += round(Phase_D_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_3_4219 == 'E':
                    Duration_4219 += round(Phase_E_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_3_4219 == 'F':
                    Duration_4219 += round(Phase_F_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                else:
                    Duration_4219 += round(Phase_G_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
            else:
                if transition_counter_4219 >= 6:
                    transition_counter_4219 = 0
                    transition_complete_4219 = True
                    Duration_4219 += 1
                elif transition_counter_4219 == 0:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", transition_cycle_4219_2)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 10)
                elif transition_counter_4219 == 4:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", (transition_cycle_4219_2+1))
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
                else:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
        elif cycle_4219 == 3 and step == Duration_4219:
            if transition_complete_4219 == True:
                cycle_4219 = 4
                if cycle_4_4219 == 'A':
                    Duration_4219 += round(Phase_A_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_4_4219 == 'B':
                    Duration_4219 += round(Phase_B_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", B_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_4_4219 == 'C':
                    Duration_4219 += round(Phase_C_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", C_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_4_4219 == 'D':
                    Duration_4219 += round(Phase_D_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_4_4219 == 'E':
                    Duration_4219 += round(Phase_E_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_4_4219 == 'F':
                    Duration_4219 += round(Phase_F_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                else:
                    Duration_4219 += round(Phase_G_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
            else:
                if transition_counter_4219 >= 6:
                    transition_counter_4219 = 0
                    transition_complete_4219 = True
                    Duration_4219 += 1
                elif transition_counter_4219 == 0:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", transition_cycle_4219_3)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 10)
                elif transition_counter_4219 == 4:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", (transition_cycle_4219_3+1))
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
                else:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
        elif cycle_4219 == 4 and step == Duration_4219:
            if transition_complete_4219 == True:
                cycle_4219 = 5
                if cycle_5_4219 == 'A':
                    Duration_4219 += round(Phase_A_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_5_4219 == 'B':
                    Duration_4219 += round(Phase_B_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", B_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_5_4219 == 'C':
                    Duration_4219 += round(Phase_C_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", C_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_5_4219 == 'D':
                    Duration_4219 += round(Phase_D_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_5_4219 == 'E':
                    Duration_4219 += round(Phase_E_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_5_4219 == 'F':
                    Duration_4219 += round(Phase_F_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                else:
                    Duration_4219 += round(Phase_G_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
            else:
                if transition_counter_4219 >= 6:
                    transition_counter_4219 = 0
                    transition_complete_4219 = True
                    Duration_4219 += 1
                elif transition_counter_4219 == 0:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", transition_cycle_4219_4)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 10)
                elif transition_counter_4219 == 4:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", (transition_cycle_4219_4+1))
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
                else:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
        elif cycle_4219 == 5 and step == Duration_4219:
            if transition_complete_4219 == True:
                cycle_4219 = 6
                if cycle_2_4219 == 'A':
                    Duration_4219 += round(Phase_A_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_6_4219 == 'B':
                    Duration_4219 += round(Phase_B_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", B_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_6_4219 == 'C':
                    Duration_4219 += round(Phase_C_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", C_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_6_4219 == 'D':
                    Duration_4219 += round(Phase_D_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_6_4219 == 'E':
                    Duration_4219 += round(Phase_E_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_6_4219 == 'F':
                    Duration_4219 += round(Phase_F_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                else:
                    Duration_4219 += round(Phase_G_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
            else:
                if transition_counter_4219 >= 6:
                    transition_counter_4219 = 0
                    transition_complete_4219 = True
                    Duration_4219 += 1
                elif transition_counter_4219 == 0:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", transition_cycle_4219_5)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 10)
                elif transition_counter_4219 == 4:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", (transition_cycle_4219_5+1))
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
                else:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
        elif cycle_4219 == 6 and step == Duration_4219:
            if transition_complete_4219 == True:
                cycle_4219 = 7
                if cycle_7_4219 == 'A':
                    Duration_4219 += round(Phase_A_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    end_phase_4219 = 'A'
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", A_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_7_4219 == 'B':
                    Duration_4219 += round(Phase_B_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    end_phase_4219 = 'B'
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", B_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_7_4219 == 'C':
                    Duration_4219 += round(Phase_C_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    end_phase_4219 = 'C'
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", C_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_7_4219 == 'D':
                    Duration_4219 += round(Phase_D_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    end_phase_4219 = 'D'
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", D_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_7_4219 == 'E':
                    Duration_4219 += round(Phase_E_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    end_phase_4219 = 'E'
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", E_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                elif cycle_7_4219 == 'F':
                    Duration_4219 += round(Phase_F_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    end_phase_4219 = 'F'
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", F_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
                else:
                    Duration_4219 += round(Phase_G_4219_Density * scale_4219 + offset_4219)
                    transition_complete_4219 = False
                    end_phase_4219 = 'G'
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", G_4219)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 60)
            else:
                if transition_counter_4219 >= 6:
                    transition_counter_4219 = 0
                    transition_complete_4219 = True
                    Duration_4219 += 1
                elif transition_counter_4219 == 0:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", transition_cycle_4219_6)
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 10)
                elif transition_counter_4219 == 4:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
                    traci.trafficlight.setPhase("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", (transition_cycle_4219_6+1))
                    traci.trafficlight.setPhaseDuration("cluster_25977365_314059191_314060044_314061754_314061758_314062509_314062525", 5)
                else:
                    transition_counter_4219 += 1
                    Duration_4219 += 1
        elif cycle_4219 == 7 and step == Duration_4219:
            cycle_4219 = 0
            
        Phase_A_4220_Density = traci.lane.getLastStepVehicleNumber("122089528#1_0") + traci.lane.getLastStepVehicleNumber("122089528#1_1") + traci.lane.getLastStepVehicleNumber("122089528#1_2") + traci.lane.getLastStepVehicleNumber("620377517_0") + traci.lane.getLastStepVehicleNumber("620377517_1") + traci.lane.getLastStepVehicleNumber("620377517_2") + traci.lane.getLastStepVehicleNumber("gneE74_0") + traci.lane.getLastStepVehicleNumber("gneE74_1") + traci.lane.getLastStepVehicleNumber("gneE74_2") + traci.lane.getLastStepVehicleNumber("gneE77_0") + traci.lane.getLastStepVehicleNumber("gneE77_1") + traci.lane.getLastStepVehicleNumber("gneE77_2") + traci.lane.getLastStepVehicleNumber("158617319_1") + traci.lane.getLastStepVehicleNumber("158617319_2") + traci.lane.getLastStepVehicleNumber("158617319_3") + traci.lane.getLastStepVehicleNumber("547793365_0") + traci.lane.getLastStepVehicleNumber("547793365_1")
        Phase_B_4220_Density = traci.lane.getLastStepVehicleNumber("122089528#1_0") + traci.lane.getLastStepVehicleNumber("122089528#1_1") + traci.lane.getLastStepVehicleNumber("122089528#1_2") + traci.lane.getLastStepVehicleNumber("620377517_0") + traci.lane.getLastStepVehicleNumber("620377517_1") + traci.lane.getLastStepVehicleNumber("620377517_2") + traci.lane.getLastStepVehicleNumber("gneE74_0") + traci.lane.getLastStepVehicleNumber("gneE74_1") + traci.lane.getLastStepVehicleNumber("gneE74_2") + traci.lane.getLastStepVehicleNumber("gneE74_3") + traci.lane.getLastStepVehicleNumber("gneE78_0") + + traci.lane.getLastStepVehicleNumber("gneE78_1") + traci.lane.getLastStepVehicleNumber("gneE47_0")
        Phase_C_4220_Density = traci.lane.getLastStepVehicleNumber("gneE77_0") + traci.lane.getLastStepVehicleNumber("gneE77_1") + traci.lane.getLastStepVehicleNumber("gneE77_2") + traci.lane.getLastStepVehicleNumber("gneE77_3") + traci.lane.getLastStepVehicleNumber("158617319_1") + traci.lane.getLastStepVehicleNumber("158617319_2") + traci.lane.getLastStepVehicleNumber("158617319_3") + traci.lane.getLastStepVehicleNumber("547793365_0") + traci.lane.getLastStepVehicleNumber("547793365_1")
        Phase_D_4220_Density = traci.lane.getLastStepVehicleNumber("gneE73_0") + traci.lane.getLastStepVehicleNumber("gneE73_1") + traci.lane.getLastStepVehicleNumber("gneE10.441_0") + traci.lane.getLastStepVehicleNumber("gneE10.441_0")
        Phase_E_4220_Density = traci.lane.getLastStepVehicleNumber("gneE74_3") + traci.lane.getLastStepVehicleNumber("gneE73_0") + traci.lane.getLastStepVehicleNumber("620377517_2") + traci.lane.getLastStepVehicleNumber("122089528#1_2") + traci.lane.getLastStepVehicleNumber("547793365_1") + traci.lane.getLastStepVehicleNumber("158617319_3")
        
        Density_4220 = {    
            'A' : Phase_A_4220_Density,
            'B' : Phase_B_4220_Density,
            'C' : Phase_C_4220_Density,
            'D' : Phase_D_4220_Density,
            'E' : Phase_E_4220_Density
        }
        sorted_density_4220 = sorted(Density_4220.items(),key=lambda x:x[1], reverse=True)
        
        if cycle_4220 == 0:
            cycle_1_4220 = sorted_density_4220[0][0]
            cycle_2_4220 = sorted_density_4220[1][0]
            cycle_3_4220 = sorted_density_4220[2][0]
            cycle_4_4220 = sorted_density_4220[3][0]
            cycle_5_4220 = sorted_density_4220[4][0]
            transition_cycle_4220_0 = transition_dictionary4220[end_phase_4220 + '-' + cycle_1_4220]
            transition_cycle_4220_1 = transition_dictionary4220[cycle_1_4220 + '-' + cycle_2_4220]
            transition_cycle_4220_2 = transition_dictionary4220[cycle_2_4220 + '-' + cycle_3_4220]
            transition_cycle_4220_3 = transition_dictionary4220[cycle_3_4220 + '-' + cycle_4_4220]
            transition_cycle_4220_4 = transition_dictionary4220[cycle_4_4220 + '-' + cycle_5_4220]
            if transition_complete_4220 == True:
                cycle_4220 = 1
                if cycle_1_4220 == 'A':
                    Duration_4220 += round(Phase_A_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", A_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_1_4220 == 'B':
                    Duration_4220 += round(Phase_B_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", B_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_1_4220 == 'C':
                    Duration_4220 += round(Phase_C_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", C_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_1_4220 == 'D':
                    Duration_4220 += round(Phase_D_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", D_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                else:
                    Duration_4220 += round(Phase_E_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", E_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
            else:
                if transition_counter_4220 >= 6:
                    transition_counter_4220 = 0
                    transition_complete_4220 = True
                    Duration_4220 += 1
                elif transition_counter_4220 == 0:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", transition_cycle_4220_0)
                    traci.trafficlight.setPhaseDuration("gneJ41", 10)
                    
                elif transition_counter_4220 == 4:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", (transition_cycle_4220_0+1))
                    traci.trafficlight.setPhaseDuration("gneJ41", 5)
                else:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
        elif cycle_4220 == 1 and step == Duration_4220:
            if transition_complete_4220 == True:
                cycle_4220 = 2
                if cycle_2_4220 == 'A':
                    Duration_4220 += round(Phase_A_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", A_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_2_4220 == 'B':
                    Duration_4220 += round(Phase_B_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", B_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_2_4220 == 'C':
                    Duration_4220 += round(Phase_C_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", C_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_2_4220 == 'D':
                    Duration_4220 += round(Phase_D_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", D_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                else:
                    Duration_4220 += round(Phase_E_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", E_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
            else:
                if transition_counter_4220 >= 6:
                    transition_counter_4220 = 0
                    transition_complete_4220 = True
                    Duration_4220 += 1
                elif transition_counter_4220 == 0:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", transition_cycle_4220_1)
                    traci.trafficlight.setPhaseDuration("gneJ41", 10)
                    
                elif transition_counter_4220 == 4:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", (transition_cycle_4220_1+1))
                    traci.trafficlight.setPhaseDuration("gneJ41", 5)
                else:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
        elif cycle_4220 == 2 and step == Duration_4220:
            if transition_complete_4220 == True:
                cycle_4220 = 3
                if cycle_3_4220 == 'A':
                    Duration_4220 += round(Phase_A_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", A_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_3_4220 == 'B':
                    Duration_4220 += round(Phase_B_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", B_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_3_4220 == 'C':
                    Duration_4220 += round(Phase_C_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", C_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_3_4220 == 'D':
                    Duration_4220 += round(Phase_D_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", D_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                else:
                    Duration_4220 += round(Phase_E_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", E_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
            else:
                if transition_counter_4220 >= 6:
                    transition_counter_4220 = 0
                    transition_complete_4220 = True
                    Duration_4220 += 1
                elif transition_counter_4220 == 0:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", transition_cycle_4220_2)
                    traci.trafficlight.setPhaseDuration("gneJ41", 10)
                    
                elif transition_counter_4220 == 4:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", (transition_cycle_4220_2+1))
                    traci.trafficlight.setPhaseDuration("gneJ41", 5)
                else:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
        elif cycle_4220 == 3 and step == Duration_4220:
            if transition_complete_4220 == True:
                cycle_4220 = 4
                if cycle_4_4220 == 'A':
                    Duration_4220 += round(Phase_A_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", A_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_4_4220 == 'B':
                    Duration_4220 += round(Phase_B_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", B_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_4_4220 == 'C':
                    Duration_4220 += round(Phase_C_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", C_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                elif cycle_4_4220 == 'D':
                    Duration_4220 += round(Phase_D_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", D_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                else:
                    Duration_4220 += round(Phase_E_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", E_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
            else:
                if transition_counter_4220 >= 6:
                    transition_counter_4220 = 0
                    transition_complete_4220 = True
                    Duration_4220 += 1
                elif transition_counter_4220 == 0:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", transition_cycle_4220_3)
                    traci.trafficlight.setPhaseDuration("gneJ41", 10)
                    
                elif transition_counter_4220 == 4:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", (transition_cycle_4220_3+1))
                    traci.trafficlight.setPhaseDuration("gneJ41", 5)
                else:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
        elif cycle_4220 == 4 and step == Duration_4220:
            if transition_complete_4220 == True:
                cycle_4220 = 5
                if cycle_5_4220 == 'A':
                    Duration_4220 += round(Phase_A_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", A_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                    end_phase_4220 = 'A'
                elif cycle_5_4220 == 'B':
                    Duration_4220 += round(Phase_B_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", B_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                    end_phase_4220 = 'B'
                elif cycle_5_4220 == 'C':
                    Duration_4220 += round(Phase_C_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", C_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                    end_phase_4220 = 'C'
                elif cycle_5_4220 == 'D':
                    Duration_4220 += round(Phase_D_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", D_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                    end_phase_4220 = 'D'
                else:
                    Duration_4220 += round(Phase_E_4220_Density * scale_4220 + offset_4220)
                    transition_complete_4220 = False
                    traci.trafficlight.setPhase("gneJ41", E_4220)
                    traci.trafficlight.setPhaseDuration("gneJ41", 60)
                    end_phase_4220 = 'E'
            else:
                if transition_counter_4220 >= 6:
                    transition_counter_4220 = 0
                    transition_complete_4220 = True
                    Duration_4220 += 1
                elif transition_counter_4220 == 0:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", transition_cycle_4220_4)
                    traci.trafficlight.setPhaseDuration("gneJ41", 10)
                    
                elif transition_counter_4220 == 4:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
                    traci.trafficlight.setPhase("gneJ41", (transition_cycle_4220_4+1))
                    traci.trafficlight.setPhaseDuration("gneJ41", 5)
                else:
                    transition_counter_4220 += 1
                    Duration_4220 += 1
        elif cycle_4220 == 5 and step == Duration_4220:
            cycle_4220 = 0
        
        Phase_A_4221_Density = traci.lane.getLastStepVehicleNumber("7635630#1_0") + traci.lane.getLastStepVehicleNumber("7635630#1_1") + traci.lane.getLastStepVehicleNumber("7635630#0_0") + traci.lane.getLastStepVehicleNumber("7635630#0_1") + traci.lane.getLastStepVehicleNumber("620377512_0") + traci.lane.getLastStepVehicleNumber("620377512_1") + traci.lane.getLastStepVehicleNumber("28573008_0") + traci.lane.getLastStepVehicleNumber("28573008_1")
        Phase_B_4221_Density = traci.lane.getLastStepVehicleNumber("620377512_0") + traci.lane.getLastStepVehicleNumber("620377512_1")  + traci.lane.getLastStepVehicleNumber("620377512_2") + traci.lane.getLastStepVehicleNumber("28573008_0") + traci.lane.getLastStepVehicleNumber("28573008_1")
        Phase_C_4221_Density = traci.lane.getLastStepVehicleNumber("7635630#1_0") + traci.lane.getLastStepVehicleNumber("7635630#1_1") + traci.lane.getLastStepVehicleNumber("7635630#1_2") + traci.lane.getLastStepVehicleNumber("7635630#1_3") + traci.lane.getLastStepVehicleNumber("7635630#0_0") + traci.lane.getLastStepVehicleNumber("7635630#0_1") + traci.lane.getLastStepVehicleNumber("7635630#0_2") + traci.lane.getLastStepVehicleNumber("7635630#0_3")
        Phase_D_4221_Density = traci.lane.getLastStepVehicleNumber("gneE82_0") + traci.lane.getLastStepVehicleNumber("gneE82_1") + traci.lane.getLastStepVehicleNumber("gneE82_2") + traci.lane.getLastStepVehicleNumber("gneE81_0") + traci.lane.getLastStepVehicleNumber("gneE81_1") + traci.lane.getLastStepVehicleNumber("gneE81_2") + traci.lane.getLastStepVehicleNumber("gneE81_3") + traci.lane.getLastStepVehicleNumber("gneE80_0") + traci.lane.getLastStepVehicleNumber("gneE80_1") + traci.lane.getLastStepVehicleNumber("gneE80_2") + traci.lane.getLastStepVehicleNumber("gneE80_3")
        Phase_E_4221_Density = traci.lane.getLastStepVehicleNumber("28573025.65_0") + traci.lane.getLastStepVehicleNumber("28573025.65_1") + traci.lane.getLastStepVehicleNumber("28573025.65_2") + traci.lane.getLastStepVehicleNumber("28573025.46_0") + traci.lane.getLastStepVehicleNumber("28573025.46_1") + traci.lane.getLastStepVehicleNumber("28573025_0") + traci.lane.getLastStepVehicleNumber("28573015#1_0") + traci.lane.getLastStepVehicleNumber("28573015#2_0")
        Phase_F_4221_Density = traci.lane.getLastStepVehicleNumber("620377512_2") + traci.lane.getLastStepVehicleNumber("28573008_1") + traci.lane.getLastStepVehicleNumber("7635630#1_2") + traci.lane.getLastStepVehicleNumber("7635630#1_3") + traci.lane.getLastStepVehicleNumber("7635630#0_2") + traci.lane.getLastStepVehicleNumber("7635630#0_3")
        
        
        Density_4221 = {
            'A' : Phase_A_4221_Density,
            'B' : Phase_B_4221_Density,
            'C' : Phase_C_4221_Density,
            'D' : Phase_D_4221_Density,
            'E' : Phase_E_4221_Density,
            'F' : Phase_F_4221_Density
        }

        sorted_density_4221 = sorted(Density_4221.items(),key=lambda x:x[1], reverse=True)
        
        if cycle_4221 == 0:
            cycle_1_4221 = sorted_density_4221[0][0]
            cycle_2_4221 = sorted_density_4221[1][0]
            cycle_3_4221 = sorted_density_4221[2][0]
            cycle_4_4221 = sorted_density_4221[3][0]
            cycle_5_4221 = sorted_density_4221[4][0]
            cycle_6_4221 = sorted_density_4221[5][0]
            transition_cycle_4221_0 = transition_dictionary4221[end_phase_4221 + '-' + cycle_1_4221]
            transition_cycle_4221_1 = transition_dictionary4221[cycle_1_4221 + '-' + cycle_2_4221]
            transition_cycle_4221_2 = transition_dictionary4221[cycle_2_4221 + '-' + cycle_3_4221]
            transition_cycle_4221_3 = transition_dictionary4221[cycle_3_4221 + '-' + cycle_4_4221]
            transition_cycle_4221_4 = transition_dictionary4221[cycle_4_4221 + '-' + cycle_5_4221]
            transition_cycle_4221_5 = transition_dictionary4221[cycle_5_4221 + '-' + cycle_6_4221]
            
            if transition_complete_4221 == True:
                cycle_4221 = 1
                if cycle_1_4221 == 'A':
                    Duration_4221 += round(Phase_A_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_1_4221 == 'B':
                    Duration_4221 += round(Phase_B_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", B_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_1_4221 == 'C':
                    Duration_4221 += round(Phase_C_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", C_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_1_4221 == 'D':
                    Duration_4221 += round(Phase_D_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_1_4221 == 'E':
                    Duration_4221 += round(Phase_E_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                else:
                    Duration_4221 += round(Phase_F_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
            else:
                if transition_counter_4221 >= 6:
                    transition_counter_4221 = 0
                    transition_complete_4221 = True
                    Duration_4221 += 1
                elif transition_counter_4221 == 0:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", transition_cycle_4221_0)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 10)
                    
                elif transition_counter_4221 == 4:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", (transition_cycle_4221_0+1))
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 5)
                else:
                    transition_counter_4221 += 1
                    Duration_4221 += 1

        
        elif cycle_4221 == 1 and step == Duration_4221:
            if transition_complete_4221 == True:
                cycle_4221 = 2
                if cycle_2_4221 == 'A':
                    Duration_4221 += round(Phase_A_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_2_4221 == 'B':
                    Duration_4221 += round(Phase_B_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", B_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_2_4221 == 'C':
                    Duration_4221 += round(Phase_C_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", C_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_2_4221 == 'D':
                    Duration_4221 += round(Phase_D_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_2_4221 == 'E':
                    Duration_4221 += round(Phase_E_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                else:
                    Duration_4221 += round(Phase_F_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
            else:
                if transition_counter_4221 >= 6:
                    transition_counter_4221 = 0
                    transition_complete_4221 = True
                    Duration_4221 += 1
                elif transition_counter_4221 == 0:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", transition_cycle_4221_1)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 10)
                elif transition_counter_4221 == 4:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", (transition_cycle_4221_1+1))
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 5)
                else:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
        elif cycle_4221 == 2 and step == Duration_4221:
            if transition_complete_4221 == True:
                cycle_4221 = 3
                if cycle_3_4221 == 'A':
                    Duration_4221 += round(Phase_A_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_3_4221 == 'B':
                    Duration_4221 += round(Phase_B_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", B_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_3_4221 == 'C':
                    Duration_4221 += round(Phase_C_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", C_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_3_4221 == 'D':
                    Duration_4221 += round(Phase_D_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_3_4221 == 'E':
                    Duration_4221 += round(Phase_E_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                else:
                    Duration_4221 += round(Phase_F_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
            else:
                if transition_counter_4221 >= 6:
                    transition_counter_4221 = 0
                    transition_complete_4221 = True
                    Duration_4221 += 1
                elif transition_counter_4221 == 0:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", transition_cycle_4221_2)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 10)
                elif transition_counter_4221 == 4:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", (transition_cycle_4221_2+1))
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 5)
                else:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
        elif cycle_4221 == 3 and step == Duration_4221:
            if transition_complete_4221 == True:
                cycle_4221 = 4
                if cycle_4_4221 == 'A':
                    Duration_4221 += round(Phase_A_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_4_4221 == 'B':
                    Duration_4221 += round(Phase_B_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", B_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_4_4221 == 'C':
                    Duration_4221 += round(Phase_C_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", C_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_4_4221 == 'D':
                    Duration_4221 += round(Phase_D_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_4_4221 == 'E':
                    Duration_4221 += round(Phase_E_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                else:
                    Duration_4221 += round(Phase_F_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
            else:
                if transition_counter_4221 >= 6:
                    transition_counter_4221 = 0
                    transition_complete_4221 = True
                    Duration_4221 += 1
                elif transition_counter_4221 == 0:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", transition_cycle_4221_3)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 10)
                elif transition_counter_4221 == 4:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", (transition_cycle_4221_3+1))
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 5)
                else:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
        elif cycle_4221 == 4 and step == Duration_4221:
            if transition_complete_4221 == True:
                cycle_4221 = 5
                if cycle_5_4221 == 'A':
                    Duration_4221 += round(Phase_A_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_5_4221 == 'B':
                    Duration_4221 += round(Phase_B_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", B_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_5_4221 == 'C':
                    Duration_4221 += round(Phase_C_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", C_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_5_4221 == 'D':
                    Duration_4221 += round(Phase_D_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                elif cycle_5_4221 == 'E':
                    Duration_4221 += round(Phase_E_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                else:
                    Duration_4221 += round(Phase_F_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
            else:
                if transition_counter_4221 >= 6:
                    transition_counter_4221 = 0
                    transition_complete_4221 = True
                    Duration_4221 += 1
                elif transition_counter_4221 == 0:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", transition_cycle_4221_4)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 10)
                elif transition_counter_4221 == 4:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", (transition_cycle_4221_4+1))
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 5)
                else:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
        elif cycle_4221 == 5 and step == Duration_4221:
            if transition_complete_4221 == True:
                cycle_4221 = 6
                if cycle_2_4221 == 'A':
                    Duration_4221 += round(Phase_A_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", A_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                    end_phase_4221 = 'A'
                elif cycle_6_4221 == 'B':
                    Duration_4221 += round(Phase_B_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", B_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                    end_phase_4221 = 'B'
                elif cycle_6_4221 == 'C':
                    Duration_4221 += round(Phase_C_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", C_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                    end_phase_4221 = 'C'
                elif cycle_6_4221 == 'D':
                    Duration_4221 += round(Phase_D_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", D_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                    end_phase_4221 = 'D'
                elif cycle_6_4221 == 'E':
                    Duration_4221 += round(Phase_E_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", E_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                    end_phase_4221 = 'E'
                else:
                    Duration_4221 += round(Phase_F_4221_Density * scale_4221 + offset_4221)
                    transition_complete_4221 = False
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", F_4221)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 60)
                    end_phase_4221 = 'F'
            else:
                if transition_counter_4221 >= 6:
                    transition_counter_4221 = 0
                    transition_complete_4221 = True
                    Duration_4221 += 1
                elif transition_counter_4221 == 0:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", transition_cycle_4221_5)
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 10)
                elif transition_counter_4221 == 4:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
                    traci.trafficlight.setPhase("cluster_25953432_313863435_313863521_314053282", (transition_cycle_4221_5+1))
                    traci.trafficlight.setPhaseDuration("cluster_25953432_313863435_313863521_314053282", 5)
                else:
                    transition_counter_4221 += 1
                    Duration_4221 += 1
        elif cycle_4221 == 6 and step == Duration_4221:
            cycle_4221 = 0
            
        
        
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
                             "--tripinfo-output", "output/tripinfo.xml", "--no-internal-links", "--summary", "output/summary.xml", "--queue-output","output/queue.xml"])
    run()
