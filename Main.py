# Traci coding
# Jiayi GUO
# 04/2023--

# Imports
# --------------------------------------------------------------
from RM import rm
from microVSL import vsl2_0
from RouteTT import routetraveltime
from RoadMonitor import monitor
from RoadMonitor_plot import animonitor
from RMswitch import rmonoff
from Clean import process
from RGswitch import rg, rg_mpc
from VariableLists import VarLists
from Plot import rmplot, routeplot, speedplot, trajectoryplots, contour
from Trajectories import traj
from COSCAL import congdet, speedcontrol, datasorting, statefunc
from contour import edgespeed
from MPC_RG import mpcrg
from VSL import vsl, recover
from DrakesFD_estimation import drake_est
from datetime import datetime

from scipy.optimize import curve_fit
import os
import sys
import time
import numpy as np
import pandas as pd
import json
import traci
import pickle
import traci.constants as tc
import matplotlib
import random
import csv
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
matplotlib.rcParams['backend'] = 'Qt5Agg'

os.environ["SUMO_HOME"] = "/usr/local/opt/sumo"
sys.path.append(os.path.join(os.environ.get("SUMO_HOME"), 'tools'))
sumoBinary = "/usr/local/bin/sumo"
sumoCmd = [sumoBinary, "-c", "/Users/guojiayi/Desktop/sumo practice/Network1/Network1.sumocfg"]
# sumoCmd = [sumoBinary, "-c", "/Users/guojiayi/Desktop/sumo practice/Network1/Network1.sumocfg", "--seed 23423"]
# ---------------------------------------------------------------


#traci.start(sumoCmd, port=8813)
traci.start(sumoCmd)
Var = VarLists()
s = 0

#######################
onoffall = [0, 0, 0, 0]  # RM1, RM3, RG, VSL

tar = [4000, 4000]

rm_off_or_max = 0  # switch between 2 and 0
note = "no"
step_limit = 3600  # total number of steps [1s per step] !! has to be multiple of 60
# sce = "Scenario_3500_3500_5rm0_rg0_vslnew"
demand_2main = 3000
demand_1ramp = 1000
sce = "3200_3000_rm13"
#######################

# Chapter 1: initialise required variables
# -----------------------------------------------------
q_Road3_u = []
flow_multiplier = 60  # multiplier to convert counted vehicle numbers to flow
########################
# Section 1.0 For data analysis
speed_route_change = []
########################
# Section 1.1 RM parameters
rm_switch = [onoffall[0], 0, onoffall[1]]
paras = [3800, 3300, 22, 24]  # I_on, I_off, V_on, V_off
########################
rmonoff_l = [0, 0, 0]
RMdet1 = ["RM1m1", "RM1m2", "RM1dm1", "RM1dm2", "RM1r_u_flow", "em4_5", "em5_6"]
RMedge1 = ["onramp1"]
RMdet2 = ["RM2m1", "RM2m2", "RM2dm1", "RM2dm2", "RM2r_u_flow", "em10_11", "em11_12"]
RMedge2 = ["onramp2"]
RMdet3 = ["RM3m1", "RM3m2", "RM3dm1", "RM3dm2", "RM3r_u_flow", "es4_5", "es5_6"]
RMedge3 = ["offramp1_2", "offramp1_3"]

aftmg1 = []
aftmg2 = []
aftmg3 = []
aftmg4 = []
aftmg5 = []
aftmg6 = []
aftmg7 = []
aftmg8 = []

queue_ramp1 = []
queue_ramp3 = []
rm_q_old1 = 0
rm_q_old3 = 0

#cumulative vehicle numbers
N_1 = []
N_2 = []
Nu_1 = []
Nu_2 = []

################
# vsl Section 1.2 VSL parameters
vslswitch = onoffall[3]
vsl_on_spd = 12
vsl_off_spd = 15
v_desire1 = 24
v_desire2 = 24
N_change = 1
COSCAL_switch = 1
COSCAL_switchR2 = 0
bottleneck_location = 2000
bottleneck_locationR2 = 2000
vsl1 = [30]
vsl2 = [30]
vtd1 = [30]
vtd2 = [30]
vsl_trend = 30
vsl_trend2 = 30
count_change = 0
count_change2 = 0
detected_v1 = [0]
detected_v2 = [0]
vr1 = []
vr2 = []
################
v_d_last = 30
count_cong = 0  # for vsl to count the integrated congestion time
count_free = 0  # for vsl to count the integrated freeflow time
v_state = []
v_cong = []
veh_ids = []
v_speed = []
v_stateR2 = []
v_congR2 = []
veh_idsR2 = []
v_speedR2 = []
t_dec = []
t_reach = []
para_COSCAL = [15, 10, -10]
para_COSCALR2 = [15, 10, -10]
v_eff = 18
v_head = 7
dec = -2
acc = 1


#################
# Section 1.3 RG parameters
rgnew = 1  #
alpha = 0.7
compl = 80
compl_mpc = 80
rgmpc = onoffall[2]
prediction_speed = []
spd_act_r1 = []
spd_act_r2 = []
spd_act_all = []
tt_act_all = []
proportion = [1]
tt1 = 1
tt2 = 0
outflow1 = 4000
outflow2 = 4000
cap1 = 7000
cap2 = 7000
L1 = 0.6
L2 = 0.6
vehdemand = 3000
T = 20/3600
Vehnr_1 = 0
Vehnr_2 = 0
vnumber1 = []
vnumber2 = []
#################
# Monitor
ListM_1 = ["em1_2", "em2_3", "em3_4", "em4_5", "em5_6", "em6_7", "em7_8", "em8_9", "em9_10", "em10_11", "em11_12",
           "em12_13", "e13_14", "e14_15", "e15_16", "e16_17"]
hect_1 = ["-", "-", "Off-ramp1", "-", "On-ramp1", "-", "-", "-", "Off-ramp2", "-", "On-ramp2", "-", "Merge", "-"]
ListM_2 = ["es1_2", "es2_3", "es3_4", "es4_5", "es5_6", "es6_7", "ns8_nm13", "e13_14", "e14_15", "e15_16", "e16_17"]

hect_2 = ["-", "-", "Off-ramp3", "-", "On-ramp3", "-", "-", "Merge", "-"]

 # edgelist = ["em3_4", "em4_5", "em5_6", "em6_7", "em7_8", "em8_9", "em9_10", "em10_11", "em11_12", "em12_13", "e13_14",
 #             "e14_15", "e15_16"]
# edgelist = ["em8_9", "em9_10", "em10_11", "em11_12", "em12_13", "e13_14", "e14_15"]
edgelist = {}
edgelist[0] = ListM_1
edgelist[1] = ListM_2
edgelist[2] = ["onramp1", "onramp1_connection"]
edgelist[3] = ["onramp2", "onramp2_connection"]
edgelist[4] = ["offramp1_1", "offramp1_2", "offramp1_3", "onramp3"]
# indicators
# typical_edges = ["em5_6", "em12_13", "e13_14", "e15_16", "e16_17", "ns8_nm13"]
# typical_edge_length = [0.3, 0.3, 0.3, 0.3, 0.3, 1.5]

# typical_edges = ["em5_6", "em6_7", "es5_6", "es6_7"]
typical_edges = ["es5_6", "es6_7", "em5_6", "em6_7"]
typical_edge_length = [0.3, 0.3, 0.3, 0.4]

den = [[] for _ in range(len(typical_edges))]
spd = [[] for _ in range(len(typical_edges))]
ocp = [[] for _ in range(len(typical_edges))]
total_delay = 0
delay_v = {}
loss_v = {}
total_waiting = 0
edge_ids = ["em2_3", "em4_5", "em6_7", "em10_11", "em12_13", "es4_5", "es7_8", "e14_15"]
edge_name = ['R1 before or1', 'R1 before rm1', 'R1 after rm1', 'R1 before rm2', 'R1 after rm2',
             'R2 before rm3', 'R2 after rm3', 'After merge before lanedrop']
laneid = [[], [], [], [], [], []]
# road 1 route 1
laneid[0] = ["em1_2_0", "em2_3_0", "em3_4_0", "em4_5_0", "em5_6_0", "em6_7_0", "em7_8_0", "em8_9_0", "em9_10_0", "em10_11_0", "em11_12_0", "em12_13_0", "e13_14_0", "e14_15_0", "e15_16_0"]
# road 1 route 2
laneid[1] = ["em1_2_0", "em2_3_0", "offramp1_1_0", "offramp1_2_0", "offramp1_3_0", "onramp3_0", "es5_6_0", "es6_7_0", "es7_8_0", "ns8_nm13_0", "e13_14_0", "e14_15_0", "e15_16_0"]
#road 2
laneid[2] = ["es1_2_0", "es2_3_0", "es3_4_0", "es4_5_0", "es5_6_0", "es6_7_0", "es7_8_0", "ns8_nm13_0", "e13_14_0", "e14_15_0", "e15_16_0"]
#on-ramp 1
laneid[3] = ["onramp1_connection_0", "onramp1_0"]
#on-ramp 1_2
laneid[4] = ["onramp1_connection_0", "onramp1_0", "em5_6_0", "em6_7_0", "em7_8_0", "em8_9_0", "em9_10_0", "em10_11_0", "em11_12_0", "em12_13_0", "e13_14_0", "e14_15_0", "e15_16_0"]
# on-ramp 3
laneid[5] = ["offramp1_1_0", "offramp1_2_0", "offramp1_3_0", "onramp3_0"]



num_edges = len(edge_ids)
halting_numbers = [[] for _ in range(num_edges)]
tt = [[] for _ in range(num_edges)]
v_edge = [[] for _ in range(num_edges)]
trajectory = []
trajectoryR2 = []
speedcont1 = []
speedcont2 = []
net_Entry1_old = []
net_Entry2_old = []
net_Entry3_old = []
net_exit_old = []
r1_Entry_old = []
r2_Entry_old = []
r_all1 = {}
r_all2 = {}
r_2a = {}
r_ramp = {}
r_rp = {}
r1_t = {}
r1_a = {}
r2_t = {}
r2_a = {}



merge_old = []
# trajectory
################
# Section 1.4 trajectory parameters
traj_lim_l = 1200
traj_lim_u = 1
# traj_lim_d_l = 0
# traj_lim_d_u = 4500
################

# Running time
Milestones = [round(step_limit / 5), round(2 * step_limit / 5), round(3 * step_limit / 5), round(4 * step_limit / 5),
              round(step_limit)]
start_time = time.time()
five = 0

# ------------------------------------------------------------------

# Chapter 2: the main simulation

# -------------------------------------------------------------------

# Section 2.1 Simulation process

while s <= step_limit:
    traci.simulationStep()
    if s == Milestones[0]:
        print("Simulation 1/5: --- %s seconds ---" % (time.time() - start_time))
    elif s == Milestones[1]:
        print("Simulation 2/5: --- %s seconds ---" % (time.time() - start_time))
    elif s == Milestones[2]:
        print("Simulation 3/5: --- %s seconds ---" % (time.time() - start_time))
    elif s == Milestones[3]:
        print("Simulation 4/5: --- %s seconds ---" % (time.time() - start_time))
    elif s == Milestones[4]:
        print("Simulation 5/5: --- %s seconds ---" % (time.time() - start_time))
    elif (time.time() - start_time) > 300 * five:
        print("5 min check: step %s" % s)
        five = five+1

# Section 2.1.1 calculate length of each route

    if s == 0:
        total_length11 = [0, 0, 0, 0, 0, 0]
        tt_standard = [0, 0, 0, 0]
        for i in range(len(total_length11)):
            for lane in laneid[i]:
                total_length11[i] += traci.lane.getLength(lane)
        tt_standard[0] = total_length11[0]/30
        tt_standard[2] = total_length11[2]/30
        t_ramp1 = total_length11[3] / 15
        t_rest = (total_length11[4]-total_length11[3])/30
        tt_standard[3] = t_ramp1+t_rest
        t_ramp3 = total_length11[5] / 15
        t_rest2 = (total_length11[1]-total_length11[5])/30
        tt_standard[1] = t_ramp3+t_rest2
        # tt_standard[4] = total_length11[4] / 15



# Section 2.2 RG on and off

    if s % 10 == 0 and rgnew == 1:
        vehicle_ids = traci.edge.getLastStepVehicleIDs("em1_2")

        # Calculate the number of elements to select
        num_elements = np.round(len(vehicle_ids)/2, decimals=0)  # Select 50% of the elements
        num_elements1 = max(int(num_elements), 0)
        # Randomly select 50% of elements from the list
        VMSset = random.sample(vehicle_ids, num_elements1)
        MPCset = [elem for elem in vehicle_ids if elem not in VMSset]

        rg(tt1, tt2, VMSset, compl)

        if rgmpc == 1:
            p1, objvalue = mpcrg(demand_2main, demand_1ramp, vehdemand, Vehnr_1, Vehnr_2, cap1, cap2, outflow1, outflow2, T, L1, L2)
            p = alpha*p1+(1-alpha)*proportion[-1]
            rg_mpc(p, MPCset, rgmpc, compl_mpc)
        # Result of predictive model and real data
            obj_convert = -objvalue
            prediction_speed.append(obj_convert)
            proportion.append(p)
            spd_r1_1 = traci.edge.getLastStepMeanSpeed("em5_6")
            spd_r1_2 = traci.edge.getLastStepMeanSpeed("em6_7")
            spd_r2_1 = traci.edge.getLastStepMeanSpeed("es5_6")
            spd_r2_2 = traci.edge.getLastStepMeanSpeed("es6_7")

            vnr_act1 = len(traci.edge.getLastStepVehicleIDs("em5_6"))+len(traci.edge.getLastStepVehicleIDs("em6_7"))
            vnr_act2 = len(traci.edge.getLastStepVehicleIDs("es5_6")) + len(traci.edge.getLastStepVehicleIDs("es6_7"))

            spd_r1 = (spd_r1_1 + spd_r1_2) / 2
            spd_r2 = (spd_r2_1 + spd_r2_2) / 2
            spd_all = (spd_r1 + spd_r2) / 2
            tt1 = traci.edge.getTraveltime("em5_6")+traci.edge.getTraveltime("em6_7")
            tt2 = traci.edge.getTraveltime("es5_6") + traci.edge.getTraveltime("es6_7")
            tt_act1 = tt1*vnr_act1
            tt_act2 = tt2*vnr_act2
            if vnr_act1+vnr_act2>0:
                tt_act = (tt_act1+tt_act2)/(vnr_act1+vnr_act2)
            else:
                tt_act = 0
            spd_act_r1.append(spd_r1)
            spd_act_r2.append(spd_r2)
            spd_act_all.append(spd_all)
            tt_act_all.append(tt_act)



        vehdemand = 60*(traci.inductionloop.getLastIntervalVehicleNumber("Entry_R1L1") + traci.inductionloop.getLastIntervalVehicleNumber("Entry_R1L2") + traci.inductionloop.getLastIntervalVehicleNumber("Entry_R1L3"))
        # print(vehdemand)
        vid11 = traci.edge.getLastStepVehicleIDs("em5_6")
        vid12 = traci.edge.getLastStepVehicleIDs("em6_7")
        vid21 = traci.edge.getLastStepVehicleIDs("es5_6")
        vid22 = traci.edge.getLastStepVehicleIDs("es6_7")
        Vehnr_1 = len(vid11)+len(vid12)
        Vehnr_2 = len(vid21)+len(vid22)
        vnumber1.append(Vehnr_1)
        vnumber2.append(Vehnr_2)
        outflow1 = 60 * (traci.inductionloop.getLastIntervalVehicleNumber("R1_outflow_L1")
                         + traci.inductionloop.getLastIntervalVehicleNumber("R1_outflow_L2"))
        outflow2 = 60 * (traci.inductionloop.getLastIntervalVehicleNumber("R2_outflow_L1")
                         + traci.inductionloop.getLastIntervalVehicleNumber("R2_outflow_L2"))
        demand_1ramp = traci.inductionloop.getLastIntervalVehicleNumber("RM1r_d_flow")




# Section 2.3 FD estimation: Density and speed collection:
    for i in range(len(typical_edges)):
        ocp1 = traci.edge.getLastStepOccupancy(typical_edges[i])
        # density = ocp * typical_edge_length[i]/0.002
        speed = traci.edge.getLastStepMeanSpeed(typical_edges[i])
        veh = traci.edge.getLastStepVehicleIDs(typical_edges[i])
        veh_num = len(veh)
        density = veh_num / typical_edge_length[i]
        ocp[i].append(ocp1)
        spd[i].append(speed)
        den[i].append(density)

    vehicle_ids = traci.vehicle.getIDList()
    # for veh in vehicle_ids:
    #     taus = traci.vehicle.getTau(veh)
    #     print(taus)

    # #vehicle lane change model
    # for veh in vehicle_ids:
    #     lc = traci.vehicle.getLaneChangeMode(veh)
    #     print(lc)

# Section 2.4 VSL simplified
    v11 = traci.lane.getLastStepMeanSpeed("em5_6_1")
    v22 = traci.lane.getLastStepMeanSpeed("es5_6_1")

    # v11 = traci.inductionloop.getIntervalMeanSpeed("R1_inflow_L2")
    # v22 = traci.inductionloop.getIntervalMeanSpeed("R2_inflow_L2")

    vr1.append(v11)
    vr2.append(v22)
    resolution = 10
    if vslswitch == 1 and s % resolution == 0:
        vele1 = vr1[-resolution:]
        v1 = np.mean(vele1)
        vele2 = vr2[-resolution:]
        v2 = np.mean(vele2)

        # v1 = v11
        # v2 = v22

        detected_v1.append(v1)
        detected_v2.append(v2)


        # control_section1 = ["em1_2", "em2_3", "em3_4", "em4_5"]
        # control_section2 = ["es1_2", "es2_3", "es3_4", "es4_5"]

        control_section1 = ["em3_4", "em4_5"]
        control_section2 = ["es3_4", "es4_5"]

        if v1 < vsl_on_spd:
            vsl_trend = v_desire1
        elif v1 > vsl_off_spd:
            vsl_trend = 30
        else:
            if vtd1[-1] == v_desire1:
                vsl_trend = v_desire1
            else:
                vsl_trend = 30

        if vsl_trend != vtd1[-1]:
            count_change = 0
        else:
            count_change += 1
        # print(vsl_trend, count_change, v1)
        if count_change > N_change:
            vsl(control_section1, vsl_trend)
            vsl1.append(vsl_trend)
        else:
            vsl(control_section1, vsl1[-1])
            vsl1.append(vsl1[-1])




        vtd1.append(vsl_trend)

        if v2 < vsl_on_spd:
            vsl_trend2 = v_desire2
        elif v2 > vsl_off_spd:
            vsl_trend2 = 30
        else:
            if vtd2[-1] == v_desire2:
                vsl_trend2 = v_desire2
            else:
                vsl_trend2 = 30

        if vsl_trend2 != vtd2[-1]:
            count_change2 = 0
        else:
            count_change2 += 1
        if count_change2 > N_change:
            vsl(control_section2, vsl_trend2)
            vsl2.append(vsl_trend2)
        else:
            vsl(control_section2, vsl2[-1])
            vsl2.append(vsl2[-1])


        vtd2.append(vsl_trend2)

        # if v1 < vsl_on_spd:
        #     vsl(control_section1, v_desire)
        #     vsl1.append(20)
        # elif v1 > vsl_off_spd:
        #     recover(control_section1)
        #     vsl1.append(30)
        # else:
        #     if vsl1[-1] == 20:
        #         vsl(control_section1, v_desire)
        #         vsl1.append(20)
        #     else:
        #         recover(control_section1)
        #         vsl1.append(30)
        #
        # if v2 < vsl_on_spd:
        #     vsl(control_section2, v_desire)
        #     vsl2.append(20)
        # elif v2 > vsl_off_spd:
        #     recover(control_section2)
        #     vsl2.append(30)
        # else:
        #     if vsl2[-1] == 20:
        #         vsl(control_section2, v_desire)
        #         vsl2.append(20)
        #     else:
        #         recover(control_section2)
        #         vsl2.append(30)

    aplarea_veh1 = traci.edge.getLastStepVehicleIDs("em3_4")+traci.edge.getLastStepVehicleIDs("em4_5")
    aplarea_veh2 = traci.edge.getLastStepVehicleIDs("es3_4")+traci.edge.getLastStepVehicleIDs("es4_5")

    veh_out_of_area1 = traci.edge.getLastStepVehicleIDs("em5_6") + traci.edge.getLastStepVehicleIDs("es5_6")
    for ve in veh_out_of_area1:
        traci.vehicle.setTau(ve, 1)

    if vsl1[-1] == v_desire1:
        for veh in aplarea_veh1:
            traci.vehicle.setTau(veh, 0.97)
    else:
        for veh in aplarea_veh1:
            traci.vehicle.setTau(veh, 1)

    if vsl2[-1] == v_desire2:
        for veh in aplarea_veh2:
            traci.vehicle.setTau(veh, 0.97)
    else:
        for veh in aplarea_veh2:
            traci.vehicle.setTau(veh, 1)

    Q1 = traci.edge.getLastStepHaltingNumber("em4_5")
    Q2 = traci.edge.getLastStepHaltingNumber("em10_11")

# Section 2.5 Data from induction loops

    # Ramp 1: mainline and ramp demand [veh/min]
    R1m = traci.inductionloop.getLastIntervalVehicleNumber("RM1m1") + traci.inductionloop.getLastIntervalVehicleNumber(
        "RM1m2")
    R1r = traci.inductionloop.getLastIntervalVehicleNumber("RM1r_u_flow")
    R1rd = traci.inductionloop.getLastIntervalVehicleNumber("RM1r_d_flow")

    # Ramp 1: presence detectors before and after the stop line of traffic light [veh/s]
    R1_app = traci.inductionloop.getLastIntervalVehicleNumber("RM1r_u")
    R1_pas = traci.inductionloop.getLastIntervalVehicleNumber("RM1r_d")


    # Ramp 2: mainline and ramp demand [veh/min]
    R2m = traci.inductionloop.getLastIntervalVehicleNumber("RM2m1") + traci.inductionloop.getLastIntervalVehicleNumber(
        "RM2m2")
    R2r = traci.inductionloop.getLastIntervalVehicleNumber("RM2r_u_flow")
    R2rd = traci.inductionloop.getLastIntervalVehicleNumber("RM2r_d_flow")

    # Ramp 2: presence detectors before and after the stop line of traffic light [veh/s]
    R2_app = traci.inductionloop.getLastIntervalVehicleNumber("RM2r_u")
    R2_pas = traci.inductionloop.getIntervalOccupancy("RM2r_d")

    # Ramp 3: mainline and ramp demand [veh/min]
    R3m = traci.inductionloop.getLastIntervalVehicleNumber("RM3m1") + traci.inductionloop.getLastIntervalVehicleNumber(
        "RM3m2")
    R3r = traci.inductionloop.getLastIntervalVehicleNumber("RM3r_u_flow")
    R3rd = traci.inductionloop.getLastIntervalVehicleNumber("RM3r_d_flow")
    # Ramp 1: presence detectors before and after the stop line of traffic light [veh/s]
    R3_app = traci.inductionloop.getLastIntervalVehicleNumber("RM3r_u")
    R3_pas = traci.inductionloop.getLastIntervalVehicleNumber("RM3r_d")

    # Road 2 downstream
    R2 = traci.inductionloop.getLastIntervalVehicleNumber("R2m1") + traci.inductionloop.getLastIntervalVehicleNumber(
        "R2m2")


    # Road 3 before lane drop
    R3u = traci.inductionloop.getLastIntervalVehicleNumber("R3u1") + traci.inductionloop.getLastIntervalVehicleNumber(
            "R3u2") + traci.inductionloop.getLastIntervalVehicleNumber("R3u3") + traci.inductionloop.getLastIntervalVehicleNumber("R3u4")
    # Road 3 after lane drop
    R3 = traci.inductionloop.getLastIntervalVehicleNumber("R3m1") + traci.inductionloop.getLastIntervalVehicleNumber(
        "R3m2") + traci.inductionloop.getLastIntervalVehicleNumber("R3m3")

    Route_id = traci.route.getIDList()

# Section 2.6 RM and corresponding value retrieving

    # Ramp 1

    ramp_id_1 = "R1on1_connection"
    [rm_s, vu1, vd1] = rmonoff(RMdet1, RMedge1, paras, rmonoff_l[0])
    rmonoff_l[0] = rm_s
    # qlength_1 = traci.lanearea.getJamLengthMeters("onramp1")
    qlength_1 = traci.lanearea.getIntervalMeanSpeed("onramp1")
    queue_ramp1.append(qlength_1)
    # print(qlength_1)
    if qlength_1 >= 12:
        rm_q = 1
    elif 0 < qlength_1 <= 3:
        rm_q = rm_off_or_max
    else:
        rm_q = rm_q_old1
    rm_q_old1 = rm_q
    #rm_q = 1
    rm_switch1 = rm_s * rm_switch[0] * rm_q

    rmarea_veh1 = traci.edge.getLastStepVehicleIDs("em4_5") + traci.edge.getLastStepVehicleIDs(
        "em5_6") + traci.edge.getLastStepVehicleIDs("em6_7")

    if rm_switch1 == 1:
        for veh in rmarea_veh1:
            traci.vehicle.setTau(veh, 0.97)
    else:
        for veh in rmarea_veh1:
            traci.vehicle.setTau(veh, 1)

    rm(ramp_id_1, R1m, R1r, R1_app, tar[0], rm_switch1)
    [phaseR1, mrR1] = rm(ramp_id_1, R1m, R1r, R1_pas, tar[0], rm_switch1)



    Var.vu_R1.append(vu1)
    Var.vd_R1.append(vd1)
    Var.RM1state.append(rm_switch1)

    # Ramp 2
    ramp_id_2 = "R1on2_connection"
    [rm_s, vu2, vd2] = rmonoff(RMdet2, RMedge2, paras, rmonoff_l[1])
    rmonoff_l[1] = rm_s
    rm_switch2 = rm_s * rm_switch[1]
    rm(ramp_id_2, R2m, R2r, R2_app, R2_pas, rm_switch2)
    [phaseR2, mrR2] = rm(ramp_id_2, R2m, R2r, R2_pas, R2_app, rm_switch2)
    Var.vu_R2.append(vu2)
    Var.vd_R2.append(vd2)
    Var.RM2state.append(rm_switch2)

    # Ramp 3
    ramp_id_3 = "R1off1_3"
    [rm_s, vu3, vd3] = rmonoff(RMdet3, RMedge3, paras, rmonoff_l[2])
    rmonoff_l[2] = rm_s
    qlength_3 = traci.lanearea.getIntervalMeanSpeed("offramp1_2")
    queue_ramp3.append(qlength_3)

    if qlength_3 >= 12:
        rm_q = 1
    elif 0 < qlength_3 <= 3:
        rm_q = rm_off_or_max
    else:
        rm_q = rm_q_old3
    rm_q_old3 = rm_q
    #rm_q = 1
    rm_switch3 = rm_s * rm_switch[2] * rm_q

    rmarea_veh2 = traci.edge.getLastStepVehicleIDs("es4_5") + traci.edge.getLastStepVehicleIDs(
        "es5_6") + traci.edge.getLastStepVehicleIDs("es6_7")

    if rm_switch3 == 1:
        for veh in rmarea_veh2:
            traci.vehicle.setTau(veh, 0.97)
    else:
        for veh in rmarea_veh2:
            traci.vehicle.setTau(veh, 1)

    rm(ramp_id_3, R3m, R3r, R3_pas, tar[1], rm_switch3)
    [phaseR3, mrR3] = rm(ramp_id_3, R3m, R3r, R3_pas, tar[1], rm_switch3)
    Var.vu_R3.append(vu3)
    Var.vd_R3.append(vd3)
    Var.RM3state.append(rm_switch3)


# Section 2.7 LC check: vehicle on each lane after merge

    aftmg1.append(traci.inductionloop.getLastIntervalVehicleNumber("R3u1"))
    aftmg2.append(traci.inductionloop.getLastIntervalVehicleNumber("R3u2"))
    aftmg3.append(traci.inductionloop.getLastIntervalVehicleNumber("R3u3"))
    aftmg4.append(traci.inductionloop.getLastIntervalVehicleNumber("R3u4"))
    aftmg5.append(traci.inductionloop.getLastIntervalVehicleNumber("R3u5"))
    aftmg6.append(traci.inductionloop.getLastIntervalVehicleNumber("R3u6"))
    aftmg7.append(traci.inductionloop.getLastIntervalVehicleNumber("R3u7"))
    aftmg8.append(traci.inductionloop.getLastIntervalVehicleNumber("R3u8"))





# Section 2.8 Other values

    # Route switching
    R1_to_R2 = traci.inductionloop.getLastIntervalVehicleNumber("R1toR2")

    # Edges
    for i in range(num_edges):
        halting_numbers[i].append(traci.edge.getLastStepHaltingNumber(edge_ids[i]))
        tt[i].append(traci.edge.getTraveltime(edge_ids[i]))
        v_edge[i].append(traci.edge.getLastStepMeanSpeed(edge_ids[i]))

    # Travel time
    tt_2_3_1 = (traci.lane.getTraveltime("em2_3_1")+traci.lane.getTraveltime("em2_3_1"))/2
    tt1 = routetraveltime(0)+tt_2_3_1

    Var.TT_Route1.append(tt1)
    tt_2_3_2 = traci.lane.getTraveltime("em2_3_0")
    tt2 = routetraveltime(1)+tt_2_3_2
    Var.TT_Route2.append(tt2)
    # print(tt_2_3_1, tt_2_3_2)
    tt_R1 = monitor(ListM_1)
    tt_R2 = monitor(ListM_2)
    Var.edge_tt_R1.append(tt_R1)
    Var.edge_tt_R2.append(tt_R2)

    ## All data outputs

    # Data on-ramps
    Var.qr_R1.append(flow_multiplier * R1r)
    Var.qm_R1.append(flow_multiplier * R1m)
    Var.phase_R1.append(phaseR1)
    Var.metering_rate_R1.append(mrR1)
    Var.qrd_R1.append(flow_multiplier * R1rd)

    Var.qr_R2.append(flow_multiplier * R2r)
    Var.qm_R2.append(flow_multiplier * R2m)
    Var.phase_R2.append(phaseR2)
    Var.metering_rate_R2.append(mrR2)
    Var.qrd_R2.append(flow_multiplier * R2rd)

    Var.qr_R3.append(flow_multiplier * R3r)
    Var.qm_R3.append(flow_multiplier * R3m)
    Var.phase_R3.append(phaseR3)
    Var.metering_rate_R3.append(mrR3)
    Var.qrd_R3.append(flow_multiplier * R3rd)

    # # data vsls
    # Var.v_vsl1.append(v)
    # Var.vd_vsl1.append(v_d1)

    # queue lengths
    Var.queue_merge1.append(Q1)
    Var.queue_merge2.append(Q2)

    # flow route 2 and flow after merge
    Var.q_Road2.append(flow_multiplier * R2)
    Var.q_Road3.append(flow_multiplier * R3)
    q_Road3_u.append(flow_multiplier*R3u)

    # flow road 1 to road 2
    Var.q_switch.append(flow_multiplier * R1_to_R2)
    veh_delay = 0
    for veh_id in traci.vehicle.getIDList():
            delay_v[veh_id] = traci.vehicle.getAccumulatedWaitingTime(veh_id)
            loss_v[veh_id] = traci.vehicle.getTimeLoss(veh_id)

        # veh_delay = traci.vehicle.getTimeLoss(veh_id)


    # v_e = traci.inductionloop.getLastStepMeanSpeed("Entry_R1L2")

    # Var.ve.append(v_e)
    Var.delay.append(veh_delay)
    # Var.cong.append(count_cong)

# Section 2.9 Edge-based contour

    spd1 = edgespeed(ListM_1)
    speedcont1.append(spd1)
    spd2 = edgespeed(ListM_2)
    speedcont2.append(spd2)

# Section 2.10 Route travel time
    if s >= 0:
        net_Entry1 = traci.edge.getLastStepVehicleIDs("em1_2")
        net_Entry2 = traci.edge.getLastStepVehicleIDs("es1_2")
        net_Entry3 = traci.edge.getLastStepVehicleIDs("onramp1")
        net_exit = traci.edge.getLastStepVehicleIDs("e16_17")
        r1_Entry = traci.edge.getLastStepVehicleIDs("em3_4")
        r2_Entry = traci.edge.getLastStepVehicleIDs("offramp1_1")
        merge = traci.edge.getLastStepVehicleIDs("e13_14")

        for veh in net_Entry1:
            if veh not in net_Entry1_old:
                r_all1[veh] = s
        for veh in net_Entry2:
            if veh not in net_Entry2_old:
                r_all2[veh] = s
        for veh in net_Entry3:
            if veh not in net_Entry3_old:
                r_ramp[veh] = s

        for veh in r1_Entry:
            if veh not in r1_Entry_old:
                r1_t[veh] = s
        for veh in r2_Entry:
            if veh not in r2_Entry_old:
                r2_t[veh] = s

        for veh in net_exit:
            if veh not in net_exit_old:
                if veh in r_all1 and veh in r1_t:
                    etime = r_all1[veh]
                    r1_a[veh] = s-etime
                if veh in r_all1 and veh in r2_t:
                    etime = r_all1[veh]
                    r2_a[veh] = s-etime
                if veh in r_all2:
                    etime = r_all2[veh]
                    r_2a[veh] = s - etime
                if veh in r_ramp:
                    etime = r_ramp[veh]
                    r_rp[veh] = s-etime



        # for veh in merge:
        #     if veh not in merge_old:
        #         if veh in r1_t:
        #             etime = r1_t[veh]
        #             r1_a[veh] = s-etime
        #         elif veh in r2_t:
        #             etime = r2_t[veh]
        #             r2_a[veh] = s-etime

        r1_Entry_old = r1_Entry
        r2_Entry_old = r2_Entry
        merge_old = merge
        net_Entry1_old = net_Entry1
        net_Entry2_old = net_Entry2
        net_Entry3_old = net_Entry3
        net_exit_old = net_exit


    # cumulative vehicle number
    n1 = traci.inductionloop.getLastStepVehicleNumber("R1_outflow_L1")+traci.inductionloop.getLastStepVehicleNumber("R1_outflow_L2")
    nu1 = traci.inductionloop.getLastStepVehicleNumber("R1_inflow_L1")+traci.inductionloop.getLastStepVehicleNumber("R1_inflow_L2")+traci.inductionloop.getLastStepVehicleNumber("R1_inflow_L3")
    N_1.append(n1)
    Nu_1.append(nu1)
    n2 = traci.inductionloop.getLastStepVehicleNumber("R2_outflow_L1") + traci.inductionloop.getLastStepVehicleNumber(
        "R2_outflow_L2")
    nu2 = traci.inductionloop.getLastStepVehicleNumber("R2_inflow_L1") + traci.inductionloop.getLastStepVehicleNumber(
        "R2_inflow_L2") + traci.inductionloop.getLastStepVehicleNumber("R2_inflow_L3")
    N_2.append(n2)
    Nu_2.append(nu2)

    #speed edge1
    speed_e1 = traci.edge.getTraveltime("em1_2")
    speed_route_change.append(speed_e1)

    s += 1

traci.close()

# ---------------------------------------------------------------------
# Chapter 3: Visualisation
# ---------------------------------


# colors = {0: 'green', 1: 'yellow', 2: 'red'}
# col = [colors[val] for val in phase_R2]

x = range(step_limit + 1)

# Fig 1: RM
# Data processing: speed
[x1, y1] = process(x, Var.vd_R1)
[x2, y2] = process(x, Var.vd_R2)
[x3, y3] = process(x, Var.vd_R3)

rmplot(paras, sce, x, Var.qm_R1, Var.qr_R1, Var.metering_rate_R1, Var.qrd_R1, Var.qm_R2, Var.qr_R2,
       Var.metering_rate_R2, Var.qrd_R2, Var.qm_R3, Var.qr_R3, Var.metering_rate_R3, Var.qrd_R3, x1, y1, x2, y2, x3, y3,
       Var.RM1state, Var.RM2state, Var.RM3state)

xx = range(len(Var.metering_rate_R1))
plt.figure()
plt.plot(xx, Var.metering_rate_R1, label='RM1')

# plt.plot(xx, Var.metering_rate_R3, label='RM3')
xx1 = range(0, len(proportion)*10, 10)
proportion_adj = [(i-0.4)*600 for i in proportion]
plt.plot(xx1, proportion_adj, label='RG')

# # FIG 6: flow on both routes
# fig, ax = plt.subplots(figsize=(12, 6))
#
# # Plot the data and label each line
# ax.plot(x, Var.qm_R2, label='flow road1')
# ax.plot(x, Var.q_Road2, label='flow road2')
# ax.plot(x, Var.q_switch, label='flow road1 to road2')
# ax.plot(x, q_Road3_u, label='flow before lane drop')
# ax.plot(x, Var.q_Road3, label='flow after lane drop')
#
# # Add axis labels
# ax.set_xlabel('Simulation step [s]')
# ax.set_ylabel('Flow [veh/h]')
#
# ax.set_title("Flow on each part")
# # Add a legend
# ax.legend()

# # FIG 6.1: vehicle on each lane after merge
# fig, ax = plt.subplots(1, 2, figsize=(12, 6))
#
# # Plot the data and label each line
# ax[0].plot(x, aftmg5, label='Lane 0')
# ax[0].plot(x, aftmg6, label='Lane 1')
# ax[0].plot(x, aftmg7, label='Lane 2')
# ax[0].plot(x, aftmg8, label='Lane 3')
#
# # Add axis labels
# ax[0].set_xlabel('Simulation step [s]')
# ax[0].set_ylabel('Vehicle number')
# ax[0].set_title("Edge 13")
# ax[0].legend()
# ax[1].plot(x, aftmg1, label='Lane 0')
# ax[1].plot(x, aftmg2, label='Lane 1')
# ax[1].plot(x, aftmg3, label='Lane 2')
# ax[1].plot(x, aftmg4, label='Lane 3')
#
# # Add axis labels
# ax[1].set_xlabel('Simulation step [s]')
# ax[1].set_ylabel('Vehicle number')
# ax[1].set_title("Edge 14")
# # Add a legend
#
# ax[1].legend()
# fig.savefig("/Users/guojiayi/Desktop/sumo practice/Figs/Lanechange " + sce + ".png")

#
# # FIG 7: halting numbers
# fig, ax = plt.subplots(figsize=(12, 6))
# for i in range(num_edges):
#     ax.plot(x, halting_numbers[i], label=edge_name[i])
#
# ax.legend()
# ax.set_title("Halting numbers on different segments")
# ax.set_xlabel('Simulation step [s]')
# ax.set_ylabel('Halting number')

# # FIG 8: travel time
# fig, ax = plt.subplots()
#
# for i in range(num_edges):
#     for j in range(len(x)):
#         if tt[i][j] > 3000:
#             tt[i][j] = 0
#     ax.plot(x, tt[i], label=edge_name[i])
#
# ax.legend()
# ax.set_title("Travel time on different segments")
# ax.set_xlabel('Simulation step [s]')
# ax.set_ylabel('Travel time [s]')


# FIG 9:Route travel time and switch


routeplot(sce, x, Var.TT_Route1, Var.TT_Route2, Var.q_switch)

# # FIG 10: monitor road 1
# e1 = range(len(edge_tt_R1[0]))
# e2 = range(len(edge_tt_R2[0]))
# step_names = range(len(edge_tt_R1))
# # FIG 10: travel time adjustment based on edge length
# for sublist in edge_tt_R1:
#     sublist[0] *= 0.5
# for sublist in edge_tt_R2:
#     sublist[6] *= 0.2
#
# animonitor(step_names, e1, edge_tt_R1, hect_1, e2, edge_tt_R2, hect_2)

# FIG 11: Average speed
# width = int(np.ceil(num_edges / 2))
# speedplot(sce, x, width, num_edges, v_edge, edge_name)

# # FIG 12 trajectories
#
# vehicle_curves = {}
# vehicle_color = {}
# vehicle_speed = {}
# vehicle_curves_R2 = {}
# vehicle_color_R2 = {}
# vehicle_speed_R2 = {}
#
# starting_step = traj_lim_l
#
# vehicle_curves = datasorting(starting_step, trajectory)
# vehicle_color = datasorting(starting_step, v_state)
# vehicle_cong = datasorting(starting_step, v_cong)
# vehicle_speed = datasorting(starting_step, v_speed)
#
# vehicle_speed_check = vehicle_speed.copy()
#
# vehicle_curves_R2 = datasorting(starting_step, trajectoryR2)
# vehicle_color_R2 = datasorting(starting_step, v_stateR2)
# vehicle_cong_R2 = datasorting(starting_step, v_congR2)
# vehicle_speed_R2 = datasorting(starting_step, v_speedR2)
#
# road1 = "Road 1"
# road2 = "Road 2"
# trajectoryplots(road1, vehicle_curves, vehicle_color, vehicle_speed, sce, starting_step, traj_lim_l, traj_lim_u,
#                 para_COSCAL)
# trajectoryplots(road2, vehicle_curves_R2, vehicle_color_R2, vehicle_speed_R2, sce, starting_step, traj_lim_l, traj_lim_u,
#                 para_COSCALR2)

values = loss_v.values()
total_delay = sum(values)
# variables_dict = {
#     'total_delay': total_delay,
#     'trajectory': trajectory,
#     'vehicle_curves': vehicle_curves,
#     'vehicle_speed': vehicle_speed,
#     'vehicle_color': vehicle_color,
#     'trajectoryR2': trajectoryR2,
#     'vehicle_curvesR2': vehicle_curves_R2,
#     'vehicle_speedR2': vehicle_speed_R2,
#     'vehicle_colorR2': vehicle_color_R2
# }
# with open(sce+'.json', 'w') as file:
#     json.dump(variables_dict, file)
# print("Variables saved to JSON.")

# FIG 13 Speed-density relationship

minu = int(step_limit/60)

den1 = [row[:-1] for row in den]
den_ag = np.reshape(den1, (len(typical_edges), minu, 60))
den_agg = np.mean(den_ag, axis=2)

spd1 = [row[:-1] for row in spd]
spd_ag = np.reshape(spd1, (len(typical_edges), minu, 60))
spd_agg = np.mean(spd_ag, axis=2)
spd_agg_km = [element * 3.6 for element in spd_agg]

fig, ax = plt.subplots(1, len(typical_edges))
x11 = range(len(den_agg[0]))
for i in range(len(typical_edges)):
    ax[i].plot(x11, den_agg[i])
    ax[i].set_xlabel("Time [10s]")
    ax[i].set_ylabel("Density [veh/km]")
    ax[i].set_title(typical_edges[i])






# for i in range(len(typical_edges)):
#     # q = [3.6 * x * y for x, y in zip(den[i], spd[i])]
#     #
#     # ax.scatter(den[i], q, s=1, label=typical_edges[i])
#     fig, ax = plt.subplots()
#     ax.scatter(den_agg[i], spd_agg_km[i], s=1, label=typical_edges[i])
#     ax.set_xlabel('Density [veh/km]')
#     ax.set_ylabel('Speed [veh/h]')
#     ax.set_title("Density Speed relationship"+typical_edges[i])
#     # plt.scatter(spd[i], den[i], s=1)
#     ax.legend()



# FIG 13.5--FD estimation

RMSE, v0, kc = drake_est(typical_edges, den_agg, spd_agg_km)

#
# def speed_density_function(dens, v0, k_c):
#     return v0*np.exp(-1/2*(dens/k_c)**2)
#
# bounds = ([90, 50], [110, 90])
#
# #
# # def speed_density_function(dens, v0, k_c, k_1):
# #     if dens <k_c:
# #         value = v0
# #     if dens >= k_1:
# #         v0*k_1/dens - (dens-k_1)/k
# #
# #     return
#
# v0 = [None for _ in range(len(typical_edges))]
# kc = [None for _ in range(len(typical_edges))]
#
# for i in range(len(typical_edges)):
#
#     popt, pcov = curve_fit(speed_density_function, den_agg[i], spd_agg_km[i], bounds=bounds)
#
#     # Extract the optimized parameters
#     v0[i], kc[i] = popt
#
#     # Generate a range of densities for plotting
#     density_range = np.linspace(min(den_agg[i]), max(den_agg[i]), 100)
#
#     # Calculate the corresponding speeds using the fitted parameters
#     speed_predicted = speed_density_function(density_range, v0[i], kc[i])
#
#     # Plot the original data and the fitted curve
#     plt.figure()
#     plt.scatter(den_agg[i], spd_agg_km[i], s=1, label='Original Data')
#     plt.plot(density_range, speed_predicted, 'r', label='Fitted Curve')
#     plt.xlabel('Density [veh/km]')
#     plt.ylabel('Speed [km/h]')
#     plt.title('Density Speed relationship ' + typical_edges[i])
#     plt.legend()
#     plt.show()





# FIG 14: speed contour
row1 = len(ListM_1)    # Number of rows
ytick1 = range(row1)
col1 = step_limit+1    # Number of columns

spdmtx1 = [[None] * col1 for _ in range(row1)]


for i in range(len(speedcont1)):
    ct = 0
    for edge in speedcont1[i]:
        spdmtx1[ct][i] = speedcont1[i][edge]
        ct += 1

row2 = len(ListM_2)    # Number of rows
ytick2 = range(row2)
col2 = step_limit+1    # Number of columns

spdmtx2 = [[None] * col2 for _ in range(row2)]

roadname1 = "Road1"
plt.figure()
plt.rcParams['font.size'] = 14

contour(step_limit+1, ListM_1, spdmtx1, roadname1)
plt.yticks(ytick1, ListM_1)
plt.xlabel("Time[s]")
plt.ylabel("Road segments")


plt.colorbar(label='Speed [m/s]')

scenario = str(rm_switch[0])+str(rm_switch[2])+str(rgmpc)+str(vslswitch)
path = "/Users/guojiayi/Desktop/sumo practice/Figs1/"
name = scenario+roadname1+ note + ".png"
plt.savefig(path + "/" + name)

for i in range(len(speedcont2)):
    ct = 0
    for edge in speedcont2[i]:
        spdmtx2[ct][i] = speedcont2[i][edge]
        ct += 1
roadname2 = "Road2"
plt.figure()
contour(step_limit+1, ListM_2, spdmtx2, roadname2)
plt.yticks(ytick2, ListM_2)
plt.colorbar(label='Speed [m/s]')
plt.xlabel("Time[s]")
plt.ylabel("Road segments")


path = "/Users/guojiayi/Desktop/sumo practice/Figs1/"
name = scenario+roadname2 + note+ ".png"
plt.savefig(path + "/" + name)


# FIG 15: experienced travel time histograms

ttr11 = list(r1_a.values())
ttr12 = list(r2_a.values())
ttr2 = list(r_2a.values())
ttrp = list(r_rp.values())
ttall_all = ttr11 + ttr12 + ttr2 + ttrp

meantt = [None] * 5
meantt[0] = np.mean(ttr11)
meantt[1] = np.mean(ttr12)
meantt[2] = np.mean(ttr2)
meantt[3] = np.mean(ttrp)
meantt[4] = np.mean(ttall_all)

delay = [0, 0, 0, 0, 0]
# delay[0] = len(ttr11)*(meantt[0]-tt_standard[0])
# delay[1] = len(ttr12)*(meantt[1]-tt_standard[1])
# delay[2] = len(ttr2)*(meantt[2]-tt_standard[2])
# delay[3] = len(ttrp)*(meantt[3]-tt_standard[3])
# delay[4] = sum(delay[:4])

delay[0] = meantt[0]-tt_standard[0]
delay[1] = meantt[1]-tt_standard[1]
delay[2] = meantt[2]-tt_standard[2]
delay[3] = meantt[3]-tt_standard[3]
dl = len(ttr11)*delay[0]+len(ttr12)*delay[1]+len(ttr2)*delay[2]+len(ttrp)*delay[3]
delay[4] = dl/(len(ttr11)+len(ttr12)+len(ttr2)+len(ttrp))


#
# hist_dict1, bins_dict1 = np.histogram(ttr11, bins=10)
# hist_dict2, bins_dict2 = np.histogram(ttr12, bins=10)
#
# # Plot histograms
# plt.figure(figsize=(10, 5))
# plt.subplot(1, 2, 1)
# plt.hist(ttr11, bins=bins_dict1, alpha=0.5, color='b', label='Route 1')
# plt.xlabel('Travel time [s]')
# plt.ylabel('Frequency')
# plt.xlim(100, 300)
# plt.title('Travel Time Distribution - Route 1')
# plt.legend()
#
# plt.subplot(1, 2, 2)
# plt.hist(ttr12, bins=bins_dict2, alpha=0.5, color='g', label='Route 2')
# plt.xlabel('Travel time [s]')
# plt.ylabel('Frequency')
# plt.xlim((100, 300))
# plt.title('Travel Time Distribution - Route 2')
# plt.legend()
#
# plt.tight_layout()
# plt.show()


output1 = [rm_switch[0], rm_switch[2], rgmpc, vslswitch]
output2 = total_delay
output3 = meantt
# Get the current date and time
current_datetime = datetime.now()


output4 = current_datetime.strftime("%Y-%m-%d %H:%M:%S")

output5 = delay

output6 = np.mean(speed_route_change)

# Name of the CSV file
csv_file = "result.csv"

# Writing data to the CSV file in a new row
with open(csv_file, mode='a', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([*output1, output2, *output3, output4, *output5, output6])

print("Data written to CSV successfully.")

print("RM1: " + str(rm_switch[0]) + "; RM3: " + str(rm_switch[2]) + "; RG: " + str(rgmpc) + "; VSL: " + str(vslswitch))
print("VSL Parameters--vsl_on: "+str(vsl_on_spd)+"; vsl_off: "+str(vsl_off_spd)+"; desired_speed: "+str(v_desire1))
print("Total_delay: " + str(total_delay))
print("Average travel time: " + str(meantt))
print("Measured delay: " + str(delay))

# FIG 16: Comparison predictive model and data


plt.figure()
plt.rcParams['font.size'] = 14
x1 = range(len(prediction_speed))

plt.plot(x1, prediction_speed, label='Prediction')
# plt.plot(x1, spd_act_r1, label='Route 1')
# plt.plot(x1, spd_act_r2, label='Route 2')
# plt.plot(x1, spd_act_all, label='Actual mean')
plt.plot(x1, tt_act_all, label='Actual mean')
plt.xlabel("Time [10s]")
plt.ylabel("Mean travel time [s]")
plt.legend()

x_p = range(len(proportion))
plt.figure()
plt.plot(x_p, proportion)

# dens = np.linspace(0, 180, 180)
# y = 90 * np.exp(-1 / 2 * (dens / 100) ** 2)
# z = dens * 90 * np.exp(-1 / 2 * (dens / 100) ** 2)
# plt.figure()
# plt.plot(dens, y)
# plt.figure()
# plt.plot(dens, z)



# FIG 17 vsl curves
fig, ax = plt.subplots(1, 2)
plt.rcParams['font.size'] = 14
detected_v1[0] = 30
detected_v2[0] = 30
x_s = range(len(vsl1))
ax[0].plot(x_s, vsl1, label="Speed limit")
ax[0].plot(x_s, detected_v1, label="Detected speed")
ax[0].axhline(y=vsl_on_spd, color='red', linestyle='--', label="v_on")
ax[0].axhline(y=vsl_off_spd, color='green', linestyle='--', label="v_off")
ax[0].set_xlabel("Time")
ax[0].set_ylabel("Speed [m/s]")
ax[0].set_title("Road 1")
ax[0].legend()
ax[1].plot(x_s, vsl2, label="Speed limit")
ax[1].plot(x_s, detected_v2, label="Detected speed")
ax[1].axhline(y=vsl_on_spd, color='red', linestyle='--', label="v_on")
ax[1].axhline(y=vsl_off_spd, color='green', linestyle='--', label="v_off")
ax[1].set_xlabel("Time")
ax[1].set_ylabel("Speed [m/s]")
ax[1].set_title("Road 2")
ax[1].legend()

fig, ax1 = plt.subplots()
plt.rcParams['font.size'] = 14
detected_v1[0] = 30
detected_v2[0] = 30
x_s = range(1, 10*len(vsl1), 10)
ax1.plot(x_s, vsl1, label="Speed limit", color="tab:blue", linestyle="--")


ax1.set_xlabel("Time")
ax1.set_ylabel("Speed limit [m/s]", color="tab:blue")
ax1.set_title("Road 1")
ax1.tick_params(axis='y', labelcolor="tab:blue")



ax2 = ax1.twinx()
ax2.plot(x, Var.qrd_R1, label="Ramp flow", color="tab:orange")
ax2.set_ylabel("Ramp flow [veh/h]", color="tab:orange")
ax2.tick_params(axis='y', labelcolor="tab:orange")

fig, ax1 = plt.subplots()
plt.rcParams['font.size'] = 14
detected_v1[0] = 30
detected_v2[0] = 30
x_s = range(1, 10*len(vsl1), 10)
ax1.plot(x_s, vsl2, label="Speed limit", color="tab:blue", linestyle="--")


ax1.set_xlabel("Time")
ax1.set_ylabel("Speed limit [m/s]", color="tab:blue")
ax1.set_title("Road 2")
ax1.tick_params(axis='y', labelcolor="tab:blue")



ax2 = ax1.twinx()
ax2.plot(x, Var.qrd_R3, label="Ramp flow", color="tab:orange")
ax2.set_ylabel("Ramp flow [veh/h]", color="tab:orange")
ax2.tick_params(axis='y', labelcolor="tab:orange")

#RM and RG
fig, ax1 = plt.subplots()
plt.rcParams['font.size'] = 14
detected_v1[0] = 30
detected_v2[0] = 30
x_s = range(1, 10*len(proportion), 10)
ax1.plot(x_s, proportion, label="Speed limit", color="tab:blue")


ax1.set_xlabel("Time")
ax1.set_ylabel("Speed limit [m/s]", color="tab:blue")
ax1.set_title("Road 1")
ax1.tick_params(axis='y', labelcolor="tab:blue")



ax2 = ax1.twinx()
ax2.plot(x, Var.qrd_R1, label="Ramp flow", color="tab:orange")
ax2.set_ylabel("Ramp flow [veh/h]", color="tab:orange")
ax2.tick_params(axis='y', labelcolor="tab:orange")

fig, ax1 = plt.subplots()
plt.rcParams['font.size'] = 14
detected_v1[0] = 30
detected_v2[0] = 30
x_s = range(1, 10*len(vsl1), 10)
ax1.plot(x_s, vsl2, label="Speed limit", color="tab:blue", linestyle="--")


ax1.set_xlabel("Time")
ax1.set_ylabel("Speed limit [m/s]", color="tab:blue")
ax1.set_title("Road 2")
ax1.tick_params(axis='y', labelcolor="tab:blue")



ax2 = ax1.twinx()
ax2.plot(x, Var.qrd_R3, label="Ramp flow", color="tab:orange")
ax2.set_ylabel("Ramp flow [veh/h]", color="tab:orange")
ax2.tick_params(axis='y', labelcolor="tab:orange")



plt.figure()
plt.plot(x, queue_ramp1)
plt.plot(x, queue_ramp3)

x_s = range(len(queue_ramp1))
plt.rcParams['font.size'] = 14
fig, ax = plt.subplots(2, 2, figsize=(12, 6))


ax[0][0].plot(x_s, Var.phase_R1, label='Ramp1')
ax[0][0].set_xlabel("time")
ax[0][0].set_ylabel("Traffic signal phase")
ax[0][0].set_title("Ramp1")
ax[0][1].plot(x_s, Var.phase_R3, label='Ramp3')
ax[0][1].set_xlabel("time")
ax[0][1].set_ylabel("Traffic signal phase")
ax[0][1].set_title("Ramp2")
ax[1][0].plot(x_s, queue_ramp1, label='Ramp1:queue')
ax[1][0].set_xlabel("time")
ax[1][0].set_ylabel("Average speed of queue detection area [m/s]")
ax[1][1].plot(x_s, queue_ramp3, label='Ramp3:queue')
ax[1][1].set_xlabel("time")
ax[1][1].set_ylabel("Average speed of queue detection area [m/s]")

plt.close("all")
print(kc, v0, [RMSE["es5_6"], RMSE["es6_7"], RMSE["em5_6"], RMSE["em6_7"]])


# cumulative curve

slanted1 = [None]*61
slanted2 = [None]*61
slantedu1 = [None]*61
slantedu2 = [None]*61

q01 = 4200
q02 = 4200

n_cum1 = np.cumsum(N_1)
n_cum2 = np.cumsum(N_2)
nu_cum1 = np.cumsum(Nu_1)
nu_cum2 = np.cumsum(Nu_2)

chunk_size = 60  # Determine the number of elements per data point
n1_agg = []
n2_agg = []
nu1_agg = []
nu2_agg = []


for i in range(0, len(n_cum1), chunk_size):
    chunk1 = n_cum1[i:i+chunk_size]
    chunk2 = n_cum2[i:i+chunk_size]
    chunk3 = nu_cum1[i:i + chunk_size]
    chunk4 = nu_cum2[i:i + chunk_size]
    average1 = sum(chunk1) / len(chunk1)
    n1_agg.append(average1)
    average2 = sum(chunk2) / len(chunk2)
    n2_agg.append(average2)
    average3 = sum(chunk3) / len(chunk3)
    nu1_agg.append(average3)
    average4 = sum(chunk4) / len(chunk4)
    nu2_agg.append(average4)

x_sc = range(len(n1_agg))

for i in range(len(n1_agg)):
    slanted1[i] = n1_agg[i]-(i+1)*q01/(3600/chunk_size)
    slantedu1[i] = nu1_agg[i]-(i+1)*q01/(3600/chunk_size)

plt.figure()
plt.plot(x_sc, slanted1, label="downstream")
# plt.plot(x_sc, slantedu1, label="upstream")
plt.xlabel("Time [min]")
plt.ylabel("Number of vehicles")
plt.title("Slanted cumulative curve after bottleneck 1: q0=4200veh/h")

for i in range(len(n2_agg)):
    slanted2[i] = n2_agg[i]-(i+1)*q02/(3600/chunk_size)
    slantedu2[i] = nu2_agg[i] - (i + 1) * q02 / (3600 / chunk_size)
plt.figure()
plt.plot(x_sc, slanted2, label="downstream")
# plt.plot(x_sc, slantedu2, label="upstream")
plt.xlabel("Time [min]")
plt.ylabel("Number of vehicles")
plt.title("Slanted cumulative curve after bottleneck 2: q0=4200veh/h")

plt.figure()
plt.plot(x1, vnumber1)
plt.plot(x1, vnumber2)

plt.figure()
plt.plot(x, speed_route_change)
# plt.plot(x, imported_array)
#
# with open('copied_array.pkl', 'wb') as file:
#     pickle.dump(speed_route_change, file)
#
# with open('copied_array.pkl', 'rb') as file:
#     imported_array = pickle.load(file)