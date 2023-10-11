# Master_thesis
This repository includes codes for SUMO simulation described in the master thesis

Two Excel spreadsheets summarise the simulation results.

The functions of each file are:
1. Network new_no_dsbnk_new.net.xml: Determines the network structure;
2. Network3.rou.xml: Determines the demand and vehicle parameters;
3. Network1.sumocfg:Configuration file;
4. Main.py: Main simulation codes including calling DTM functions and traffic state collection;
5. Main_Coordination.py: Main simulation codes for coordination scenarios including calling DTM functions and traffic state collection;
6. CTM.py: MPC algorithm, including the CTM model and the GA optimisation process
7. test_ctm.py: A test of MPC control algorithm including several visualisation
8. RGswitch.py: Apply route change on vehicles (both VMS and predictive RG)
9. MPC_RG.py: Predictive algorithm to determine the optimal split rate
10. RM.py: Change traffic signal according to Demand-capacity algorithm
11. RM_for_coordination.py: Change traffic signal according to desired cycle time for coordinated control
12. RMswitch.py: Turn RM on/off based on the traffic states
13. VSL.py: Change the speed limit on VSL application area
14. Plot.py: Includes functions for generating required plots





