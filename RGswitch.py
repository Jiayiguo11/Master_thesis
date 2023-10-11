import traci
import random


def rg(r1, r2, vehicle_ids, compl_rate):
    # vehicle_ids = traci.edge.getLastStepVehicleIDs("em1_2")
    #                   # + traci.edge.getLastStepVehicleIDs("em2_3")
    for id1 in vehicle_ids:
        rand = random.randint(1, 100)
        if r1 >= r2:
            if rand > 100-compl_rate:
                traci.vehicle.setRouteID(id1, "route2")
            else:
                traci.vehicle.setRouteID(id1, "route1")
        else:
            traci.vehicle.setRouteID(id1, "route1")


def rg_mpc(p, vehicle_ids, onoff, compl_rate):
    # vehicle_ids = traci.edge.getLastStepVehicleIDs("em1_2")
    #                   # + traci.edge.getLastStepVehicleIDs("em2_3")
    if onoff == 1:
        route = "route2"
    else:
        route = "route1"
    for id1 in vehicle_ids:
        rand1 = random.randint(1, 100)
        rand2 = random.randint(1, 100)
        if rand1 > 100-compl_rate:  # compliance rate
            if rand2 > 100*p:
                traci.vehicle.setRouteID(id1, route)
            else:
                traci.vehicle.setRouteID(id1, "route1")
        else:
            traci.vehicle.setRouteID(id1, "route1")



