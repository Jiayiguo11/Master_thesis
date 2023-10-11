import traci
import numpy as np


def vsl(edge, v_desire):
    # v_desire = min(30, q_desire/(rho*3.6))

    v_limit1 = v_desire
    v_limit = np.round(v_limit1, decimals=0)
    for i in edge:
        traci.edge.setMaxSpeed(i, v_limit)

    return v_limit


def recover(edge):
    for i in edge:
        traci.edge.setMaxSpeed(i, 30)
