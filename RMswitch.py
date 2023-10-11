import traci

# input det needs to contain:
# [0] Main road upstream; [1] Main road upstream lane 2; [2] Main road downstream;
# [3]  Main road downstream; [4]Onramp upstream;
# input para needs to contain:
# [0] Flow on threshold; [1] Flow off threshold; [2] Speed on threshold; [3] Speed off threshold;
# [4] maximum queue length on ramp
def rmonoff(det, edge, para, lastrm):
    v_u = -1
    v_d = -1
    queue = 0
    v_on = 0  # initialisation
    v_off = 0  # initialisation
    I_u = traci.inductionloop.getLastIntervalVehicleNumber(det[0]) + traci.inductionloop.getLastIntervalVehicleNumber(det[1])
    Ir_u = traci.inductionloop.getLastIntervalVehicleNumber(det[4])
    # v_u1 = traci.inductionloop.getLastStepMeanSpeed(det[0])
    # v_u2 = traci.inductionloop.getLastStepMeanSpeed(det[1])
    # for id in edge:
    #     queue += traci.lanearea.getJamLengthMeters(id)
    #
    # if v_u1 == -1 and v_u2 == -1:
    #     v_u = -1
    # elif v_u1+v_u2 > v_u1:
    #     v_u = (v_u1+v_u2)/2
    # else:
    #     v_u = max(v_u1, v_u2)
    #
    # v_d1 = traci.inductionloop.getLastStepMeanSpeed(det[2])
    # v_d2 = traci.inductionloop.getLastStepMeanSpeed(det[3])
    #
    # if v_d1 == -1 and v_d2 == -1:
    #     v_d = -1
    # elif v_d1 + v_d2 > v_d1:
    #     v_d = (v_d1 + v_d2) / 2
    # else:
    #     v_d = max(v_d1, v_d2)
    I = 60*(I_u + Ir_u)

    v_u = traci.edge.getLastStepMeanSpeed(det[5])
    v_d = traci.edge.getLastStepMeanSpeed(det[6])

    if I > para[0]:
        I_on = 1
    else:
        I_on = 0
    if I < para[1]:
        I_off = 1
    else:
        I_off = 0


    if v_u == -1 or v_d == -1:
        v_on = 0
        v_off = 0
    else:
        if v_u < para[2] or v_d < para[2]:
            v_on = 1
        elif v_u >= para[2] and v_d >= para[2]:
            v_on = 0
        elif v_u > para[3] or v_d > para[3]:
            v_off = 1
        elif v_u <= para[3] and v_d <= para[3]:
            v_off = 0

    # if I_on + v_on >= 1:
    if I_on >= 1:
        rm = 1
    elif I_off * v_off >= 1:
        rm = 0
    else:
        rm = lastrm

    if v_u == -1:
        v_u = None
    if v_d == -1:
        v_d = None
    return rm, v_u, v_d



