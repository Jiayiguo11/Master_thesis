import traci


def rm(rid, mveh, rveh, r_pass, cap, switch):


    ##test inputs
    # rid = "R1on1_connection"
    # mveh = 6100
    # rveh = 700
    # r_pass = 1
    # r_app = 1

    if switch == 0:
        traci.trafficlight.setProgram(rid, "green")
        traci.trafficlight.setPhase(rid, 0)
        # t = traci.trafficlight.getProgram(rid)
        # print(t)
        ramp_rate = rveh * 60
        cp = traci.trafficlight.getPhase(rid)

    elif switch == 2:
        ramp_rate = 700
        desired_cycle_time = 3600 / ramp_rate
        green_time = 2  # fixed green time
        yellow_time = green_time / 2  # fixed yellow time (1/4 of the green time)
        red_time = max(desired_cycle_time - green_time - yellow_time, 2)  # fixed red time

        phase_time = [green_time, yellow_time, red_time]

        # Define the initial phase and time values
        current_phase = traci.trafficlight.getPhase(rid)
        # current_phase_time = traci.trafficlight.getPhaseDuration(rid)
        current_phase_time = phase_time[current_phase]

        # red time
        if current_phase == 1:
            traci.trafficlight.setPhase(rid, 2)
            traci.trafficlight.setPhaseDuration(rid, phase_time[2])

        cp = traci.trafficlight.getPhase(rid)
    else:
        traci.trafficlight.setProgram(rid, "0")
        # Calculate the current mainline flow rate
        mainline_flow = 60*mveh
        ramp_flow = 60*rveh

        # Calculate the maximum ramp flow allowed based on demand-capacity control
        capacity = cap
        qr_min = 300
        left_capacity = max(capacity - mainline_flow, 300)
        ramp_rate = min(ramp_flow, left_capacity)



        # Calculate the maximum ramp flow allowed based on the desired cycle time
        desired_cycle_time = 3600 / max(ramp_rate, qr_min)
        green_time = 2  # fixed green time
        yellow_time = 1  # fixed yellow time (1/4 of the green time)
        red_time = max(desired_cycle_time - green_time - yellow_time, 2)  # fixed red time


        phase_time = [green_time, yellow_time, red_time]

        # Define the initial phase and time values
        current_phase = traci.trafficlight.getPhase(rid)
        # current_phase_time = traci.trafficlight.getPhaseDuration(rid)
        current_phase_time = phase_time[current_phase]

        # red time
        if current_phase == 1:
            traci.trafficlight.setPhase(rid, 2)
            traci.trafficlight.setPhaseDuration(rid, phase_time[2])

        # switch to yellow after detected passed vehicle after signal stop line

        # elif current_phase == 0:
        #
        #     if r_pass > 0:
        #         traci.trafficlight.setPhase(rid, 1)
        #         traci.trafficlight.setPhaseDuration(rid, phase_time[1])
        #     if r_pass == 0:
        #         traci.trafficlight.setPhase(rid, 0)
        #         traci.trafficlight.setPhaseDuration(rid, phase_time[0])
        #
        # elif current_phase == 2:
        #
        #     if r_pass > 0:
        #         traci.trafficlight.setPhase(rid, 2)
        #         traci.trafficlight.setPhaseDuration(rid, phase_time[2])
        #     if r_pass == 0:
        #         traci.trafficlight.setPhase(rid, 0)
        #         traci.trafficlight.setPhaseDuration(rid, phase_time[0])

        cp = traci.trafficlight.getPhase(rid)
    return cp, ramp_rate






