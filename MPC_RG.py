from scipy.optimize import minimize
import numpy as np
import pygad
import matplotlib.pyplot as plt

def mpcrg(demand_2main, demand_1ramp, vehdemand, Vehnr_1, Vehnr_2, cap1, cap2, outflow1, outflow2, T, L1, L2):
    # vehdemand: detected vehicle demand at upstream;
    # Vehnr_1&2: current vehicle number at route1 and 2 (from edge function)
    # outflow1&2: current outflow (from loop detector)
    # T: time step

    # demand_2main = 3000
    # demand_1ramp = 1000
    # vehdemand = 3200
    # Vehnr_1 = 7
    # Vehnr_2 = 0
    # outflow1 = 4000
    # outflow2 = 3000
    # cap1 = 7000
    # cap2 = 7000
    # T = 10/3600
    # L1 = 0.6
    # L2 = 0.3

    def speed_density_function1(dens):
        return 105 * np.exp(-1 / 2 * (dens / 75) ** 2)
    def speed_density_function2(dens):
        return 105 * np.exp(-1 / 2 * (dens / 75) ** 2)

    def qrhofd(dens, kc):
        return dens * 105 * np.exp(-1 / 2 * (dens / kc) ** 2)

    of1_d = qrhofd(Vehnr_1/L1, 75)
    of2_d = qrhofd(Vehnr_2/L2, 75)
    of1 = min(of1_d, 4000)
    of2 = min(of2_d, 4000)

    cur_spd1 = speed_density_function1(Vehnr_1/L1)
    cur_spd2 = speed_density_function2(Vehnr_2/L2)
    if cur_spd1 <= 0.85 * cur_spd2:
        mtplr = 2.5
    else:
        mtplr = 1
    # print(mtplr)
    def objective(vnr1):
        vnr2 = vehdemand*T-vnr1
        demand1 = demand_1ramp * T + vnr1
        demand2 = demand_2main * T + mtplr * vnr2
        # roomleft1 = max(0, cap1*T - Vehnr_1 + outflow1 * T)
        # roomleft2 = max(0, cap2*T - Vehnr_2 + outflow2 * T)
        # nnxt1 = Vehnr_1 - outflow1 * T + min(demand1, roomleft1)
        # nnxt2 = Vehnr_2 - outflow2 * T + min(demand2, roomleft2)
        nnxt1 = Vehnr_1 - of1 * T + demand1
        nnxt2 = Vehnr_2 - of2 * T + demand2

        rho1 = nnxt1/L1
        rho2 = nnxt2/L2
        # rho3 = nnxt3/L3
        v1 = speed_density_function1(rho1)
        v2 = speed_density_function2(rho2)
        if np.abs((v1-v2)/v1) > 0.1:
            pnt = 10
        else:
            pnt = 0

        # print(v1, v2)
        # obj = np.abs((L1 / v1)*3600 - (L2 / v2)*3600)
        obj = -3600 * (L1/v1 * nnxt1 + L2/v2 * nnxt2)/(nnxt1+nnxt2) + pnt
        prediction = obj-pnt
        #obj = np.abs(rho1-rho2)
        return obj, prediction


    # Define the initial guess
    x0 = 3.0  # Example initial guess

    bdrlow = max(0, (vehdemand-1000)*T)
    bdrupper = vehdemand*T

    # Define the constraints (if any)
    # constraints = [{'type': 'ineq', 'fun': lambda x: x[0] + x[1] - 1}]  # Example constraint: x + y >= 1

    # # Solve the optimization problem
    # result = minimize(objective, x0, bounds=[(bdrlow, bdrupper)], method='L-BFGS-B')
    #
    # # Print the optimized result
    # # print(result)
    # # print(bdrlow, bdrupper)
    #
    # proportion = min(1, result.x/(vehdemand*T))
    # speed = result.fun



        # Define the number of genes and bounds for each variable

    range1 = [vehdemand * T * x / 100 for x in range(40, 100, 1)]

    obj1 = []
    pre = []
    for value in range1:
        [ob, pr] = objective(value)
        obj1.append(ob)
        pre.append(pr)

    # print(ob, pr)


    # x_ob = range(len(obj1))
    # plt.plot(x_ob, obj1)

    optimal_variable = obj1.index(max(obj1))

    proportion = np.round(optimal_variable*0.01+0.4, decimals=3)
    speed = pre[optimal_variable]

    # print(proportion, speed)

    return proportion, speed


