import traci
import numpy as np
from MPC_RG import mpcrg
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import pygad
import time
import itertools
import random


# d0 = 3500  # demand road 1
# d0_2 = 3400  # demand road 2
# dr = 800  # Demand Ramp 1
# rho1 = [10, 10, 10]  # density three cells road 1
# rho2 = [10, 10, 10]  # density three cells road 2
# q1 = 20
# q2 = 20
# w1 = 0
# w2 = 0
# inputs = [d0, d0_2, dr, rho1, rho2, q1, q2, w1, w2]

pextra = 1.5

def mpc_coordination(inputs):

    def speed_density_function1(dens, kc):
        return 105 * np.exp(-1 / 2 * (dens / kc) ** 2)

    def qrhofd(dens, kc):
        return dens * 105 * np.exp(-1 / 2 * (dens / kc) ** 2)


    #predictive model

    # print(inputs)


    # d0 = inputs[0]   # demand road 1
    # d0_2 = inputs[1]  # demand road 2
    # dr = inputs[2]  # Demand Ramp 1
    # rho1 = inputs[3] # density three cells road 1
    # rho2 = inputs[4]  # density three cells road 2
    # queue1 = inputs[5]
    # queue2 = inputs[6]
    # w1 = inputs[7]
    # r1 = inputs[5]
    # r2 = inputs[6]
    # vsl1 = inputs[7]
    # vsl2 = inputs[8]
    # p = inputs[9]

    ## =======================test from here ==============================
    # horizon = 18
    horizon = 12

    # # mock inputs

    # r1 = 0
    # r2 = 0
    # vsl1 = 30
    # vsl2 = 30
    # p = 1

    # states = [d0, d0_2, dr, rho1, rho2, queue1, queue2]

    nr_iter = 10
    dx = 0.3
    dt = 10 / 3600

    target_flow = 4000

    cap1 = [6000, 4000, 4000]  # Capacity three cells road 1
    kc1 = [90, 65, 65]  # congested density road 1
    cap2 = [6000, 6000, 4000]  # Capacity three cells road 2
    kc2 = [90, 90, 65]  # congested density road 2


    d1 = [0, 0, 0, 0]
    s1 = [0, 0, 0, 0]
    q1 = [0, 0, 0, 0]
    d2 = [0, 0, 0, 0]
    s2 = [0, 0, 0, 0]
    q2 = [0, 0, 0, 0]
    spd1 = [0, 0, 0]
    spd2 = [0, 0, 0]


    def celltransmission(d0_old, d0_2_old, dr_old, rho1_old, rho2_old, Queue_r1, Queue_r2, w1, w2, r1, r2, vsl1, vsl2, p):

        d0 = d0_old
        d0_2 = d0_2_old
        dr = dr_old
        rho1 = rho1_old
        rho2 = rho2_old



        def speed_density_function1(dens, kc):
            return 105 * np.exp(-1 / 2 * (dens / kc) ** 2)

        def qrhofd(dens, kc):
            return dens * 105 * np.exp(-1 / 2 * (dens / kc) ** 2)

        cur_spd1 = speed_density_function1(rho1[0], kc1[0])+speed_density_function1(rho1[1], kc1[1])+speed_density_function1(rho1[2], kc1[2])
        cur_spd2 = speed_density_function1(rho2[0], kc2[0])+speed_density_function1(rho2[1], kc2[1])+speed_density_function1(rho2[2], kc2[2])
        if cur_spd1 <= 0.85 * cur_spd2:
            beta_adj = pextra
        else:
            beta_adj = 1

        # rho = [1, 1, 1]
        d1_main = d0 * p * vsl1 / 30  # Main road upstream
        dr1act = min(r1, dr)
        d1[0] = d1_main + dr1act  # Main road all
        for j in range(len(d1) - 1):
            if rho1[j] <= kc1[j]:
                d1[j + 1] = qrhofd(rho1[j], kc1[j])  # demand: uncongested
            else:
                d1[j + 1] = cap1[j]  # Demand: congested
            if rho1[j] >= kc1[j]:
                s1[j] = qrhofd(rho1[j], kc1[j])  # supply: congested
            else:
                s1[j] = cap1[j]  # supply: uncongested
            if j == 2:
                q1[3] = qrhofd(rho1[j], kc1[j])  # outflow out of the network
            q1[j] = min(d1[j], s1[j])  # Flow update
            rho_cur = rho1[j]
            rho1[j] = rho_cur + (q1[j] - q1[j + 1]) * dt / dx  # Density update



        d2_main = d0_2 * vsl2 / 30
        dr2 = (1 - p) * d0 * beta_adj
        dr2_act = min(r2, dr2)
        d2[0] = d2_main + dr2_act

        for j in range(len(d2) - 1):
            if rho2[j] <= kc2[j]:
                d2[j + 1] = qrhofd(rho2[j], kc2[j])
            else:
                d2[j + 1] = cap2[j]
            if rho2[j] >= kc2[j]:
                s2[j] = qrhofd(rho2[j], kc2[j])
            else:
                s2[j] = cap2[j]
            if j == 2:
                q2[3] = qrhofd(rho2[j], kc2[j])
            q2[j] = min(d2[j], s2[j])
            rho_cur = rho2[j]
            rho2[j] = rho_cur + (q2[j] - q2[j + 1]) * dt / dx

        for j in range(len(rho1)):
            spd1[j] = speed_density_function1(rho1[j], kc1[j])
            spd2[j] = speed_density_function1(rho2[j], kc2[j])

        if rho1[0] > kc1[0]:
            if1 = min(qrhofd(rho1[0], kc1[0]), d1[0]+w1/dt)
        else:
            if1 = min(cap1[0], d1[0] + w1 / dt)

        if rho2[0] > kc2[0]:
            if2 = min(qrhofd(rho2[0], kc2[0]), d2[0]+w2/dt)
        else:
            if2 = min(cap2[0], d2[0]+w2/dt)

        # print(d1[0], d2[0])
        #
        # print(if1, if2)

        w1 = max(0, w1 + (d0 + dr1act - if1-dr2) * dt)
        w2 = max(0, w2 + (d0_2 + dr2_act - if2) * dt)
        # print(w1, w2)

        qu1 = max((dr - dr1act) * dt + Queue_r1, 0)
        queue_new1 = max(qu1, 0)
        qu2 = max((dr2 - dr2_act) * dt + Queue_r2, 0)
        queue_new2 = max(qu2, 0)


        return d0, d0_2, dr, rho1, rho2, queue_new1, queue_new2, w1, w2, q1, q2







    def objective_function(dtm_inputs):
        obj = []
        obj_outflow = []
        obj_rampflow = []
        obj_tt1 = []
        obj_tt2 = []
        obj_ttr1 = []
        obj_ttr2 = []
        obj_ttm1 = []
        obj_ttm2 = []
        d0 = inputs[0]  # demand road 1
        d0_2 = inputs[1]  # demand road 2
        dr = inputs[2]  # Demand Ramp 1
        rho1 = inputs[3]  # density three cells road 1
        rho2 = inputs[4]  # density three cells road 2
        queue1 = inputs[5]
        queue2 = inputs[6]
        w1 = inputs[7]
        w2 = inputs[8]
        states1 = [d0, d0_2, dr, rho1, rho2, queue1, queue2, w1, w2]
        penalty_imbalance = 0

        for i in range(horizon):
            if i % 6 == 0:
                Np = int(i/6)
                inputsnew = dtm_inputs[(Np*5):((Np+1)*5)]

                new_state = celltransmission(*states1[:9], *inputsnew)
            else:
                new_state = celltransmission(*inputs_new, *inputsnew)

            d0 = new_state[0]
            d0_2 = new_state[1]
            dr = new_state[2]
            rho1 = new_state[3]
            rho2 = new_state[4]
            queue1 = new_state[5]
            queue2 = new_state[6]
            inputs_new = new_state[:9]
            r1 = inputsnew[0]
            r2 = inputsnew[1]
            vsl1 = inputsnew[2]
            vsl2 = inputsnew[3]
            p = inputsnew[4]
            q1 = new_state[9]
            q2 = new_state[10]

            if i % 6 == 5:
                states1 = new_state
            for j in range(len(new_state[3])):
                spd1[j] = speed_density_function1(rho1[j], kc1[j])
                spd2[j] = speed_density_function1(rho2[j], kc2[j])
            # of1 = qrhofd(rho1[-1], kc1[-1])
            # of2 = qrhofd(rho2[-1], kc2[-1])
            of1 = q1[1]
            of2 = q2[2]
            w1 = inputs_new[7]
            w2 = inputs_new[8]
            tt1 = np.sum(rho1)*dt
            tt2 = np.sum(rho2)*0.9*dt
            ttr1 = queue1*dt
            ttr2 = queue2*dt





            # print(of1, of2)
            # print(np.mean(spd1), np.mean(spd2))
            # vnr1 = (rho1[0] + rho1[1])/2 * L1
            # vnr2 = (rho2[0] + rho2[1])/2 * L2
            # average_speed = (np.mean(spd1) * vnr1 + np.mean(spd2) * vnr2) / (vnr1 + vnr2)
            # obj.append(average_speed)
            obj_outflow.append(of1+of2)
            # obj_rampflow.append(r1+r2)
            obj_tt1.append(tt1)
            obj_tt2.append(tt2)
            obj_ttr1.append(ttr1)
            obj_ttr2.append(ttr2)
            obj_ttm1.append(w1*dt)
            obj_ttm2.append(w2*dt)
            flow_imbalance = np.abs(of1 - of2) / of1
            spd_imbalance = np.abs(np.mean(spd1) - np.mean(spd2)) / np.mean(spd1)
            if flow_imbalance >= 0.3 or spd_imbalance >= 0.3:
                penalty_imbalance += 300
            else:
                penalty_imbalance += 0
        tt_extra = 1
        # print(np.sum(obj_tt1), np.sum(obj_tt2), np.sum(obj_ttr1), np.sum(obj_ttr2), np.sum(obj_ttm1), np.sum(obj_ttm2))
        travel_time = np.sum(obj_tt1)+np.sum(obj_tt2)+tt_extra*np.sum(obj_ttm1)+tt_extra*np.sum(obj_ttm2)
        # print(np.sum(obj_tt1), np.sum(obj_tt2), tt_extra*np.sum(obj_ttr1), tt_extra*np.sum(obj_ttr2), tt_extra*np.sum(obj_ttm1), tt_extra*np.sum(obj_ttm2))

        # print(flow_imbalance, spd_imbalance)

        penalty_control = 0
        r1 = dtm_inputs[0]
        r2 = dtm_inputs[1]
        v1 = dtm_inputs[2]
        v2 = dtm_inputs[3]
        p = dtm_inputs[4]
        if v1 == 24:
            penalty_control += 0.25 * d0 * horizon * dt * horizon * 3600 *dt
        if v2 == 24:
            penalty_control += 0.25 * d0_2 * horizon * dt * horizon * 3600 *dt
        if v1 < 30 and r1 > 400:
            penalty_control += 0
        elif v2 < 30 and r2 > 400:
            penalty_control += 0
        elif p > 0.9 and r1 > 0:
            penalty_control += 0
        elif np.min(ttr1)/dt > 30 or np.min(ttr2)/dt > 20:
            penalty_control += 0
        else:
            penalty_control += 0
        diff = [0, 0, 0, 0, 0]
        for i in range(horizon//6-1):
            for j in range(5):
                diff[j] += np.abs(dtm_inputs[(i+1)*5+j]-dtm_inputs[i*5+j])
        # print(penalty_control)

        # print(6 / (horizon - 6) * (10 * (diff[0] + diff[1]) + 100 * (diff[2] + diff[3]) + 1000 * diff[4]))



        # obj_value = -np.mean(obj)-np.mean(obj_outflow)/50 + penalty_control + penalty_imbalance - r1/100 - r2/100

        #obj_value = -np.mean(obj_outflow) + penalty_control + penalty_imbalance + 6/(horizon-6)*(10*(diff[0]+diff[1]) + 100 * (diff[2] + diff[3])+1000*diff[4])
        obj_value = -np.mean(obj_outflow) + penalty_imbalance
        #obj_value = 3600*travel_time+penalty_control
        # print(obj_value)
        return obj_value


    # x0 = [dr, 600, 30, 30, 1]
    #
    # bound = [[0, 800], [0, 800], [24, 30], [24, 30], [0.5, 1]]
    #
    # hist = []
    #
    #
    # def callback1(*args):
    #     x = args[0]
    #     value = objective_function(x)
    #     print(x)
    #     hist.append(value)


    # result = minimize(objective_function, x0, bounds=bound, method="trust-constr", callback=callback1)
    #
    # x = range(result.nit)
    # plt.plot(x, hist)
    # objective_function([800, 600, 30, 30, 0.83])
    #
    # objective_function([200, 500, 30, 30, 1])

    if 0 < inputs[5] < 7:
        space_vsl1 = [24]
        space_rm1 = [800]
    else:
        space_vsl1 = [30]
        space_rm1 = range(300, 800, 10)

    if 0 < inputs[6] < 7:
        space_vsl2 = [24]
        space_rm2 = [800]

    else:
        space_vsl2 = [30]
        space_rm2 = range(300, 800, 10)
    # space_rm1 = range(300, 810, 50)
    # space_rm2 = range(300, 810, 50)
    # space_vsl1 = range(24, 31, 6)
    # space_vsl2 = range(24, 31, 6)
    space_rg = [x / 100 for x in range(30, 105, 5)]

    combinations = list(itertools.product(space_rm1, space_rm2, space_vsl1, space_vsl2, space_rg))

    filtered_combinations = [
        combo for combo in combinations if not (
            (combo[0] > 300 and combo[2] != 30) or
            (combo[1] > 300 and combo[3] != 30)
        )
    ]
    filtered_combinations = combinations

    def fitness_function(ga_instance, solution, solution_idx):
        dtm_inputs = solution
        total_v_ave = objective_function(dtm_inputs)
        return -total_v_ave  # Negative to maximize speed

    def fitness_function_test(solution):
        dtm_inputs = solution
        total_v_ave = objective_function(dtm_inputs)
        return -total_v_ave  # Negative to maximize speed

    # Define the number of genes and bounds for each variable
    # num_genes = 5
    # variable_bounds = [(300, 1200), (300, 1200), (24, 30), (24, 30), (0.5, 1)]
    #
    # # Create the initial population
    # num_solutions = 100
    # initial_population1 = np.random.uniform(low=np.array(variable_bounds)[:, 0], high=np.array(variable_bounds)[:, 1], size=(num_solutions, num_genes))

    initial_population = []

    for _ in range(300):
        # Randomly select two different 5-element lists
        selected_lists = random.sample(filtered_combinations, horizon//6)
        combined_list = []
        # Combine the selected lists into a single 10-element list
        for i in range(len(selected_lists)):
            combined_list += selected_lists[i]

        # Append the combined list to the combinations list
        initial_population.append(combined_list)
    rvalue1 = []
    rvalue2 = []
    for liste in initial_population:
        rvalue1.append(liste[0])
        rvalue2.append(liste[1])

    # Create a PyGAD instance
    ga_instance = pygad.GA(num_generations=20,
                           num_parents_mating=20,
                           initial_population=initial_population,
                           # gene_type=[int, int, int, int, [float, 2], int, int, int, int, [float, 2]],
                           # gene_space=[space_rm, space_rm, space_vsl, space_vsl, space_rg, space_rm, space_rm, space_vsl, space_vsl, space_rg],
                           fitness_func=fitness_function,
                           mutation_type=None,
                           parent_selection_type="rank",
                           crossover_type="uniform")

    # Record start time
    start_time = time.time()
    # Run the genetic algorithm
    ga_instance.run()

    best_fitness = ga_instance.best_solutions_fitness

    # Generate the convergence process plot
    plt.figure(figsize=(10, 6))
    plt.rcParams['font.size'] = 14
    plt.plot(best_fitness, marker='o')
    plt.title("Convergence Process Plot")
    plt.xlabel("Generation")
    plt.ylabel("Objective")
    plt.grid()
    plt.show()

    # Get the best solution found
    best_solution, best_solution_fitness, other = ga_instance.best_solution()



    print("Best Solution:", best_solution)
    print("Best Fitness (Objective):", -best_solution_fitness)

    # Record end time
    end_time = time.time()

    # Calculate elapsed time
    elapsed_time = end_time - start_time

    for i in range(6):
        if i == 0:
            states_ne = celltransmission(*inputs[:9], *best_solution[:5])
            updtd_input = states_ne[:9]
        else:
            states_ne = celltransmission(*updtd_input[:9], *best_solution[:5])
            updtd_input = states_ne[:9]
        if i == 5:
            waiting1 = states_ne[7]
            waiting2 = states_ne[8]


    # print(waiting1, waiting2)

    print(f"Running time: {elapsed_time:.6f} seconds")

    fitness_function_test([300, 300, 24, 24, 1, 300, 300, 24, 24, 1])
    fitness_function_test([700, 500, 30, 30, 1, 700, 500, 30, 30, 1])

    return best_solution, best_solution_fitness, waiting1, waiting2







