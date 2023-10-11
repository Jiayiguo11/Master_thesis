
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def drake_est(typical_edges, den_agg, spd_agg_km):
    bounds = ([90, 50], [110, 90])
    # def speed_density_function(dens, v0, k_c):
    #     return v0 * np.exp(-1/2 * (dens/k_c)**2)

    def speed_density_function(dens, v0, kc):
        return v0 * np.exp(-1 / 2 * (dens / kc) ** 2)

    def rmse(x, x_pre):
        sum = 0
        for i in range(len(x)):
            sum += (x[i]-x_pre[i])**2
        RMSE = np.sqrt(sum/len(x))
        return RMSE
    RMSE = {}
    v0 = [None for _ in range(len(typical_edges))]
    kc = [None for _ in range(len(typical_edges))]

    for i in range(len(typical_edges)):

        popt, pcov = curve_fit(speed_density_function, den_agg[i], spd_agg_km[i], bounds=bounds)

        # Extract the optimized parameters
        v0[i], kc[i] = popt

        # Generate a range of densities for plotting
        density_range = np.linspace(min(den_agg[i]), max(den_agg[i]), 100)

        # Calculate the corresponding speeds using the fitted parameters
        speed_predicted = speed_density_function(density_range, v0[i], kc[i])

        speed_pre = speed_density_function(den_agg, v0[i], kc[i])

        RMSE[typical_edges[i]] = rmse(speed_pre[i], spd_agg_km[i])
        # Plot the original data and the fitted curve
        plt.figure()
        plt.scatter(den_agg[i], spd_agg_km[i], s=1, label='Original Data')
        plt.plot(density_range, speed_predicted, 'r', label='Fitted Curve')
        plt.xlabel('Density [veh/km]')
        plt.ylabel('Speed [km/h]')
        plt.title('Density Speed relationship ' + typical_edges[i])
        plt.legend()
        plt.show()

        q = [x * y for x, y in zip(density_range, speed_predicted)]
        plt.figure()
        plt.plot(density_range, q)

    return RMSE, v0, kc


