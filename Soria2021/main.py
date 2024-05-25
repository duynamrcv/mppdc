

from Drone import Drone
from config import *

import numpy as np
import time
import matplotlib.pyplot as plt

if __name__ == "__main__":
    drones = []
    # Initialize Drone
    for i in range(NUM_UAV):
        drone = Drone(i, STARTS[i,:], np.array([0,0,0]), ROBOT_RADIUS)
        drone.setupController(horizon_length=10, dt=TIMESTEP)
        drones.append(drone)
    
    compute_times = []
    iter = 0
    try:
        print("[INFO] Start")
        run = True
        while run:
            times = []
            for i in range(NUM_UAV):
                # compute velocity using nmpc
                start = time.time()
                control = drones[i].computeControlSignal(drones)
                times.append(time.time()-start)
                drones[i].updateState(control, TIMESTEP)

            compute_times.append(times)
            iter += 1
            if iter % 10 == 0:
                print("Iteration {}".format(iter))

            # Reach terminal condition
            count = 0
            for i in range(NUM_UAV):
                if drones[i].state[0] > X_GOAL:
                    count += 1
            run = count < NUM_UAV
    finally:
        print("[INFO] Saving")
        # Saving
        import pickle
        data = {}
        for i in range(NUM_UAV):
            data[i] = np.array(drones[i].path)
        with open(SAVE_FILE, 'wb') as file:
            pickle.dump(data, file, protocol=pickle.HIGHEST_PROTOCOL)

        compute_times = np.array(compute_times)
        print("Average time: {:.6}s".format(compute_times.mean()))
        print("Max time: {:.6}s".format(compute_times.max()))   
        print("Min time: {:.6}s".format(compute_times.min()))
