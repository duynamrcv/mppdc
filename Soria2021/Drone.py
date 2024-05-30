import numpy as np
import math
import time

from enum import Enum
from scipy.optimize import minimize, Bounds
from sklearn.cluster import DBSCAN
from config import *

class Mode(Enum):
    FORMATION = 0
    TAILGATING = 1

class Drone:
    def __init__(self, index:int, state:np.array, control:np.array, radius):
        self.index = index
        self.mode = Mode.FORMATION
        self.scaling_factor = 1
        
        # Drone state and control
        self.time_stamp = 0.0
        self.state = state
        self.control = control

        self.n_state = 6
        self.n_control = 3

        # Drone radius
        self.radius = radius

        # Drone control bounds
        self.control_max = [ 2.0, 2.0, 0.0]
        self.control_min = [-0.0,-2.0, 0.0]
        
        # Store drone path
        self.path = [np.concatenate([[self.time_stamp], self.state, self.control, [self.mode.value, self.scaling_factor]])]
        self.errors = []

    def updateState(self, control:np.array, dt:float):
        """
        Computes the states of drone after applying control signals
        """
        
        # Update
        position = self.state[:3]
        velocity = self.state[3:]

        next_velocity = velocity + control*dt
        next_position = position + next_velocity*dt

        self.state = np.concatenate([next_position, next_velocity])
        self.control = control
        self.time_stamp = self.time_stamp + dt

        # Store
        self.path.append(np.concatenate([[self.time_stamp], self.state, self.control, [self.mode.value, self.scaling_factor]]))

    def setupController(self, horizon_length=20, dt=0.1):
        # Boundary
        self.upper_bound = self.control_max*horizon_length
        self.lower_bound = self.control_min*horizon_length

        # nmpc timestep
        self.nmpc_timestep = NMPC_SCALE*dt

        # Predictive length
        self.horizon_length = horizon_length

        # History predictive horizon
        self.states_prediction = np.zeros(self.n_state*horizon_length)
        self.controls_prediction = np.random.rand(self.n_control*self.horizon_length)
    
    def computeControlSignal(self, drones):
        """
        Computes control velocity of the copter
        """
        state = self.state.copy()
        observed_obstacles = self.observerObstacles()

        # u0 = np.random.rand(self.control.shape[0]*self.horizon_length)
        u0 = self.controls_prediction
        def cost_fn(u): return self.costFunction(u, state, drones, observed_obstacles)

        bounds = Bounds(self.lower_bound, self.upper_bound)

        res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds, tol=10e-6, options=dict(maxiter=1e5))
        control = res.x[:3]
        self.states_prediction = self.predictTrajectory(state, res.x)
        self.controls_prediction = res.x

        # For save error
        pos_des = 0
        for i in range(NUM_UAV):
            pos_des += drones[i].state[:3] - (TOPOLOGY[i,:]-TOPOLOGY[self.index,:])

        self.errors.append(np.linalg.norm(self.state[:3]-pos_des/NUM_UAV))
        return control

    def costFunction(self, u, state, drones, obstacles):
        traj = self.predictTrajectory(state, u)

        c_u = self.costControl(u)
        c_sep = self.costSeparation(traj, drones)
        c_dir = self.costDirection(traj)
        c_nav = self.costNavigation(traj)
        c_obs = self.costObstacle(traj, obstacles)
        c_col = self.costInterAgent(traj, drones)
        total = W_sep*c_sep + W_dir*c_dir + W_nav*c_nav + W_u*c_u + W_obs*c_obs + W_col*c_col

        return total

    def costControl(self, u):
        return np.sum(u**2)

    def costSeparation(self, traj, drones):
        cost_sep = 0
        for j in range(NUM_UAV):
            if j == self.index:
                continue
            for i in range(self.horizon_length):
                pos_rel = drones[j].states_prediction[self.n_state*i:self.n_state*i+3] \
                                               - traj[self.n_state*i:self.n_state*i+3]
                cost_sep += (np.sum(pos_rel**2) - np.sum((TOPOLOGY[j,:]-TOPOLOGY[self.index,:])**2))**2
        return cost_sep/(NUM_UAV-1)

    def costDirection(self, traj):
        cost_dir = 0
        for i in range(self.horizon_length):
            vel = traj[self.n_state*i+3:self.n_state*(i+1)]
            cost_dir += (1 - np.sum(vel*UREF)**2/np.sum(vel**2))**2
        return cost_dir
    
    def costNavigation(self, traj):
        cost_nav = 0
        for i in range(self.horizon_length):
            vel = traj[self.n_state*i+3:self.n_state*(i+1)]
            cost_nav += (np.sum(vel**2) - VREF**2)**2
        return cost_nav

    def costObstacle(self, traj, obstacles):
        cost_obs = 0
        if obstacles.shape[0] == 0:
            return cost_obs
        retraj = traj.reshape(self.horizon_length, self.n_state)
        dx = retraj[:,0]-obstacles[:,0,None]
        dy = retraj[:,1]-obstacles[:,1,None]
        r = np.hypot(dx, dy)
        rs = np.min(r,axis=0)
        for i in range(self.horizon_length):
            cost_obs += 1/(1 + np.exp(4*(rs[i] - ROBOT_RADIUS)))

        return cost_obs
    
    def costInterAgent(self, traj, drones):
        cost_igt = 0
        for j in range(NUM_UAV):
            if j == self.index:
                continue
            for i in range(self.horizon_length):
                pos_rel = drones[j].states_prediction[self.n_state*i:self.n_state*i+3] - traj[self.n_state*i:self.n_state*i+3]
                pos_dis = np.linalg.norm(pos_rel)
                if pos_dis < 3*ROBOT_RADIUS:
                    cost_igt += (3*ROBOT_RADIUS-pos_dis)/ROBOT_RADIUS
        return cost_igt

    def observerObstacles(self):
        observed_obstacles = []
        for j in range(OBSTACLES.shape[0]):
            obs = OBSTACLES[j,:]
            if np.linalg.norm(self.state[:2]-obs[:2]) <= SENSING_RADIUS:
                observed_obstacles.append(obs)
        return np.array(observed_obstacles)

    def predictTrajectory(self, state, controls):
        """
        Computes the states of the system after applying a sequence of control signals u on
        initial state x0
        """
        trajectory = []
        for i in range(self.horizon_length):
            # Update
            position = state[:3]
            velocity = state[3:]
            control = controls[self.n_control*i:self.n_control*(i+1)]

            next_position = position + velocity*self.nmpc_timestep
            next_velocity = velocity + control*self.nmpc_timestep
            state = np.concatenate([next_position, next_velocity])
            trajectory.append(state)
        return np.array(trajectory).reshape((state.shape[0]*self.horizon_length))

    def getTrajectoryFormation(self, drones):
        traj_ref = np.zeros(self.n_state*self.horizon_length)
        delta_idx = np.vstack([np.eye(self.n_state)]*self.horizon_length)@np.concatenate([TOPOLOGY[self.index,:], [0,0,0]])
        for i in range(NUM_UAV):
            if i == self.index:
                continue
            delta_i = np.vstack([np.eye(self.n_state)]*self.horizon_length)@np.concatenate([TOPOLOGY[i,:], [0,0,0]])
            traj_ref += drones[i].states_prediction - (delta_i - delta_idx)*self.scaling_factor
        traj_ref /= (NUM_UAV-1)
        return traj_ref

    def getTrajectoryFollow(self, leader_idx, drones):
        if leader_idx == -1: # leader
            return None
        leader_prediction = drones[leader_idx].states_prediction
        traj_ref = np.zeros(self.n_state*self.horizon_length)
        for i in range(self.horizon_length):
            traj_ref[self.n_state*i:self.n_state*(i+1)] = leader_prediction[self.n_state*i:self.n_state*(i+1)] \
                                                        - np.concatenate([DREF*UREF/NMPC_SCALE, [0,0,0]])
        return traj_ref
    
    def selectLeader(self, drones):
        position = self.state[:2]
        positions = []
        for i in range(NUM_UAV):
            positions.append(drones[i].state[:2])
        positions = np.array(positions)
            
        vec = (positions[:,0]-position[0])*UREF[0] + (positions[:,1]-position[1])*UREF[1]
        vec[np.where(vec<=0)]=np.inf
        leader_index = np.argmin(vec)
        if leader_index == self.index:   # is leader
            return -1
        return leader_index

    def modeChanging(self, obstacles:np.array):
        # Check obstacles
        if obstacles.shape[0] == 0:
            self.mode = Mode.FORMATION
            self.scaling_factor = 1.0
            return
        
        # Estimate width of environment
        we = self.estimateEnvironmentWidth(obstacles)
        if we is None:
            self.mode = Mode.FORMATION
            self.scaling_factor = 1.0
            return

        # Mode selection
        if we <= 5*ROBOT_RADIUS:
            self.mode = Mode.TAILGATING
            self.scaling_factor = -1
        else:
            # Estimate width of formation
            wf = self.estimateFormationWidth()
            scaling_factor = 1.0
            if we - 2*ROBOT_RADIUS < wf:
                scaling_factor = (we - 2*ROBOT_RADIUS)/wf
            self.mode = Mode.FORMATION
            self.scaling_factor = scaling_factor

    @staticmethod
    def estimateFormationWidth():
        y_left = np.min(TOPOLOGY[:,1])
        y_right = np.max(TOPOLOGY[:,1])
        return y_right - y_left
    
    def estimateEnvironmentWidth(self, obstacles):
        # Get obstacles in front of robot in motion direction
        vec = self.state[:2] - obstacles
        obstacles = obstacles[np.where(vec*UREF[:2]<0),:][0]

        # DBSCAN clustering
        try:
            clusters = DBSCAN(eps = 2*ROBOT_RADIUS, min_samples = 3).fit(obstacles)
        except:
            return None
        label = np.unique(clusters.labels_)
        if label.shape[0] != 2:
            return None
        
        x = []
        for i in label:
            xi = obstacles[np.where(clusters.labels_==i),:][0]
            # Find the environment's width
            distance = abs( (xi[:,0]-self.state[0])*UREF[1] +\
                            (xi[:,1]-self.state[1])*UREF[0])
            index = np.argmin(distance)
            x.append(xi[index,:])
        
        theta = math.atan2(UREF[1], UREF[0])
        width = abs((x[0][0]-x[1][0])*np.sin(theta) + (x[0][1]-x[1][1])*np.cos(theta))
        return width