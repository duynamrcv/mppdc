import numpy as np
import math
import time

from enum import Enum
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

        self.gt = self.state

        # Drone radius
        self.radius = radius
        
        # Store drone path
        self.path = [np.concatenate([[self.time_stamp], self.state, self.control, [self.mode.value, self.scaling_factor], self.gt])]

    def updateState(self, control:np.array, dt:float):
        """
        Computes the states of drone after applying control signals
        """
        # Update
        position = self.state[:3]
        velocity = self.state[3:]
        
        next_velocity = velocity + control*dt

        # Control signal alignment
        norm_vel = np.linalg.norm(next_velocity)
        if norm_vel > VMAX:
            next_velocity = next_velocity/norm_vel*VMAX

        next_position = position + velocity*dt

        self.state = np.concatenate([next_position, next_velocity])
        self.control = control
        self.time_stamp = self.time_stamp + dt

        # Store
        if self.gt is None:
            self.gt = self.state
        self.path.append(np.concatenate([[self.time_stamp], self.state, self.control, [self.mode.value, self.scaling_factor], self.gt]))

    def setupController(self, dt=0.1):
        # nmpc timestep
        self.timestep = dt

    def computeControlSignal(self, drones):
        """
        Computes control velocity of the copter
        """
        # state = self.state.copy()
        observed_obstacles = self.observerObstacles()

        self.modeChanging(observed_obstacles)
        if self.mode == Mode.FORMATION:
            # Behaviors for formation
            v_m = self.behaviorMigration()
            v_f = self.behaviorFormation(drones)
            v_o = self.behaviorObstacle(observed_obstacles)
            v_c = self.behaviorCollision(drones)
            v_r = self.behaviorRandom()
            vel = W_nav*v_m + W_sep*v_f + W_obs*v_o + W_col*v_c + W_r*v_r
        else:
            leader_idx = self.selectLeader(drones)
            # Behaviors for tailgating
            v_m = self.behaviorMigration()
            v_t = self.behaviorTailgating(drones,leader_idx)
            v_o = self.behaviorObstacle(observed_obstacles)
            v_c = self.behaviorCollision(drones)
            v_r = self.behaviorRandom()
            vel = W_nav*v_m + W_sep*v_t + W_obs*v_o + W_col*v_c + W_r*v_r

        control = (vel - self.state[3:])/self.timestep

        # Control signal alignment
        norm_control = np.linalg.norm(control)
        if norm_control > UMAX:
            control = control/norm_control*UMAX
        # print(control)
        return control

    @staticmethod
    def behaviorMigration():
        return VREF*UREF
    
    def behaviorFormation(self, drones):
        v_f = np.zeros(self.n_control)
        gt = 0
        for i in range(NUM_UAV):
            if i == self.index:
                continue
            v_f += (drones[i].state[:3]-self.state[:3]) -\
                   (TOPOLOGY[i,:] - TOPOLOGY[self.index,:])*self.scaling_factor
            gt += drones[i].state[:3] - (TOPOLOGY[i,:] - TOPOLOGY[self.index,:])*self.scaling_factor
        self.gt = np.concatenate([gt,self.state[3:]])/(NUM_UAV-1)
        return v_f
    
    def behaviorTailgating(self, drones, leader_idx):
        if leader_idx == -1:
            self.gt = self.state
            return self.state[3:]
        v_t = drones[leader_idx].state[:3] - self.state[:3] - DREF*UREF + drones[leader_idx].state[3:]
        self.gt = drones[leader_idx].state - np.concatenate([DREF*UREF, drones[leader_idx].state[3:]])
        return v_t

    def behaviorObstacle(self, obstacles):
        if obstacles.shape[0] == 0:
            return np.zeros(self.n_control)
        
         # Get obstacles in front of robot in motion direction
        vec = self.state[:2] - obstacles
        obstacles = obstacles[np.where(vec*UREF[:2]<0),:][0]

        # DBSCAN clustering
        try:
            clusters = DBSCAN(eps = 2*ROBOT_RADIUS, min_samples = 3).fit(obstacles)
        except:
            return np.zeros(self.n_control)
        label = np.unique(clusters.labels_)
        vel_obs = np.zeros(self.n_control)
        
        for i in label:
            xi = obstacles[np.where(clusters.labels_==i),:][0]
            dx = self.state[0]-xi[:,0]
            dy = self.state[1]-xi[:,1]
            r = np.hypot(dx, dy)
            rs = np.min(r)
            rsidx = np.argmin(r)
            dir = np.array([dx[rsidx],dy[rsidx], 0])/rs
            # if rs <= ROBOT_RADIUS:
            #     return np.ones(self.n_control)*np.inf
            # vel_obs += dir*(1 - np.tanh(BETA*(rs - ROBOT_RADIUS)))/2
            vel_obs += dir*np.exp(-BETA*(rs-ROBOT_RADIUS))/(rs-ROBOT_RADIUS)
            # vel_obs += -1/2*(1/rs - 1/ROBOT_RADIUS)**2*dir
        return vel_obs/label.shape[0]
    
    def behaviorCollision(self, drones):
        vel_col = np.zeros(self.n_control)
        for j in range(NUM_UAV):
            if j == self.index:
                continue
            pos_rel = drones[j].state[:3] - self.state[:3]
            rs = np.linalg.norm(pos_rel)
            if rs >= DREF:
                continue
            dir = -pos_rel/rs
            # vel_col += dir*(1 - np.tanh(BETA*(rs - 2*ROBOT_RADIUS)))/2
            # vel_col += dir*np.exp(-BETA*(rs-2*ROBOT_RADIUS))/(rs-2*ROBOT_RADIUS)
            # vel_col += -1/2*(1/rs - 1/(2*ROBOT_RADIUS))**2*dir
            vel_col += (3*ROBOT_RADIUS-rs)/ROBOT_RADIUS*dir
        vel_col /= (NUM_UAV-1)
        return vel_col
    
    def behaviorRandom(self):
        return np.random.rand(self.n_control)

    def observerObstacles(self):
        observed_obstacles = []
        for j in range(OBSTACLES.shape[0]):
            obs = OBSTACLES[j,:]
            if np.linalg.norm(self.state[:2]-obs[:2]) <= SENSING_RADIUS:
                observed_obstacles.append(obs)
        return np.array(observed_obstacles)

    def selectLeader(self, drones):
        position = self.state[:2]
        positions = []
        for i in range(NUM_UAV):
            positions.append(drones[i].state[:2])
        positions = np.array(positions)
            
        vec = (positions[:,0]-position[0])*UREF[0] + (positions[:,1]-position[1])*UREF[1]
        vec[np.where(vec<=0)]=np.inf
        if np.all(np.isinf(vec)): # is leader
            return -1
        leader_index = np.argmin(vec)
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