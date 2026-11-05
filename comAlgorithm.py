import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# collected raw angular velocity of rw
dataFile = "MotorData.csv"

# read data
df = pd.read_csv(dataFile, header=None) 

# pandas data frame of time stamps, all rows, column 0
timestamp_us = df.iloc[:, 0]

# ang vel rw data column 2, 3, 4
qCurrentMotorA = df.iloc[:, 2] 
qCurrentMotorB = df.iloc[:, 3] 
qCurrentMotorC = df.iloc[:, 4] 

# convert to numpy arrays for analysis
timestamp_us = timestamp_us.to_numpy()

qCurrentMotorA = qCurrentMotorA.to_numpy(dtype=float)
qCurrentMotorB = qCurrentMotorB.to_numpy(dtype=float)
qCurrentMotorC = qCurrentMotorC.to_numpy(dtype=float)

# Torque constant from motor data sheet
kT = 0.08 # [Nm/A]

# Torque of motors [Nm]
tauRwX =  qCurrentMotorA*kT 
tauRwY = qCurrentMotorB*kT
tauRwZ = qCurrentMotorC*kT

gravityVector = [0, 0, 9.81] # [m/s^2]

# Reaction wheel allocation to the body frame
R_b_rw = [[np.sqrt(2)/2,  -np.sqrt(2)/4,  -np.sqrt(2)/4]
          [      0,        np.sqrt(6)/4,  -np.sqrt(6)/4]
          [np.sqrt(2)/2,   np.sqrt(2)/2,   np.sqrt(2)/2]]

# Moment of inertia of each reaction wheel about its CM
IrwA = 6.1
IrwB = 6.1
IrwC = 6.1

# Total Mass of prototype assembly [kg]
M = 8888  

# Adjustable Mass [kg]
m = 3      
g = 9.81

# Mass moment of inertia of reaction wheels
Irw = [[IrwA, 0, 0]
      [0, IrwB, 0]
      [0, 0, IrwC]]

# External torque acting on the system in the inertial ref frame
# tauExt = Irw*angAccelRw*R_b_rw
tauExtX = IrwA*tauRwX*R_b_rw
tauExtY = IrwB*tauRwY*R_b_rw

# The gravity torque is the main external torque acting on the system
# tauGrav = r x m*g = -tauExt
tauGravX = -tauExtX
tauGravY = -tauExtY

# Solved for distance offset [m]
rX_minus = -tauGravY / (M*g)
rY_minus = tauGravX / (M*g)

# Displacement of adjustable mass needed to correct [m]
# rCM_plus = rCM_minus + (m/M)*deltaR_a
deltaR_x = -rX_minus * (M/m)
deltaR_y = -rY_minus * (M/m)



