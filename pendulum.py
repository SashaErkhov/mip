import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

def quintic_trajectory(t, th_0, th_d, T):
    if t <= 0:
        return th_0, 0, 0
    if t >= T:
        return th_d, 0, 0
    tau = t/T
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = (30 * t**2 / T**3) - (60 * t**3 / T**4) + (30 * t**4 / T**5)
    dds = (60 * t / T**3) - (180 * t**2 / T**4) + (120 * t**3 / T**5)

    th_ref = th_0 + s * (th_d - th_0)
    dth_ref = ds * (th_d - th_0)
    ddth_ref = dds * (th_d - th_0)
    return th_ref, dth_ref, ddth_ref


guiFlag = True

dt = 1/240 # pybullet simulation step
th0 = 0.1  # starting position (radian)
thd = 1.0  # desired position (radian)
kp = 40.0  # proportional coefficient
ki = 40.0
kd = 20.0
g = 10     # m/s^2
L = 0.8    # m
m = 1      # kg

physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-g)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf.xml", useFixedBase=True)

# get rid of all the default damping forces
# think of it as imagined "air drag"
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=th0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

maxTime = 5 # seconds
logTime = np.arange(0, 5, dt)
sz = len(logTime)
logThetaSim = np.zeros(sz)
logThetaRef = np.zeros(sz) # Опорная траектория
logVelSim = np.zeros(sz)
logTauSim = np.zeros(sz)
logAccSim = np.zeros(sz) # Ускорение
idx = 0
T = 2
last_vel = 0
for t in logTime:
    th = p.getJointState(boxId, 1)[0]
    vel = p.getJointState(boxId, 1)[1]

    th_ref, dth_ref, ddth_ref = quintic_trajectory(t, th0, thd, T)

    logThetaSim[idx] = th
    logThetaRef[idx] = th_ref
    logAccSim[idx] = (vel - last_vel)/dt
    last_vel = vel

    e = th - th_ref
    de = vel - dth_ref

    # Feedback linearization
    tau = (m*L*L)*(g/L*np.sin(th) + ddth_ref - kp * e - kd * de)

    logTauSim[idx] = tau

    p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=tau, controlMode=p.TORQUE_CONTROL)
    p.stepSimulation()
    vel = p.getJointState(boxId, 1)[1]
    logVelSim[idx] = vel

    idx += 1
    if guiFlag:
        time.sleep(dt)
p.disconnect()

plt.subplot(4,1,1)
plt.plot(logTime, logThetaSim, 'b', label="Sim Pos")
plt.plot([logTime[0], logTime[-1]], [thd, thd], 'r--', label="Ref Pos")
plt.plot(logTime, logThetaRef, 'y--', label="Ref Pos (Quintic)")
plt.grid(True)
plt.legend()

plt.subplot(4,1,2)
plt.plot(logTime, logVelSim, 'b', label="Sim Vel")
plt.grid(True)
plt.legend()

plt.subplot(4,1,3)
plt.subplot(4,1,3)
plt.plot(logTime, logAccSim, 'b', label="Sim Acc (Calculated)")
plt.grid(True)
plt.legend()

plt.subplot(4,1,4)
plt.plot(logTime, logTauSim, 'b', label="Sim Tau")
plt.grid(True)
plt.legend()
plt.show()

# dt = 0.1
# dx = f(x,t) t = [0, 0.1, 0.2]
# x(t)
# dx/dt = f(x,t)
# dx = dt * f(x,t)
# x[n+1] - x[n] = dt * f(x,t)
# x[n+1] = x[n] + dt*f(x,t) # Euler method

# lim(dx/dt) dt -> 0

# ddth = -g/L * sin(th)
# mL^2*ddth + mgLsin(th) = tau
# ddth = - g/Lsin(th) + tau/(mL^2)
# tau = (mL^2)g/Lsin(th) + u(t) -> ddth = -g/Lsin(th) + ((mL^2)g/Lsin(th) + u(t))/(mL^2)
# ddth = -g/Lsin(th) + g/Lsin(th) + u(t)
# ddth = u(t)
# ddth = kp(th-thd)
# Feedback linearization
# Линеаризация обратной связью

# dx = ax
# dx(t) = f(x,t)
# dth = w
# dw = -g/Lsin(th)

# dth = w
# dw = -g/L*th
# X = (th, w)
# dX = A*X = [0 1; -g/L 0]
# X = e^(A*t)

# dx/dt = ax
# dx / x = a dt
# ln(x) = at + C
# x = e^(at)

# LTI
# dX = A*X + B*tau
# tau = K*X
# dx = A*X + B*K*X = (A+BK)X

# Forward Kinematics
# x = -L1*sin(th1) - L2*sin(th1+th2)
# z = H - L1*cos(th1) - L2*cos(th1+th2)

# dx = -L1*cos(th1)*dth1 - L2*cos(th1+th2)*(dth1+dth2)
# dz = L1*sin(th1)*dth1 + L2*sin(th1+th2)*(dth1+dth2)

# dx = (-L1*cos(th1) - L2*cos(th1+th2))*dth1 - L2*cos(th1+th2) * dth2
# dz = (L1*sin(th1) + L2*sin(th1+th2))*dth1 + L2*sin(th1+th2) * dth2
# X = (x,z)'
# Th = (th1, th2)'
# dX = J(Th) * dTh
# dTh = inv(J) * dX
# dX = k(Xd - X)