import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

guiFlag = True

dt = 1/240 # pybullet simulation step
th0 = 0.5  # starting position (radian)
g = 10     # m/s^2
L = 0.8    # m
xd = 0.5
zd = 1

physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-g)
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

pos0 = p.getLinkState(boxId, 2)[0]
X0 = np.array([[pos0[0]],[pos0[2]]])

maxTime = 5 # seconds
logTime = np.arange(0, maxTime, dt)
sz = len(logTime)
logXsim = np.zeros(sz)
logZsim = np.zeros(sz)
idx = 0
T = 2
for t in logTime:
    th = p.getJointState(boxId, 1)[0]

    pos = p.getLinkState(boxId, 2)[0]
    logXsim[idx] = pos[0]
    logZsim[idx] = pos[2]

    jac = np.array([
        [-L * np.cos(th)],
        [ L * np.sin(th)]
    ])

    jac_pinv = np.linalg.pinv(jac)
    X = np.array([[pos[0]],[pos[2]]])
    Xd = np.array([[xd],[zd]])

    s = 1
    if t < T:
        tau = t / T
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    Xd_curr = X0 + s * (Xd - X0)

    vel_d = -100.0 * jac_pinv @ (X-Xd_curr)
    vel_d = vel_d.flatten()

    p.setJointMotorControl2(
        bodyIndex=boxId,
        jointIndex=1,
        targetVelocity=vel_d[0],
        controlMode=p.VELOCITY_CONTROL
    )    
    p.stepSimulation()

    idx += 1
    if guiFlag:
        time.sleep(dt)
p.disconnect()

plt.subplot(2,1,1)
plt.plot(logTime, logXsim)
plt.subplot(2,1,2)
plt.plot(logTime, logZsim)
plt.show()