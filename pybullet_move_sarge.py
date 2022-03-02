from ast import match_case
from fileinput import filename
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw = 45, cameraPitch = -45, cameraTargetPosition=[0,0,0])

cubeStartOrientation = p.getQuaternionFromEuler([1.57075,0,0])
cubeStartPos = [0,0,2]

sarge5_Id = p.loadURDF("URDF/05-sarge5_v1.urdf",cubeStartPos, cubeStartOrientation)
sarge5EndEffectorIndex = 3
numJoints = p.getNumJoints(sarge5_Id)

step = 0

maxForce = 800

commandMFB = 60 # value between -127 and 127

# LEG POSITIONS

xb = 2.76907 #starting x base position of the front left foot
yb = -.8402 #starting y base position of the front left foot
zb = 0 #starting z base position of the front left foot

xs = 2.0 #stride forward & back
ys = -2.0 #stride left & right
zs = 0.2 #stride up & down

pos = [xb, yb, zb]
orn = p.getQuaternionFromEuler([0, 0, 3.14])



jointPoses = p.calculateInverseKinematics(sarge5_Id,
                                          sarge5EndEffectorIndex,
                                          pos)

for i in range(numJoints - 1):
    p.setJointMotorControl2(sarge5_Id, i, p.POSITION_CONTROL, targetPosition = jointPoses[i], force=maxForce)

print(p.getJointInfo(sarge5_Id, 3))

while(1):
    time.sleep(1./240.)
    # frontRightHipTarget = p.readUserDebugParameter(frontRightHipTargetId)
    p.stepSimulation()
    if step > 8:
        step = 1
    step += 1 # add to step counter
    # print(step)
    
    FLxe = xb
    FLye = yb
    FLze = zb

    floatMFB = commandMFB / 127.0

    if step == 1:
        FLze = zb + zs
        FLxe = xb + (floatMFB * xs)
    elif step == 2:
        FLxe = xb + (floatMFB * xs)
    elif step == 3:
        FLxe = xb + (floatMFB * xs)
    elif step == 5:
        FLxe = xb - (floatMFB * xs)
    elif step == 6:
        FLxe = xb - (floatMFB * xs)
    elif step == 7:
        FLze = zb + zs
        FLxe = xb - (floatMFB * xs)
    elif step == 8:
        FLze = zb + zs

    
    pos = [FLxe, FLye, FLze]
    # print(pos)
    orn = p.getQuaternionFromEuler([0, 0, 3.14])

    """ jointPoses = p.calculateInverseKinematics(sarge5_Id,
                                              sarge5EndEffectorIndex,
                                              pos,
                                              orn) """
    
    """ for i in range(numJoints):
        p.setJointMotorControl2(bodyIndex = sarge5_Id,
                                jointIndex = i,
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = jointPoses[i],
                                targetVelocity = 0,
                                force = maxForce,
                                positionGain = 1,
                                velocityGain = 1) """


    
    