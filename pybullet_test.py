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
sarge5EndEffectorIndex = 2
numJoints = 3

step = 0

targetPos = 0
maxForce = 800

commandMFB = 127

# LEG POSITIONS

xb = 4.0 #starting x base position of the front left foot
yb = -4.0 #starting y base position of the front left foot
zb = -1.5 #starting z base position of the front left foot

xs = 1.0 #stride forward & back
ys = -1.0 #stride left & right
zs = 0.2 #stride up & down


for i in range(p.getNumJoints(sarge5_Id)):
    p.setJointMotorControl2(sarge5_Id, i, p.POSITION_CONTROL, targetPosition=targetPos, force=maxForce)

while(1):
    time.sleep(1./240.)
    # frontRightHipTarget = p.readUserDebugParameter(frontRightHipTargetId)
    p.stepSimulation()
    if step > 8:
        step = 0
    step += 1 # add to step counter
    # print(step)
    
    FLxe = xb
    FLye = yb
    FLze = zb

    floatMFB = commandMFB / 127.0

    if step == 1:
        FLze = zb + zs
        FLxe = xb + ((floatMFB * xs)/4.0)
        print(FLxe)
    elif step == 2:
        FLxe = xb + ((floatMFB * xs)/2.0)
        print(FLxe)
    elif step == 3:
        FLxe = xb + ((floatMFB * xs)/4.0)
    elif step == 5:
        FLxe = xb - ((floatMFB * xs)/4.0)
    elif step == 6:
        FLxe = xb - ((floatMFB * xs)/2.0)
    elif step == 7:
        FLze = zb + zs
        FLxe = xb - ((floatMFB * xs)/4.0)
    elif step == 8:
        FLze = zb + zs

    
    pos = [FLxe, FLze, FLye]
    print(pos)
    orn = p.getQuaternionFromEuler([0, 0, 3.14])

    jointPoses = p.calculateInverseKinematics(sarge5_Id,
                                              sarge5EndEffectorIndex,
                                              pos,
                                              orn)
    
    for i in range(numJoints):
        p.setJointMotorControl2(bodyIndex = sarge5_Id,
                                jointIndex = i,
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = jointPoses[i],
                                targetVelocity = 0,
                                force = maxForce,
                                positionGain = 1,
                                velocityGain = 1)


    
    