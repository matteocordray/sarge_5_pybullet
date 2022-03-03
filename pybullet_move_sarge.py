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
# numJoints = p.getNumJoints(sarge5_Id)
numJoints = 4

maxForce = 800

FLxe = p.addUserDebugParameter("FLxe", -7, 7, -3.5)
FLye = p.addUserDebugParameter("FLye", -7, 7, 3.5)
FLze = p.addUserDebugParameter("FLze", -7, 7, 3.5)

for i in range(p.getNumJoints(sarge5_Id)):
  p.setJointMotorControl2(sarge5_Id, i, p.POSITION_CONTROL, targetPosition=0, force=maxForce)

""" THIS SECTION IS FOR LEG POSITIONS """
def calcAngles(xe, ye, ze):
    xm11 = _LB*cos(_QB)

while(1):
    time.sleep(1./240.)
    # frontRightHipTarget = p.readUserDebugParameter(frontRightHipTargetId)
    p.stepSimulation()

    _FLxe = p.readUserDebugParameter(FLxe)
    _FLye = p.readUserDebugParameter(FLye)
    _FLze = p.readUserDebugParameter(FLze)
    
    
    pos = [_FLxe, _FLze, _FLye]
    # print(pos)
    orn = p.getQuaternionFromEuler([0, 0, 3.14])

    jointPoses = p.calculateInverseKinematics(sarge5_Id,
                                              sarge5EndEffectorIndex,
                                              pos,
                                              orn)
    
    for i in range(numJoints - 1):
        p.setJointMotorControl2(bodyIndex = sarge5_Id,
                                jointIndex = i,
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = jointPoses[i],
                                targetVelocity = 0,
                                force = maxForce,
                                positionGain = 0.3,
                                velocityGain = 1)


    
    