from ast import match_case
from cmath import sqrt
from fileinput import filename
from xml.etree.ElementTree import PI
import math
import pybullet as p
import time
import pybullet_data
import calc.leg as l

GRAVITY = -9.8

QB = math.pi / 4
R0FL = 0.0

LB = math.sqrt(pow(1.016,2) + pow(1.016,2)) # 101.60 mm length + 101.60 mm width
L0 = 0.935 # 93.5 mm length
L1 = 0.935 # 93.5 mm length
L2 = 1.02 # 102 mm length

RB = 0.0
PB = 0.0
YB = 0.0


# Set Legs
FL_Leg = l.Leg(QB, R0FL, LB, L0, L1, L2, RB, PB, YB)
FR_Leg = l.Leg(QB, R0FL, LB, L0, L1, L2, RB, PB, YB)
BR_Leg = l.Leg(QB, R0FL, LB, L0, L1, L2, RB, PB, YB)
BL_Leg = l.Leg(QB, R0FL, LB, L0, L1, L2, RB, PB, YB)

""" TESTING
FL_Leg.calcAngles(2.00, 2.00, -1.50)

Q0 = FL_Leg.getQ0()
Q1 = FL_Leg.getQ1()
Q2 = FL_Leg.getQ2()

print("Q0: " + str(Q0))
print("Q1: " + str(Q1))
print("Q2: " + str(Q2))

    END TESTING """

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,GRAVITY)
planeId = p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw = 45, cameraPitch = -45, cameraTargetPosition=[0,0,0])

cubeStartOrientation = p.getQuaternionFromEuler([1.57075,0,0])
cubeStartPos = [0,0,2]

sarge5_Id = p.loadURDF("../URDF/05-sarge5_v1.urdf",cubeStartPos, cubeStartOrientation)
sarge5EndEffectorIndex = 3
# numJoints = p.getNumJoints(sarge5_Id)
numJoints = p.getNumJoints(sarge5_Id) - 2
maxForce = 800

FLxe = p.addUserDebugParameter("FLxe", 2, 3, 2)
FLye = p.addUserDebugParameter("FLye", 2, 3, 2)
FLze = p.addUserDebugParameter("FLze", -2, 0, -1.5)

FRxe = p.addUserDebugParameter("FRxe", 2, 3, 2)
FRye = p.addUserDebugParameter("FRye", 2, 3, 2)
FRze = p.addUserDebugParameter("FRze", -2, 0, -1.5)

BRxe = p.addUserDebugParameter("BRxe", 2, 3, 2)
BRye = p.addUserDebugParameter("BRye", 2, 3, 2)
BRze = p.addUserDebugParameter("BRze", -2, 0, -1.5)

BLxe = p.addUserDebugParameter("BLxe", 2, 3, 2)
BLye = p.addUserDebugParameter("BLye", 2, 3, 2)
BLze = p.addUserDebugParameter("BLze", -2, 0, -1.5)

for i in range(p.getNumJoints(sarge5_Id)):
    if (i - 3) % 4 != 0:
        print(p.getJointInfo(sarge5_Id, i))
        p.setJointMotorControl2(sarge5_Id, i, p.POSITION_CONTROL, targetPosition=0, force=maxForce)


while(1):
    time.sleep(1./240.)
    # frontRightHipTarget = p.readUserDebugParameter(frontRightHipTargetId)
    p.stepSimulation()

    _FLxe = p.readUserDebugParameter(FLxe)
    _FLye = p.readUserDebugParameter(FLye)
    _FLze = p.readUserDebugParameter(FLze)

    _FRxe = p.readUserDebugParameter(FRxe)
    _FRye = p.readUserDebugParameter(FRye)
    _FRze = p.readUserDebugParameter(FRze)

    _BRxe = p.readUserDebugParameter(BRxe)
    _BRye = p.readUserDebugParameter(BRye)
    _BRze = p.readUserDebugParameter(BRze)

    _BLxe = p.readUserDebugParameter(BLxe)
    _BLye = p.readUserDebugParameter(BLye)
    _BLze = p.readUserDebugParameter(BLze)
    
    FL_Leg.calcAngles(_FLxe, _FLye, _FLze)
    FR_Leg.calcAngles(_FRxe, _FRye, _FRze)
    BR_Leg.calcAngles(_BRxe, _BRye, _BRze)
    BL_Leg.calcAngles(_BLxe, _BLye, _BLze)
    
    # pos = [_FLxe, _FLze, _FLye]
    # print(pos)
    # orn = p.getQuaternionFromEuler([0, 0, 3.14])

    """ jointPoses = p.calculateInverseKinematics(sarge5_Id,
                                              sarge5EndEffectorIndex,
                                              pos,
                                              orn) """
    

    
    jointPoses = [FL_Leg.getQ0(), FL_Leg.getQ1(), FL_Leg.getQ2(),
                     FR_Leg.getQ0(), FR_Leg.getQ1(), FR_Leg.getQ2(),
                     BR_Leg.getQ0(), BR_Leg.getQ1(), BR_Leg.getQ2(),
                     BL_Leg.getQ0(), BL_Leg.getQ1(), BL_Leg.getQ2()]

    
    # print(jointPoses)
    ind = 0
    for i in range(numJoints + 1): 
        if (i - 3) % 4 != 0:
            p.setJointMotorControl2(bodyIndex = sarge5_Id,
                                    jointIndex = i,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPosition = jointPoses[ind],
                                    targetVelocity = 0,
                                    force = maxForce,
                                    positionGain = 0.3,
                                    velocityGain = 1)
            ind += 1 #increment by one for jointPoses list


    
    