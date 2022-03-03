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
# GRAVITY = 0

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
p.setRealTimeSimulation(1)
planeId = p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw = 45, cameraPitch = -45, cameraTargetPosition=[0,0,0])

cubeStartOrientation = p.getQuaternionFromEuler([1.57075,0,0])
cubeStartPos = [0,0,2]

#Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
cyaw=10
cpitch=-20
cdist=8

sarge5_Id = p.loadURDF("../URDF/05-sarge5_v1.urdf",cubeStartPos, cubeStartOrientation)
sarge5EndEffectorIndex = 3
# numJoints = p.getNumJoints(sarge5_Id)
numJoints = p.getNumJoints(sarge5_Id) - 2
maxForce = 500

commandMFB = 60
step = 0

# LEG POSITIONS

xb = 2.0 #starting x base position of the front left foot
yb = 2.0 #starting y base position of the front left foot
zb = -1.5 #starting z base position of the front left foot

xs = 1.5 #stride forward & back
ys = 1.5 #stride left & right
zs = 0.2 #stride up & down

""" FLxe = p.addUserDebugParameter("FLxe", 2, 3, 2)
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
BLze = p.addUserDebugParameter("BLze", -2, 0, -1.5) """

for i in range(p.getNumJoints(sarge5_Id)):
    if (i - 3) % 4 != 0:
        print(p.getJointInfo(sarge5_Id, i))
        p.setJointMotorControl2(sarge5_Id, i, p.POSITION_CONTROL, targetPosition=0, force=maxForce)


t = 0

# Defaults for first run

def setDefaultBase():
    global FLxe, FLye, FLze, FRxe, FRye, FRze, BRxe, BRye, BRze, BLxe, BLye, BLze
    FLxe = xb
    FLye = yb
    FLze = zb
    FRxe = xb
    FRye = yb
    FRze = zb
    BRxe = xb
    BRye = yb
    BRze = zb
    BLxe = xb
    BLye = yb
    BLze = zb

setDefaultBase() # set default base values

while(1):
    time.sleep(1./240.)
    # frontRightHipTarget = p.readUserDebugParameter(frontRightHipTargetId)
    # p.stepSimulation()
    sargePos, sargeOrn = p.getBasePositionAndOrientation(sarge5_Id)
    p.resetDebugVisualizerCamera(cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=sargePos)
    
    keys = p.getKeyboardEvents()
    #Keys to change camera
    if keys.get(100):  #D
        cyaw+=1
    if keys.get(97):   #A
        cyaw-=1
    if keys.get(99):   #C
        cpitch+=1
    if keys.get(102):  #F
        cpitch-=1
    if keys.get(122):  #Z
        cdist+=.01
    if keys.get(120):  #X
        cdist-=.01
    
    t += 1
    if (t % 10) == 0:
        if t > 3000:
            t = 0
        print("Timer: " + str(t))
        step += 1
        if step > 8:
            step = 0
        # add to step counter
        # print(step)
        
        setDefaultBase()

        floatMFB = commandMFB / 127.0

        if step == 1:
            # z movement
            FLze = zb + zs
            BRze = zb + zs
            
            # x movement
            FLxe = xb + ((floatMFB * xs)/4.0)
            FRxe = xb - ((floatMFB * xs)/4.0)
            BRxe = xb + ((floatMFB * xs)/4.0)
            BLxe = xb - ((floatMFB * xs)/4.0)
        elif step == 2:
            # x movement
            FLxe = xb + ((floatMFB * xs)/2.0)
            FRxe = xb - ((floatMFB * xs)/2.0)
            BRxe = xb + ((floatMFB * xs)/2.0)
            BLxe = xb - ((floatMFB * xs)/2.0)
        elif step == 3:
            # z movement
            FRze = zb + zs
            BLze = zb + zs
            
            # x movement
            FLxe = xb + ((floatMFB * xs)/4.0)
            FRxe = xb + ((floatMFB * xs)/4.0)
            BRxe = xb + ((floatMFB * xs)/4.0)
            BLxe = xb + ((floatMFB * xs)/4.0)
        elif step == 4:
            # z movement
            FRze = zb + zs
            BLze = zb + zs
        elif step == 5:
            # z movement
            FRze = zb + zs
            BLze = zb + zs
            
            # x movement
            FLxe = xb - ((floatMFB * xs)/4.0)
            FRxe = xb + ((floatMFB * xs)/4.0)
            BRxe = xb - ((floatMFB * xs)/4.0)
            BLxe = xb + ((floatMFB * xs)/4.0)
        elif step == 6:
            # x movement
            FLxe = xb - ((floatMFB * xs)/2.0)
            FRxe = xb + ((floatMFB * xs)/2.0)
            BRxe = xb - ((floatMFB * xs)/2.0)
            BLxe = xb + ((floatMFB * xs)/2.0)
        elif step == 7:
            # z movement
            FLze = zb + zs
            BRze = zb + zs

            # x movement
            FLxe = xb - ((floatMFB * xs)/4.0)
            FRxe = xb + ((floatMFB * xs)/4.0)
            BRxe = xb - ((floatMFB * xs)/4.0)
            BLxe = xb + ((floatMFB * xs)/4.0)
        elif step == 8:
            # z movement
            FLze = zb + zs
            BRze = zb + zs
        
        print("step: " + str(step))
        print("FLxe: " + str(FLxe))
        print("FRxe: " + str(FRxe))
        print("BRxe: " + str(BRxe))
        print("BLxe: " + str(BLxe))

    """
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
    """

    FL_Leg.calcAngles(FLxe, FLye, FLze)
    FR_Leg.calcAngles(FRxe, FRye, FRze)
    BR_Leg.calcAngles(BRxe, BRye, BRze)
    BL_Leg.calcAngles(BLxe, BLye, BLze)
    
    # pos = [_FLxe, _FLze, _FLye]
    # print(pos)
    # orn = p.getQuaternionFromEuler([0, 0, 3.14])

    """ jointPoses = p.calculateInverseKinematics(sarge5_Id,
                                              sarge5EndEffectorIndex,
                                              pos,
                                              orn) """
    

    
    """ This will find the next angle in the sequence"""
    
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
                                    targetVelocity = 0.1,
                                    force = maxForce,
                                    positionGain = 0.3,
                                    velocityGain = 1)
            ind += 1 #increment by one for jointPoses list


    
    