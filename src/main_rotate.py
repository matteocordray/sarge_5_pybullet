from ast import match_case
from cmath import sqrt
from fileinput import filename
from xml.etree.ElementTree import PI
from simple_pid import PID
import math
import pybullet as p
import numpy as np
import time
import pybullet_data
import calc.leg as l
import calc.rotate as r

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

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,GRAVITY)
p.setRealTimeSimulation(1)
planeId = p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw = 45, cameraPitch = -45, cameraTargetPosition=[0,0,0])

cubeStartOrientation = p.getQuaternionFromEuler([1.57075,0,0])
cubeStartPos = [0,0,2]

#Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
cyaw=90
cpitch=-10# -20
cdist=5

sarge5_Id = p.loadURDF("../URDF/05-sarge5_v1.urdf",cubeStartPos, cubeStartOrientation)
maxForce = 50

# LEG POSITIONS

xb = 2.0 #starting x base position of the front left foot
yb = 2.0 #starting y base position of the front left foot
zb = -1.5 #starting z base position of the front left foot

xs = 1 #stride forward & back
ys = 1 #stride left & right
zs = 0.4 #stride up & down

#Scenery e.g. an inclined box
boxHalfLength = 20
boxHalfWidth = 20
boxHalfHeight = 0.2
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
sh_colBox_down = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
box_orn = p.getQuaternionFromEuler([-0.1, 0, 0])
box_down_orn = p.getQuaternionFromEuler([0.1, 0, 0])
block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [0,-6,-0.1],baseOrientation = [box_orn[0], box_orn[1], box_orn[2], box_orn[3]])
block_down = p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox_down,
                        basePosition = [0,-45.6,-0.1],baseOrientation = [box_down_orn[0], box_down_orn[1], box_down_orn[2], box_down_orn[3]])
p.changeDynamics(block, -1, lateralFriction=2)
p.changeDynamics(block_down, -1, lateralFriction=2)

# DEBUG PARAMETERS

trailDuration = 5

FL_Pose = [0, 0, 0]
FL_prevPose = [0, 0, 0]
FL_hasPrevPose = 0

FR_Pose = [0, 0, 0]
FR_prevPose = [0, 0, 0]
FR_hasPrevPose = 0

BR_Pose = [0, 0, 0]
BR_prevPose = [0, 0, 0]
BR_hasPrevPose = 0

time_c = int(p.addUserDebugParameter("time", 5, 20, 10))
comMFB = p.addUserDebugParameter("speed", 0, 127, 50)
rotation = p.addUserDebugParameter("Rotation (radians)", -1.57079, 1.57079, 0)

p.changeDynamics(sarge5_Id,3,lateralFriction=2)
p.changeDynamics(sarge5_Id,7,lateralFriction=2)
p.changeDynamics(sarge5_Id,11,lateralFriction=2)
p.changeDynamics(sarge5_Id,15,lateralFriction=2)
    
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


def setRotatedBase(pitch):
    global FLxe, FLye, FLze, FRxe, FRye, FRze, BRxe, BRye, BRze, BLxe, BLye, BLze
    global xb, yb, zb
    # ROTATE TESTS
    
    p1 = [xb, yb, zb]
    
    alpha = 0 #(30/180.0)*np.pi
    beta  = pitch  #(int(pitch)/180.0)*np.pi
    gamma = 0 #(70/180.0)*np.pi
    
    p2 = r.rotate(p1, alpha, beta, gamma)
    p2_n = r.rotate(p1, alpha, -beta, gamma)
    # print(f'Pitch: {int((pitch*180.0)/np.pi)} P2: {p2}')

    _xb = p2[0]
    _yb = p2[1]
    _zb = p2[2]

    n_xb = p2_n[0]
    n_yb = p2_n[1]
    n_zb = p2_n[2]
    
    FLxe = _xb
    FLye = _yb
    FLze = _zb
    FRxe = _xb
    FRye = _yb
    FRze = _zb
    BRxe = n_xb
    BRye = n_yb
    BRze = n_zb
    BLxe = n_xb
    BLye = n_yb
    BLze = n_zb

    

setDefaultBase() # set default base values
# setRotatedBase((10/180.0)*np.pi)
t = 0

sarge_orn = 0

f = open("matlab/output.txt", "w")

running = False
backwards = False
while(1):
    
    
    # print(f'Yaw: {(sarge_orn[0]*180.0/np.pi) - 90.0}° Pitch: {(sarge_orn[1]*180.0/np.pi)}° Roll: {(sarge_orn[2]*180.0/np.pi)}°')
    # print(f'Pitch: {(sarge_orn[1]*180.0/np.pi)}°')
    sargePos = p.getBasePositionAndOrientation(sarge5_Id)[0]
    p.resetDebugVisualizerCamera(cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=sargePos)
    
    keys = p.getKeyboardEvents() 
    #Keys to change camera
    if keys.get(100):  #D
        cyaw+=0.1
    if keys.get(97):   #A
        cyaw-=0.1
    if keys.get(99):   #C
        cpitch+=0.01
    if keys.get(102):  #F
        cpitch-=0.01
    if keys.get(122):  #Z
        cdist+=0.01
    if keys.get(120):  #X
        cdist-=0.01
    
    time_control = int(p.readUserDebugParameter(time_c))
    # while (0):
    # sarge_orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(sarge5_Id)[1])
    sarge_orn = p.readUserDebugParameter(rotation)

    setRotatedBase(sarge_orn)

    FL_Leg.calcAngles(FLxe, FLye, FLze)
    FR_Leg.calcAngles(FRye, FRxe, FRze) # THIS HAS DIFFERENT COORDINATE SYSTEM
    BR_Leg.calcAngles(BRxe, BRye, BRze)
    BL_Leg.calcAngles(BLye, BLxe, BLze) # THIS HAS DIFFERENT COORDINATE SYSTEM   

    
    """ This will find the next angle in the sequence"""
    
    jointPoses = [FL_Leg.getQ0(), FL_Leg.getQ1(), FL_Leg.getQ2(),
                     FR_Leg.getQ0(), FR_Leg.getQ1(), FR_Leg.getQ2(),
                     BR_Leg.getQ0(), BR_Leg.getQ1(), BR_Leg.getQ2(),
                     BL_Leg.getQ0(), BL_Leg.getQ1(), BL_Leg.getQ2()]

    FL_Pose = [FLye, FLze, FLxe]
    FR_Pose = [-FRxe, FRye, FRze]
    BR_Pose = [-BRxe, BRze, -BRye]

    # DEBUG
    # if (FL_hasPrevPose and FR_hasPrevPose):
        # p.addUserDebugLine(FL_prevPose, FL_Pose, [1, 0, 0], 1, trailDuration, sarge5_Id)
        # p.addUserDebugLine(FR_prevPose, FR_Pose, [1, 0, 0], 1, trailDuration, sarge5_Id)
        # p.addUserDebugLine(BR_prevPose, BR_Pose, [1, 0, 0], 1, trailDuration, sarge5_Id)

    FL_prevPose = FL_Pose
    FL_hasPrevPose = 1
    FR_prevPose = FR_Pose
    FR_hasPrevPose = 1
    BR_prevPose = BR_Pose
    BR_hasPrevPose = 1
    
    
    # print(jointPoses)
    ind = 0
    for i in range(p.getNumJoints(sarge5_Id)): 
        if (i - 3) % 4 != 0:
            p.setJointMotorControl2(bodyIndex = sarge5_Id,
                                    jointIndex = i,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPosition = jointPoses[ind],
                                    maxVelocity = 3,
                                    force = maxForce)
            ind += 1 #increment by one for jointPoses list


    
    