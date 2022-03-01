from fileinput import filename
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")

cubeStartOrientation = p.getQuaternionFromEuler([1.57075,0,0])
cubeStartPos = [0,0,1]

sarge5_Id = p.loadURDF("URDF/05-sarge5_v1.urdf",cubeStartPos, cubeStartOrientation)

targetPos = 0
maxForce = 100
for i in range(p.getNumJoints(sarge5_Id)):
    p.setJointMotorControl2(sarge5_Id, i, p.POSITION_CONTROL, targetPosition=targetPos, force=maxForce)

frontRightHipTargetId = p.addUserDebugParameter("Front Right Hip Angle", -1.570796327, 1.570796327, -1)

frontRightHipJointIndex = 0

while(1):
    time.sleep(1./240.)
    frontRightHipTarget = p.readUserDebugParameter(frontRightHipTargetId)
    p.setJointMotorControl2(sarge5_Id,
                            frontRightHipJointIndex,
                            p.POSITION_CONTROL,
                            targetPosition=frontRightHipTarget,
                            force=maxForce)
    p.stepSimulation()
    
p.disconnect()