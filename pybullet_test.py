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
while(1):
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()