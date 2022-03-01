from fileinput import filename
import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="STL/Sarge_5_Body_Frame_Matteo_v1.stl",
)
body_id = p.createMultiBody(
    baseCollisionShapeIndex=shape_id,
    basePosition=(0, 0, 0),
    baseOrientation=(0, 0, 0, 1),
)
cuid = p.createCollisionShape(p.GEOM_BOX, halfExtents = [1, 1, 1])
mass= 0 #static box
p.createMultiBody(mass,cuid)

cubeStartPos = [0,0,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("URDF/04-materials.urdf",cubeStartPos, cubeStartOrientation)
while(1):
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()