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
    fileName="OBJ/Sarge_5_Body_Frame_v1.obj",
    meshScale=[0.25,0.25,0.25]
)
cubePosition = [0, 0, 0]
cubeStartOrientation = p.getQuaternionFromEuler([1.57075,0,0])

body_id = p.createMultiBody(
    baseCollisionShapeIndex=shape_id,
    basePosition=(cubePosition[0], cubePosition[1], cubePosition[2]),
    baseOrientation=(cubeStartOrientation[0], cubeStartOrientation[1], 
                     cubeStartOrientation[2], cubeStartOrientation[3]),
)

cubeStartPos = [0,0,0.5]

# boxId = p.loadURDF("URDF/04-materials.urdf",cubeStartPos, cubeStartOrientation)
while(1):
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()