import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Remove debug visualizer
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0,0)

# Creating base collision and visual ids
base_width = 0.2
col_base_id = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[base_width, base_width, base_width]
)
vis_base_id = -1

# Base position and orientation
base_pos = [0.2, 0.2, 0.2]
base_ori = [0, 0, 0, 1]

# Creating stem collision visual ids
stem_width = 0.2
col_stem_id = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[stem_width, stem_width, 3 * stem_width],
    collisionFramePosition=[0, 0, 0.4]
)
vis_stem_id = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[stem_width, stem_width, 3 * stem_width]
)

col_v1_id = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[0, 0, 3 * stem_width],
    collisionFramePosition=[0, 0, 0.4]
)
vis_v1_id = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[stem_width, stem_width, 3 * stem_width]
)

link_Masses = [1, 1]
linkCollisionShapeIndices = [col_stem_id]
linkVisualShapeIndices = [vis_stem_id]
linkPositions = [[0, 0, 0.7]]
linkOrientations = [[0, 0, 0, 1]]
linkInertialFramePositions = [[10, 10, 10]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
indices = [0]
jointTypes = [p.JOINT_REVOLUTE]
axis = [[0, 1, 0]]

# Making the plant
base_id = p.createMultiBody(
    100000, col_base_id, vis_base_id, base_pos, base_ori,
    linkMasses=link_Masses, linkCollisionShapeIndices=linkCollisionShapeIndices,
    linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions,
    linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions,
    linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=indices,
    linkJointTypes=jointTypes, linkJointAxis=axis
)


p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

while(1):
    time.sleep(1/240.0)