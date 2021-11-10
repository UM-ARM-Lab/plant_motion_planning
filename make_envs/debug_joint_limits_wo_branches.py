import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Remove debug visualizer
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)

# Creating base collision and visual ids
base_width = 0.2
col_base_id = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[base_width, base_width, base_width]
)
vis_base_id = -1

# Base position and orientation
base_pos = [0, 0, base_width]
base_ori = [0, 0, 0, 1]

def create_stem_element(length, width, height):

    col_stem_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[length, width, height],
        collisionFramePosition=[0, 0, stem_base_spacing + stem_half_height]
    )
    vis_stem_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[length, width, height]
    )

    col_v1_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0, 0, 0],
        collisionFramePosition=[0, 0, 0.0]
    )
    vis_v1_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0, 0, 0]
        # collisionFramePosition=[0, 0, 0]
    )

    return [col_v1_id, col_stem_id], [vis_v1_id, vis_stem_id]

# Creating stem collision visual ids
stem_half_width = 0.2
stem_base_spacing = 0.3
stem_half_height = 1 * stem_half_width

# col_stem1_id, vis_stem1_id = create_stem_element(stem_half_width, stem_half_width, 2 * stem_half_height)
# col_stem2_id, vis_stem2_id = create_stem_element(stem_half_width, stem_half_width, stem_half_height)
#
# link_Masses = [1, 1, 1, 1]
# # linkCollisionShapeIndices = [col_v1_id, col_stem_id, col_v2_id, col_stem_id1]
# # linkVisualShapeIndices = [vis_v1_id, vis_stem_id, vis_v2_id, vis_stem_id1]
# linkCollisionShapeIndices = col_stem1_id + col_stem2_id
# linkVisualShapeIndices = vis_stem1_id + vis_stem2_id
# linkPositions = [[0, 0, base_width], [0, 0, 0.0], [0, 0, stem_base_spacing + (2 * stem_half_height)], [0, 0, 0]]
# linkOrientations = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]]
# linkInertialFramePositions = [[0, 0, 0], [0, 0, 3],[0, 0, 0],[0, 0, 0]]
# linkInertialFrameOrientations = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]]
# indices = [0, 1, 2, 3]
# # indices = [-1, 0, 1, 2]
# # jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
# jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
# axis = [[1, 0, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0]]



col_stem1_id, vis_stem1_id = create_stem_element(stem_half_width, stem_half_width, 2 * stem_half_height)

link_Masses = [1, 1]
# linkCollisionShapeIndices = [col_v1_id, col_stem_id, col_v2_id, col_stem_id1]
# linkVisualShapeIndices = [vis_v1_id, vis_stem_id, vis_v2_id, vis_stem_id1]
linkCollisionShapeIndices = col_stem1_id
linkVisualShapeIndices = vis_stem1_id
linkPositions = [[0, 0, base_width], [0, 0, 0.0]]
linkOrientations = [[0, 0, 0, 1], [0, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0], [0, 0, 3]]
linkInertialFrameOrientations = [[0, 0, 0, 1], [0, 0, 0, 1]]
indices = [0, 1]
# indices = [-1, 0, 1, 2]
# jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
axis = [[1, 0, 0], [0, 1, 0]]


# Making the plant
base_id = p.createMultiBody(
    100000, col_base_id, vis_base_id, base_pos, base_ori,
    linkMasses=link_Masses, linkCollisionShapeIndices=linkCollisionShapeIndices,
    linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions,
    linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions,
    linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=indices,
    linkJointTypes=jointTypes, linkJointAxis=axis
)

for j in range(p.getNumJoints(base_id)):
    print(p.getJointInfo(base_id, j))

p.changeDynamics(base_id, -1, jointLowerLimit=-0.5, jointUpperLimit=0.5)
p.changeDynamics(base_id, 0, jointLowerLimit=-0.5, jointUpperLimit=0.5)
p.changeDynamics(base_id, 1, jointLowerLimit=-0.5, jointUpperLimit=0.5)

# input("")


p.setGravity(0, 0, -10)
# p.setRealTimeSimulation(1)

while(1):

    p.stepSimulation()
    time.sleep(1/240.0)