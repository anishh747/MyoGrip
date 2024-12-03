import pybullet as p
import pybullet_data
import os
import time

# Connect to PyBullet
p.connect(p.GUI)

# Set search path for default PyBullet data (e.g., plane.urdf)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0, 0, -9.81)

# Load the robotic hand
current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(current_dir, "urdf/hand.urdf")
hand_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.5])

# Add a plane (ground) at the XY-plane to stop the sphere from falling
plane_id = p.loadURDF("plane.urdf", basePosition=[
                      0, 0, 0])  # Ground at (0, 0, 0)


# Get the number of joints in the robotic hand
num_joints = p.getNumJoints(hand_id)
print(f"Number of joints in the robotic hand: {num_joints}")

# Print joint information to identify the finger joints
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(hand_id, joint_index)
    print(f"Joint {joint_index}: {joint_info[1].decode('utf-8')} (Type: {joint_info[2]})")


palm_pos, palm_orient = p.getLinkState(hand_id, 0)[:2]

# print("Palm Position:", palm_pos)
# print("Palm Orientation:", palm_orient)


sphere_radius = 0.03  # 5 cm radius
# In front of the hand (adjust x for distance)
sphere_start_position = (palm_pos[0] + 0.2, palm_pos[1], palm_pos[2]+0.03)
sphere_collision_shape = p.createCollisionShape(
    p.GEOM_SPHERE, radius=sphere_radius)
sphere_visual_shape = p.createVisualShape(
    p.GEOM_SPHERE, radius=sphere_radius, rgbaColor=[1, 0, 0, 1])  # Red sphere
sphere_id = p.createMultiBody(
    baseMass=0.1,  # Mass of the sphere
    baseCollisionShapeIndex=sphere_collision_shape,
    baseVisualShapeIndex=sphere_visual_shape,
    basePosition=sphere_start_position
)

# Specify the joint index for a finger you want to move (replace with the correct joint index)
# finger_joint_index = 4  # Example index, replace with your actual finger joint index
# target_position = 0.8  # Target position in radians
# finger_joint_indices = [2, 3, 5, 6, 7, 10, 11, 12, 13, 15, 16]
# target_positions = [1, 1, -0.5, 1, 1, 1, 1, 1, 1, 1, 1, 1]
finger_joint_indices = [5]
target_positions = [1]

# Move the finger in a loop
while True:
    # Set a target position for the finger joint
    # p.setJointMotorControl2(
    #     bodyUniqueId=hand_id,
    #     jointIndex=finger_joint_index,
    #     controlMode=p.POSITION_CONTROL,
    #     targetPosition=target_position,
    #     force=50.0  # Maximum force to apply
    # )

    for joint_index, target_position in zip(finger_joint_indices, target_positions):
        p.setJointMotorControl2(
            bodyUniqueId=hand_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=1.0
        )

    # Step the simulation
    p.stepSimulation()

    # Pause for visualization
    # time.sleep(0.5)
