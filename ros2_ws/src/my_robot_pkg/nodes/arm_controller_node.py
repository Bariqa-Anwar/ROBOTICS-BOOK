import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose # For target pose
from sensor_msgs.msg import JointState # For joint commands
import numpy as np

# Conceptual IK solver demonstrating pseudo-inverse Jacobian logic
class SimpleIKSolver:
    def __init__(self, link_lengths=[0.1, 0.1, 0.1]): # Example link lengths
        self.link_lengths = np.array(link_lengths)
        self.num_joints = len(link_lengths)
        # The Node object isn't available here to call get_logger()
        # if self.num_joints != 3:
        #    self.get_logger().warn("This conceptual IK solver is designed for a 3-DOF arm.")
        self.joint_angles = np.zeros(self.num_joints) # Initial joint angles

    def forward_kinematics(self, joint_angles):
        # Conceptual Forward Kinematics for a 3-DOF arm (e.g., planar or simple serial)
        # This is a highly simplified example for illustration.
        # Real FK involves Denavit-Hartenberg parameters.
        x = self.link_lengths[0] * np.cos(joint_angles[0]) + \
            self.link_lengths[1] * np.cos(joint_angles[0] + joint_angles[1]) + \
            self.link_lengths[2] * np.cos(joint_angles[0] + joint_angles[1] + joint_angles[2])
        y = self.link_lengths[0] * np.sin(joint_angles[0]) + \
            self.link_lengths[1] * np.sin(joint_angles[0] + joint_angles[1]) + \
            self.link_lengths[2] * np.sin(joint_angles[0] + joint_angles[1] + joint_angles[2])
        z = 0.0 # Assuming planar for simplicity of conceptual Jacobian

        return np.array([x, y, z])

    def calculate_jacobian(self, joint_angles):
        # Conceptual Jacobian for a 3-DOF planar arm (dx/dq, dy/dq)
        # Simplified for illustration purposes.
        # J = [ dx/dq1 dx/dq2 dx/dq3 ]
        #     [ dy/dq1 dy/dq2 dy/dq3 ]
        l1, l2, l3 = self.link_lengths
        q1, q2, q3 = joint_angles

        # Partial derivatives with respect to q1, q2, q3 (conceptual)
        J11 = -l1*np.sin(q1) - l2*np.sin(q1+q2) - l3*np.sin(q1+q2+q3)
        J12 = -l2*np.sin(q1+q2) - l3*np.sin(q1+q2+q3)
        J13 = -l3*np.sin(q1+q2+q3)

        J21 = l1*np.cos(q1) + l2*np.cos(q1+q2) + l3*np.cos(q1+q2+q3)
        J22 = l2*np.cos(q1+q2) + l3*np.cos(q1+q2+q3)
        J23 = l3*np.cos(q1+q2+q3)

        # For a 3-DOF arm in 3D space, this would be a 6x3 Jacobian
        # For conceptual purposes, we'll use a 3x3 for position
        jacobian = np.array([
            [J11, J12, J13],
            [J21, J22, J23],
            [0.0, 0.0, 0.0] # Assuming Z is constant for this planar example
        ])
        return jacobian


    def solve_ik(self, target_pose: Pose, current_joint_state: JointState):
        # Iterative Pseudo-Inverse Jacobian IK solver (conceptual)
        # This is a highly simplified iteration for demonstration.
        # In practice, this requires more robust convergence criteria and damping.

        self.joint_angles = np.array(current_joint_state.position[:self.num_joints])

        target_position = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        
        # Iteration parameters
        max_iterations = 100
        tolerance = 1e-4
        learning_rate = 0.1 # Small step for stability

        for i in range(max_iterations):
            current_position = self.forward_kinematics(self.joint_angles)
            error = target_position - current_position

            # Check if error is within tolerance
            if np.linalg.norm(error) < tolerance:
                break

            jacobian = self.calculate_jacobian(self.joint_angles)

            # Calculate pseudo-inverse (J_pinv = J.T @ inv(J @ J.T))
            # Handle singular cases for conceptual robustness
            if jacobian.shape[0] < jacobian.shape[1]: # Under-actuated
                jacobian_pinv = jacobian.T @ np.linalg.pinv(jacobian @ jacobian.T)
            else: # Over-actuated or square
                jacobian_pinv = np.linalg.pinv(jacobian)
            
            # Update joint angles
            delta_q = learning_rate * jacobian_pinv @ error
            self.joint_angles += delta_q[:self.num_joints] # Apply changes to our joints

        return self.joint_angles
class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        self.subscription = self.create_subscription(
            Pose,
            '/target_pose', # Topic for desired end-effector pose
            self.target_pose_callback,
            10)
        self.subscription # prevent unused variable warning
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_commands', 10)
        self.ik_solver = SimpleIKSolver()

        # Initialize current joint state (dummy for now)
        self.current_joint_state = JointState()
        self.current_joint_state.name = ['joint1', 'joint2', 'joint3']
        self.current_joint_state.position = [0.0, 0.0, 0.0]

        self.get_logger().info('Arm Controller Node initialized. Waiting for target poses...')

    def target_pose_callback(self, msg: Pose):
        self.get_logger().info(f'Received target pose: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}')

        # Solve IK to get joint angles
        new_joint_angles = self.ik_solver.solve_ik(msg, self.current_joint_state)

        # Publish joint commands
        joint_cmd_msg = JointState()
        joint_cmd_msg.header.stamp = self.get_clock().now().to_msg()
        joint_cmd_msg.name = self.current_joint_state.name
        joint_cmd_msg.position = new_joint_angles.tolist() # Convert numpy array to list
        self.joint_state_publisher.publish(joint_cmd_msg)
        self.get_logger().info(f'Published joint commands: {joint_cmd_msg.position}')

        # Update current joint state (for simplicity, assume immediate execution)
        self.current_joint_state.position = new_joint_angles.tolist()

def main(args=None):
    rclpy.init(args=args)
    arm_controller_node = ArmControllerNode()
    rclpy.spin(arm_controller_node)
    arm_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
