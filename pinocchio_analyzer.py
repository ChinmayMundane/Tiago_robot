#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pinocchio as pin
import numpy as np

class PinocchioAnalyzer(Node):
    def __init__(self):
        super().__init__('pinocchio_analyzer')
        
        # Create subscription to robot_description
        self.subscription = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            10)
        
        self.robot_description = None
        self.model = None
        self.data = None
        self.geometry_model = None
        self.geometry_data = None
        
        self.get_logger().info('Pinocchio Analyzer Node has started')

    def robot_description_callback(self, msg):
        """Callback for receiving robot description"""
        self.get_logger().info('Received robot description message')
        if self.robot_description is None:
            self.robot_description = msg.data
            self.initialize_pinocchio()
            self.run_tests()
            
    def initialize_pinocchio(self):
        """Initialize Pinocchio model from URDF"""
        try:
            # Load the robot model from the URDF string
            self.model = pin.buildModelFromXML(self.robot_description)
            self.data = self.model.createData()  # Older API
            
            # Create geometry model for collision checking
            self.geometry_model = pin.buildGeomFromUrdfString(
                self.model, 
                self.robot_description,
                pin.GeometryType.COLLISION
            )
            self.geometry_data = self.geometry_model.createData()  # Older API
            
            self.get_logger().info('Successfully initialized Pinocchio model')
            self.get_logger().info(f'Number of joints: {self.model.nq}')
            self.get_logger().info(f'Number of bodies: {self.model.nbodies}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Pinocchio model: {str(e)}')

    def run_tests(self):
        """Execute all requested tests"""
        if self.model is None or self.data is None:
            self.get_logger().error('Model not initialized, cannot run tests')
            return

        # Test 1: Create Pinocchio Model (already done in initialization)
        self.get_logger().info('Test 1: Pinocchio model creation - PASSED')

        # Test 2: Forward Kinematics
        self.test_forward_kinematics()

        # Test 3: Jacobian Calculation
        self.test_jacobian()

        # Test 4: Collision Checking
        self.test_collision()

    def test_forward_kinematics(self):
        """Test forward kinematics calculation"""
        try:
            # Set neutral joint configuration (all zeros)
            q = pin.neutral(self.model)
            
            # Perform forward kinematics
            pin.forwardKinematics(self.model, self.data, q)
            
            # Get transform of the last frame (assuming end effector)
            frame_id = self.model.nframes - 1
            M = self.data.oMf[frame_id]
            
            self.get_logger().info('Test 2: Forward Kinematics')
            self.get_logger().info(f'Frame name: {self.model.frames[frame_id].name}')
            self.get_logger().info(f'Transform: \n{str(M)}')
            
        except Exception as e:
            self.get_logger().error(f'Forward kinematics test failed: {str(e)}')

    def test_jacobian(self):
        """Test Jacobian calculation"""
        try:
            # Set neutral joint configuration
            q = pin.neutral(self.model)
            
            # Update kinematics
            pin.computeJointJacobians(self.model, self.data, q)
            
            # Get Jacobian for the last frame
            frame_id = self.model.nframes - 1
            J = pin.getFrameJacobian(
                self.model,
                self.data,
                frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            
            self.get_logger().info('Test 3: Jacobian Calculation')
            self.get_logger().info(f'Frame name: {self.model.frames[frame_id].name}')
            self.get_logger().info(f'Jacobian shape: {J.shape}')
            self.get_logger().info(f'Jacobian: \n{str(J)}')
            
        except Exception as e:
            self.get_logger().error(f'Jacobian test failed: {str(e)}')

    def test_collision(self):
        """Test collision checking between two links"""
        try:
            if self.geometry_model is None or self.geometry_data is None:
                self.get_logger().error('Geometry model not initialized')
                return
                
            self.get_logger().info(f'Number of geometry objects: {len(self.geometry_model.geometryObjects)}')
            self.get_logger().info(f'Number of collision pairs before: {len(self.geometry_model.collisionPairs)}')
            
            # Add a collision pair between first two geometry objects
            if len(self.geometry_model.geometryObjects) >= 2:
                collision_pair = pin.CollisionPair(0, 1)
                self.geometry_model.addCollisionPair(collision_pair)
                self.get_logger().info(f'Added collision pair: {collision_pair.first} vs {collision_pair.second}')
            
            self.get_logger().info(f'Number of collision pairs after: {len(self.geometry_model.collisionPairs)}')
            
            # Set neutral joint configuration
            q = pin.neutral(self.model)
            
            # Update geometry placements
            pin.updateGeometryPlacements(
                self.model,
                self.data,
                self.geometry_model,
                self.geometry_data,
                q
            )
            
            if len(self.geometry_model.collisionPairs) > 0:
                # Compute all collisions to populate collisionResults
                pin.computeCollisions(
                    self.model,
                    self.data,
                    self.geometry_model,
                    self.geometry_data,
                    q,
                    False  # False = stop at first collision (optional optimization)
                )
                
                # Get the result for the first collision pair
                pair_index = 0
                result = self.geometry_data.collisionResults[pair_index].isCollision()
                
                collision_pair = self.geometry_model.collisionPairs[pair_index]
                geom1 = self.geometry_model.geometryObjects[collision_pair.first]
                geom2 = self.geometry_model.geometryObjects[collision_pair.second]
                
                self.get_logger().info('Test 4: Collision Checking')
                self.get_logger().info(f'Checking collision between: {geom1.name} and {geom2.name}')
                self.get_logger().info(f'Collision result: {"True" if result else "False"}')
            else:
                self.get_logger().warn('No collision pairs defined in the model')
                
        except Exception as e:
            self.get_logger().error(f'Collision test failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PinocchioAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()