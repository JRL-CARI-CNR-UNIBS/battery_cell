#! /usr/bin/python3

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
from rclpy.duration import Duration
import subprocess, sys, time, os, signal

class OmronOptimNode(Node):
    def __init__(self):
        super().__init__('omron_optim_node', parameter_overrides=[])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.static_handeye_acquired = False
        self.handeye_detection_started = False
        self.Thm = None

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'omron/tip'
        t.child_frame_id = 'handeye_target'

        t.transform.translation.x = 0.09
        t.transform.translation.y = -0.07
        t.transform.translation.z = 0.09
        
        t.transform.rotation.x = 0.707
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.707

        self.TF_omrontip_handeye = t

        self.get_logger().info('Handeye target pose w.r.t. omron/tip acquired.')
        self.get_logger().info('Transformation: \n' + str(self.TF_omrontip_handeye))

        self.tf_static_broadcaster.sendTransform(self.TF_omrontip_handeye)


    def timer_callback(self):
        if not self.static_handeye_acquired:
            self.get_logger().warn('STATE 1')
            self.tf_static_broadcaster.sendTransform(self.TF_omrontip_handeye)
            time.sleep(3)

            ################ TEMP ##############
            self.Thm = TransformStamped()
            # self.Thm = self.get_transform('handeye_target', 'omron/map')
            self.Thm = self.get_transform('kuka_flange', 'kuka_base_link')

            if self.Thm is not None:
                # Extract the translation and rotation from the transform
                translation = self.Thm.transform.translation
                rotation = self.Thm.transform.rotation

                # Convert the quaternion to a rotation matrix
                r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
                rotation_matrix = r.as_matrix()

                # Create the transformation matrix
                transformation_matrix = np.eye(4)
                transformation_matrix[0:3, 0:3] = rotation_matrix
                transformation_matrix[0:3, 3] = [translation.x, translation.y, translation.z]
                self.get_logger().info('Transformation: \n' + str(transformation_matrix))

                # Save the transformation matrix to a text file
                np.savetxt('/home/gino/transformation_matrix.txt', transformation_matrix)
            ################ TEMP ##############

            if self.Thm is not None:
                self.static_handeye_acquired = True
                self.get_logger().info('Handeye target pose w.r.t. omron/map acquired.')
                # self.get_logger().info('Transformation: \n' + str(self.Thm))

        elif self.static_handeye_acquired and not self.handeye_detection_started:
            self.get_logger().warn('STATE 2')
            command = "ros2 run apriltag_ros apriltag_node " + \
                      "--ros-args -r image_rect:=/zed/zed_node/rgb/image_rect_color " + \
                      "-r camera_info:=/zed/zed_node/rgb/camera_info " + \
                      "--params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml"
            
            # The os.setsid() is passed in the argument preexec_fn so
            # it's run after the fork() and before  exec() to run the shell.
            # https://stackoverflow.com/questions/4789837/how-to-terminate-a-python-subprocess-launched-with-shell-true/4791612#4791612
            self.process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)

            self.handeye_detection_started = True
            self.get_logger().info('Handeye detection started.')

        else:
            self.get_logger().warn('STATE 3')
            self.get_logger().info('Computing map pose w.r.t. world.')
            self.compute_map_pose()
            self.Thm = None
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)  # Send the signal to all the process groups
            self.static_handeye_acquired = False
            self.handeye_detection_started = False


    def get_transform(self, frame_orig, frame_goal) -> TransformStamped | None:
        trans = None
        try:
            trans = self.tf_buffer.lookup_transform(frame_orig, frame_goal, Time())
            # self.get_logger().info('Transformation: \n' + str(trans))
        except Exception as e:
            self.get_logger().error('Could not find transform: ' + str(e))
            
        return trans
    
    def compute_map_pose(self):
        Twh = self.get_transform('world', 'handeye_target') # 'world', 'handeye_target'
        # Thm = self.get_transform('handeye_target', 'omron/map') # 'handeye_target', 'omron/map'

        if Twh is None or self.Thm is None:
            self.get_logger().error('Could not compute map pose w.r.t. world.')
            return
        
        rot_wh = Twh.transform.rotation
        t_wh = Twh.transform.translation 
        rot_hm = self.Thm.transform.rotation
        t_hm = self.Thm.transform.translation

        Rwh = R.from_quat([rot_wh.x, rot_wh.y, rot_wh.z, rot_wh.w]).as_matrix()
        Rhm = R.from_quat([rot_hm.x, rot_hm.y, rot_hm.z, rot_hm.w]).as_matrix()

        Twh = np.eye(4)
        Twh[:3, :3] = Rwh
        Twh[:3, 3] = [t_wh.x, t_wh.y, t_wh.z]

        Thm_mat = np.eye(4)
        Thm_mat[:3, :3] = Rhm
        Thm_mat[:3, 3] = [t_hm.x, t_hm.y, t_hm.z]

        Twm = np.dot(Twh, Thm_mat)

        self.get_logger().info('Computed map pose w.r.t. world: \n' + str(Twm))

        t_wm = Twm[:3, 3]
        R_wm = Twm[:3, :3]
        quat_wm = R.from_matrix(R_wm).as_quat()

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'omron/map'

        t.transform.translation.x = t_wm[0]
        t.transform.translation.y = t_wm[1]
        t.transform.translation.z = t_wm[2]
        
        t.transform.rotation.x = quat_wm[0]
        t.transform.rotation.y = quat_wm[1]
        t.transform.rotation.z = quat_wm[2]
        t.transform.rotation.w = quat_wm[3]

        self.tf_static_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    omron_optim_node = OmronOptimNode()

    rclpy.spin(omron_optim_node)

    omron_optim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()