# -*- coding: utf-8 -*-

# ROS2
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

import sys

import math
from isaacgym import gymapi
from isaacgym import gymutil
import numpy as np

import torch
import os
from isaacgymenvs.utils.torch_jit_utils import *
import rtde_control


current_directory = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current_directory)


class FestivalEnv:
    def __init__(self, fps=60.0, urdf_file="ur/robots/ur3e.urdf") -> None:
        self.gym = gymapi.acquire_gym()
        self.args = gymutil.parse_arguments(
            description="Festival Reinforcement Learning"
        )

        self.fps = fps
        self.urdf_file = urdf_file
        self.init_lock = True

        self.festival_default_dof_pos = [1.57, -2.355, 1.57, -2.355, -1.57, 0.0]

        roll = torch.tensor([3.14], dtype=torch.float32)
        pitch = torch.tensor([0.0], dtype=torch.float32)
        yaw = torch.tensor([0.0], dtype=torch.float32)
        quat = quat_from_euler_xyz(roll, pitch, yaw).flatten().tolist()

        self.festival_default_ee = [0.4, 0.0, 0.4 + 1.017]
        self.festival_default_ee.extend(quat)

        print(self.festival_default_ee)

        self.create_sim(self.fps)
        self.physics_step()

    def create_sim(self, fps=60.0):
        self.sim_params = gymapi.SimParams()
        self.sim_params.dt = 1.0 / fps
        self.sim_params.substeps = 2
        self.sim_params.up_axis = gymapi.UP_AXIS_Z
        self.sim_params.gravity.x = 0
        self.sim_params.gravity.y = 0
        self.sim_params.gravity.z = -9.81
        if self.args.physics_engine == gymapi.SIM_FLEX:
            self.sim_params.flex.solver_type = 5
            self.sim_params.flex.num_outer_iterations = 4
            self.sim_params.flex.num_inner_iterations = 15
            self.sim_params.flex.relaxation = 0.75
            self.sim_params.flex.warm_start = 0.8
        elif self.args.physics_engine == gymapi.SIM_PHYSX:
            self.sim_params.physx.solver_type = 1
            self.sim_params.physx.num_position_iterations = 6
            self.sim_params.physx.num_velocity_iterations = 1
            self.sim_params.physx.num_threads = self.args.num_threads
            self.sim_params.physx.use_gpu = self.args.use_gpu

        self.sim_params.use_gpu_pipeline = False
        if self.args.use_gpu_pipeline:
            print("WARNING: Forcing CPU pipeline.")

        self.sim = self.gym.create_sim(
            self.args.compute_device_id,
            self.args.graphics_device_id,
            self.args.physics_engine,
            self.sim_params,
        )

        if self.sim is None:
            print("*** Failed to create sim")
            quit()

        self._create_ground_plane()
        self._viewer()
        self._create_env(self.urdf_file)

    def _create_ground_plane(self):
        # Add ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        self.gym.add_ground(self.sim, plane_params)

    def _create_env(self, urdf_file):
        # Load franka asset
        asset_root = os.path.join(parent_directory, "./assets")
        franka_asset_file = urdf_file
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        asset_options.flip_visual_attachments = True
        asset_options.collapse_fixed_joints = False
        asset_options.disable_gravity = True
        asset_options.armature = 0.01
        asset_options.use_mesh_materials = True

        print("Loading asset '%s' from '%s'" % (franka_asset_file, asset_root))
        franka_asset = self.gym.load_asset(
            self.sim, asset_root, franka_asset_file, asset_options
        )

        # Create table asset
        table_pos = [-0.52, 0.0, 0.361]
        table_scale = [1.44, 0.4, 0.722]

        table_init_pose = gymapi.Transform()
        table_init_pose.p = gymapi.Vec3(*table_pos)
        table_init_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        table_color = gymapi.Vec3(0.204, 0.204, 0.204)

        table_opt = gymapi.AssetOptions()
        table_opt.fix_base_link = True
        table_asset = self.gym.create_box(self.sim, *table_scale, table_opt)

        # Create stand asset
        stand_pos = [0.0, 0.0, 0.8695]
        stand_scale = [0.2, 0.2, 0.295]

        stand_init_pose = gymapi.Transform()
        stand_init_pose.p = gymapi.Vec3(*stand_pos)
        stand_init_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        stand_color = gymapi.Vec3(0.204, 0.204, 0.204)

        stand_opt = gymapi.AssetOptions()
        stand_opt.fix_base_link = True
        stand_asset = self.gym.create_box(self.sim, *stand_scale, stand_opt)

        # Create controlbox asset
        controlbox_pos = [-0.94, 0.0, 0.922]
        controlbox_scale = [0.6, 0.2, 0.4]

        controlbox_init_pose = gymapi.Transform()
        controlbox_init_pose.p = gymapi.Vec3(*controlbox_pos)
        controlbox_init_pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)
        controlbox_color = gymapi.Vec3(0.51, 0.51, 0.51)

        controlbox_opt = gymapi.AssetOptions()
        controlbox_opt.fix_base_link = True
        controlbox_asset = self.gym.create_box(
            self.sim, *controlbox_scale, controlbox_opt
        )

        # Some common handles for later use
        franka_hand = "display"

        # Attractor setup
        attractor_handles = []
        attractor_properties = gymapi.AttractorProperties()
        attractor_properties.stiffness = 5e5
        attractor_properties.damping = 5e3

        # Make attractor in all axes
        attractor_properties.axes = gymapi.AXIS_ALL
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, 1.017)
        pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)

        # Create helper geometry used for visualization
        # Create an wireframe axis
        self.axes_geom = gymutil.AxesGeometry(0.1)
        # Create an wireframe sphere
        sphere_rot = gymapi.Quat.from_euler_zyx(0.5 * math.pi, 0, 0)
        sphere_pose = gymapi.Transform(r=sphere_rot)
        self.sphere_geom = gymutil.WireframeSphereGeometry(
            0.03, 12, 12, sphere_pose, color=(1, 0, 0)
        )

        # Set up the self.env grid
        spacing = 1.0
        env_lower = gymapi.Vec3(-spacing, -spacing, 0.0)
        env_upper = gymapi.Vec3(spacing, spacing, spacing)

        self.env = self.gym.create_env(self.sim, env_lower, env_upper, 1)

        # add franka
        franka_handle = self.gym.create_actor(
            self.env, franka_asset, pose, "franka", 0, 2
        )
        body_dict = self.gym.get_actor_rigid_body_dict(self.env, franka_handle)

        table_handle = self.gym.create_actor(
            self.env, table_asset, table_init_pose, "table", 1, 0
        )

        stand_handle = self.gym.create_actor(
            self.env, stand_asset, stand_init_pose, "stand", 1, 1
        )

        controlbox_handle = self.gym.create_actor(
            self.env, controlbox_asset, controlbox_init_pose, "controlbox", 1, 2
        )

        # set colors
        self.gym.set_rigid_body_color(
            self.env, table_handle, 0, gymapi.MESH_VISUAL, table_color
        )
        self.gym.set_rigid_body_color(
            self.env, stand_handle, 0, gymapi.MESH_VISUAL, stand_color
        )
        self.gym.set_rigid_body_color(
            self.env, controlbox_handle, 0, gymapi.MESH_VISUAL, controlbox_color
        )

        # print(body_dict)
        props = self.gym.get_actor_rigid_body_states(
            self.env, franka_handle, gymapi.STATE_POS
        )
        hand_handle = body = self.gym.find_actor_rigid_body_handle(
            self.env, franka_handle, franka_hand
        )

        # Initialize the attractor
        attractor_properties.target = props["pose"][:][body_dict[franka_hand]]
        attractor_properties.target.p = gymapi.Vec3(
            self.festival_default_ee[0],
            self.festival_default_ee[1],
            self.festival_default_ee[2],
        )
        attractor_properties.target.r = gymapi.Quat(
            self.festival_default_ee[3],
            self.festival_default_ee[4],
            self.festival_default_ee[5],
            self.festival_default_ee[6],
        )
        attractor_properties.rigid_handle = hand_handle

        # Draw axes and sphere at attractor location
        gymutil.draw_lines(
            self.axes_geom, self.gym, self.viewer, self.env, attractor_properties.target
        )
        gymutil.draw_lines(
            self.sphere_geom,
            self.gym,
            self.viewer,
            self.env,
            attractor_properties.target,
        )

        self.attractor_handle = self.gym.create_rigid_body_attractor(
            self.env, attractor_properties
        )

        franka_dof_props = self.gym.get_actor_dof_properties(self.env, franka_handle)
        # get joint limits and ranges for Franka
        # franka_lower_limits = franka_dof_props['lower']
        # franka_upper_limits = franka_dof_props['upper']
        # franka_ranges = franka_upper_limits - franka_lower_limits
        # franka_mids = 0.5 * (franka_upper_limits + franka_lower_limits)
        # franka_num_dofs = len(franka_dof_props)

        # override default stiffness and damping values
        franka_dof_props["driveMode"].fill(gymapi.DOF_MODE_VEL)
        franka_dof_props["stiffness"].fill(0.0)
        franka_dof_props["damping"].fill(1000.0)

        # Give a desired pose for first 2 robot joints to improve stability
        franka_dof_props["driveMode"][0:2] = gymapi.DOF_MODE_POS
        franka_dof_props["driveMode"][7:] = gymapi.DOF_MODE_POS

        franka_dof_props["stiffness"][7:] = 1e10
        franka_dof_props["damping"][7:] = 1.0

        self.gym.set_actor_dof_properties(self.env, franka_handle, franka_dof_props)

    def _viewer(self):
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        if self.viewer is None:
            print("*** Failed to create viewer")
            quit()
        cam_pos = gymapi.Vec3(-4.0, 0.0, 1.0)
        cam_target = gymapi.Vec3(1.0, 0.0, 1.0)
        self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

    def point_move(self, pre, post, dt=75):
        a = (post[0] - pre[0]) / dt
        b = (post[1] - pre[1]) / dt
        c = (post[2] - pre[2]) / dt
        return [a, b, c]

    def physics_step(
        self,
        update_time=1.5,
        target_xyz=[0.0, 0.0, 0.0],
        target_point=[0.0, 0.3, 0.5 + 1.017, 0.0, 0.0, 0.0, 1.0],
    ):
        try:
            if self.gym.query_viewer_has_closed(self.viewer):
                raise Exception("Viewer Closed")

            if target_point == None:
                target_point = self.festival_default_ee

            # Every 0.01 seconds the pose of the attactor is updated
            t = self.gym.get_sim_time(self.sim)

            if t >= update_time:
                self.gym.clear_lines(self.viewer)

                attractor_properties = self.gym.get_attractor_properties(
                    self.env, self.attractor_handle
                )
                display_state = self.gym.get_actor_rigid_body_states(
                    self.env, self.attractor_handle, 0
                )[9][0][0]
                display_state = list(display_state)

                if self.init_lock:
                    target_xyz = self.point_move(display_state, target_point[:3])
                    self.init_lock = False

                target_dist = torch.norm(
                    (torch.tensor(display_state) - torch.tensor(target_point[:3])),
                    dim=-1,
                )

                pose = attractor_properties.target

                if target_dist > 0.025:
                    pose.p.x = pose.p.x + target_xyz[0]
                    pose.p.y = pose.p.y + target_xyz[1]
                    pose.p.z = pose.p.z + target_xyz[2]

                else:
                    self.init_lock = True

                self.gym.set_attractor_target(self.env, self.attractor_handle, pose)

                gymutil.draw_lines(
                    self.axes_geom, self.gym, self.viewer, self.env, pose
                )
                gymutil.draw_lines(
                    self.sphere_geom, self.gym, self.viewer, self.env, pose
                )

                update_time += 0.1

            # Step the physics
            self.gym.simulate(self.sim)
            self.gym.fetch_results(self.sim, True)

            # Step rendering
            self.gym.step_graphics(self.sim)
            self.gym.draw_viewer(self.viewer, self.sim, False)
            self.gym.sync_frame_time(self.sim)

        except Exception as e:
            print(e)

    def target_point(self, target_point):
        target_xyz = self.point_move(target_point[0], target_point[1])
        pass

    def __del__(self) -> None:
        self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)


class FestivalSimNode(Node):
    def __init__(self, simenv: FestivalEnv) -> None:
        super().__init__("festival_sim_node")

        self.target_frame = (
            self.declare_parameter("target_frame", "human_link")
            .get_parameter_value()
            .string_value
        )
        self.source_frame = (
            self.declare_parameter("source_frame", "camera_link")
            .get_parameter_value()
            .string_value
        )

        self.festival_env = simenv
        self.timer = self.create_timer(0.01, self.timerloop)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Festival Simulation Node has been started")

    def timerloop(self):
        target_point = self.trasnformLister()
        self.festival_env.physics_step(target_point=target_point)

    def trasnformLister(self) -> list:
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time()
            )

            return [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {self.target_frame} to {self.source_frame}: {ex}"
            )
            return

    def __del__(self) -> None:
        del self.festival_env


def main(args=None):
    rclpy.init(args=args)
    simenv = FestivalEnv()
    festival_sim_node = FestivalSimNode(simenv)
    try:
        rclpy.spin(festival_sim_node)
    except KeyboardInterrupt:
        print("Interrupted!")
        festival_sim_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
