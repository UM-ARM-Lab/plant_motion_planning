import time

from itertools import count

import numpy as np
import pybullet

import plant_motion_planning.utils as s_utils
from .pr2_utils import get_top_grasps, get_side_grasps, get_side_only_grasps
from .utils import get_pose, set_pose, get_movable_joints, \
    set_joint_positions, add_fixed_constraint, enable_real_time, disable_real_time, joint_controller, \
    enable_gravity, wait_for_duration, link_from_name, get_body_name, sample_placement, \
    end_effector_from_body, approach_from_grasp, plan_joint_motion, GraspInfo, Pose, INF, Point, \
    inverse_kinematics, pairwise_collision, remove_fixed_constraint, Attachment, get_sample_fn, \
    step_simulation, refine_path, plan_direct_joint_motion, get_joint_positions, wait_if_gui, flatten, \
    pairwise_contact, plan_joint_motion_with_angle_contraints, \
    plan_joint_motion2, plan_joint_motion_with_angle_contraints_v2, \
    plan_joint_motion_with_angle_contraints_v3, plan_joint_motion_with_angle_contraints_v4, \
    plan_joint_motion_with_angle_constraints_v5, plan_joint_motion_with_controls, \
    plan_joint_motion_with_angle_contraints_v6, plan_joint_motion_with_angle_contraints_v7, \
    plan_joint_motion_single_plant, plan_joint_motion_with_angle_contraints_multi_world, \
    plan_joint_motion_multiworld_benchmark

# from plant_motion_planning.utils import compute_total_cost, compute_path_cost

# TODO: deprecate

GRASP_INFO = {
    'top': GraspInfo(lambda body: get_top_grasps(body, under=True, tool_pose=Pose(), max_width=INF,  grasp_length=0),
                     approach_pose=Pose(0.1*Point(z=1))),
    'side': GraspInfo(lambda body: get_side_grasps(body, under=True, tool_pose=Pose(), max_width=INF, grasp_length=0),
                     approach_pose=Pose(0.1 * Point(z=0.5))),
    'side_only': GraspInfo(lambda body: get_side_only_grasps(body, under=True, tool_pose=Pose(), max_width=INF, grasp_length=0),
                      approach_pose=Pose(0.1 * Point(z=0.1))),
}

TOOL_FRAMES = {
    'iiwa14': 'iiwa_link_ee_kuka', # iiwa_link_ee | iiwa_link_ee_kuka
}

DEBUG_FAILURE = False

##################################################

class BodyPose(object):
    num = count()
    def __init__(self, body, pose=None):
        if pose is None:
            pose = get_pose(body)
        self.body = body
        self.pose = pose
        self.index = next(self.num)
    @property
    def value(self):
        return self.pose
    def assign(self):
        set_pose(self.body, self.pose)
        return self.pose
    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'p{}'.format(index)


class BodyGrasp(object):
    num = count()
    def __init__(self, body, grasp_pose, approach_pose, robot, link):
        self.body = body
        self.grasp_pose = grasp_pose
        self.approach_pose = approach_pose
        self.robot = robot
        self.link = link
        self.index = next(self.num)
    @property
    def value(self):
        return self.grasp_pose
    @property
    def approach(self):
        return self.approach_pose
    #def constraint(self):
    #    grasp_constraint()
    def attachment(self):
        return Attachment(self.robot, self.link, self.grasp_pose, self.body)
    def assign(self):
        return self.attachment().assign()
    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'g{}'.format(index)

class BodyConf(object):
    num = count()
    def __init__(self, body, configuration=None, joints=None):
        if joints is None:
            joints = get_movable_joints(body)
        if configuration is None:
            configuration = get_joint_positions(body, joints)
        self.body = body
        self.joints = joints
        self.configuration = configuration
        self.index = next(self.num)
    @property
    def values(self):
        return self.configuration
    def assign(self):
        set_joint_positions(self.body, self.joints, self.configuration)
        return self.configuration


    def assign_with_controls_old(self):
        set_joint_positions(self.body, self.joints, self.configuration)
        pybullet.setJointMotorControlArray(self.body, self.joints, pybullet.POSITION_CONTROL, self.configuration,
                                           positionGains=7 * [0.01])
        for t in range(10):
            s_utils.step_sim()

        return self.configuration

    def assign_with_controls(self):
        set_joint_positions(self.body, self.joints, self.configuration)
        pybullet.setJointMotorControlArray(self.body, self.joints, pybullet.POSITION_CONTROL, self.configuration,
                                           positionGains=7 * [0.01])
        for t in range(10):
            s_utils.step_sim_v2()

        return self.configuration

    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'q{}'.format(index)

class BodyPath(object):
    def __init__(self, body, path, joints=None, attachments=[]):
        if joints is None:
            joints = get_movable_joints(body)
        self.body = body
        self.path = path
        self.joints = joints
        self.attachments = attachments
    def bodies(self):
        return set([self.body] + [attachment.body for attachment in self.attachments])
    def iterator(self):
        # TODO: compute and cache these
        # TODO: compute bounding boxes as well
        for i, configuration in enumerate(self.path):
            set_joint_positions(self.body, self.joints, configuration)
            for grasp in self.attachments:
                grasp.assign()
            yield i
    def iterator_with_control(self):
        # TODO: compute and cache these
        # TODO: compute bounding boxes as well

        position_gains = 7 * [0.01]

        for i, configuration in enumerate(self.path):

            # set_joint_positions(self.body, self.joints, configuration)
            pybullet.setJointMotorControlArray(self.body, self.joints, pybullet.POSITION_CONTROL, configuration,
                                               positionGains=position_gains)

            for t in range(11):
                s_utils.step_sim()

            for grasp in self.attachments:
                grasp.assign()

            yield i



    def iterator_multi_world(self, multi_world_env):

        position_gains = 7 * [0.01]

        for i, configuration in enumerate(self.path):

            # set_joint_positions(self.body, self.joints, configuration)
            # pybullet.setJointMotorControlArray(self.body, self.joints, pybullet.POSITION_CONTROL, configuration,
            #                                    positionGains=position_gains)

            # for t in range(11):
            #     s_utils.step_sim_v2()
            multi_world_env.step(configuration)

            for grasp in self.attachments:
                grasp.assign()

            yield i

    def iterator_with_control_v3(self, single_plant_env):

        position_gains = 7 * [0.01]

        for i, configuration in enumerate(self.path):

            # set_joint_positions(self.body, self.joints, configuration)
            # pybullet.setJointMotorControlArray(self.body, self.joints, pybullet.POSITION_CONTROL, configuration,
            #                                    positionGains=position_gains)

            # for t in range(11):
            #     s_utils.step_sim_v2()
            single_plant_env.step(configuration)

            for grasp in self.attachments:
                grasp.assign()

            yield i

    def iterator_with_control_v2(self):

        position_gains = 7 * [0.01]

        for i, configuration in enumerate(self.path):

            # set_joint_positions(self.body, self.joints, configuration)
            pybullet.setJointMotorControlArray(self.body, self.joints, pybullet.POSITION_CONTROL, configuration,
                                               positionGains=position_gains)

            for t in range(11):
                s_utils.step_sim_v2()

            for grasp in self.attachments:
                grasp.assign()

            yield i

    def control(self, real_time=False, dt=0):
        # TODO: just waypoints
        if real_time:
            enable_real_time()
        else:
            disable_real_time()
        for values in self.path:
            for _ in joint_controller(self.body, self.joints, values):
                enable_gravity()
                if not real_time:
                    step_simulation()
                time.sleep(dt)
    # def full_path(self, q0=None):
    #     # TODO: could produce sequence of savers
    def refine(self, num_steps=0):
        return self.__class__(self.body, refine_path(self.body, self.joints, self.path, num_steps), self.joints, self.attachments)
    def reverse(self):
        return self.__class__(self.body, self.path[::-1], self.joints, self.attachments)
    def __repr__(self):
        return '{}({},{},{},{})'.format(self.__class__.__name__, self.body, len(self.joints), len(self.path), len(self.attachments))

##################################################

class ApplyForce(object):
    def __init__(self, body, robot, link):
        self.body = body
        self.robot = robot
        self.link = link
    def bodies(self):
        return {self.body, self.robot}
    def iterator(self, **kwargs):
        return []
    def refine(self, **kwargs):
        return self
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.robot, self.body)

class Attach(ApplyForce):
    def control(self, **kwargs):
        # TODO: store the constraint_id?
        add_fixed_constraint(self.body, self.robot, self.link)
    def reverse(self):
        return Detach(self.body, self.robot, self.link)

class Detach(ApplyForce):
    def control(self, **kwargs):
        remove_fixed_constraint(self.body, self.robot, self.link)
    def reverse(self):
        return Attach(self.body, self.robot, self.link)


# def compute_ee_path_cost(path):
#
#     for i in range(len(path) - 1):
#         q1 = path[i]
#         q2 = path



class Command(object):
    num = count()
    def __init__(self, body_paths):
        self.body_paths = body_paths
        self.index = next(self.num)
    def bodies(self):
        return set(flatten(path.bodies() for path in self.body_paths))
    # def full_path(self, q0=None):
    #     if q0 is None:
    #         q0 = Conf(self.tree)
    #     new_path = [q0]
    #     for partial_path in self.body_paths:
    #         new_path += partial_path.full_path(new_path[-1])[1:]
    #     return new_path
    def step(self):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                msg = '{},{}) step?'.format(i, j)
                wait_if_gui(msg)
                #print(msg)
    def execute(self, time_step=0.05):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                #time.sleep(time_step)
                wait_for_duration(time_step)


    def execute_with_movable_plant_angle_constraint(self,robot, block, plant_id = [], movable = [],
                                                    deflection_limit = 0, time_step=0.05):

        # distance_fn, alpha = cost_utils

        total_cost = 0
        total_ee_path_cost = 0
        deflection_over_limit = 0
        alpha = 0.1

        text_id = []

        # text_ori = pybullet.getQuaternionFromEuler([1.57, 0.0, 0.0])
        text_ori = pybullet.getQuaternionFromEuler([0.62, 0.0, 0.5])

        pybullet.addUserDebugText("Deflection limit: %0.3f rad" % (deflection_limit), [-1.95, 1, 1.05], textColorRGB=[1, 0, 0])

        for e, b in enumerate(movable):
            b.observe()
            text_id.append(pybullet.addUserDebugText("Deflection of plant " + str(e + 1) + ": " + str(b.deflection),
                                                     [-2 - 0.2 * e, 1, 1 - 0.2 * e], textColorRGB=[0, 0, 0]))

        prev_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
            # for j in body_path.iterator_with_control():


                for pid in plant_id:
                    plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = pybullet.getJointState(pid, 0)
                    plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = pybullet.getJointState(pid, 1)
                    pybullet.applyExternalTorque(pid, linkIndex=1,
                                                 torqueObj=[-200 * plant_rot_joint_displacement_x,
                                                            -200 * plant_rot_joint_displacement_y, 0],
                                                 flags=pybullet.WORLD_FRAME)

                # if(pairwise_collision(robot, 2)):
                #     print("Collision Detected!!")
                #     exit()

                for e, b in enumerate(movable):
                    b.observe()

                    if(b.deflection > deflection_limit):
                        deflection_over_limit = deflection_over_limit + (b.deflection - deflection_limit)

                    pybullet.addUserDebugText("Deflection of plant " + str(e + 1) + ": %.3f rad" % (b.deflection),
                                              [-2 - 0.07 * e, 1, 1 - 0.07 * e], textColorRGB=[0, 0, 0],
                                              replaceItemUniqueId=text_id[e])

                    print(f"Deflection of plant {e+1}: {b.deflection}")

                print("========================================")

                cur_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                # print("prev: ", prev_ee_pos, "cur: ", cur_ee_pos, "diff: ", prev_ee_pos - cur_ee_pos)
                # print("path cost: ", np.linalg.norm(prev_ee_pos - cur_ee_pos))
                # input("")

                prev_ee_pos = cur_ee_pos

                for t in range(100):
                    pybullet.stepSimulation()

                    #time.sleep(time_step)
                    # wait_for_duration(time_step)
                # wait_if_gui("press enter to move to next step")

            # total_path_cost = compute_path_cost(body_path.path, distance_fn)

        total_cost = total_ee_path_cost + (alpha * deflection_over_limit)

        print("====================================================")
        print("total path cost: ", total_ee_path_cost)
        print("total Deflection over limit: ", deflection_over_limit)
        print("alpha: ", alpha)
        print("====================================================")
        print("Total cost: ", total_cost)
        print("====================================================")



    def execute_with_controls(self,robot, init_conf, block, plant_id = [], movable = [],
                                                    deflection_limit = 0, time_step=0.05):

        # distance_fn, alpha = cost_utils

        total_cost = 0
        total_ee_path_cost = 0
        deflection_over_limit = 0
        alpha = 0.1

        text_id = []

        # text_ori = pybullet.getQuaternionFromEuler([1.57, 0.0, 0.0])
        text_ori = pybullet.getQuaternionFromEuler([0.62, 0.0, 0.5])

        pybullet.addUserDebugText("Deflection limit: %0.3f rad" % (deflection_limit), [-1.95, 1, 1.05], textColorRGB=[1, 0, 0])

        for e, b in enumerate(movable):
            b.observe()
            text_id.append(pybullet.addUserDebugText("Deflection of plant " + str(e + 1) + ": " + str(b.deflection),
                                                     [-2 - 0.2 * e, 1, 1 - 0.2 * e], textColorRGB=[0, 0, 0]))

        prev_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

        joints = get_movable_joints(robot)
        position_gains = 7 * [0.01]

        set_joint_positions(robot, joints, init_conf)
        pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, init_conf,
                                           positionGains=position_gains)
        for t in range(20):
            s_utils.step_sim()

        for i, body_path in enumerate(self.body_paths):
            # for j in body_path.iterator():
            for j in body_path.iterator_with_control():


                #for pid in plant_id:
                #    plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = pybullet.getJointState(pid, 0)
                #    plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = pybullet.getJointState(pid, 1)
                #    pybullet.applyExternalTorque(pid, linkIndex=1,
                #                                 torqueObj=[-200 * plant_rot_joint_displacement_x,
                #                                            -200 * plant_rot_joint_displacement_y, 0],
                #                                 flags=pybullet.WORLD_FRAME)

                # if(pairwise_collision(robot, 2)):
                #     print("Collision Detected!!")
                #     exit()

                for e, b in enumerate(movable):
                    b.observe()

                    pybullet.addUserDebugText("Deflection of plant " + str(e + 1) + ": %.3f rad" % (b.deflection),
                                              [-2 - 0.07 * e, 1, 1 - 0.07 * e], textColorRGB=[0, 0, 0],
                                              replaceItemUniqueId=text_id[e])

                    print("Deflection of plant %d: %f" % (e, b.deflection))

                    if (b.deflection > deflection_limit):
                        print("Error! Deflection limit exceeded!")
                        deflection_over_limit = deflection_over_limit + (b.deflection - deflection_limit)

                        # import pickle
                        # with open('path_smoothed.pkl', 'wb') as f:
                        #     pickle.dump([path.body_paths[0].path], f)
                        #
                        # input("")

                print("========================================")

                cur_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                prev_ee_pos = cur_ee_pos

                #for t in range(23):
                #    pybullet.stepSimulation()

                    #time.sleep(time_step)
                    # wait_for_duration(time_step)
                # wait_if_gui("press enter to move to next step")

        total_cost = total_ee_path_cost + (alpha * deflection_over_limit)

        print("====================================================")
        print("total path cost: ", total_ee_path_cost)
        print("total Deflection over limit: ", deflection_over_limit)
        print("alpha: ", alpha)
        print("====================================================")
        print("Total cost: ", total_cost)
        print("====================================================")



    def execute_multi_world(self, init_conf, multi_world_env, num_worlds=4):

        # distance_fn, alpha = cost_utils

        total_cost = 0
        total_ee_path_cost = 0
        deflection_over_limit = np.zeros(num_worlds)
        alpha = 0.1

        # text_id = []

        # text_ori = pybullet.getQuaternionFromEuler([1.57, 0.0, 0.0])
        # text_ori = pybullet.getQuaternionFromEuler([0.62, 0.0, 0.5])

        # pybullet.addUserDebugText("Deflection limit: %0.3f rad" % (multi_world_env.deflection_limit), [-1.95, 1, 1.05],
        #                           textColorRGB=[1, 0, 0])

        # for b in single_plant_env.movable:
        #     b.observe_all()
        #
        #     for e, deflection in enumerate(b.deflections):
        #         text_id.append(pybullet.addUserDebugText("Deflection of link " + str(e + 1) + ": " + str(deflection),
        #                                                  [-2 - 0.2 * e, 1, 1 - 0.2 * e], textColorRGB=[0, 0, 0]))

        prev_ee_pos = np.array(pybullet.getLinkState(multi_world_env.sample_robot, 9)[0])

        multi_world_env.step_after_restoring(init_conf)

        print("Executing path...")

        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator_multi_world(multi_world_env):

                # for pid in plant_id:
                #    plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = pybullet.getJointState(pid, 0)
                #    plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = pybullet.getJointState(pid, 1)
                #    pybullet.applyExternalTorque(pid, linkIndex=1,
                #                                 torqueObj=[-200 * plant_rot_joint_displacement_x,
                #                                            -200 * plant_rot_joint_displacement_y, 0],
                #                                 flags=pybullet.WORLD_FRAME)

                # if(pairwise_collision(robot, 2)):
                #     print("Collision Detected!!")
                #     exit()

                for e_env, env_item in enumerate(multi_world_env.envs.items()):

                    xy, env = env_item

                    print("Environment: ", e_env)

                    for b in env.movable:
                        b.observe_all()

                        for e, deflection in enumerate(b.deflections):

                            print("\tDeflection of plant %d: %f" % (e, deflection))

                            if (deflection > multi_world_env.deflection_limit):
                                print("Error! Deflection limit exceeded!")
                                deflection_over_limit[e_env] = deflection_over_limit[e_env] + (deflection - multi_world_env.deflection_limit)

                    print("========================================")

                cur_ee_pos = np.array(pybullet.getLinkState(multi_world_env.sample_robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                prev_ee_pos = cur_ee_pos

                # for t in range(23):
                #    pybullet.stepSimulation()

                # time.sleep(time_step)
                # wait_for_duration(time_step)
                # wait_if_gui("press enter to move to next step")

        total_cost = total_ee_path_cost + (alpha * deflection_over_limit)

        print("====================================================")
        print("total path cost: ", total_ee_path_cost)
        print("total Deflection over limit: ", deflection_over_limit)
        print("alpha: ", alpha)
        print("====================================================")
        print("Total cost: ", total_cost)
        print("====================================================")


    def execute_with_controls_v3(self, robot, init_conf, single_plant_env):

        # distance_fn, alpha = cost_utils

        total_cost = 0
        total_ee_path_cost = 0
        deflection_over_limit = 0
        alpha = 0.1

        text_id = []

        # text_ori = pybullet.getQuaternionFromEuler([1.57, 0.0, 0.0])
        text_ori = pybullet.getQuaternionFromEuler([0.62, 0.0, 0.5])

        pybullet.addUserDebugText("Deflection limit: %0.3f rad" % (single_plant_env.deflection_limit), [-1.95, 1, 1.05],
                                  textColorRGB=[1, 0, 0])

        for b in single_plant_env.movable:
            b.observe_all()

            for e, deflection in enumerate(b.deflections):
                text_id.append(pybullet.addUserDebugText("Deflection of link " + str(e + 1) + ": " + str(deflection),
                                                         [-2 - 0.2 * e, 1, 1 - 0.2 * e], textColorRGB=[0, 0, 0]))

        prev_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

        joints = get_movable_joints(robot)
        position_gains = 7 * [0.01]

        set_joint_positions(robot, joints, init_conf)
        # pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, init_conf,
        #                                    positionGains=position_gains)

        single_plant_env.step(init_conf)
        # for t in range(10):
        #     s_utils.step_sim_v2()

        for i, body_path in enumerate(self.body_paths):
            # for j in body_path.iterator():
            for j in body_path.iterator_with_control_v3(single_plant_env):
            # for j in body_path.iterator_with_control_v2():

                # for pid in plant_id:
                #    plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = pybullet.getJointState(pid, 0)
                #    plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = pybullet.getJointState(pid, 1)
                #    pybullet.applyExternalTorque(pid, linkIndex=1,
                #                                 torqueObj=[-200 * plant_rot_joint_displacement_x,
                #                                            -200 * plant_rot_joint_displacement_y, 0],
                #                                 flags=pybullet.WORLD_FRAME)

                # if(pairwise_collision(robot, 2)):
                #     print("Collision Detected!!")
                #     exit()

                for b in single_plant_env.movable:
                    b.observe_all()

                    for e, deflection in enumerate(b.deflections):
                        pybullet.addUserDebugText("Deflection of link " + str(e + 1) + ": %.3f rad" % (deflection),
                                                  [-2 - 0.07 * e, 1, 1 - 0.07 * e], textColorRGB=[0, 0, 0],
                                                  replaceItemUniqueId=text_id[e])

                        print("Deflection of plant %d: %f" % (e, deflection))

                        if (deflection > single_plant_env.deflection_limit):
                            print("Error! Deflection limit exceeded!")
                            deflection_over_limit = deflection_over_limit + (deflection - single_plant_env.deflection_limit)

                            # import pickle
                            # with open('path_smoothed.pkl', 'wb') as f:
                            #     pickle.dump([path.body_paths[0].path], f)
                            #
                            # input("")

                print("========================================")

                cur_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                prev_ee_pos = cur_ee_pos

                # for t in range(23):
                #    pybullet.stepSimulation()

                # time.sleep(time_step)
                # wait_for_duration(time_step)
                # wait_if_gui("press enter to move to next step")

        total_cost = total_ee_path_cost + (alpha * deflection_over_limit)

        print("====================================================")
        print("total path cost: ", total_ee_path_cost)
        print("total Deflection over limit: ", deflection_over_limit)
        print("alpha: ", alpha)
        print("====================================================")
        print("Total cost: ", total_cost)
        print("====================================================")

    def execute_with_controls_v2(self, robot, init_conf, movable=[],
                              deflection_limit=0):

        # distance_fn, alpha = cost_utils

        total_cost = 0
        total_ee_path_cost = 0
        deflection_over_limit = 0
        alpha = 0.1

        text_id = []

        # text_ori = pybullet.getQuaternionFromEuler([1.57, 0.0, 0.0])
        text_ori = pybullet.getQuaternionFromEuler([0.62, 0.0, 0.5])

        pybullet.addUserDebugText("Deflection limit: %0.3f rad" % (deflection_limit), [-1.95, 1, 1.05],
                                  textColorRGB=[1, 0, 0])

        for b in movable:
            b.observe_all()

            for e, deflection in enumerate(b.deflections):
                text_id.append(pybullet.addUserDebugText("Deflection of link " + str(e + 1) + ": " + str(deflection),
                                                         [-2 - 0.2 * e, 1, 1 - 0.2 * e], textColorRGB=[0, 0, 0]))

        prev_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

        joints = get_movable_joints(robot)
        position_gains = 7 * [0.01]

        set_joint_positions(robot, joints, init_conf)
        pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, init_conf,
                                           positionGains=position_gains)
        for t in range(20):
            s_utils.step_sim_v2()

        for i, body_path in enumerate(self.body_paths):
            # for j in body_path.iterator():
            for j in body_path.iterator_with_control_v2():

                # for pid in plant_id:
                #    plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = pybullet.getJointState(pid, 0)
                #    plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = pybullet.getJointState(pid, 1)
                #    pybullet.applyExternalTorque(pid, linkIndex=1,
                #                                 torqueObj=[-200 * plant_rot_joint_displacement_x,
                #                                            -200 * plant_rot_joint_displacement_y, 0],
                #                                 flags=pybullet.WORLD_FRAME)

                # if(pairwise_collision(robot, 2)):
                #     print("Collision Detected!!")
                #     exit()

                for b in movable:
                    b.observe_all()

                    for e, deflection in enumerate(b.deflections):
                        pybullet.addUserDebugText("Deflection of link " + str(e + 1) + ": %.3f rad" % (deflection),
                                                  [-2 - 0.07 * e, 1, 1 - 0.07 * e], textColorRGB=[0, 0, 0],
                                                  replaceItemUniqueId=text_id[e])

                        print("Deflection of plant %d: %f" % (e, deflection))

                        if (deflection > deflection_limit):
                            print("Error! Deflection limit exceeded!")
                            deflection_over_limit = deflection_over_limit + (deflection - deflection_limit)

                            # import pickle
                            # with open('path_smoothed.pkl', 'wb') as f:
                            #     pickle.dump([path.body_paths[0].path], f)
                            #
                            # input("")

                print("========================================")

                cur_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                prev_ee_pos = cur_ee_pos

                # for t in range(23):
                #    pybullet.stepSimulation()

                # time.sleep(time_step)
                # wait_for_duration(time_step)
                # wait_if_gui("press enter to move to next step")

        total_cost = total_ee_path_cost + (alpha * deflection_over_limit)

        print("====================================================")
        print("total path cost: ", total_ee_path_cost)
        print("total Deflection over limit: ", deflection_over_limit)
        print("alpha: ", alpha)
        print("====================================================")
        print("Total cost: ", total_cost)
        print("====================================================")


    def execute_with_movable_plant(self,robot, plant_ids, time_step=0.05):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():

                for pid in plant_ids:
                    plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = pybullet.getJointState(pid, 0)
                    plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = pybullet.getJointState(pid, 1)
                    pybullet.applyExternalTorque(pid, linkIndex=1,
                                          torqueObj=[-200 * plant_rot_joint_displacement_x,
                                                     -200 * plant_rot_joint_displacement_y, 0],
                                          flags=pybullet.WORLD_FRAME)

                # contact_points = get_contact_points(bodyA = robot, bodyB = plant_id)
                # if(len(contact_points) > 0):
                # for cp in contact_points:
                #     if(len(cp) > 0 and cp.normalForce > 0):
                #         print("normal force during simulation: ", cp.normalForce)
                # print("normal force: ", contact_points["normalForce"])

                # print("I iterate every time step!!")
                pybullet.stepSimulation()

                #time.sleep(time_step)
                wait_for_duration(time_step)

    def control(self, real_time=False, dt=0): # TODO: real_time
        for body_path in self.body_paths:
            body_path.control(real_time=real_time, dt=dt)
    def refine(self, **kwargs):
        return self.__class__([body_path.refine(**kwargs) for body_path in self.body_paths])
    def reverse(self):
        return self.__class__([body_path.reverse() for body_path in reversed(self.body_paths)])
    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'c{}'.format(index)

#######################################################

def get_tool_link(robot):
    return link_from_name(robot, TOOL_FRAMES[get_body_name(robot)])


def get_grasp_gen(robot, grasp_name='top'):
    grasp_info = GRASP_INFO[grasp_name]
    tool_link = get_tool_link(robot)
    def gen(body):
        grasp_poses = grasp_info.get_grasps(body)
        # TODO: continuous set of grasps
        for grasp_pose in grasp_poses:
            body_grasp = BodyGrasp(body, grasp_pose, grasp_info.approach_pose, robot, tool_link)
            yield (body_grasp,)
    return gen


def get_stable_gen(fixed=[]):
    def gen(body, surface):
        while True:
            pose = sample_placement(body, surface)
            if (pose is None) or any(pairwise_collision(body, b) for b in fixed):
                continue
            body_pose = BodyPose(body, pose)
            yield (body_pose,)
    return gen



def get_ik_fn(robot, fixed=[], teleport=False, num_attempts=10):
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)

    def fn(body, pose, grasp):

        obstacles = [body] + fixed

        gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
        approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)

        for _ in range(num_attempts):
            set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
            # TODO: multiple attempts?
            q_approach = inverse_kinematics(robot, grasp.link, approach_pose)
            if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue
            conf = BodyConf(robot, q_approach)
            q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)
            if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue
            if teleport:
                path = [q_approach, q_grasp]
            else:
                conf.assign()
                #direction, _ = grasp.approach_pose
                #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                #                                   quat_from_pose(approach_pose))
                path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles)

                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Approach motion failed')
                    continue

            command = Command([BodyPath(robot, path),
                               Attach(body, robot, grasp.link),
                               BodyPath(robot, path[::-1], attachments=[grasp])])
            return (conf, command)
            # TODO: holding collisions
        return None
    return fn


def get_ik_fn_angle(robot, fixed=[], movable = [], deflection_limit = 0, orientation_limit = 0, teleport=False, num_attempts=10):
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)

    def fn(body, pose, grasp):

        obstacles = [body] + fixed
        # obstacles = [body] + fixed + movable

        gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
        approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)

        for _ in range(num_attempts):
            set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
            # TODO: multiple attempts?
            q_approach = inverse_kinematics(robot, grasp.link, approach_pose)

            ## Check for collision with any fixed object
            if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue

            conf = BodyConf(robot, q_approach)
            q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)


            if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue

            # Go through movable plants and check their deflection and orientation angles
            flag = 0
            step_simulation()
            for b in movable:
                b.observe()

                if(b.deflection > deflection_limit):
                    flag = 1
                    break

            if(flag == 1):
                continue

                # pybullet.stepSimulation()
                #
                # print("====================================")
                # print(f"b.deflection: {b.deflection}")
                # print("====================================")


            if teleport:
                path = [q_approach, q_grasp]
            else:
                conf.assign()
                #direction, _ = grasp.approach_pose
                #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                #                                   quat_from_pose(approach_pose))
                path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles)

                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Approach motion failed')
                    continue

            command = Command([BodyPath(robot, path),
                               Attach(body, robot, grasp.link),
                               BodyPath(robot, path[::-1], attachments=[grasp])])
            return (conf, command)
            # TODO: holding collisions
        return None
    return fn

def get_ik_fn2(robot, fixed=[], movable = [], force_limit = 0, teleport=False, num_attempts=10):
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)

    def fn(body, pose, grasp):

        obstacles = [body] + fixed
        # obstacles = [body] + fixed + movable

        gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
        approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)

        for _ in range(num_attempts):
            set_joint_positions(robot, movable_joints, sample_fn()) # Random seed
            # TODO: multiple attempts?
            q_approach = inverse_kinematics(robot, grasp.link, approach_pose)

            ## Check for collision with any fixed object
            if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue


            conf = BodyConf(robot, q_approach)
            q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)


            if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue

            step_simulation()
            # Check if normal force against the movable object is over the force_limit
            for b in movable:

                # print("contact? ", pairwise_contact(robot, b, force_limit=force_limit))
                # print("collision? ", pairwise_collision(robot, b))
                if(pairwise_contact(robot, b, force_limit=force_limit)):
                    continue

                # contact_points = get_contact_points(bodyA = robot, bodyB = b)
                # if(len(contact_points) > 0):
                #     for cp in contact_points:
                #         if(cp.normalForce > 0):
                #         #     print("\tExceeding normal force: ", cp.normalForce)
                #             print("\tNormal force: ", cp.normalForce)
                #         continue



            if teleport:
                path = [q_approach, q_grasp]
            else:
                conf.assign()
                #direction, _ = grasp.approach_pose
                #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                #                                   quat_from_pose(approach_pose))
                path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles)

                if path is None:
                    if DEBUG_FAILURE: wait_if_gui('Approach motion failed')
                    continue

            command = Command([BodyPath(robot, path),
                               Attach(body, robot, grasp.link),
                               BodyPath(robot, path[::-1], attachments=[grasp])])
            return (conf, command)
            # TODO: holding collisions
        return None
    return fn

##################################################

def assign_fluent_state(fluents):
    obstacles = []
    for fluent in fluents:
        name, args = fluent[0], fluent[1:]
        if name == 'atpose':
            o, p = args
            obstacles.append(o)
            p.assign()
        else:
            raise ValueError(name)
    return obstacles


def get_free_motion_gen2(robot, fixed=[], movable=[], force_limit = 0,
                         teleport=False, self_collisions=True):

    def fn(conf1, conf2, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            obstacles = fixed + assign_fluent_state(fluents)
            path = plan_joint_motion2(robot, conf2.joints, conf2.configuration, obstacles=obstacles, movable = movable,
                                      force_limit = force_limit, self_collisions=self_collisions)
            if path is None:
                if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)
    return fn

def get_free_motion_gen(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            obstacles = fixed + assign_fluent_state(fluents)
            path = plan_joint_motion(robot, conf2.joints, conf2.configuration, obstacles=obstacles, self_collisions=self_collisions)
            if path is None:
                if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)
    return fn


def get_free_motion_gen_single_plant(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign_with_controls_old()
            obstacles = fixed + assign_fluent_state(fluents)

            path = plan_joint_motion_single_plant(robot, conf2.joints, conf2.configuration, obstacles=obstacles,
                                                   self_collisions=self_collisions)
            if path is None:
                if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)
    return fn



def get_free_motion_gen_multiworld_benchmark(robot, fixed, multi_world_env, teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            # conf1.assign_with_controls()
            multi_world_env.step(conf1.configuration)
            # obstacles = fixed + assign_fluent_state(fluents)
            path = plan_joint_motion_multiworld_benchmark(robot, conf2.joints, conf2.configuration, multi_world_env,
                                                   obstacles=fixed,
                                                   self_collisions=self_collisions)
            if path is None:
                if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)
    return fn

def get_free_motion_gen_with_controls(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign_with_controls()
            obstacles = fixed + assign_fluent_state(fluents)
            path = plan_joint_motion_with_controls(robot, conf2.joints, conf2.configuration, obstacles=obstacles,
                                                   self_collisions=self_collisions)
            if path is None:
                if DEBUG_FAILURE: wait_if_gui('Free motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)
    return fn

def get_free_motion_gen_with_angle_constraints(robot, fixed=[], movable = [], deflection_limit = 0, teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        # assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        # if teleport:
        #     path = [conf1.configuration, conf2.configuration]

        conf1.assign()
        obstacles = fixed + assign_fluent_state(fluents)
        path = plan_joint_motion_with_angle_contraints(robot, conf2.joints, conf2.configuration, obstacles=obstacles, movable = movable,
                                                        deflection_limit = deflection_limit, self_collisions=self_collisions)

        if path is None:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)

    return fn


def get_free_motion_gen_with_angle_constraints_v4(robot, start_state_id, fixed=[], movable = [], deflection_limit = 0,
                                                  self_collisions=True):
    """
    Method to fina a path between two configurations.

    :param robot: Body ID of the robot.
    :param start_state_id: The saved state ID of the initial state of the environment. This is returned by pybullet
    when saving the environment.
    :param fixed: List of entities in our experiment that will be rigid and fixed during the entire simulation.
    :param movable: List of entities in our experiment that will be allowed to deflect and move. These are the
    characterization objects that are found during the creation of the plants
    :param deflection_limit: Maximum deflection limit each link is allowed to deflect.
    :param self_collisions: Flag that toggles self-collisions during simulation.

    :return: A function that can be used to find a path between two configurations for the given robot and environment.
    """

    def fn(conf1, conf2, fluents=[]):
        """

        :param conf1: Initial configuration
        :param conf2: Final configuration
        :param fluents: fluent states

        :return: A command object that contains the path(s) between the initial and final configurations.
        """

        # Assign the initial configuration
        conf1.assign_with_controls_old()
        obstacles = fixed + assign_fluent_state(fluents)

        # Plan a path between conf1 and conf2
        path = plan_joint_motion_with_angle_contraints_v4(robot, start_state_id, conf2.joints, conf2.configuration,
                                                          obstacles=obstacles, movable=movable,
                                                          deflection_limit=deflection_limit,
                                                          self_collisions=self_collisions)

        if path is None:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)

    return fn



def get_free_motion_gen_with_angle_constraints_multi_world(robot, start_state_id, multi_world_env, self_collisions=True):

    """
    Method to fina a path between two configurations.

    :param robot: Body ID of the robot.
    :param start_state_id: The saved state ID of the initial state of the environment. This is returned by pybullet
    when saving the environment.
    :param fixed: List of entities in our experiment that will be rigid and fixed during the entire simulation.
    :param movable: List of entities in our experiment that will be allowed to deflect and move. These are the
    characterization objects that are found during the creation of the plant
    :param deflection_limit: Maximum deflection limit each link is allowed to deflect.
    :param self_collisions: Flag that toggles self-collisions during simulation.

    :return: A function that can be used to find a path between two configurations for the given robot and environment.
    """

    def fn(conf1, conf2):
        """

        :param conf1: Initial configuration
        :param conf2: Final configuration
        :param fluents: fluent states

        :return: A command object that contains the path(s) between the initial and final configurations.
        """

        # Assign the initial configuration
        # obstacles = fixed

        # conf1.assign_with_controls()
        # for env in multi_world_env.envs:
        #     set_joint_positions(multi_world_env.envs[env].robot, multi_world_env.joints, conf1.configuration)
        # multi_world_env.step(conf1.configuration)

        multi_world_env.step(conf1.configuration, True)

        # Plan a path between conf1 and conf2
        # path = plan_joint_motion_with_angle_contraints_v6(robot, start_state_id, conf2.joints, conf2.configuration,
        #                                                   obstacles=obstacles, movable=movable,
        #                                                   deflection_limit=deflection_limit,
        #                                                   self_collisions=self_collisions)
        path = plan_joint_motion_with_angle_contraints_multi_world(robot, start_state_id, conf2.configuration,
                                                          multi_world_env, self_collisions=self_collisions)

        if path is None:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command, )

    return fn


def get_free_motion_gen_with_angle_constraints_v7(robot, start_state_id, single_plant_env, self_collisions=True):

    """
    Method to fina a path between two configurations.

    :param robot: Body ID of the robot.
    :param start_state_id: The saved state ID of the initial state of the environment. This is returned by pybullet
    when saving the environment.
    :param fixed: List of entities in our experiment that will be rigid and fixed during the entire simulation.
    :param movable: List of entities in our experiment that will be allowed to deflect and move. These are the
    characterization objects that are found during the creation of the plant
    :param deflection_limit: Maximum deflection limit each link is allowed to deflect.
    :param self_collisions: Flag that toggles self-collisions during simulation.

    :return: A function that can be used to find a path between two configurations for the given robot and environment.
    """

    def fn(conf1, conf2):
        """

        :param conf1: Initial configuration
        :param conf2: Final configuration
        :param fluents: fluent states

        :return: A command object that contains the path(s) between the initial and final configurations.
        """

        # Assign the initial configuration
        # obstacles = fixed

        # conf1.assign_with_controls()
        set_joint_positions(robot, single_plant_env.joints, conf1.configuration)
        single_plant_env.step(conf1.configuration)

        # Plan a path between conf1 and conf2
        # path = plan_joint_motion_with_angle_contraints_v6(robot, start_state_id, conf2.joints, conf2.configuration,
        #                                                   obstacles=obstacles, movable=movable,
        #                                                   deflection_limit=deflection_limit,
        #                                                   self_collisions=self_collisions)
        path = plan_joint_motion_with_angle_contraints_v7(robot, start_state_id, conf2.configuration,
                                                          single_plant_env, self_collisions=self_collisions)

        if path is None:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command, )

    return fn

def get_free_motion_gen_with_angle_constraints_v6(robot, start_state_id, fixed=[], movable = [], deflection_limit = 0,
                                                  self_collisions=True):

    """
    Method to fina a path between two configurations.

    :param robot: Body ID of the robot.
    :param start_state_id: The saved state ID of the initial state of the environment. This is returned by pybullet
    when saving the environment.
    :param fixed: List of entities in our experiment that will be rigid and fixed during the entire simulation.
    :param movable: List of entities in our experiment that will be allowed to deflect and move. These are the
    characterization objects that are found during the creation of the plant
    :param deflection_limit: Maximum deflection limit each link is allowed to deflect.
    :param self_collisions: Flag that toggles self-collisions during simulation.

    :return: A function that can be used to find a path between two configurations for the given robot and environment.
    """

    def fn(conf1, conf2, fluents=[]):
        """

        :param conf1: Initial configuration
        :param conf2: Final configuration
        :param fluents: fluent states

        :return: A command object that contains the path(s) between the initial and final configurations.
        """

        # Assign the initial configuration
        conf1.assign_with_controls()
        obstacles = fixed + assign_fluent_state(fluents)

        # Plan a path between conf1 and conf2
        path = plan_joint_motion_with_angle_contraints_v6(robot, start_state_id, conf2.joints, conf2.configuration,
                                                          obstacles=obstacles, movable=movable,
                                                          deflection_limit=deflection_limit,
                                                          self_collisions=self_collisions)

        if path is None:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command, )

    return fn


def get_free_motion_gen_with_angle_constraints_v5(robot, start_state_id, fixed=[], movable = [], deflection_limit = 0, saved_state = 0,
                                                  teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        # assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        # if teleport:
        #     path = [conf1.configuration, conf2.configuration]

        conf1.assign()
        obstacles = fixed + assign_fluent_state(fluents)

        path = plan_joint_motion_with_angle_constraints_v5(robot, start_state_id, conf2.joints, conf2.configuration,
                                                          obstacles=obstacles, movable=movable,
                                                          deflection_limit=deflection_limit,
                                                          self_collisions=self_collisions)

        if path is None:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)

    return fn

def get_free_motion_gen_with_angle_constraints_v2(robot, start_state_id, fixed=[], movable = [], deflection_limit = 0, saved_state = 0,
                                                  teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        # assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        # if teleport:
        #     path = [conf1.configuration, conf2.configuration]

        conf1.assign()
        obstacles = fixed + assign_fluent_state(fluents)
        path = plan_joint_motion_with_angle_contraints_v2(robot, start_state_id, conf2.joints, conf2.configuration, obstacles=obstacles,
                                                       movable = movable,
                                                       deflection_limit = deflection_limit, self_collisions=self_collisions)
        if path is None:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)

    return fn



def get_free_motion_gen_with_angle_constraints_v3(robot, start_state_id, fixed=[], movable = [], deflection_limit = 0, saved_state = 0,
                                                  teleport=False, self_collisions=True):
    def fn(conf1, conf2, fluents=[]):
        # assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        # if teleport:
        #     path = [conf1.configuration, conf2.configuration]

        conf1.assign()
        obstacles = fixed + assign_fluent_state(fluents)
        path = plan_joint_motion_with_angle_contraints_v3(robot, start_state_id, conf2.joints, conf2.configuration, obstacles=obstacles,
                                                          movable = movable,
                                                          deflection_limit = deflection_limit, self_collisions=self_collisions)
        if path is None:
            if DEBUG_FAILURE: wait_if_gui('Free motion failed')
            return None
        command = Command([BodyPath(robot, path, joints=conf2.joints)])
        return (command,)

    return fn


def get_holding_motion_gen(robot, fixed=[], teleport=False, self_collisions=True):
    def fn(conf1, conf2, body, grasp, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            obstacles = fixed + assign_fluent_state(fluents)
            path = plan_joint_motion(robot, conf2.joints, conf2.configuration,
                                     obstacles=obstacles, attachments=[grasp.attachment()], self_collisions=self_collisions)
            if path is None:
                if DEBUG_FAILURE: wait_if_gui('Holding motion failed')
                return None
        command = Command([BodyPath(robot, path, joints=conf2.joints, attachments=[grasp])])
        return (command,)
    return fn


##################################################

def get_movable_collision_test():
    def test(command, body, pose):
        if body in command.bodies():
            return False
        pose.assign()
        for path in command.body_paths:
            moving = path.bodies()
            if body in moving:
                # TODO: cannot collide with itself
                continue
            for _ in path.iterator():
                # TODO: could shuffle this
                if any(pairwise_collision(mov, body) for mov in moving):
                    if DEBUG_FAILURE: wait_if_gui('Movable collision')
                    return True
        return False
    return test
