from random import random
import time

from .utils import irange, argmin, RRT_ITERATIONS, apply_alpha, RED, INF, elapsed_time
import plant_motion_planning.pybullet_tools.utils as pyb_tools_utils
import plant_motion_planning.utils as s_utils

class TreeNode_v2(object):

    def __init__(self, config, state_id = -1, parent=None):
        self.config = config
        self.parent = parent

        # Store the state of the simulation
        self.state_id = state_id

    #def retrace(self):
    #    if self.parent is None:
    #        return [self]
    #    return self.parent.retrace() + [self]

    def save_state(self):
        self.state_id = pyb_tools_utils.save_state()

    def restore_state(self):
        pyb_tools_utils.restore_state(self.state_id)

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def clear(self):
        self.node_handle = None
        self.edge_handle = None

    def draw(self, env, color=apply_alpha(RED, alpha=0.5)):
        # https://github.mit.edu/caelan/lis-openrave
        from manipulation.primitives.display import draw_node, draw_edge
        self.node_handle = draw_node(env, self.config, color=color)
        if self.parent is not None:
            self.edge_handle = draw_edge(
                env, self.config, self.parent.config, color=color)

    def __str__(self):
        return 'TreeNode(' + str(self.config) + ')'
    __repr__ = __str__


class TreeNode(object):

    def __init__(self, config, parent=None):
        self.config = config
        self.parent = parent

    #def retrace(self):
    #    if self.parent is None:
    #        return [self]
    #    return self.parent.retrace() + [self]

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def clear(self):
        self.node_handle = None
        self.edge_handle = None

    def draw(self, env, color=apply_alpha(RED, alpha=0.5)):
        # https://github.mit.edu/caelan/lis-openrave
        from manipulation.primitives.display import draw_node, draw_edge
        self.node_handle = draw_node(env, self.config, color=color)
        if self.parent is not None:
            self.edge_handle = draw_edge(
                env, self.config, self.parent.config, color=color)

    def __str__(self):
        return 'TreeNode(' + str(self.config) + ')'
    __repr__ = __str__


def configs(nodes):
    if nodes is None:
        return None
    return list(map(lambda n: n.config, nodes))


def rrt_with_angle_constraints(start, start_state_id, goal_sample, distance_fn, sample_fn, extend_fn, collision_fn, goal_test=lambda q: False,
        goal_probability=.8, max_iterations=RRT_ITERATIONS, max_time=INF):
    """
    :param start: Start configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :return: Path [q', ..., q"] or None if unable to find a solution
    """

    input("Starting RRT... Press enter to continue...")

    start_time = time.time()

    if collision_fn(start):
        return None

    # start_state_id = pyb_tools_utils.save_state()
    pyb_tools_utils.restore_state(start_state_id)

    if not callable(goal_sample):
        g = goal_sample
        goal_sample = lambda: g

    nodes = [TreeNode_v2(start, state_id=start_state_id)]

    max_iterations = 50

    for i in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break

        s_utils.step_sim()

        # print(f"Iteration: {i}")
        # input("press enter to continue...")

        goal = random() < goal_probability or i == 0

        s = goal_sample() if goal else sample_fn()

        last = argmin(lambda n: distance_fn(n.config, s), nodes)
        last.restore_state()

        for q in extend_fn(last.config, s):

            s_utils.step_sim()

            if collision_fn(q):
                break
            last = TreeNode_v2(q, parent=last)
            last.save_state()

            nodes.append(last)

            if goal_test(last.config):
                return configs(last.retrace())
        else:
            if goal:
                nodes[0].restore_state()
                return configs(last.retrace())
    return None
