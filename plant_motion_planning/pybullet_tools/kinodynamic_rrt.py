from cmath import cos
from re import X
import torch
import random
import pybullet as p

class Node:
    def __init__(self, parent, x, u, q):
        self.parent = parent # Parent node (None for root)
        self.x = x # State configuration
        self.u = u # Control input used to get to state (None for root)
        self.q = q # Arm joint configuration
    
    def extract_path(self):
        reverse_path = []
        n = self
        while(n.parent is not None):
            reverse_path.append(n)
            n = n.parent
        path = reverse_path[::-1]
        return path
        
def rrt_solve(start, goal, dynamics_fn, steering_fn, connect_fn, collision_fn, base_cost_fn, arms_cost_fn, sample_fn, prims, execute_fn, goal_sampling=0.1, max_iterations=5000, epsilon=1e-1, smoothing_iterations=20):
    xi = start[0]
    xg = goal[0]
    qi = start[1]
    qg = goal[1]
    
    root = Node(None, xi, None, qi)
    nodes = [root]
    print("rrt start")
    path = None
    goal_attempts = set()

    for i in range(max_iterations):
        # Goal biasing
        if random.uniform(0, 1) < goal_sampling:
            targetx = xg
            targetq = qg
            is_target_goal = True
        else:
            targetx, targetq = sample_fn()
            is_target_goal = False
        
        # Find nearest neighbor
        min_cost = torch.inf
        nearest_node = None
        for n in nodes:
            cost = base_cost_fn(n.x, targetx) + arms_cost_fn(n.q, targetq) * 3
            if cost < min_cost and not(is_target_goal and n.x in goal_attempts):
                min_cost = cost
                nearest_node = n

        if nearest_node is None:
            continue

        # Use steering function for goal samples
        if is_target_goal:
            x = nearest_node.x
            q = nearest_node.q
            goal_attempts.add(x)
            print(x)
            z = steering_fn(x, targetx)
            arms_path = connect_fn(q, targetq, len(z))
            if arms_path is None:
                continue

            end_path = []
            is_collision = False

            # Check path for collisions
            for u, q in zip(z, arms_path):
                x = dynamics_fn(x, u)
                end_path.append(Node(None, x, u, q))
                if collision_fn(x, q):
                    is_collision = True
                    break
            
            # If we got to the goal, success
            if not is_collision and base_cost_fn(x, targetx) < epsilon and arms_cost_fn(q, targetq) < epsilon:
                path = nearest_node.extract_path()
                path = path + end_path
                break

        # Use motion primitives otherwise
        else:
            cost_to_target = torch.inf
            while True:
                # Try prims to find most optimal one
                min_cost_to_target = torch.inf
                min_cost_state = None
                min_cost_prim = None
                for z in prims:
                    x = nearest_node.x
                    for u in z:
                        x = dynamics_fn(x, u)
                    curr_cost_to_target = base_cost_fn(x, targetx)
                    if curr_cost_to_target < min_cost_to_target:
                        min_cost_to_target = curr_cost_to_target
                        min_cost_state = x
                        min_cost_prim = z

                # Check collision
                x = nearest_node.x
                arms_path = connect_fn(nearest_node.q, targetq, len(min_cost_prim))
                if arms_path is None:
                    break
                parent = nearest_node
                for u, q in zip(min_cost_prim, arms_path):
                    x = dynamics_fn(x, u)
                    if collision_fn(x, q):
                        break
                    n = Node(parent, x, u, q)
                    nodes.append(n)
                    parent = n

                if min_cost_to_target < cost_to_target:
                    nearest_node = parent
                    cost_to_target = min_cost_to_target
                else:
                    break
    
    if path is None:
        print("Could not find path")
        return path, path

    old_path = path.copy()

    # for i in range(smoothing_iterations):
    #     print("Smoothing iteration", i)
    #     # Sample two random states on path
    #     i0 = random.randint(0, len(path) - 2)
    #     i1 = random.randint(i0+1, len(path) - 1)

    #     if i0 != i1:
    #         n0 = path[i0]
    #         n1 = path[i1]

    #         p.addUserDebugLine((n0.x[0, 0], n0.x[0, 1], 0.1), (n1.x[0, 0], n1.x[0, 1], 0.1), lifeTime=5, lineColorRGB=(0, 0, 1))

    #         # Connect states
    #         z = steering_fn(n0.x, n1.x)

    #         # Only proceed if steering function did better than original path
    #         # if len(z) == (i1 - i0):
    #         #     continue

    #         x = n0.x
    #         smoothed_path = []
    #         is_collision = False
    #         print(i1 - i0, len(z))
    #         # Check collision
    #         for u in z:
    #             x = dynamics_fn(x, u)
    #             if collision_fn(x):
    #                 is_collision = True
    #                 break
    #             n = Node(None, x, u)
    #             smoothed_path.append(n)
    #         if is_collision:
    #             continue
    #         execute_fn(smoothed_path[0], smoothed_path)

    #         # Check rest of path
    #         for i in range(i1+1, len(path)):
    #             z = steering_fn(x, path[i].x, max_iterations=5)
    #             for u in z:
    #                 x = dynamics_fn(x, u)
    #                 if collision_fn(x):
    #                     is_collision = True
    #                     break
    #                 n = Node(None, x, u)
    #                 smoothed_path.append(n)

    #         if is_collision:
    #             continue

    #         # If we end up outside our goal radius, try connecting to the goal again
    #         if base_cost_fn(x, xg) > epsilon:
    #             z = steering_fn(x, xg, 10)
    #             for u in z:
    #                 x = dynamics_fn(x, u)
    #             if collision_fn(x):
    #                 is_collision = True
    #                 break
    #             n = Node(None, x, u)
    #             smoothed_path.append(n)

    #         if is_collision:
    #             continue

    #         # Check if we're still within our goal tolerance
    #         if base_cost_fn(x, xg) < epsilon:
    #             print("Smoothing success")
    #             path[i0+1:i1+1] = smoothed_path

    print("Ran for ", i, " iterations")
    return path, old_path
        