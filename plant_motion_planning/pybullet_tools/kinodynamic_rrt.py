from cmath import cos
import torch
import random

class Node:
    def __init__(self, parent, x, u):
        self.parent = parent # Parent node (None for root)
        self.x = x # State configuration
        self.u = u # Control input used to get to state (None for root)
    
    def extract_path(self):
        reverse_path = []
        n = self
        while(n.parent is not None):
            reverse_path.append(n)
            n = n.parent
        path = reverse_path[::-1]
        return path
        
def rrt_solve(xi, xg, dynamics_fn, steering_fn, collision_fn, cost_fn, sample_fn, prims, goal_sampling=0.1, max_iterations=2000, epsilon=0.5, smoothing_iterations=15):
    root = Node(None, xi, None)
    nodes = [root]
    print("rrt start")
    path = None
    for i in range(max_iterations):
        # Goal biasing
        if random.uniform(0, 1) < goal_sampling:
            target = xg
            is_target_goal = True
        else:
            target = sample_fn()
            is_target_goal = False
        
        # Find nearest neighbor
        min_cost = torch.inf
        nearest_node = None
        for n in nodes:
            cost = cost_fn(n.x, target)
            if cost < min_cost:
                min_cost = cost
                nearest_node = n

        # Use steering function for goal samples
        if is_target_goal:
            x = nearest_node.x
            z = steering_fn(x, target)
            end_path = []

            # Check path for collisions
            for u in z:
                x = dynamics_fn(x, u)
                end_path.append(Node(None, x, u))
                if collision_fn(x):
                    continue
            
            # If we got to the goal, success
            if cost_fn(x, target) < (epsilon / 5):
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
                    curr_cost_to_target = cost_fn(x, target)
                    if curr_cost_to_target < min_cost_to_target:
                        min_cost_to_target = curr_cost_to_target
                        min_cost_state = x
                        min_cost_prim = z

                # Check collision
                x = nearest_node.x
                parent = nearest_node
                for u in min_cost_prim:
                    x = dynamics_fn(x, u)
                    if collision_fn(x):
                        break
                    n = Node(parent, x, u)
                    nodes.append(n)
                    parent = n

                if min_cost_to_target < cost_to_target:
                    nearest_node = parent
                    cost_to_target = min_cost_to_target
                else:
                    break
    
    old_path = path.copy()

    for i in range(smoothing_iterations):
        print("Smoothing iteration", i)
        # Sample two random states on path
        i0 = random.randint(0, len(path) - 2)
        i1 = random.randint(i0+1, len(path) - 1)

        if i0 != i1:
            n0 = path[i0]
            n1 = path[i1]

            # Connect states
            z = steering_fn(n0.x, n1.x, max_iterations=i1-i0)

            # Only proceed if steering function did better than original path
            # if len(z) == (i1 - i0):
            #     continue

            x = n0.x
            smoothed_path = []
            is_collision = False
            print(i1 - i0, len(z))
            # Check collision
            for u in z:
                x = dynamics_fn(x, u)
                if collision_fn(x):
                    is_collision = True
                    break
                n = Node(None, x, u)
                smoothed_path.append(n)
            if is_collision:
                continue
            
            # Check rest of path
            for i in range(i1+1, len(path)):
                x = dynamics_fn(x, path[i].u)
                if collision_fn(x):
                    is_collision = True
                    break
                n = Node(None, x, path[i].u)
                smoothed_path.append(n)
            
            if is_collision:
                continue

            # If we end up outside our goal radius, try connecting to the goal again
            if cost_fn(x, xg) > epsilon:
                z = steering_fn(x, xg, 10)
                for u in z:
                    x = dynamics_fn(x, u)
                if collision_fn(x):
                    is_collision = True
                    break
                n = Node(None, x, u)
                smoothed_path.append(n)

            if is_collision:
                continue

            # Check if we're still within our goal tolerance
            if cost_fn(x, xg) < epsilon:
                print("Smoothing success")
                path[i0+1:i1+1] = smoothed_path

    print("Ran for ", i, " iterations")
    return path, old_path
        