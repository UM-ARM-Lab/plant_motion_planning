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
            reverse_path.append(n.x)
            n = n.parent
        path = reverse_path[::-1]
        return path
        
def rrt_solve(xi, xg, dynamics_fn, steering_fn, collision_fn, cost_fn, sample_fn, prims, goal_sampling=0.1, max_iterations=2000, epsilon=0.5, smoothing_iterations=50):
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
                end_path.append(x)
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
    
    startIndex = 0
    endIndex = len(path)-1
    nodeStack = []

    while (startIndex != endIndex or nodeStack) and startIndex != len(path)-1:
        if startIndex == endIndex:
            endIndex = startIndex - 1
            n = nodeStack.pop()
            startIndex = n[0]

        # Try to connect two points
        print('asdf', path[endIndex])
        z = steering_fn(path[startIndex], path[endIndex], max_iterations=endIndex-startIndex)

        # Check if steering function worked
        if len(z) == (endIndex - startIndex):
            endIndex -= 1
        
        # Check path
        else:
            is_collision = False
            x = path[startIndex]
            xs = []
            for u in z:
                x = dynamics_fn(x, u)
                xs.append(x)
                if collision_fn(x):
                    is_collision = True
                    break
            
            # Check if path was invalid
            if is_collision or cost_fn(x, path[endIndex]) > epsilon:
                endIndex -= 1
            
            # Path is valid, push start index on stack and continue
            else:
                nodeStack.append((startIndex, xs))
                startIndex = endIndex
                endIndex = len(path)-1
    
    smoothed_path = []

    # Success
    if startIndex == len(path)-1:
        print("Smoothed path successfully")

        # Get path
        for n in nodeStack:
            xs = n[1]
            smoothed_path = smoothed_path + xs

    # Failure, just return unsmoothed path
    else:
        print("Could not smooth path")
        smoothed_path = path

    print("Ran for ", i, " iterations")
    return smoothed_path, path
        