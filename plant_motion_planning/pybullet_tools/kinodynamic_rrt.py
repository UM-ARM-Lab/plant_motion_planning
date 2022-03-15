import torch
import random

class Node:
    def __init__(self, parent, x, z):
        self.parent = parent # Parent node (None for root)
        self.x = x # State configuration
        self.z = z # Motion primitive used to get to state (None for root)
    
    def extract_path(self):
        reverse_path = []
        n = self
        while(n.parent is not None):
            reverse_path.append(n.z)
            n = n.parent
        path = reverse_path[::-1]
        return path
        
def rrt_solve(xi, xg, dynamics_fn, steering_fn, collision_fn, cost_fn, sample_fn, prims, goal_sampling=0.1, max_iterations=2000, epsilon=0.2):
    root = Node(None, xi, None)
    nodes = [root]

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

            # Check path for collisions
            for u in z:
                x = dynamics_fn(x, u)
                if collision_fn(x):
                    continue
            
            # If we got to the goal, success
            if cost_fn(x, target) < epsilon:
                path = nearest_node.extract_path()
                path.append(z)
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
                newNode = Node(nearest_node, min_cost_state, [])
                x = nearest_node.x
                for u in min_cost_prim:
                    x = dynamics_fn(x, u)
                    if collision_fn(x):
                        break
                    newNode.z.append(u)
                if newNode.z:
                    nodes.append(newNode)

                if min_cost_to_target < cost_to_target:
                    nearest_node = newNode
                    cost_to_target = min_cost_to_target
                else:
                    break
        
    print("Ran for ", i, " iterations")
    return path
        