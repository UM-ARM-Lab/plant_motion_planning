import torch
import random

class Node:
    def __init__(self, parent, x, u, q, state_id):
        self.parent = parent # Parent node (None for root)
        self.xs = x[::-1] # State configuration
        self.us = u[::-1] if u is not None else None # Control input used to get to state (None for root)
        self.qs = q[::-1] # Arm joint configuration
        self.state_id = state_id
    
    def extract_path(self):
        reverse_path = []
        n = self
        while(n.parent is not None):
            for x, q, u in zip(n.xs, n.qs, n.us):
                reverse_path.append((x, q, u))
            n = n.parent
        path = reverse_path[::-1]
        return path
        
def rrt_solve(start, goal, dynamics_fn, steering_fn, connect_fn, collision_fn, save_state_fn, restore_state_fn,
                 base_cost_fn, arms_cost_fn, sample_fn, prims, execute_fn, goal_sampling=0.1, max_iterations=5000, 
                 epsilon=1e-1, smoothing_iterations=15):
    xi = start[0]
    xg = goal[0]
    qi = start[1]
    qg = goal[1]
    state_id = save_state_fn()
    
    root = Node(None, [xi], None, [qi], state_id)
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
            cost = base_cost_fn(n.xs[0], targetx) + arms_cost_fn(n.qs[0], targetq) * 3
            if cost < min_cost and not(is_target_goal and n.xs[0] in goal_attempts):
                min_cost = cost
                nearest_node = n

        if nearest_node is None:
            continue

        # Use steering function for goal samples
        if is_target_goal:
            x = nearest_node.xs[0]
            q = nearest_node.qs[0]
            restore_state_fn(nearest_node.state_id)
            goal_attempts.add(x)
            z = steering_fn(x, targetx)
            arms_path = connect_fn(q, targetq, len(z))
            if arms_path is None:
                continue

            end_path = []
            is_collision = False

            # Check path for collisions
            for u, q in zip(z, arms_path):
                x = dynamics_fn(x, u)
                
                if collision_fn(x, q):
                    is_collision = True
                    break
                state_id = save_state_fn()
                end_path.append((x, q, u))
            
            # If we got to the goal, success
            if not is_collision and base_cost_fn(x, targetx) < epsilon and arms_cost_fn(q, targetq) < epsilon:
                print("steering length", len(end_path))
                path = nearest_node.extract_path()
                path = path + end_path
                break

        # # Use motion primitives otherwise
        # else:
        cost_to_target = torch.inf
        #while True:
        restore_state_fn(nearest_node.state_id)

        # Try prims to find most optimal one
        min_cost_to_target = torch.inf
        min_cost_state = None
        min_cost_prim = None
        for z in prims:
            x = nearest_node.xs[0]
            for u in z:
                x = dynamics_fn(x, u)
            curr_cost_to_target = base_cost_fn(x, targetx)
            if curr_cost_to_target < min_cost_to_target:
                min_cost_to_target = curr_cost_to_target
                min_cost_state = x
                min_cost_prim = z

        # Check collision
        x = nearest_node.xs[0]
        arms_path = connect_fn(nearest_node.qs[0], targetq, len(min_cost_prim))

        parent = nearest_node
        xs = []
        qs = []
        us = []
        is_collision = False
        for u, q in zip(min_cost_prim, arms_path):
            x = dynamics_fn(x, u)
            if collision_fn(x, q):
                is_collision = True
                break
            xs.append(x)
            qs.append(q)
            us.append(u)
        
        if not is_collision:
            state_id = save_state_fn()
            nodes.append(Node(parent, xs, us, qs, state_id))
            # # Keep going as long as cost to target keeps decreasing
            # if min_cost_to_target < cost_to_target:
            #     nearest_node = parent
            #     cost_to_target = min_cost_to_target
            # else:
            #     break
    
    if path is None:
        print("Could not find path")
        return path, path

    old_path = path.copy()

    for i in range(smoothing_iterations):
        print("Smoothing iteration", i)
        # Sample two random states on path
        i0 = random.randint(0, len(path) - 2)
        i1 = random.randint(i0+1, len(path) - 1)

        print(i1-i0)

        if i0 != i1:
            x0, q0, _ = path[i0]
            x1, q1, _ = path[i1]

            # Connect states
            z = steering_fn(x0, x1, max_iterations=i1-i0)

            # Only proceed if steering function did better than original path
            # if len(z) == (i1 - i0):
            #     continue

            smoothed_path = []
            qs = connect_fn(q0, q1, len(z))
            x = x0
            is_collision = False
            print(i1 - i0, len(z))
            # Check collision
            for u, q in zip(z, qs):
                x = dynamics_fn(x, u)
                if collision_fn(x, q):
                    is_collision = True
                    break
                smoothed_path.append((x, q, u))
            if is_collision:
                continue
                
            # Connect rest of path
            q = qs[-1]
            for i in range(i1+1, len(path)):
                _, q, u = path[i]
                x = dynamics_fn(x, u)
                if collision_fn(x, q):
                    is_collision = True
                    break
                smoothed_path.append((x, q, u))
                q = qs[-1]
            if is_collision:
                continue

            # Check if we're still within our goal tolerance
            if base_cost_fn(x, xg) + arms_cost_fn(q, qg) < epsilon:
                print("Smoothing success")
                path[i0+1:-1] = smoothed_path
            else:
                print("End cost too high")

    print("Ran for ", i, " iterations")
    return path, old_path
        