import torch
import random

# Node for configuration space tree
class Node_F:
    def __init__(self, parent, x, u, q, state_id):
        self.parent = parent
        self.x = x
        self.u = u
        self.q = q
        self.state_id = state_id
    
    def extract_path(self):
        reverse_path = []
        n = self
        while(n.parent is not None):
            reverse_path.append((n.x, n.q))
            n = n.parent
        path = reverse_path[::-1]
        return path

# Node for goal space tree
class Node_B:
    def __init__(self, parent, b, node_id=None):
        self.parent = parent
        self.b = b
        self.node_id = node_id

# @param start: start -> (xi, qi) represent starting base state and arm configuration
# @param goal: SE3 representation of goal pose for end effector
# @param dynamics_fn: f(x0, u) -> x1 represents dynamics of base given control input u
# @param connect_fn: f(qi, qg, num_steps) -> qs returns list of arm conf tensors representing connecting initial and final arm with max steps specified
# @param collision_fn: f(x, q) -> True/False
# @param save_state_fn: f() -> state_id
# @param restore_state_fn: f(state_id) resets env to state including plant deflections
  
def bispace_rrt(start, goal, dynamics_fn, connect_fn, collision_fn, save_state_fn, restore_state_fn, base_cost_fn, arms_cost_fn, conf_to_goal_fn,
                goal_cost_fn, sample_fn, sample_conf_step_fn, sample_goal_step_fn, prims, follow_prob=0.2, f_iters=5, b_iters=3, follow_iters=500, 
                follow_samples=100, max_iters=5000, epsilon=1e-2, debug_fn=None):
    
    xi, qi = start
    bg = goal

    if collision_fn(xi, qi):
        print("Error. Initial state in collision")
        return None

    state_id = save_state_fn()
    f_tree = [Node_F(None, xi, None, qi, state_id)]
    b_tree = [Node_B(None, bg)]

    is_forward = False
    
    for i in range(max_iters):
        if is_forward:
            for j in range(f_iters):
                # Extend forward tree
                targetx, targetq = sample_fn()

                # Find nearest neighbor
                min_cost = torch.inf
                nearest_node = None
                for n in f_tree:
                    cost = base_cost_fn(n.x, targetx) + arms_cost_fn(n.q, targetq) * 3
                    if cost < min_cost:
                        min_cost = cost
                        nearest_node = n
                
                # Find min cost prim
                min_cost = torch.inf
                min_prim = None
                for z in prims:
                    x = nearest_node.x
                    for u in z:
                        x = dynamics_fn(x, u)
                    cost = base_cost_fn(x, targetx)
                    if cost < min_cost:
                        min_cost = cost
                        min_prim = z
                
                # Find arms path
                qs = connect_fn(nearest_node.q, targetq, len(min_prim))

                # Check collision
                #restore_state_fn(nearest_node.state_id)
                nodes = []
                parent = nearest_node
                is_collision = False
                x = nearest_node.x
                for u, q in zip(min_prim, qs):
                    x = dynamics_fn(x, u)
                    if collision_fn(x, q):
                        is_collision = True
                        break
                    n = Node_F(parent, x, u, q, save_state_fn())
                    nodes.append(n)
                    parent = n

                if is_collision:
                    continue
                    
                # Add nodes to tree
                for n in nodes:
                    f_tree.append(n)

                # Follow tree to goal
                if random.uniform(0, 1) < follow_prob:
                    print("Attempting follow")
                    start_node = n
                    end_path = []

                    # Find nearest neighbor in goal tree
                    curr_x = x
                    curr_q = q
                    curr_b = conf_to_goal_fn(x, q)
                    min_cost = torch.inf
                    nearest_node = None
                    for n in b_tree:
                        cost = goal_cost_fn(curr_b, n.b)
                        if cost < min_cost:
                            min_cost = cost
                            nearest_node = n
                    
                    # Repeatidly try to follow goal tree to goal
                    is_goal = nearest_node.parent is None
                    for k in range(follow_iters):
                        if debug_fn is not None:
                            debug_fn(follow_node=True, node_id=nearest_node.node_id)
                    
                        # Sample lowest cost action to goal
                        best_cost = goal_cost_fn(curr_b, nearest_node.b)
                        min_u = None
                        min_q = None
                        for l in range(follow_samples):
                            u, q = sample_conf_step_fn(curr_q)
                            x = dynamics_fn(curr_x, u)

                            if collision_fn(x, q):
                                continue
                            
                            cost = goal_cost_fn(conf_to_goal_fn(x, q), nearest_node.b)
                            if cost < best_cost:
                                best_cost = cost
                                min_u = u
                                min_q = q
                        
                        print("Cost to goal", best_cost)
                        if min_u is None or best_cost < (epsilon if is_goal else 50*epsilon): 
                            if debug_fn is not None:
                                debug_fn(unfollow_node=True, node_id=nearest_node.node_id)
                            nearest_node = nearest_node.parent
                            print("Moving up to parent")
                            if nearest_node is None:
                                if best_cost < epsilon:
                                    print("Path found!")
                                    path = start_node.extract_path()
                                    x, q = end_path[-1]
                                    print("Final pose", conf_to_goal_fn(x, q))
                                    return path + end_path
                                else:
                                    print("Follow failed")
                                    break
                            is_goal = nearest_node.parent is None
                        else:
                            curr_x = dynamics_fn(curr_x, min_u)
                            curr_q = min_q
                            end_path.append((x, q))
        else:
            for j in range(b_iters):
                # Pick a random node to extend from
                idx = random.randint(0, len(b_tree)-1)
                parent = b_tree[idx]
                bstep = sample_goal_step_fn()
                pose = parent.b + bstep
                node_id = None
                if debug_fn is not None:
                    node_id = debug_fn(create_node=True, node_position=pose[0:3])
                n = Node_B(parent, pose, node_id)
                b_tree.append(n)

        is_forward = not is_forward

    # No path found
    return None