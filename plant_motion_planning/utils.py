import pybullet as p
from motion_planners.motion_planners.utils import waypoints_from_path, compute_path_cost

def draw_frame_axes(start, length, width = 2, lifetime = 0, parentid = None, parentlinkid = None):

    if(parentid == None):
        x_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0] + length,start[1],start[2]), lineColorRGB=[1, 0, 0],
                                          lineWidth=width,
                                          lifeTime=lifetime)
        y_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0],start[1] + length,start[2]), lineColorRGB=[0, 1, 0],
                                          lineWidth=width,
                                          lifeTime=lifetime)
        z_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0],start[1],start[2] + length), lineColorRGB=[0, 0, 1],
                                          lineWidth=width,
                                          lifeTime=lifetime)
    elif(parentlinkid == None):
        x_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0] + length,start[1],start[2]), lineColorRGB=[1, 0, 0],
                                          lineWidth=width,
                                          lifeTime=lifetime,parentObjectUniqueId=parentid)
        y_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0],start[1] + length,start[2]), lineColorRGB=[0, 1, 0],
                                          lineWidth=width,
                                          lifeTime=lifetime,parentObjectUniqueId=parentid)
        z_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0],start[1],start[2] + length), lineColorRGB=[0, 0, 1],
                                          lineWidth=width,
                                          lifeTime=lifetime,parentObjectUniqueId=parentid)
    else:
        x_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0] + length,start[1],start[2]), lineColorRGB=[1, 0, 0],
                                          lineWidth=width,
                                          lifeTime=lifetime,parentObjectUniqueId=parentid, parentLinkIndex=parentlinkid)
        y_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0],start[1] + length,start[2]), lineColorRGB=[0, 1, 0],
                                          lineWidth=width,
                                          lifeTime=lifetime,parentObjectUniqueId=parentid, parentLinkIndex=parentlinkid)
        z_axis_world = p.addUserDebugLine(lineFromXYZ=start, lineToXYZ=(start[0],start[1],start[2] + length), lineColorRGB=[0, 0, 1],
                                          lineWidth=width,
                                          lifeTime=lifetime,parentObjectUniqueId=parentid, parentLinkIndex=parentlinkid)

def compute_total_cost(path, cost_fn, deflection_over_limit, alpha = 1.0):

    waypoints = waypoints_from_path(path)
    path_length_cost = compute_path_cost(waypoints, cost_fn)

    total_cost = path_length_cost + (alpha * deflection_over_limit)

    return total_cost