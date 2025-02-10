import numpy as np
from scipy.interpolate import CubicSpline
# from scipy.interpolate import Akima1DInterpolator
from scipy.interpolate import UnivariateSpline
from scipy.optimize import minimize

from scipy.interpolate import interp1d
import math

# Distance squared function for optimization
def distance_squared(t, xy, spline_x, spline_y):
    tx = np.array([t, spline_x(t)])
    ty = np.array([t, spline_y(t)])
    return (tx[1] - xy[0])**2 + (ty[1] - xy[1])**2


def get_ref_trajectory_ds(traj_points, N, ds):
    # Convert to NumPy arrays for vectorized operations
    x = np.array([point['x'] for point in traj_points])
    y = np.array([point['y'] for point in traj_points])
    speed = np.array([point['speed'] for point in traj_points])

    # Initialize time, distance, yaw, and curvature
    t = np.zeros(len(traj_points))
    distance = np.zeros(len(traj_points))
    yaw = np.zeros(len(traj_points))
    curvature = np.zeros(len(traj_points))

    # Compute differences
    xdiff = np.diff(x)
    ydiff = np.diff(y)
    gap = np.sqrt(xdiff**2 + ydiff**2)
    gap_vx = np.maximum(speed[:-1] + speed[1:], 1.0) / 2.0
    gap_time = gap / gap_vx

    # Accumulate time and distance
    t[1:] = np.cumsum(gap_time)
    distance[1:] = np.cumsum(gap)

    # Compute yaw
    yaw[1:] = np.arctan2(ydiff, xdiff)
    yaw[0] = yaw[1]

    # Compute curvature using the three-point method
    x0, y0 = x[:-2], y[:-2]
    x1, y1 = x[1:-1], y[1:-1]
    x2, y2 = x[2:], y[2:]
    k1 = 0.5 * (x1**2 + y1**2 - x0**2 - y0**2) / (x1 - x0)
    k2 = (y1 - y0) / (x1 - x0)
    b = 0.5 * (x2**2 + y2**2 - x0**2 - y0**2 - 2 * k1 * (x2 - x0)) / ((y2 - y0) - k2 * (x2 - x0))
    a = k1 - k2 * b
    radius = np.sqrt((x1 - a)**2 + (y1 - b)**2)
    curvature_mid = 1 / radius
    cross_product = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
    curvature_sign = np.sign(cross_product)
    curvature_mid *= curvature_sign

    curvature[1:-1] = curvature_mid
    curvature[0] = curvature[1]
    curvature[-1] = curvature[-2]

    # Assign computed values back to traj_points
    for i, point in enumerate(traj_points):
        point['time'] = t[i]
        point['distance'] = distance[i]
        point['yaw'] = yaw[i]
        point['curvature'] = curvature[i]


    # Vectorized computation for finding the minimum distance
    # num_points_to_check = np.int32(len(traj_points) / 3.0)
    num_points_to_check = np.int32(len(traj_points))
    distances = np.sqrt(x[:num_points_to_check]**2 + y[:num_points_to_check]**2)
    dist_min_idx = np.argmin(distances)


    # traj_t = np.array([point['time'] for point in traj_points])
    traj_s = np.array([point['distance'] for point in traj_points])
    traj_x = np.array([point['x'] for point in traj_points])
    traj_y = np.array([point['y'] for point in traj_points])
    traj_yaw = np.array([point['yaw'] for point in traj_points])
    traj_yaw_unwrapped = np.unwrap(traj_yaw, period=2*np.pi)
    traj_v = np.array([point['speed'] for point in traj_points])
    traj_curvature = np.array([point['curvature'] for point in traj_points])

    if len(traj_points) < 2:
        print("Cannot generate target interpolated trajectory!!")
        return


    sampled_trajectory = np.zeros((N, 7))
    sx = CubicSpline(traj_s, traj_x, extrapolate=True)
    sy = CubicSpline(traj_s, traj_y, extrapolate=True)
    sspeed = CubicSpline(traj_s, traj_v, extrapolate=True)
    # scurvature = CubicSpline(traj_s, traj_curvature, extrapolate=True)
    scurvature = UnivariateSpline(traj_s, traj_curvature, s=0.001, ext=3)
    syaw_unwrapped = CubicSpline(traj_s, traj_yaw_unwrapped, extrapolate=True)

    start_s = traj_s[dist_min_idx] 
    ego_xy = np.array([0.0, 0.0])
    res = minimize(distance_squared, x0=start_s, args=(ego_xy, sx, sy), bounds=[(start_s-1.0, start_s+1.0)])
    s_min_dist = res.x[0]
    n_min_dist = math.sqrt(res.fun)

    sx_tangent = sx.derivative()(s_min_dist)
    sy_tangent = sy.derivative()(s_min_dist)
    tangent_vector = np.array([sx_tangent, sy_tangent])
    normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])
    ego_to_path_vector = np.array([sx(s_min_dist) - ego_xy[0], sy(s_min_dist) - ego_xy[1]])
    dot_product = np.dot(normal_vector, ego_to_path_vector)
    if dot_product > 0:
        n_min_dist = -n_min_dist
    else:
        n_min_dist = n_min_dist
    
    # Vectorized sampling
    s_values = s_min_dist + np.arange(N) * ds
    x_values = sx(s_values)
    y_values = sy(s_values)
    yaw_values = syaw_unwrapped(s_values)
    yaw_values = np.mod(yaw_values + np.pi, 2 * np.pi) - np.pi
    speed_values = sspeed(s_values)
    curvature_values = scurvature(s_values)
    t_values = np.zeros(N)

    sampled_trajectory[:, 0] = t_values
    sampled_trajectory[:, 1] = x_values
    sampled_trajectory[:, 2] = y_values
    sampled_trajectory[:, 3] = yaw_values
    sampled_trajectory[:, 4] = speed_values
    sampled_trajectory[:, 5] = curvature_values
    sampled_trajectory[:, 6] = s_values - s_min_dist

    return n_min_dist, sampled_trajectory


def get_ref_trajectory_dt(traj_points, N, dt):
    # Convert to NumPy arrays for vectorized operations
    x = np.array([point['x'] for point in traj_points])
    y = np.array([point['y'] for point in traj_points])
    speed = np.array([point['speed'] for point in traj_points])
    

    # Initialize time, distance, yaw, and curvature
    t = np.zeros(len(traj_points))
    distance = np.zeros(len(traj_points))
    yaw = np.zeros(len(traj_points))
    curvature = np.zeros(len(traj_points))

    # Compute differences
    xdiff = np.diff(x)
    ydiff = np.diff(y)
    gap = np.sqrt(xdiff**2 + ydiff**2)
    gap_vx = np.maximum(speed[:-1] + speed[1:], 1.0) / 2.0
    gap_time = gap / gap_vx

    # Accumulate time and distance
    t[1:] = np.cumsum(gap_time)
    distance[1:] = np.cumsum(gap)

    # Compute yaw
    yaw[1:] = np.arctan2(ydiff, xdiff)
    yaw[0] = yaw[1]

    # Compute curvature using the three-point method
    x0, y0 = x[:-2], y[:-2]
    x1, y1 = x[1:-1], y[1:-1]
    x2, y2 = x[2:], y[2:]
    k1 = 0.5 * (x1**2 + y1**2 - x0**2 - y0**2) / (x1 - x0)
    k2 = (y1 - y0) / (x1 - x0)
    b = 0.5 * (x2**2 + y2**2 - x0**2 - y0**2 - 2 * k1 * (x2 - x0)) / ((y2 - y0) - k2 * (x2 - x0))
    a = k1 - k2 * b
    radius = np.sqrt((x1 - a)**2 + (y1 - b)**2)
    curvature_mid = 1 / radius
    cross_product = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
    curvature_sign = np.sign(cross_product)
    curvature_mid *= curvature_sign

    curvature[1:-1] = curvature_mid
    curvature[0] = curvature[1]
    curvature[-1] = curvature[-2]

    # Assign computed values back to traj_points
    for i, point in enumerate(traj_points):
        point['time'] = t[i]
        point['distance'] = distance[i]
        point['yaw'] = yaw[i]
        point['curvature'] = curvature[i]


    # Vectorized computation for finding the minimum distance
    num_points_to_check = np.int32(len(traj_points) / 1.5)
    distances = np.sqrt(x[:num_points_to_check]**2 + y[:num_points_to_check]**2)
    dist_min_idx = np.argmin(distances)

    traj_t = np.array([point['time'] for point in traj_points])
    # traj_s = np.array([point['distance'] for point in traj_points])
    traj_x = np.array([point['x'] for point in traj_points])
    traj_y = np.array([point['y'] for point in traj_points])
    traj_yaw = np.array([point['yaw'] for point in traj_points])
    traj_yaw_unwrapped = np.unwrap(traj_yaw, period=2*np.pi)
    traj_v = np.array([point['speed'] for point in traj_points])
    traj_curvature = np.array([point['curvature'] for point in traj_points])

    if len(traj_points) < 2:
        print("Cannot generate target interpolated trajectory!!")
        return


    sampled_trajectory = np.zeros((N, 7))
    tx = CubicSpline(traj_t, traj_x, extrapolate=True)
    ty = CubicSpline(traj_t, traj_y, extrapolate=True)
    tspeed = CubicSpline(traj_t, traj_v, extrapolate=True)
    tcurvature = CubicSpline(traj_t, traj_curvature, extrapolate=True)
    # tcurvature = UnivariateSpline(traj_t, traj_curvature, s=0.001, ext=3)
    tyaw_unwrapped = CubicSpline(traj_t, traj_yaw_unwrapped, extrapolate=True)

    start_t = traj_t[dist_min_idx]
    ego_xy = np.array([0.0, 0.0])
    res = minimize(distance_squared, x0=start_t, args=(ego_xy, tx, ty), bounds=[(start_t-0.2, start_t+0.2)])
    t_min_dist = res.x[0]
    n_min_dist = math.sqrt(res.fun)

    tx_tangent = tx.derivative()(t_min_dist)
    ty_tangent = ty.derivative()(t_min_dist)
    tangent_vector = np.array([tx_tangent, ty_tangent])
    normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])
    ego_to_path_vector = np.array([tx(t_min_dist) - ego_xy[0], ty(t_min_dist) - ego_xy[1]])
    dot_product = np.dot(normal_vector, ego_to_path_vector)
    if dot_product > 0:
        n_min_dist = -n_min_dist
    else:
        n_min_dist = n_min_dist
    
    # Vectorized sampling
    t_values = t_min_dist +  np.arange(N) * dt
    x_values = tx(t_values)
    y_values = ty(t_values)
    yaw_values = tyaw_unwrapped(t_values)
    yaw_values = np.mod(yaw_values + np.pi, 2 * np.pi) - np.pi
    speed_values = tspeed(t_values)
    curvature_values = tcurvature(t_values)
    s_values = np.zeros(N)

    sampled_trajectory[:, 0] = t_values - t_min_dist
    sampled_trajectory[:, 1] = x_values
    sampled_trajectory[:, 2] = y_values
    sampled_trajectory[:, 3] = yaw_values
    sampled_trajectory[:, 4] = speed_values
    sampled_trajectory[:, 5] = curvature_values
    sampled_trajectory[:, 6] = s_values

    return n_min_dist, sampled_trajectory



def transform_traj_to_ego(vehicle_state, trajectory):
    world_trajectory = np.empty((len(trajectory.point), 3))
    for i in range(len(trajectory.point)):
        world_trajectory[i, 0] = trajectory.point[i].x
        world_trajectory[i, 1] = trajectory.point[i].y
        world_trajectory[i, 2] = trajectory.point[i].yaw

    rotation_mat = np.array([
        [np.cos(vehicle_state.yaw), np.sin(vehicle_state.yaw), -vehicle_state.x*np.cos(vehicle_state.yaw) - vehicle_state.y*np.sin(vehicle_state.yaw)],
        [-np.sin(vehicle_state.yaw), np.cos(vehicle_state.yaw), vehicle_state.x*np.sin(vehicle_state.yaw) - vehicle_state.y*np.cos(vehicle_state.yaw)],
        [0.0, 0.0, 1.0]
    ])

    ones = np.ones((world_trajectory.shape[0], 1))
    homogeneous_path = np.hstack((world_trajectory[:, :2], ones))
    ego_xy = homogeneous_path.dot(rotation_mat.T)
    ego_yaw = (world_trajectory[:, 2] - vehicle_state.yaw) % (2*np.pi)
    ego_yaw = (ego_yaw + np.pi) % (2 * np.pi) - np.pi

    # 연속된 중복을 확인하는 마스크 생성
    mask = np.concatenate(([True], (np.diff(ego_xy[:, 0]) != 0) | (np.diff(ego_xy[:, 1]) != 0)))

    ego_traj = []
    for i in range(len(trajectory.point)):
        if(mask[i]==True):
            point = {
                'time': 0.0,
                'x': ego_xy[i, 0],
                'y': ego_xy[i, 1],
                'z': trajectory.point[i].z,
                'yaw': ego_yaw[i],
                'curvature': trajectory.point[i].curvature,
                'distance': 0.0,
                'speed': trajectory.point[i].speed,
                'acceleration': 0.0
            }
            ego_traj.append(point)

    return ego_traj



def transform_to_ego(vehicle_state, target_state):
    rotation_mat = np.array([
        [np.cos(vehicle_state.yaw), np.sin(vehicle_state.yaw), -vehicle_state.x*np.cos(vehicle_state.yaw) - vehicle_state.y*np.sin(vehicle_state.yaw)],
        [-np.sin(vehicle_state.yaw), np.cos(vehicle_state.yaw), vehicle_state.x*np.sin(vehicle_state.yaw) - vehicle_state.y*np.cos(vehicle_state.yaw)],
        [0.0, 0.0, 1.0]
    ])

    target_homogeneous = np.array([target_state.x, target_state.y, 1])

    ego_coordinates = target_homogeneous.dot(rotation_mat.T)
    ego_x = ego_coordinates[0]
    ego_y = ego_coordinates[1]
    ego_yaw = (target_state.yaw - vehicle_state.yaw) % (2 * np.pi)
    ego_yaw = (ego_yaw + np.pi) % (2 * np.pi) - np.pi

    ego_state = np.array([ego_x, ego_y, ego_yaw]) 

    return ego_state



def transform_boundary_to_ego(vehicle_state, trajectory):
    lb_world = np.empty((len(trajectory.point), 2))
    rb_world = np.empty((len(trajectory.point), 2))

    for i in range(np.shape(lb_world)[0]):
        lb_world[i, 0] = trajectory.point[i].lb_point.x
        lb_world[i, 1] = trajectory.point[i].rb_point.y
        
    for i in range(np.shape(rb_world)[0]):
        rb_world[i, 0] = trajectory.point[i].rb_point.x
        rb_world[i, 1] = trajectory.point[i].rb_point.y

    rotation_mat = np.array([
        [np.cos(vehicle_state.yaw), np.sin(vehicle_state.yaw), -vehicle_state.x * np.cos(vehicle_state.yaw) - vehicle_state.y * np.sin(vehicle_state.yaw)],
        [-np.sin(vehicle_state.yaw), np.cos(vehicle_state.yaw), vehicle_state.x * np.sin(vehicle_state.yaw) - vehicle_state.y * np.cos(vehicle_state.yaw)],
        [0.0, 0.0, 1.0]
    ])

    # Convert left boundary to homogeneous coordinates and transform
    lb_homogeneous = np.hstack((lb_world, np.ones((lb_world.shape[0], 1))))
    lb_ego = lb_homogeneous.dot(rotation_mat.T)[:, :2]

    # Convert right boundary to homogeneous coordinates and transform
    rb_homogeneous = np.hstack((rb_world, np.ones((rb_world.shape[0], 1))))
    rb_ego = rb_homogeneous.dot(rotation_mat.T)[:, :2]


    # Resample and smooth boundaries at 1m intervals
    lb_ego = resample_and_smooth(lb_ego)
    rb_ego = resample_and_smooth(rb_ego)

    return [lb_ego, rb_ego]


def transform_object_to_ego(vehicle_state, objects):
    # 회전 행렬 생성
    rotation_mat = np.array([
        [np.cos(vehicle_state.yaw), np.sin(vehicle_state.yaw), -vehicle_state.x*np.cos(vehicle_state.yaw) - vehicle_state.y*np.sin(vehicle_state.yaw)],
        [-np.sin(vehicle_state.yaw), np.cos(vehicle_state.yaw), vehicle_state.x*np.sin(vehicle_state.yaw) - vehicle_state.y*np.cos(vehicle_state.yaw)],
        [0.0, 0.0, 1.0]
    ])

    # 변환할 objects의 shape
    num_objects, num_timesteps, _ = objects.shape

    # homogeneous coordinates 로 확장 (x, y) -> (x, y, 1)
    ones = np.ones((num_objects, num_timesteps, 1))
    hom_objects = np.concatenate((objects, ones), axis=2)
    
    # 자차 프레임으로 변환
    transformed_objects = np.einsum('ij,tkj->tki', rotation_mat, hom_objects)[:, :, :2]
    
    return transformed_objects

def resample_and_smooth(points):
    # Calculate cumulative distance along the points
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))
    cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

    # New cumulative distances for resampling at 0.5m intervals
    new_cumulative_distances = np.arange(0, cumulative_distances[-1], 0.5)

    # Interpolation functions
    interp_func_x = interp1d(cumulative_distances, points[:, 0], kind='linear')
    interp_func_y = interp1d(cumulative_distances, points[:, 1], kind='linear')

    # Resample and smooth
    resampled_x = interp_func_x(new_cumulative_distances)
    resampled_y = interp_func_y(new_cumulative_distances)

    resampled_points = np.vstack((resampled_x, resampled_y)).T
    
    # Calculate yaw values
    yaw = np.arctan2(np.diff(resampled_y), np.diff(resampled_x))
    yaw = np.append(yaw, yaw[-1])  # Append the last yaw value to match the length of resampled_points

    resampled_points_with_yaw = np.hstack((resampled_points, yaw[:, np.newaxis]))
    return resampled_points_with_yaw
