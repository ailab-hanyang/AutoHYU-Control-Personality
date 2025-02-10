import numpy as np
from scipy.interpolate import CubicSpline
# from scipy.interpolate import Akima1DInterpolator
from scipy.interpolate import UnivariateSpline
from scipy.optimize import minimize
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
    num_points_to_check = np.int32(len(traj_points) / 3.0)
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
    scurvature = CubicSpline(traj_s, traj_curvature, extrapolate=True)
    # scurvature = UnivariateSpline(traj_s, traj_curvature, s=0.005, ext=3)
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
    num_points_to_check = np.int32(len(traj_points) / 3.0)
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

    sx_tangent = tx.derivative()(t_min_dist)
    sy_tangent = ty.derivative()(t_min_dist)
    tangent_vector = np.array([sx_tangent, sy_tangent])
    normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])
    ego_to_path_vector = np.array([tx(t_min_dist) - ego_xy[0], ty(t_min_dist) - ego_xy[1]])
    dot_product = np.dot(normal_vector, ego_to_path_vector)
    if dot_product > 0:
        n_min_dist = -n_min_dist
    else:
        n_min_dist = n_min_dist
    
    # Vectorized sampling
    t_values = t_min_dist + np.arange(N) * dt
    x_values = tx(t_values)
    y_values = ty(t_values)
    yaw_values = tyaw_unwrapped(t_values)
    yaw_values = np.mod(yaw_values + np.pi, 2 * np.pi) - np.pi
    speed_values = tspeed(t_values)
    curvature_values = tcurvature(t_values)
    s_values = np.zeros(N)

    sampled_trajectory[:, 0] = t_values
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

    ego_traj = []
    for i in range(len(trajectory.point)):
        point = {
            'time': trajectory.point[i].time,
            'x': ego_xy[i, 0],
            'y': ego_xy[i, 1],
            'z': trajectory.point[i].z,
            'yaw': ego_yaw[i],
            'curvature': trajectory.point[i].curvature,
            'distance': trajectory.point[i].distance,
            'speed': trajectory.point[i].speed,
            'acceleration': trajectory.point[i].acceleration
        }
        ego_traj.append(point)

    return ego_traj


