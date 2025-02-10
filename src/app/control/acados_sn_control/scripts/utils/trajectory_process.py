import numpy as np
from scipy.interpolate import CubicSpline
from scipy.interpolate import Akima1DInterpolator
from scipy.interpolate import UnivariateSpline
from scipy.optimize import minimize
from scipy.interpolate import interp1d
import math
import time
from acados_sn_control import libfrenet3 as frenet


# Distance squared function for optimization
def distance_squared(t, xy, spline_x, spline_y):
    tx = np.array([t, spline_x(t)])
    ty = np.array([t, spline_y(t)])
    return (tx[1] - xy[0])**2 + (ty[1] - xy[1])**2

def get_ref_trajectory_dt(traj_points, N, dt):
    # Convert to NumPy arrays for vectorized operations
    traj_x = np.array([point['x'] for point in traj_points])
    traj_y = np.array([point['y'] for point in traj_points])
    traj_speed = np.array([point['speed'] for point in traj_points])
  
    # Initialize time, distance, yaw, and curvature
    t = np.zeros(len(traj_points))
    distance = np.zeros(len(traj_points))
    yaw = np.zeros(len(traj_points))
    curvature = np.zeros(len(traj_points))

    # Compute differences
    xdiff = np.diff(traj_x)
    ydiff = np.diff(traj_y)
    gap = np.sqrt(xdiff**2 + ydiff**2)
    gap_vx = np.maximum(traj_speed[:-1] + traj_speed[1:], 1.0) / 2.0
    gap_time = gap / gap_vx

    # Accumulate time and distance
    t[1:] = np.cumsum(gap_time)
    distance[1:] = np.cumsum(gap)

    # Compute yaw
    yaw[1:] = np.arctan2(ydiff, xdiff)
    yaw[0] = yaw[1]

    # Compute curvature using the three-point method
    x0, y0 = traj_x[:-2], traj_y[:-2]
    x1, y1 = traj_x[1:-1], traj_y[1:-1]
    x2, y2 = traj_x[2:], traj_y[2:]
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

    # traj_t = np.array([point['time'] for point in traj_points])
    traj_s = np.array([point['distance'] for point in traj_points])
    traj_x = np.array([point['x'] for point in traj_points])
    traj_y = np.array([point['y'] for point in traj_points])
    traj_yaw = np.array([point['yaw'] for point in traj_points])
    traj_yaw_unwrapped = np.unwrap(traj_yaw, period=2*np.pi)
    traj_v = np.array([point['speed'] for point in traj_points])
    traj_curvature = np.array([point['curvature'] for point in traj_points])

    # Vectorized computation for finding the minimum distance
    num_points_to_check = np.int32(len(traj_points) / 3.0)
    # num_points_to_check = np.int32(len(traj_points))
    distances = np.sqrt(traj_x[:num_points_to_check]**2 + traj_y[:num_points_to_check]**2)
    dist_min_idx = np.argmin(distances)

    if len(traj_points) < 2:
        print("Cannot generate target interpolated trajectory!!")
        return

    sampled_trajectory = np.zeros((N, 7))

    # 시간 기반 보간을 위한 스플라인 생성
    sx = Akima1DInterpolator(t, traj_x)
    sy = Akima1DInterpolator(t, traj_y)
    sspeed = Akima1DInterpolator(t, traj_v)
    scurvature = Akima1DInterpolator(t, traj_curvature)
    syaw_unwrapped = Akima1DInterpolator(t, traj_yaw_unwrapped)
    
    # 외삽 허용
    sx.extrapolate = True
    sy.extrapolate = True
    sspeed.extrapolate = True
    scurvature.extrapolate = True
    syaw_unwrapped.extrapolate = True

    # 현재 위치에서 가장 가까운 시간점 찾기
    start_t = t[dist_min_idx]
    ego_xy = np.array([0.0, 0.0])
    res = minimize(distance_squared, x0=start_t, args=(ego_xy, sx, sy), bounds=[(start_t-1.0, start_t+1.0)])
    t_min_dist = res.x[0]
    n_min_dist = math.sqrt(res.fun)

    # 법선 벡터 계산
    sx_tangent = sx.derivative()(t_min_dist)
    sy_tangent = sy.derivative()(t_min_dist)
    tangent_vector = np.array([sx_tangent, sy_tangent])
    normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])
    ego_to_path_vector = np.array([sx(t_min_dist) - ego_xy[0], sy(t_min_dist) - ego_xy[1]])
    dot_product = np.dot(normal_vector, ego_to_path_vector)
    n_min_dist = -n_min_dist if dot_product > 0 else n_min_dist
    
    # 시간 기반 샘플링
    t_values = t_min_dist + np.arange(N) * dt
    x_values = sx(t_values)
    y_values = sy(t_values)
    yaw_values = syaw_unwrapped(t_values)
    yaw_values = np.mod(yaw_values + np.pi, 2 * np.pi) - np.pi
    speed_values = sspeed(t_values)
    curvature_values = scurvature(t_values)
    # 결과 저장
    sampled_trajectory[:, 0] = t_values - t_min_dist  # 상대 시간으로 저장
    sampled_trajectory[:, 1] = x_values
    sampled_trajectory[:, 2] = y_values
    sampled_trajectory[:, 3] = yaw_values
    sampled_trajectory[:, 4] = speed_values
    sampled_trajectory[:, 5] = curvature_values
    
    # 누적 거리 계산
    dx = np.diff(x_values)
    dy = np.diff(y_values)
    distances = np.concatenate(([0], np.cumsum(np.sqrt(dx**2 + dy**2))))
    sampled_trajectory[:, 6] = distances

    # ... rest of the code with RoadMap ...

    return n_min_dist, sampled_trajectory




def transform_traj_to_ego(vehicle_state, trajectory):
    world_trajectory = np.empty((len(trajectory.point), 7))
    for i in range(len(trajectory.point)):
        world_trajectory[i, 0] = trajectory.point[i].x
        world_trajectory[i, 1] = trajectory.point[i].y
        world_trajectory[i, 2] = trajectory.point[i].yaw


    rotation_mat = np.array([
        [np.cos(vehicle_state.yaw), np.sin(vehicle_state.yaw), -vehicle_state.x * np.cos(vehicle_state.yaw) - vehicle_state.y * np.sin(vehicle_state.yaw)],
        [-np.sin(vehicle_state.yaw), np.cos(vehicle_state.yaw), vehicle_state.x * np.sin(vehicle_state.yaw) - vehicle_state.y * np.cos(vehicle_state.yaw)],
        [0.0, 0.0, 1.0]
    ])

    ones = np.ones((world_trajectory.shape[0], 1))

    # Transform main x, y
    homogeneous_path_xy = np.hstack((world_trajectory[:, :2], ones))
    ego_xy = homogeneous_path_xy.dot(rotation_mat.T)

    # Transform yaw
    ego_yaw = (world_trajectory[:, 2] - vehicle_state.yaw) % (2 * np.pi)
    ego_yaw = (ego_yaw + np.pi) % (2 * np.pi) - np.pi

    # Create mask to check for consecutive duplicates
    mask = np.concatenate(([True], (np.diff(ego_xy[:, 0]) != 0) | (np.diff(ego_xy[:, 1]) != 0)))

    ego_traj = []
    for i in range(len(trajectory.point)):
        if mask[i]:
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


def transform_object_to_ego(vehicle_state, objects):
    vehicle_x = vehicle_state.x
    vehicle_y = vehicle_state.y
    vehicle_yaw = vehicle_state.yaw

    cos_yaw = np.cos(vehicle_yaw)
    sin_yaw = np.sin(vehicle_yaw)

    # Rotation matrix to transform coordinates to ego frame
    R = np.array([
        [cos_yaw, sin_yaw],
        [-sin_yaw, cos_yaw]
    ])

    all_data = []

    for obj in objects.object:
        obj_id = obj.id
        length = obj.dimension.length
        width = obj.dimension.width
        height = obj.dimension.height

        # Extract positions and orientations into NumPy arrays
        positions = np.array([[state.x, state.y] for state in obj.state])  # Shape: (num_states, 2)
        yaws = np.array([state.yaw for state in obj.state])                # Shape: (num_states,)

        # Translate positions to the ego frame
        translated_positions = positions - np.array([vehicle_x, vehicle_y])

        # Rotate positions to align with the ego vehicle's orientation
        transformed_positions = translated_positions @ R.T  # Shape: (num_states, 2)

        # Adjust orientations to the ego frame
        transformed_yaws = yaws - vehicle_yaw               # Shape: (num_states,)

        num_states = transformed_positions.shape[0]

        # Combine all relevant data into a single NumPy array for this object
        # Each row represents a state at a future time step
        # Columns: [object_id, length, width, height, x, y, yaw]
        obj_data = np.column_stack((
            np.full(num_states, obj_id),          # Object ID
            np.full(num_states, length),          # Object length
            np.full(num_states, width),           # Object width
            np.full(num_states, height),          # Object height
            transformed_positions,                # Transformed x and y positions
            transformed_yaws                      # Transformed yaw angles
        ))

        all_data.append(obj_data)

    # Concatenate data from all objects into a single NumPy array
    transformed_objects = np.array(all_data)
    # The resulting transformed_objects array has the following structure:
    #
    # Shape: (total_num_states, 7)
    #
    # Each row corresponds to one predicted state of an object at a future time step.
    # Columns:
    # [0] Object ID          (float)
    # [1] Object length      (float)
    # [2] Object width       (float)
    # [3] Object height      (float)
    # [4] Transformed x      (float)
    # [5] Transformed y      (float)
    # [6] Transformed yaw    (float)
    return transformed_objects
