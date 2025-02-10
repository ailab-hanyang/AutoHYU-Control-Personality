import numpy as np
import math
from scipy.interpolate import interp1d
import casadi as cs
from scipy.spatial import cKDTree

   
def postprocess_yaw(yaw):
    if isinstance(yaw, (list, np.ndarray)):
        yaw = np.fmod(yaw + np.pi, 2*np.pi) - np.pi
        return yaw
    else:
        yaw = math.fmod(yaw + math.pi, 2*math.pi) - math.pi
        return yaw

    
def LonLatDeviations(ego_yaw, ego_x, ego_y, ref_x,ref_y):
    '''
    This method is based on rotating the deviation vectors by the negative 
    of the yaw angle of the vehicle, which aligns the deviation vectors with 
    the longitudinal and lateral axes of the vehicle.
    '''
    rotcos      = np.cos(-ego_yaw)
    rotsin      = np.sin(-ego_yaw)
    dev_long    = rotcos * (ref_x - ego_x) - rotsin * (ref_y - ego_y)
    dev_lat     = rotsin * (ref_x - ego_x) + rotcos * (ref_y - ego_y)
    return dev_long, dev_lat



def frenet_to_cartesian(X_ref, pred_X):
    
    s_pred  = pred_X[:,0]
    n_pred  = pred_X[:,1]
    mu_pred = pred_X[:,2]

    kappa_interp = cs.interpolant("kappa", "linear", [X_ref[:,6]], X_ref[:,5])
    kappa = np.array(kappa_interp(s_pred)).flatten()

    s_ref = X_ref[:, 6]
    x_ref = X_ref[:, 1]
    y_ref = X_ref[:, 2]
    yaw_ref = X_ref[:, 3]

    # 보간 함수 생성
    interp_x = interp1d(s_ref, x_ref, kind='linear', fill_value="extrapolate")
    interp_y = interp1d(s_ref, y_ref, kind='linear', fill_value="extrapolate")
    interp_yaw = interp1d(s_ref, yaw_ref, kind='linear', fill_value="extrapolate")

    # 보간을 통해 참조 좌표에서 s_pred에 해당하는 좌표와 방향 구하기
    x_ref_pred = interp_x(s_pred)
    y_ref_pred = interp_y(s_pred)
    yaw_ref_pred = interp_yaw(s_pred)

    # Frenet to Cartesian 변환 수행
    x = x_ref_pred + n_pred * np.cos(yaw_ref_pred + np.pi / 2)
    y = y_ref_pred + n_pred * np.sin(yaw_ref_pred + np.pi / 2)
    yaw = yaw_ref_pred + mu_pred
    yaw_deg = np.rad2deg(yaw_ref_pred)
    # 경로의 방향을 수치 적분을 통해 계산
    return np.vstack((x,y,yaw)), kappa


def find_closest_points(pred_X, lb_ego, rb_ego):
    # Build KD-trees for fast nearest-neighbor lookup
    lb_tree = cKDTree(lb_ego[:,0:2])
    rb_tree = cKDTree(rb_ego[:,0:2])
    
    # Find the nearest points for each point in pred_X
    lb_dists, lb_indices = lb_tree.query(pred_X[:,0:2])
    rb_dists, rb_indices = rb_tree.query(pred_X[:,0:2])
    
    lb_closest_points = lb_ego[lb_indices]
    rb_closest_points = rb_ego[rb_indices]
    
    return lb_closest_points, rb_closest_points