import numpy as np
import matplotlib.pyplot as plt

class ValidationPlot(object):
    def __init__(self, model_name, gt_trajectory, predict_trajectory, input_force, input_steer, params):
        self.model_name = model_name  # Store model_name

        gt_x = [item[0] for item in gt_trajectory]
        gt_y = [item[1] for item in gt_trajectory]
        dr_x = [item[0] for item in predict_trajectory]
        dr_y = [item[1] for item in predict_trajectory]
        dr_yawrate = [item[2] for item in predict_trajectory]
        dist_list = np.zeros(len(dr_x), dtype=float)

        plt.figure(figsize=(20, 9))
        plt.suptitle(self.model_name)  # Add title using model_name
        
        ## subplot 1 : trajectory visualization
        plt.subplot(2, 3, 1)
        min_x, max_x = min(min(gt_x), min(dr_x)), max(max(gt_x), max(dr_x))
        min_y, max_y = min(min(gt_y), min(dr_y)), max(max(gt_y), max(dr_y))
        range_x = max_x - min_x
        range_y = max_y - min_y
        max_range = max(range_x, range_y)
        mid_x = (max_x + min_x) / 2
        mid_y = (max_y + min_y) / 2
        x_lim = (mid_x - max_range / 2 - 20, mid_x + max_range / 2 + 20)
        y_lim = (mid_y - max_range / 2 - 20, mid_y + max_range / 2 + 20)
        plt.xlim(x_lim)
        plt.ylim(y_lim)

        plt.plot(dr_x, dr_y, marker='o', color='b', markersize=2, label='Validation')
        plt.plot(gt_x, gt_y, marker='o', color='black', markersize=1, label='GT')
        plt.plot(dr_x[0], dr_y[0], marker='o', markersize=5, color='r') 

        plt.xlabel('Pos X')
        plt.ylabel('Pos Y')
        plt.title('Prediction Trajectory')
        plt.legend()
        plt.grid(True)

        ## subplot 2 : predicted yawrate
        plt.subplot(2, 3, 2)
        time = np.arange(0, len(dr_yawrate)*params['dt'], params['dt'])
        time = time[:len(dr_yawrate)]
        dr_yawrate = [x * (180.0/np.pi) for x in dr_yawrate]
        plt.plot(time, dr_yawrate, marker='.', linestyle='-', markersize=0.1, color='b', label='deg/s')
        plt.title('Predicted Yaw Rate')
        plt.legend()
        plt.grid(True)

        ## subplot 3 : Euclidian Distance Error
        plt.subplot(2, 3, 3)
        for i in range(len(dr_x)):
            dist_list[i] = ((dr_x[i] - gt_x[i])**2 + (dr_y[i] - gt_y[i])**2)**0.5
        time = np.arange(0, len(dr_x)*params['dt'], params['dt'])
        time = time[:len(dr_x)]
        dist_rms = np.sqrt(np.mean(dist_list**2))
        plt.text(0.01, 0.85, f'RMSE: {dist_rms:.3f}', ha='left', va='top', transform=plt.gca().transAxes,
                 fontsize=10, color='black')
        plt.plot(time, dist_list, marker='.', linestyle='-', markersize=0.1, color='r', label='m')
        plt.title('Euclidian Distance Error')
        plt.legend()
        plt.grid(True)

        ## subplot 4 : input torque
        plt.subplot(2, 3, 4)
        time = np.arange(0, len(input_force)*params['dt'], params['dt'])
        time = time[:len(input_force)]
        plt.plot(time, input_force, marker='.', linestyle='-', markersize=0.1, color='black', label='Nm')
        plt.title('Model Input Force')
        plt.legend()
        plt.grid(True)

        ## subplot 5 : input steer
        plt.subplot(2, 3, 5)
        time = np.arange(0, len(input_steer)*params['dt'], params['dt'])
        time = time[:len(input_steer)]
        plt.plot(time, input_steer, marker='.', linestyle='-', markersize=0.1, color='black', label='deg')
        plt.title('Model Input Steer')
        plt.legend()
        plt.grid(True)
        
        ## write distance
        total_dist = 0.0
        for i in range(1, len(gt_x)):
            dist = ((gt_x[i] - gt_x[i-1])**2 + (gt_y[i] - gt_y[i-1])**2)**0.5
            total_dist += dist
        
        plt.figtext(0.1, 0.95, f'Total Distance: {total_dist:.3f}', ha='left', va='top', fontsize=12)
        plt.subplots_adjust(wspace=0.3, hspace=0.3)
        plt.show()