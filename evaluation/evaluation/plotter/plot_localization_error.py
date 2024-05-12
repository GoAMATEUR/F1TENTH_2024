import numpy as np
import pandas as pd
import os
from pathlib import Path
import matplotlib.pyplot as plt

EXPERIMENT_TIME = "20240513005435"

def plot(output_file_path, x_data, y_data, x_label, y_label, name):
    fig, ax = plt.subplots()
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)

    ax.plot(x_data, y_data, label=name)

    ax.legend()
    fig.savefig(output_file_path)
    plt.close(fig)

if __name__ == "__main__":
    tmp_dir_path = Path(os.getcwd())
    csv_path = tmp_dir_path / "../datas" / EXPERIMENT_TIME / "localization_evaluation.csv"

    output_dir_path = tmp_dir_path / "evaluation_images"
    os.makedirs(output_dir_path, exist_ok=True)

    data = pd.read_csv(csv_path)

    time = data["timestamp"].astype(float).to_numpy() - data["timestamp"].astype(float).iloc[0]

    # output x error 
    gt_x = data["gt_x"] = data["gt_x"].astype(float).to_numpy()
    est_x = data["est_x"] = data["est_x"].astype(float).to_numpy()
    x_error = gt_x - est_x
    plot(output_dir_path / "x_error.png", time, x_error, "time [s]", "m", "x error")

    # output y error 
    gt_y = data["gt_y"] = data["gt_y"].astype(float).to_numpy()
    est_y = data["est_y"] = data["est_y"].astype(float).to_numpy()
    y_error = gt_y - est_y
    plot(output_dir_path / "y_error.png", time, y_error, "time [s]", "m", "y error")

    # output RMSE 
    rmse = np.sqrt(x_error**2 + y_error**2)
    plot(output_dir_path / "rmse.png", time, rmse, "time [s]", "m", "rmse")

    # output angle error 
    gt_yaw = data["gt_yaw"] = data["gt_yaw"].astype(float).to_numpy()
    est_yaw = data["est_yaw"] = data["est_yaw"].astype(float).to_numpy()
    angle_error = gt_yaw - est_yaw
    plot(output_dir_path / "angle_error.png", time, angle_error, "time [s]", "rad", "angle error")

    # output x and y ground truth
    fig, ax = plt.subplots()
    ax.set_xlabel("time [s]")
    ax.set_ylabel("m")

    ax.plot(time, gt_x, label="gt x")
    ax.plot(time, gt_y, label="gt y")

    ax.legend()
    fig.savefig(output_dir_path / "gt_x_y.png")
    plt.close(fig)

