import pandas as pd
import numpy as np
from bagpy import bagreader
import open3d as o3d

def load_bag(bag_path):
    bag = bagreader(bag_path)
    return bag

def load_scan_df(bag):
    csv_file = bag.message_by_topic('/scan')
    ld = pd.read_csv(csv_file)
    return ld

def load_odom_df(bag):
    csv_file = bag.message_by_topic('/odom_rf2o')
    od = pd.read_csv(csv_file)
    return od

def preprocess_scan(ld):
    cols = [col for col in ld.columns if 'ranges' in col]
    ld['ranges'] = ld[cols].apply(lambda row: np.array(row.dropna().tolist()), axis=1)
    ld['cos_angles'] = ld.apply(lambda row: np.cos(np.arange(row['angle_min'], row['angle_max'], row['angle_increment'])), axis=1)
    ld['sin_angles'] = ld.apply(lambda row: np.sin(np.arange(row['angle_min'], row['angle_max'], row['angle_increment'])), axis=1)
    ld = ld[['Time', 'ranges', 'cos_angles', 'sin_angles']]
    return ld

def compute_points(ld):
    x_coords = np.vstack(ld['ranges'].values) * np.vstack(ld['cos_angles'].values)
    y_coords = np.vstack(ld['ranges'].values) * np.vstack(ld['sin_angles'].values)
    points = np.stack([x_coords, y_coords], axis=2)
    return points

def find_nearest_od_time(ld_time, od):
    result = od[od.Time <= ld_time].sort_values('Time')
    if not result.empty:
        return result.iloc[-1]
    else:
        return od.sort_values('Time').iloc[0]

def enrich_with_odometry(ld, od):
    od_cols = ['Time', 'pose.pose.position.x',
        'pose.pose.position.y', 'pose.pose.position.z',
        'pose.pose.orientation.x', 'pose.pose.orientation.y',
        'pose.pose.orientation.z', 'pose.pose.orientation.w', 'pose.covariance',
        'twist.twist.linear.x', 'twist.twist.linear.y', 'twist.twist.linear.z',
        'twist.twist.angular.x', 'twist.twist.angular.y',
        'twist.twist.angular.z', 'twist.covariance']
    od = od[od_cols]
    ld = ld.copy()
    ld.rename(columns={'Time': 'Time_ld'}, inplace=True)
    ld[od_cols] = 0
    ld[od_cols] = ld['Time_ld'].apply(lambda x: find_nearest_od_time(x, od))
    dt = ld['Time_ld'] - ld['Time']
    ld['pose.pose.position.x'] += dt * ld['twist.twist.linear.x']
    ld['pose.pose.position.y'] += dt * ld['twist.twist.linear.y']
    ld['pose.pose.position.z'] += dt * ld['twist.twist.linear.z']
    ld['pose.pose.orientation.x'] += dt * ld['twist.twist.angular.x']
    ld['pose.pose.orientation.y'] += dt * ld['twist.twist.angular.y']
    ld['pose.pose.orientation.z'] += dt * ld['twist.twist.angular.z']
    ld['pose.pose.orientation.w'] += dt * ld['twist.twist.angular.z']
    return ld

def get_transformation_matrix(odom_msg):
    x, y, z = odom_msg['pose.pose.position.x'], odom_msg['pose.pose.position.y'], odom_msg['pose.pose.position.z']
    qx, qy, qz, qw = odom_msg['pose.pose.orientation.x'], odom_msg['pose.pose.orientation.y'], odom_msg['pose.pose.orientation.z'], odom_msg['pose.pose.orientation.w']
    R = np.array([
        [1-2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1-2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1-2*(qx*qx + qy*qy)]
    ])
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = [x, y, z]
    return T

def compute_global_points(ld, points):
    global_points = []
    for index, row in ld.iterrows():
        T = get_transformation_matrix(row)
        pts_local = np.vstack((points[index, :, 0], points[index, :, 1], np.zeros(points.shape[1]), np.ones(points.shape[1])))
        pts_global = T @ pts_local
        global_points.append(pts_global[:3, :])
    return np.array(global_points)

def visualize_point_cloud(global_points):
    points_3d = global_points.reshape(-1, 3)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    bag_path = './data/rf2o_data.bag'
    bag = load_bag(bag_path)
    ld = load_scan_df(bag)
    od = load_odom_df(bag)
    ld = preprocess_scan(ld)
    points = compute_points(ld)
    ld = enrich_with_odometry(ld, od)
    global_points = compute_global_points(ld, points)
    visualize_point_cloud(global_points)
