import open3d as o3d
import numpy as np
import math
import time

def create_point_cloud(num_points=1000):
    points = np.random.rand(num_points, 3)  # ポイントをランダムに生成
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def create_grid(size=10, n=10, color=[0, 0, 0]):
    lines = []
    colors = []
    points = []
    for i in range(n + 1):
        for j in range(n + 1):
            points.append([i * size / n - size / 2, j * size / n - size / 2, 0])
            if i < n:
                lines.append([i * (n + 1) + j, (i + 1) * (n + 1) + j])
                colors.append(color)
            if j < n:
                lines.append([i * (n + 1) + j, i * (n + 1) + (j + 1)])
                colors.append(color)
    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set

def create_coordinate_frame(size=0.1, origin=[0, 0, 0]):
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=origin)

def calculate_distances_from_point(points, reference_point):
    # reference_point から points までの距離を計算
    distances = np.sqrt(np.sum((points - reference_point) ** 2, axis=1))
    for i, dist in enumerate(distances):
        if i < 10:
            print(f"点{i}: 座標{points[i]}, 座標({reference_point[0]}, {reference_point[1]}, {reference_point[2]})からの距離{dist:.2f}")

# ポイントクラウド、グリッド、座標軸を作成
pcd = create_point_cloud()
grid = create_grid(size=1, n=10, color=[0.5, 0.5, 0.5])
coordinate_frame = create_coordinate_frame()

# 可視化
o3d.visualization.draw_geometries([pcd, grid, coordinate_frame])

reference_point = np.array([0, 0, 0])
calculate_distances_from_point(np.asarray(pcd.points), reference_point)
reference_point = np.array([50, 50, 50])
calculate_distances_from_point(np.asarray(pcd.points), reference_point)