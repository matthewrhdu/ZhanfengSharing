import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

for i in range(6):
    if i == 3:
        pass
    else:
        data = o3d.io.read_point_cloud(f'images/img{i}.ply')

        data.paint_uniform_color((0, 0, 1))
        data.estimate_normals()
        o3d.visualization.draw_geometries([data])
