import open3d as o3d
import numpy as np

def visualize(mesh):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.run()
    vis.destroy_window()

def main():
    mesh = o3d.io.read_point_cloud("foot.ply")
    mesh.paint_uniform_color([0.5, 0.5, 0.5])
    print(type(mesh.points))
    #print(np.asarray(mesh.points))
    visualize(mesh)


main()