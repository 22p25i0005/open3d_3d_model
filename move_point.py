import numpy as np
import open3d as o3d


if __name__ == "__main__":

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

    mesh = o3d.io.read_point_cloud("foot.ply")
    # mesh.paint_uniform_color([0.5, 0.5, 0.5])
    xyz_load = np.asarray(mesh.points)
    print(xyz_load)
    print(f"dimension : {xyz_load.shape}")

    print(xyz_load[0][0])

    xyz_load[0][2] = xyz_load[100][2] - 0.009  # [0-1224] [0-2]
    xyz_load[101][2] = xyz_load[101][2] - 0.005
    xyz_load[103][2] = xyz_load[103][2] - 0.005
    xyz_load[900][2] = xyz_load[900][2] - 0.005

    new_pc = o3d.geometry.PointCloud()
    new_pc.points = o3d.utility.Vector3dVector(xyz_load)
    print(type(mesh))

    # radii = [0.005, 0.01, 0.02, 0.04]
    # rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(new_pc, o3d.utility.DoubleVector(radii))
    # o3d.visualization.draw_geometries([new_pc,rec_mesh])


    alpha = 0.030
    print(f"alpha={alpha:.3f}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(new_pc, alpha)
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh,mesh_frame],window_name="open3d", mesh_show_back_face=True)


