import open3d as o3d
import numpy as np

def formation3DFromMesh(model_path, number_of_points):
    
    mesh = o3d.io.read_triangle_mesh(model_path)

    if mesh.is_empty():
        raise Exception("Mesh is empty.")
    
    mesh.compute_vertex_normals()
    pcd = mesh.sample_points_uniformly(number_of_points=number_of_points)
    
    # Line vectors
    coords = np.array(pcd.points)
    ones = np.ones((coords.shape[0],1))

    # Building homogeneous line vectors
    coords = np.hstack((coords, ones))
    return coords, mesh, pcd

def formation3DfromMeshZoo(mesh_name):
    pass


def visualizeMesh(mesh):
    if mesh is None:
        raise Exception("Mesh is empty.")
    o3d.visualization.draw_geometries([mesh])

def visualizePointCloud(pcd):

    if pcd is None:
        raise Exception("PointCloud is empty.")
    o3d.visualization.draw_geometries([pcd])
    
def 
    
if __name__ == '__main__':
    coords, mesh, pcd = setFormationFrom3DModel('/home/guisoares/.ros/swarm_models/christ-the-redeemer/Christ the Redeemer.obj', 100)
    visualizePointCloud(pcd)