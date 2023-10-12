from nricp_lau import nonrigidIcp

import numpy as np
import open3d as o3d
import copy
import pyvista as pv
from icp import icp,draw_registration_result


def write_ply_file_NICP(mesh, reg, savepath):
    points = reg
    faces = mesh.faces
    numVertices = len(reg)
    numFaces = mesh.n_faces
    faces = faces.reshape((numFaces, 4))
    with open(savepath, 'w+') as fileOut:
        # Writes ply header
        fileOut.write("ply\n")
        fileOut.write("format ascii 1.0\n")
        fileOut.write("comment VCGLIB generated\n")
        fileOut.write("element vertex " + str(numVertices) + "\n")
        fileOut.write("property float x\n")
        fileOut.write("property float y\n")
        fileOut.write("property float z\n")

        fileOut.write("element face " + str(numFaces) + "\n")
        fileOut.write("property list uchar int vertex_indices\n")
        fileOut.write("end_header\n")

        for i in range(numVertices):
            # writes "x y z" of current vertex
            fileOut.write(str(points[i, 0]) + " " + str(points[i, 1]) + " " + str(points[i, 2]) + "255\n")

        # Writes faces
        for f in faces:
            # print(f)
            # WARNING: Subtracts 1 to vertex ID because PLY indices start at 0 and OBJ at 1
            fileOut.write("3 " + str(f[1]) + " " + str(f[2]) + " " + str(f[3]) + "\n")


#read source file
    
sourcemesh = o3d.io.read_triangle_mesh("template_or_source_mesh.ply")
targetmesh = o3d.io.read_triangle_mesh("target_mesh.ply")
sourcemesh.compute_vertex_normals()
targetmesh.compute_vertex_normals()






#first find rigid registration
# guess for inital transform for icp
initial_guess = np.eye(4)
affine_transform = icp(sourcemesh,targetmesh,initial_guess)
print(affine_transform)

#creating a new mesh for non rigid transform estimation 
refined_sourcemesh = copy.deepcopy(sourcemesh)
refined_sourcemesh.transform(affine_transform)
refined_sourcemesh.compute_vertex_normals()

landmark_list_source = [4518-1, 4915-1, 4955-1, 4805-1,1908-1, 2346-1, 2478-1] #index for landmarks
landmark_list_target = [7045-1,26393-1,22717-1,22533-1,11501-1,1700-1,2678-1]

print(landmark_list_source)
print(landmark_list_target)

#non rigid registration
deformed_mesh = nonrigidIcp(refined_sourcemesh,targetmesh,landmark_list_source, landmark_list_target, landmark_flag=1)



sourcemesh.paint_uniform_color([0.1, 0.9, 0.1])
targetmesh.paint_uniform_color([0.9,0.1,0.1])
deformed_mesh.paint_uniform_color([0.1,0.1,0.9])
o3d.visualization.draw_geometries([targetmesh,deformed_mesh])


pv_source = pv.read('template.ply')

## here we constrain the topology (order) by connecting the NICP points (reg) to the source topology (triangles)
pv_reconstruction = write_ply_file_NICP(pv_source, np.array(deformed_mesh.vertices), 'reconstructed.ply')


