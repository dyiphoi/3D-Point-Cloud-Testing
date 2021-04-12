#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

file_data_path="N:\Research\AMRL\Photogrammetry\Tool Reconstruction\Mastercut_Reconstruction_V2.txt"
    
point_cloud= np.loadtxt(file_data_path, skiprows=1, max_rows=1000000)


# In[2]:


mean_Z=np.mean(point_cloud,axis=0)[2]
spatial_query=point_cloud[abs( point_cloud[:,2]-mean_Z)<1]
xyz=spatial_query[:,:3]
#rgb=spatial_query[:,3:]


# In[3]:


print(mean_Z)
print("test \n", spatial_query[1:4,:3])


# In[20]:


# A 3-Dimensional array
a = np.array([[0, 1, 2, 3, 4, 5],
              [6, 7, 8, 9, 10, 11],
              [12, 13, 14, 15, 16, 17],
              [18, 19, 20, 21, 22, 23],
              [24, 25, 26, 27, 28, 29],
              [30, 31, 32, 33, 34, 35]])
print("\n Array is:\n ",a)

# slicing and indexing
print("\n a[0, 3:5] = ",a[0, 2:5]) 

print("\n a[4:, 4:] = ",a[4:, 4:]) 

print("\n a[:, 2] = ",a[:, 2]) 

print("\n a[2::2, ::2] = ",a[2::2, ::2]) 


# In[4]:


ax = plt.axes(projection='3d')


# In[5]:


ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], c = 'g', s=0.01)
plt.show()


# In[31]:





# In[7]:


import open3d as o3d

input_path="N:\Research\AMRL\Photogrammetry\Tool Reconstruction/"
output_path="N:\Research\AMRL\Photogrammetry\Tool Reconstruction/"
dataname="Mastercut_Reconstruction_V2.ply"

point_cloud= np.loadtxt(input_path+dataname,skiprows=1)


# In[8]:


#Format to open3d usable objects
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
#pcd.colors = o3d.utility.Vector3dVector(point_cloud[:,3:6]/255)
pcd.normals = o3d.utility.Vector3dVector(point_cloud[:,3:6])

myPoints = np.asarray(pcd.points)

o3d.visualization.draw_geometries([pcd])


# In[9]:


#radius determination
distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist


# In[10]:


#computing the mehs
bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))


# In[11]:


#decimating the mesh
dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)


# In[12]:


dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()


# In[13]:


#computing the mesh
#poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]


# In[14]:


#cropping
bbox = pcd.get_axis_aligned_bounding_box()
#p_mesh_crop = bpa_mesh.crop(bbox)


# In[15]:


#export
o3d.io.write_triangle_mesh(output_path+"bpa_mesh.ply", dec_mesh)
#o3d.io.write_triangle_mesh(output_path+"p_mesh_c.ply", p_mesh_crop)


# In[ ]:


#function creation
def lod_mesh_export(mesh, lods, extension, path):
    mesh_lods={}
    for i in lods:
        mesh_lod = mesh.simplify_quadric_decimation(i)
        o3d.io.write_triangle_mesh(path+"lod_"+str(i)+extension, mesh_lod)
        mesh_lods[i]=mesh_lod
    print("generation of "+str(i)+" LoD successful")
    return mesh_lods


# In[ ]:


#execution of function
my_lods = lod_mesh_export(bpa_mesh, [100000,50000,10000,1000,100], ".ply", output_path)


# In[ ]:


#execution of function
my_lods2 = lod_mesh_export(bpa_mesh, [8000,800,300], ".ply", output_path)


# In[ ]:




