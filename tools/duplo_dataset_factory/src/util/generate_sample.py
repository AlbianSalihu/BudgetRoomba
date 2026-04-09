import trimesh
import numpy as np
import os
from util.generate_construction import generate_construction
import cv2 as cv
import io
from PIL import Image
import matplotlib.pyplot as plt
from trimesh.transformations import transform_points
from scipy.spatial.transform import Rotation as R
import time
import pyrender

def generate_sample(id:int, dir:str="generated", max_height:int=5, max_constructions:int=3, noise_intensity:float=0.0, timing:bool=False, empty_prob:float=0.001, texture_dir:str=None): 
    assert max_height > 0 and max_constructions > 0
    if np.random.rand() < empty_prob:
        print(noise_intensity)
        empty_image = np.ones((640,640,3), dtype=np.uint8) 
        empty_image[:,:] = np.random.normal(255 * 0.2,255 * 0.05,3).astype(np.uint8)
        if noise_intensity > 0.0:
            row,col,ch= empty_image.shape
            gauss = np.random.normal(0.0,(noise_intensity)**0.5,(row,col,ch))
            gauss = gauss.reshape(row,col,ch)
            empty_image = empty_image + gauss.astype(np.uint8)
        cv.imwrite("{}/images/im{}.jpg".format(dir, id), empty_image)
        open("{}/labels/im{}.txt".format(dir, id), "w+").close()
    
    
    if timing: start_time=time.time()
    # Generate Scene
    master_scene = trimesh.Scene()
    bounding_boxes = []
    z_shift =  30 * np.random.rand() - 60
    c = master_scene.camera_transform
    n_constructions_target = np.random.randint(1,max_constructions+1)
    n_constructions_is = 0
    while n_constructions_is < n_constructions_target:
        temp_scene = trimesh.Scene()
        if timing: start_time=time.time()
        construction = generate_construction(np.random.randint(1,max_height+1),0,0,0)
        # Generate Random Spatial Transformation for DUPLO construction
        random_rot = R.from_euler("xyz", np.random.normal(0.0, 3*np.pi/2, size=(3))).as_matrix()
        random_trans = np.diag([30,30]) @ np.random.rand(2) + np.array([-15,-15])
        construction_pose = np.eye(4)
        construction_pose[:3,:3] = random_rot
        construction_pose[:2,3] = random_trans
        construction_pose[2,3] = z_shift
        # construction_pose = np.linalg.inv(construction_pose)
        if timing: print("[Construction-Generation]\t{} s".format(time.time()-start_time))
        for layer in construction:
            for brick in layer:
                temp_scene.add_geometry(brick[0], transform=construction_pose)
        potential_border_points = np.array(temp_scene.convex_hull.vertices).transpose()
        potential_border_points = np.concatenate((potential_border_points, np.ones((1,potential_border_points.shape[1]))), axis=0)
        K = np.array([
            [772.5483399, 0, 320],
            [0, 772.5483399, 320],
            [0, 0, 1],

        ])
        # Transform points into imaging plane
        potential_border_points =  K @ (np.linalg.inv(c) @ potential_border_points)[:-1,:]
        potential_border_points = potential_border_points[:2, :] / potential_border_points[2:, :]
        potential_border_points[0,:] = 640 - potential_border_points[0,:]
        potential_border_points[1,:] = 640 - potential_border_points[1,:]
        potential_border_points = np.clip(potential_border_points, 0, 639)
        # Find extrema = Corners of BB-rectangle
        upper_border = np.max(potential_border_points, axis=1)
        lower_border = np.min(potential_border_points, axis=1)
        for bounding_box in bounding_boxes:
            if not(np.any(upper_border < bounding_box[0]) or np.any(bounding_box[1] < lower_border)):
                break
        else:
            bounding_boxes.append([lower_border, upper_border])
            for layer in construction:
                for brick in layer:
                    master_scene.add_geometry(brick[0], transform=construction_pose)
            n_constructions_is += 1
    if timing: print("[Construction2Scene]\t{} s".format(time.time()-start_time))
    # Generate Random Spatial Transformation for DUPLO construction
    master_scene.camera_transform = c
    potential_border_points = np.array(master_scene.convex_hull.vertices).transpose()

    # Insert Ground-Plane such that lowest point of construction touches it
    lowest_z = np.min(potential_border_points[2,:])
    bottom_plane_tf = np.eye(4)
    bottom_plane_tf[:3,3] = np.array([0,0,lowest_z])
    bottom_plane = trimesh.primitives.Box((100,100, 0.01,), bottom_plane_tf)
    uv = np.random.rand(bottom_plane.vertices.shape[0], 2)

    if texture_dir is None or np.random.rand() > 0.2:
        bottom_plane.visual = trimesh.visual.TextureVisuals(uv=uv,material=trimesh.visual.material.SimpleMaterial(diffuse=np.abs(np.random.normal(0.2,0.05,3)), glossiness=0, specular=np.abs(np.random.normal(0.2,0.05,3))))
    else:
        available_textures = [f for f in os.listdir(texture_dir) if os.path.isfile(os.path.join(texture_dir, f)) ]
        texture_path = os.path.join(texture_dir, available_textures[np.random.randint(0,len(available_textures))])
        texture_img = Image.open(texture_path)
        texture_img = texture_img.resize((320,320))
        bottom_plane.visual = trimesh.visual.TextureVisuals(uv=uv,image=texture_img,material=trimesh.visual.material.SimpleMaterial(image=texture_img))
    master_scene.add_geometry(bottom_plane)

    
    # Construct Pyrender-Scene from trimesh
    # TODO: Explain magic numbers (or compute them)
    if timing: start_time=time.time()
    pyrender_scene = pyrender.Scene.from_trimesh_scene(master_scene)
    cam = pyrender.IntrinsicsCamera(772.5483399, 772.5483399, 320, 320, master_scene.camera.z_near, master_scene.camera.z_far)
    cam_node = pyrender_scene.add(cam, pose=master_scene.camera_transform)
    dl = pyrender.DirectionalLight(color=np.clip(np.random.normal(0.9,0.05,3),0.8,1.0), intensity=np.clip(np.random.normal(15.0, 8.0),0.2,np.inf))
    pyrender_scene.add(dl)
    renderer = pyrender.offscreen.OffscreenRenderer(640, 640)
    # Rendering Time!
    rendered_pyrender, _ = renderer.render(pyrender_scene, pyrender.RenderFlags.SHADOWS_ALL )
    if timing: print("[Rendering]\t{} s".format(time.time()-start_time))

    rendered_pyrender = rendered_pyrender[::-1]
    if timing: start_time=time.time()

    # Extract Bounding Box from 'most excentric' points of convex hull
    

    if noise_intensity > 0.0:
        row,col,ch= rendered_pyrender.shape
        gauss = np.random.normal(0.0,noise_intensity**0.5,(row,col,ch))
        gauss = gauss.reshape(row,col,ch)
        rendered_pyrender = rendered_pyrender + gauss
    if timing: print("[Projection]\t{} s".format(time.time()-start_time))

    # Write results to disk
    if timing: start_time=time.time()
    cv.imwrite("{}/images/im{}.jpg".format(dir, id), rendered_pyrender)
    for [lower_border, upper_border] in bounding_boxes:
        cv.rectangle(rendered_pyrender, lower_border.astype(int).tolist(), upper_border.astype(int).tolist(), color=[255,0,255], thickness=5)
    
    cv.imwrite("{}/images/im{}_withBB.jpg".format(dir, id), rendered_pyrender)
    with open("{}/labels/im{}.txt".format(dir, id), "w+") as label_file:
        for [lower_border, upper_border] in bounding_boxes:
            upper_border /= 640.0
            lower_border /= 640.0
            entry = "0 {} {} {} {}\n".format(*(upper_border + lower_border)/2, *(upper_border - lower_border))
            label_file.write(entry)
    if timing: print("[Write2Disk]\t{} s".format(time.time()-start_time))
    

if __name__=="__main__":
    import sys
    try:
        id = int(sys.argv[1])
    except:
        id = 0
    try:
        max_height = int(sys.argv[2])
    except:
        max_height = 3
    try:
        noise_intensity = int(sys.argv[3])
    except:
        noise_intensity = 0
    print("Generate sample with id {} and max_height {} and noise_intensity {}".format(id, max_height, noise_intensity))
    generate_sample(id=id, max_height=max_height, noise_intensity=noise_intensity, max_constructions=8)     
    



