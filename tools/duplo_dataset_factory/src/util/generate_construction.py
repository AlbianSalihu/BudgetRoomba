import trimesh
import numpy as np
import os
import copy

COLOR_PALETTE = [
    [255, 0, 0, 255],
    [0, 255, 0, 255],
    [0, 0, 255, 255]
]

BRICK_TYPES = [
    [trimesh.primitives.Box([3,3,3], np.array([[1,0,0,0.75],[0,1,0,0.75],[0,0,1,1.5],[0,0,0,1]])), [2,2], [0,0,0]],
    [trimesh.primitives.Box([6,3,3], np.array([[1,0,0,2.25],[0,1,0,0.75],[0,0,1,1.5],[0,0,0,1]])), [4,2], [0,0,0]],
]
def rotation_matrix_2d(theta:float): return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

def transform(base_mesh: list, new_mesh: list, connection:list, layer:int):
    original_tf = base_mesh[2]
    planar_tf = np.array([connection[0], connection[1]]) * 1.5
    planar_tf = rotation_matrix_2d(original_tf[2]) @ planar_tf + np.array(original_tf[0:2])
    
    new_mesh[2] = [*planar_tf.tolist(), connection[2] * np.pi / 2]
    
    new_mesh[0].apply_transform(np.array([
        [np.cos(new_mesh[2][2]), -np.sin(new_mesh[2][2]), 0, new_mesh[2][0]],
        [np.sin(new_mesh[2][2]), np.cos(new_mesh[2][2]), 0, new_mesh[2][1]],
        [0, 0, 1, layer * 3],
        [0, 0, 0, 1],
    ]))
    return new_mesh
    

def generate_construction(max_height:int, max_width:int, width_prob:float, height_prob:float)->trimesh.Trimesh:
    objects = []
    layers = [[copy.deepcopy(BRICK_TYPES[0])]]
    uv = np.random.rand(layers[0][0][0].vertices.shape[0], 2)
    next_color = COLOR_PALETTE[np.random.randint(0,len(COLOR_PALETTE))]
    layers[0][0][0].visual = trimesh.visual.TextureVisuals(uv=uv,material=trimesh.visual.material.SimpleMaterial(diffuse=next_color, glossiness=1, ambient=next_color, specular=next_color))
    while len(layers) < max_height + 1:
        new_layer = []
        for brick in layers[-1]:
            while np.random.rand() > np.exp(0.2 * len(new_layer)) - 1:
                next_brick = copy.deepcopy(BRICK_TYPES[np.random.randint(0,len(BRICK_TYPES))])
    
                next_color = COLOR_PALETTE[np.random.randint(0,len(COLOR_PALETTE))]
                # next_brick = copy.deepcopy(BRICK_TYPES[0])
                uv = np.random.rand(next_brick[0].vertices.shape[0], 2)
                next_brick[0].visual = trimesh.visual.TextureVisuals(uv=uv,material=trimesh.visual.material.SimpleMaterial(diffuse=next_color, glossiness=1, ambient=None, specular=next_color))
                next_connection = [np.random.randint(0, brick[1][0]), np.random.randint(0, brick[1][1]), np.random.randint(0,4)]
                transform(brick, next_brick, next_connection, len(layers))
                # next_brick[0] = trimesh.Trimesh(next_brick[0].vertices, next_brick[0].faces, visual=trimesh.visual.TextureVisuals(face_materials=trimesh.visual.material.SimpleMaterial(diffuse=next_color, glossiness=1.0, ambient=next_color)))
                new_layer.append(copy.deepcopy(next_brick))
        layers.append(new_layer)
    return layers

if __name__ == "__main__":
    construction = generate_construction(5,0,0,0)
    scene = trimesh.Scene()
    for layer in construction:
        for brick in layer:
            scene.add_geometry(brick[0])
    scene.show()


