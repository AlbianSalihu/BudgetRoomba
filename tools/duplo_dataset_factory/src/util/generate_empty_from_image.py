import cv2 as cv
import numpy as np
import os
import copy
from tqdm import tqdm
import logging


def _save(id:int, image:np.ndarray, target_dir:str="generated"):
    cv.imwrite("{}/images/im{}.jpg".format(target_dir, id), image)
    open("{}/labels/im{}.txt".format(target_dir, id), "w+").close()
    return True

def _add_noise(image:np.ndarray, noise_intensity:float=0.1):
    row,col,ch= image.shape
    gauss = np.random.normal(0.0,noise_intensity**0.5,(row,col,ch))
    gauss = gauss.reshape(row,col,ch)
    image = image + gauss
    return image.astype(np.uint8)

def generate_empty_from_images(start_id:int, source_dir:str, target_dir:str="generated", rotate:bool=True, noise_intensity:float=100, modify_colors:bool=True, target_resolution:tuple=(640,640,), visualize_each_sample:bool=False):
    onlyfiles = [f for f in os.listdir(source_dir) if os.path.isfile(os.path.join(source_dir, f)) ]
    orientations = np.arange(4) if rotate else [3]
    color_transformations = np.arange(3) if modify_colors else [0]
    id = start_id
    for image_file, _ in zip(onlyfiles, tqdm(onlyfiles)):
        image_file = os.path.join(source_dir, image_file)
        original_image = cv.imread(image_file)
        original_image = cv.resize(original_image, target_resolution)
        for orientation in orientations:
            if orientation != 3: rotated_picture = cv.rotate(copy.deepcopy(original_image), orientation)
            else: rotated_picture = copy.deepcopy(original_image)
            for color_transformation in color_transformations:
                transformed_image = np.roll(copy.deepcopy(rotated_picture), color_transformation, axis=2)              
                transformed_image = _add_noise(transformed_image, noise_intensity)
                if visualize_each_sample:
                    cv.imshow("Sample (to be written on disk)", transformed_image)
                    cv.waitKey()
                _save(id, transformed_image, target_dir)
                logging.debug("Generated image")
                id = id + 1
    logging.info("Generated {} images in {} from {} images in {}".format(id - start_id, target_dir, len(onlyfiles), source_dir))             
    return id - start_id



                
        

