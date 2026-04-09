from util.generate_sample import generate_sample
from util.generate_empty_from_image import generate_empty_from_images

import os
import argparse
import threading
import logging
import copy
from tqdm import tqdm

N_SAMPLES_TARGET = 10000
N_THREADS = 1
START_INDEX = 0

def single_thread_runner(id_range:list, thread_static_arguments:dict):
    for id,_ in zip(id_range,tqdm(range(len(id_range)))):
        generate_sample(id=id, **thread_static_arguments)



if __name__=="__main__":
    logging.getLogger().setLevel(logging.INFO)
    if not os.path.isdir("generated"): os.mkdir("generated")
    if not os.path.isdir("generated/images"): os.mkdir("generated/images")
    if not os.path.isdir("generated/labels"): os.mkdir("generated/labels")
    thread_static_arguments = {
        "max_height": 5.0,
        "dir": "generated",
        "noise_intensity": 50.0,
        "empty_prob": 0.005,
        "texture_dir": "data/textures"
    }
    empty_images = generate_empty_from_images(START_INDEX, source_dir="data/textures", target_dir="generated")
    N_SAMPLES_TARGET = N_SAMPLES_TARGET - empty_images
    START_INDEX = START_INDEX + empty_images

    chunk_size = N_SAMPLES_TARGET // N_THREADS
    if(chunk_size * N_THREADS != N_SAMPLES_TARGET):
        logging.warning("N_SAMPLES_TARGET must be multiple of N_THREADS = {} - generating only {} samples (next smaller multiple)".format(chunk_size * N_THREADS, N_THREADS))
    id_range = list(range(START_INDEX, START_INDEX + chunk_size * N_THREADS))
    threads = []
    for i in range(N_THREADS):
        threads.append(threading.Thread(target=single_thread_runner, args=copy.deepcopy([id_range[i * chunk_size: (i+1) * chunk_size], thread_static_arguments])))
        threads[-1].start()
    for thread in threads:
        thread.join()

    
