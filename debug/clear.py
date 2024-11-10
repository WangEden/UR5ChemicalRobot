import os
import shutil


path = "images"

if os.path.exists(path):
    shutil.rmtree(path)
    os.makedirs(path)
