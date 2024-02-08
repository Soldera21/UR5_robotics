##
# @file dataset2Yolo.py
#
# @brief Convert the given dataset to a Yolo format dataset
#
# @section module_authors Author(s)
# - Created by Marco Morandin
#
import json
import os
import re
import shutil
import yaml

#Global constants
## Name of the folder that contains the given dataset 
ASSIGNS = ['assign1', 'assign2']

## Script working directory
PROJECT_DIR = os.getcwd()


with open(PROJECT_DIR + '/categories.json', 'r') as file:
    ## List of all categories in the dataset
    CATEGORIES = json.load(file)
file.close()

def create_yaml_file():
    """! This function is used to create the yaml file required from YOLO format
    """
    with open(PROJECT_DIR + '/yolo_dataset/data.yaml', 'w') as file:
        yaml.safe_dump({
                'train': '../images',
                'nc': len(CATEGORIES),
                'names': CATEGORIES,
            }, file)
    file.close()

def create_annotations():
    """! This function creates annotations in .txt format for each image and saves it in /labels.
    Moreover this function copy images from the assigned dataset to /images
    """
    for assign in ASSIGNS:
        for scene in os.listdir(PROJECT_DIR + '/' + assign + '/'):
            if os.path.isdir(PROJECT_DIR + '/' + assign + '/' +  scene):
                for filename in os.listdir(PROJECT_DIR + '/' + assign + '/' + scene):
                    if 'jpeg' in filename:
                        shutil.copyfile(PROJECT_DIR + '/' + assign + '/' + scene + '/' + filename, PROJECT_DIR + '/yolo_dataset/images/' + assign + scene + filename)
                        view = [int(s) for s in re.findall(r'\d+', filename)][0]
                        with open(PROJECT_DIR + '/' + assign + '/' + scene + '/view=' + str(view) + '.json', 'r') as file:
                            data = json.load(file)
                        file.close()
                        vertexes = []
                        dimensions = []
                        category = []
                        for obj, value in data.items():
                            vertex = value["3d_bbox_pixel_space"]
                            x = [item[0] for item in vertex]
                            y = [item[1] for item in vertex]
                            dimensions.append([((max(x) - min(x)) / 1024), (max(y) - min(y)) / 1024])
                            vertexes.append([((min(x) + max(x)) / 2 / 1024), ((min(y) + max(y)) / 2 / 1024)])
                            category.append(value["y"])
                        with open(PROJECT_DIR + '/yolo_dataset/labels/' + assign + scene + filename.replace('jpeg', 'txt'), 'w') as file:
                            for i in range(0, len(category)):
                                if (0 <= vertexes[i][0] <= 1) and (0 <= vertexes[i][1] <= 1) and (0 <= dimensions[i][0] <= 1) and (0 <= dimensions[i][1] <= 1):
                                    file.write(str(CATEGORIES.index(category[i])) + ' ' + str(vertexes[i][0]) + ' ' + str(vertexes[i][1]) + ' ' + str(dimensions[i][0]) + ' ' + str(dimensions[i][1]) + '\n')
                        file.close()

if __name__ == '__main__':
    print('***********************')
    print('Yolo dataset builder')
    print('***********************')
    print()
    create_yaml_file()
    print('Yaml file created')
    print()
    print('Starting creating annotations and copying images')
    create_annotations()
    print('Annotations created and images copyed')