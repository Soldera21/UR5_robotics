from ultralytics import YOLO
import os

PROJECT_DIR = os.getcwd()

model = YOLO('yolov8m.pt')

metrics = []

split = len([entry for entry in os.listdir(PROJECT_DIR + '/split_5_5-Fold_Cross-val') if os.path.isdir(os.path.join(PROJECT_DIR  + '/split_5_5-Fold_Cross-val', entry))])

for i in range(0, split):
    print()
    print('******************** Initiated train number: ' + str(i) + ' ********************')
    print()
    results = model.train(
        data=PROJECT_DIR + '/split_5_5-Fold_Cross-val/split_' + str(i+1) + '/split_' + str(i+1) + '_dataset.yaml',
        epochs=20,
        imgsz=1024,
        device=[0,1],
        save = True, 
        workers = 8,
        batch = 10,
        project = 'robotica',
    )
    print()
    print('******************** Finished train number: ' + str(i) + ' ********************')
    print()


