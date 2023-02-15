import cv2
import numpy as np
import os
from glob import glob

LABELS = ['left', 'right','stop','crosswalk','uturn','tgu','tgd','tru','trd','ty','to','ig','###','$$$$']

dir = './data/new/train/'
image_dir = dir + 'JPEGImages/'
anno_dir = dir + 'Annotations/'

# image_files = os.listdir(image_dir)
image_files = glob(image_dir+'*.png')


for image_file in image_files:
    image_file = image_file.split('\\')[-1]
    image = cv2.imread(image_dir+image_file)
    anno_file = image_file[:-3] + 'txt'
    labels, boxes = [], []
    with open(anno_dir+anno_file, 'r') as f:
        lines = f.readlines()
        for line in lines:
            line = line[:-1].split(' ')
            label = int(line[0])
            box = [float(line[1]), float(line[2]), float(line[3]), float(line[4])]
            labels.append(label)
            boxes.append(box)

    for label, box in zip(labels, boxes):
        box = [box[0] - box[2] * 0.5, box[1] - box[3] * 0.5, box[0] + box[2] * 0.5, box[1] + box[3] * 0.5]
        box[0] = int(box[0] * 640)
        box[2] = int(box[2] * 640)
        box[1] = int(box[1] * 480)
        box[3] = int(box[3] * 480)

        color = (255//(label+1), 255//(label+1), 255//(label+1))
        cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), color, 1)
        if label != 11:
            # cv2.putText(image, LABELS[label]+str(box[2]-box[0])+'x'+str(box[3]-box[1]), (box[0], box[1]-5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, color, 1)
            cv2.putText(image, LABELS[label], (box[0], box[1]-5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, color, 1)
    try:
        cv2.imshow(image_file, image)
    except cv2.error:
        print(image_file)
    key=cv2.waitKey()
    if key==27:
        break
    cv2.destroyAllWindows()

cv2.destroyAllWindows()

