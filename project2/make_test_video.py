import cv2, os, glob
import numpy as np
from scipy import stats

def classificate_traffic_light(roi):
    width = roi.shape[1]
    center = int(width * 0.5)
    dif = int(width*0.1)
    l1 = int(center-dif*1.45)
    l3 = int(center+dif*1.45)
    lines = roi[:,[l1, center, l3]].transpose((2,1,0))
    lines = np.mean(lines, 2)
    if np.max(lines) < 100 :
        light = 5 + 4
    elif np.sum((np.abs(lines[2]-lines[1]) < 4).astype(np.int32)) > 2:
        light = 5 + 3
    else:
        light = 5 + stats.mode(np.argmax(lines, 0), keepdims=False)[0]
    return light

def make_video():
    class_str = ['left', 'right', 'stop', 'crosswalk', 'uturn', 'trafic_blue', 'trafic_green', 'trafic_red', 'trafic_yellow', 'traffic_off']

    colors = [[255,0,0], [255,255,0], [255,192,32], [192,192,192], [255,0,255], [255,0,0], [0,255,0], [0,0,255], [0,255,255], [255,255,255]]

    with open('./data/new/eval/ImageSets/all.txt', 'r') as f:
        files = f.readlines()
    with open('./yolov3-pytorch/result.txt', 'r') as f:
        images_outputs = f.readlines()

    out = cv2.VideoWriter('./output_f30.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 30, (640,480))

    image_dir = './data/new/eval/JPEGImages/'
    for file, outputs in zip(files, images_outputs):
        file = file.split('\n')[0]
        if int(file) < 260 or int(file) > 6465:
            continue
        outputs = outputs.split('\n')[0]
        image = cv2.imread(image_dir+file+'.png')
        if outputs != '[]':
            outputs = outputs.split(', ')
            for output in outputs:
                output = output.strip('[]')
                output = np.array(output.split(' ')).astype(np.float32)
                x1, x2 = np.minimum(np.maximum((output[[0,2]] / 416 * 640).astype(np.int32), 0), image.shape[1])
                y1, y2 = np.minimum(np.maximum((output[[1,3]] / 416 * 480).astype(np.int32), 0), image.shape[0])
                prob = output[4]
                label = output[5].astype(np.int32)
                if label == 5:
                    # print(file)
                    label = classificate_traffic_light(image[y1:y2+1, x1:x2+1])
                color = colors[label]
                cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
                cv2.putText(image, class_str[label], (x1, y1-5), cv2.FONT_HERSHEY_COMPLEX, 0.45, color, 1)
                cv2.putText(image, '{:.2f}'.format(prob), (x1, y1+12), cv2.FONT_HERSHEY_COMPLEX, 0.45, color, 1)
        # cv2.imshow('image', image)
        out.write(image)
        # if cv2.waitKey(1) == 27:
            # break
    out.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    make_video()