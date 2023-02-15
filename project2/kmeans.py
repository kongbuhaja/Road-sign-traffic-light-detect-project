import numpy as np
from glob import glob

def load_objects():

    data = []
    
    file_path = './data/new/train/Annotations/*.txt'
    xml_list = glob(file_path)
    
    for xml in xml_list:
        with open(xml, 'r') as f:
            lines = f.readlines()
            for line in lines:
                box = line[:-1].split(' ')[3:]
                data.append(np.array(box, dtype=np.float32))
    return np.array(data)

def iou(boxes, clusters):
    box_w, box_h = boxes[..., 0:1], boxes[..., 1:2]
    clusters = np.transpose(clusters, (1,0))
    cluster_w, cluster_h = clusters[0:1], clusters[1:2]

    intersection = np.minimum(box_w, cluster_w) * np.minimum(box_h, cluster_h)
    union = box_w * box_h + cluster_w * cluster_h - intersection
    return intersection / union

def avg_iou(boxes, clusters):
    return np.mean([np.max(iou(boxes, clusters), axis=1)])

def kmeans(boxes, k):
    num_boxes = boxes.shape[0]
    distances = np.empty((num_boxes, k))
    last_cluster = np.zeros((num_boxes,))

    np.random.seed()
    clusters = boxes[np.random.choice(num_boxes, k, replace=False)]

    while True:
        distances = 1 - iou(boxes, clusters)
        mean_distance = np.mean(distances)
        
        current_nearest = np.argmin(distances, axis=1)
        if(last_cluster == current_nearest).all():
            break
        for cluster in range(k):
            clusters[cluster] = np.mean(boxes[current_nearest == cluster], axis=0)

        last_cluster = current_nearest
    return clusters

def get_anchors(k, w, h):
    print(f'{w}x{h}')
    boxes = load_objects()
    result = kmeans(boxes, k)
    avg_acc = avg_iou(boxes, result)*100
    print("Average accuracy: {:.2f}%".format(avg_acc))
    result = np.array(sorted(np.array(result), key=lambda x: x[0]*x[1]), dtype=np.float32)
    result[..., 0] = result[..., 0] * w
    result[..., 1] = result[..., 1] * h
    result = result.astype(np.int32)
    print("Anchors")
    print(result)
    return result


get_anchors(6, 640, 480)
get_anchors(6, 416, 416)