from ast import parse
import numpy as np
import sys, os
from PIL import Image
import argparse

def parse_args():
    parser = argparse.ArgumentParser(description="MNIST")
    parser.add_argument('--data_dir', dest='data_dir', help="data directory",
                        default=None, type=str)
    parser.add_argument("--imgset", dest='imgset', help='imageset',
                        default='all.txt', type=str)
    parser.add_argument('--output_dir', dest='output_dir', help="output directory",
                        default='./output', type=str)
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit()
    args = parser.parse_args()
    return args

def mask(input_img, input_anno, output_img, output_anno):
    img = Image.open(input_img)
    
    normal_box = []
    mask_box = []
    with open(input_anno, 'r') as f:
        lines = f.readlines()
        for l in lines:
            _l = l.replace('\n','')
            box_data = _l.split(" ")
            if box_data[0] in ['9', '10', '11']:
                mask_box += [[float(b) for b in box_data[1:]]]
            else:
                normal_box += [l]

    img_wh = img.size

    if len(mask_box) == 0:
        img.save(output_img)
        with open(output_anno, 'w') as f:
            for n in normal_box:
                f.write(n)
        return

    for m in mask_box:
        #xywh to xyxy
        cx = m[0]
        cy = m[1]
        w = int(m[2] * img.size[0])
        h = int(m[3] * img.size[1])
        m[0] = int((cx - m[2]/2) * img.size[0])
        m[1] = int((cy - m[3]/2) * img.size[1])
        m[2] = int((cx + m[2]/2) * img.size[0])
        m[3] = int((cy + m[3]/2) * img.size[1])
        
        mask_img = Image.new(mode="RGB", size=(w,h), color=(128,128,128))
        img.paste(mask_img, (m[0],m[1]))
    img.save(output_img)
    with open(output_anno, 'w') as f:
        for n in normal_box:
            f.write(n)
    return

def main():
    input_imgset = args.data_dir + "/ImageSets/" + args.imgset
    input_img_dir = args.data_dir + "/JPEGImages/"
    input_anno_dir = args.data_dir + "/Annotations/"

    output_imgset = args.output_dir + '/ImageSets/' + args.imgset
    output_img_dir = args.output_dir + "/JPEGImages/"
    output_anno_dir = args.output_dir + '/Annotations/'

    if not os.path.isdir(args.output_dir):
        os.mkdir(args.output_dir)
    if not os.path.isdir(output_img_dir):
        os.mkdir(output_img_dir)
    if not os.path.isdir(output_anno_dir):
        os.mkdir(output_anno_dir)
    if not os.path.isdir(args.output_dir + '/ImageSets/'):
        os.mkdir(args.output_dir + '/ImageSets/')
    
    file_names = []
   
    with open(input_imgset, 'r') as f:
        lines = f.readlines()
        for l in lines:
            if l == ' ':
                continue
            file_names += [l.replace("\n", "")]

    with open(output_imgset, 'w') as f:
        for l in lines:
            f.write(l)
    
    for i, file_name in enumerate(file_names):
        print(i, file_name)
        input_img = input_img_dir + file_name + '.png'
        input_anno = input_anno_dir + file_name +'.txt'
        output_img = output_img_dir + file_name + '.png'
        output_anno = output_anno_dir + file_name + '.txt'

        mask(input_img, input_anno, output_img, output_anno)
    

if __name__ == "__main__":
    args = parse_args()
    
    main()

# python3 masking.py --data_dir ./data/original/train --output_dir ./data/new/train