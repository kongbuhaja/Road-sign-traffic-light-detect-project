import glob

def count_classes(split):
    txt_list = glob.glob('./data/original/' + split + '/Annotations/*.txt')
    classes = [0]*12
    for txt in txt_list:
        with open(txt, 'r') as f:
            data = f.read()
            lines = data.split('\n')
            for line in lines[:-1]:
                classes[int(line.split(' ')[0])] += 1

    print(split, classes)

count_classes('train')
count_classes('eval')