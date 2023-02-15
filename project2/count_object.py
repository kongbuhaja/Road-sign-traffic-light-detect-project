import glob

LABELS = ['left', 'right','stop','crosswalk','uturn','tgu','tgd','tru','trd','ty','to','ig','error1', 'error2']

def count_classes(split):
    txt_list = glob.glob('./data/new/' + split + '/Annotations/*.txt')
    classes = [0]*len(LABELS)
    for txt in txt_list:
        with open(txt, 'r') as f:
            data = f.read()
            lines = data.split('\n')
            for line in lines[:-1]:
                classes[int(line.split(' ')[0])] += 1

    print(split, classes)

print(LABELS)
count_classes('train')
count_classes('eval')