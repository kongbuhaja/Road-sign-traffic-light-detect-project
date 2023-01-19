import glob
from count_object import count_classes

def make_file_list(split):
    count_classes(split)
    files = glob.glob('./data/original/'+split+'/JPEGImages/*.png') 
    file_names = []
    for file in files:
        name = file.split('/')[-1]
        file_names.append(name.split('.')[0])

    with open('./data/original/'+split+'/ImageSets/all.txt', 'w') as f:
        for file_name in file_names:
            f.writelines(file_name+'\n')
        f.writelines(' ')
        f.close()

make_file_list('train')
make_file_list('eval')
# make_file_list('test')