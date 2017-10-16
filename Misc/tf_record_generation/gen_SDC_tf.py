"""
Usage:
  # From tensorflow/models/
  # Create train data:
  python gen_SDC_tf.py --yaml_input=data/train.yaml  --img_dir_path=images --output_path=train.record
"""
from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import io
import pandas as pd
import tensorflow as tf

from PIL import Image
from object_detection.utils import dataset_util
from collections import namedtuple, OrderedDict

import yaml

flags = tf.app.flags
flags.DEFINE_string('yaml_input', '', 'Path to the yaml input')
flags.DEFINE_string('img_dir_path', '', 'Path to the image dir')
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS


# TO-DO replace this with label map
def class_text_to_int(row_label):
    if row_label == 'Green':
        return 1
    elif row_label == 'Red':
        return 2
    elif row_label == 'Yellow':
        return 3
    else:
        None


def split(df, group):
    data = namedtuple('data', ['filename', 'object'])
    gb = df.groupby(group)
    return [data(filename, gb.get_group(x)) for filename, x in zip(gb.groups.keys(), gb.groups)]


def create_tf_example(path, annotation):
    g = annotation['filename']
    h = g.split('/')
    img_name = h[1]
    #print(img_path)   
    with tf.gfile.GFile(os.path.join(path, '{}'.format(img_name)), 'rb') as fid:
        encoded_jpg = fid.read()
    encoded_jpg_io = io.BytesIO(encoded_jpg)
    image = Image.open(encoded_jpg_io)
    width, height = image.size
    #print('w:%d, h:%d', (width, height)) 

    filename = img_name.encode('utf8')
    image_format = b'jpg'
    xmins = []
    xmaxs = []
    ymins = []
    ymaxs = []
    classes_text = []
    classes = []

    boxes = annotation['boxes']
    num_boxes = len(boxes)
    #print('num_boxes:', num_boxes)
    for j in range(num_boxes):
        x_min = boxes[j]['xmin']
        x_width =  boxes[j]['x_width']
        y_min = boxes[j]['ymin']
        y_height =  boxes[j]['y_height']
        x_max = x_min + x_width
        y_max = y_min + y_height

        xmins.append(float(x_min) / width)
        xmaxs.append(float(x_max) / width)
        ymins.append(float(y_min) / height)
        ymaxs.append(float(y_max) / height)
        classes_text.append(boxes[j]['label'].encode('utf8'))
        classes.append(class_text_to_int(boxes[j]['label']))

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_jpg),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))
    
    return tf_example


def main(_):
    writer = tf.python_io.TFRecordWriter(FLAGS.output_path)
    #path = os.path.join(os.getcwd(), 'images')
    path = os.path.join(os.getcwd(), FLAGS.img_dir_path)
    with open(FLAGS.yaml_input, 'r') as f:
        doc = yaml.load(f)
    #grouped = split(examples, 'filename')
    for i in range(len(doc)):
        tf_example = create_tf_example(path, doc[i])
        writer.write(tf_example.SerializeToString())

    writer.close()
    output_path = os.path.join(os.getcwd(), FLAGS.output_path)
    print('Successfully created the TFRecords: {}'.format(output_path))
    #print('doc_leng: {}'.format(len(doc)))

if __name__ == '__main__':
    tf.app.run()
