#!/usr/bin/python3

import os
import shutil
import random

from absl import flags
from absl import app

FLAGS = flags.FLAGS

flags.DEFINE_string(
    'input_folder',
    '/media/nrsl/software/dataset/hit-dataset/semantic_university',
    'semantic kitti folder, which contains labels and velodyne.')

flags.DEFINE_string(
    'train_folder',
    '/media/nrsl/software/dataset/hit-dataset/semantic_dataset/sequences/00',
    '')

flags.DEFINE_string(
    'test_folder',
    '/media/nrsl/software/dataset/hit-dataset/semantic_dataset/sequences/01',
    '')


def main(argv):
    print(FLAGS.input_folder)

    labels_path = os.path.join(FLAGS.input_folder, 'labels')
    velodyne_path = os.path.join(FLAGS.input_folder, 'velodyne')
    train_folder = FLAGS.train_folder
    test_folder = FLAGS.test_folder

    # prepare output folder
    if os.path.exists(train_folder):
        shutil.rmtree(train_folder)
    os.makedirs(os.path.join(train_folder, 'labels'))
    os.makedirs(os.path.join(train_folder, 'velodyne'))
    train_labels_path = os.path.join(train_folder, 'labels')
    train_velodyne_path = os.path.join(train_folder, 'velodyne')
    if os.path.exists(test_folder):
        shutil.rmtree(test_folder)
    os.makedirs(os.path.join(test_folder, 'labels'))
    os.makedirs(os.path.join(test_folder, 'velodyne'))
    test_labels_path = os.path.join(test_folder, 'labels')
    test_velodyne_path = os.path.join(test_folder, 'velodyne')

    labels_filename = os.listdir(labels_path)
    velodyne_filename = os.listdir(velodyne_path)
    labels_filename.sort()
    velodyne_filename.sort()

    # shuffle file id
    file_count = len(labels_filename)
    file_ids = list(range(0, file_count))
    random.shuffle(file_ids)
    split_id = int(file_count * 0.7)

    train_ids = file_ids[0:split_id]
    test_ids = file_ids[split_id:]

    processed = 0
    for id in train_ids:
        shutil.copy(os.path.join(labels_path, labels_filename[id]),
                    os.path.join(train_labels_path, labels_filename[id]))
        shutil.copy(os.path.join(velodyne_path, velodyne_filename[id]),
                    os.path.join(train_velodyne_path, velodyne_filename[id]))
        processed += 1
        print(processed, file_count)
    for id in test_ids:
        shutil.copy(os.path.join(labels_path, labels_filename[id]),
                    os.path.join(test_labels_path, labels_filename[id]))
        shutil.copy(os.path.join(velodyne_path, velodyne_filename[id]),
                    os.path.join(test_velodyne_path, velodyne_filename[id]))
        processed += 1
        print(processed, file_count)


if __name__ == '__main__':
    app.run(main)
