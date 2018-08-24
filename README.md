# ros_inception_v3

This repository contains the code which run tensorflow image recognition in ROS ,and also enable to train your own model for image recognition.

## reference

https://github.com/akshaypai/tfClassifier
https://github.com/OTL/rostensorflow

## Developer

*PouChun, Kung (k3083518729@gmail.com)

## Usage & Tutorial

Medium Notes: https://medium.com/@k3083518729/tensorflow-image-recognition-58b0ac77c263

- Install TensorFlow (see [tensor flow install guide](https://www.tensorflow.org/install/install_linux))
- Install ROS (see http://wiki.ros.org)

ROS camera driver & dependencies install
-------------------------------------------
```bash
$ sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-opencv3
```

Install camera driver (for example:cv_camera)

```bash
$ sudo apt-get install ros-kinetic-cv-camera
```

TensorFlow install
-------------------------------------------
Please read official guide.
https://www.tensorflow.org/install/install_linux

image_recognition.py
--------------------------------

* publish: /result (std_msgs/String)
* subscribe: /image (sensor_msgs/Image)

Usage

```bash
$ roscore
$ rosrun cv_camera cv_camera_node
$ python image_recognition.py image:=/cv_camera/image_raw
$ rostopic echo /result
```

Retrain your own model
--------------------------------
Download retrain code

```bash
$ git clone https://github.com/akshaypai/tfClassifier
$ cd tfClassifier/image_classification
```
Download dataset(for example:flower_photos dataset)

```bash
cd ~
curl -O http://download.tensorflow.org/example_images/flower_photos.tgz
tar xzf flower_photos.tgz
```
Retrain

```bash
python retrain.py --model_dir ./inception --image_dir ~/flower_photos ./output --how_many_training_steps 1000
```

After retrain process finish you can find ./output.pb and ./labels.txt in /tmp folder .

Move or copy them to ros_inception_v3 folder.

Replace original ./output.pb and ./labels.txt file with new file.

Start test

```bash
$ roscore
$ rosrun cv_camera cv_camera_node
$ python image_recognition.py image:=/cv_camera/image_raw
$ rostopic echo /result
```
