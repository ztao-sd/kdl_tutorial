#!/bin/bash

echo Publishing JointState

rostopic pub -1 joint_state sensor_msgs/JointState "{header: {seq: 123, stamp: 0, frame_id: none}, name: [jnt_state_1], position: [0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0], effort: [0.0, 0.0, 0.0, 0.0, 0.0]}"

rostopic pub -1 cart_state geometry_msgs/Transform "{translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}"