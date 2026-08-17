#!/usr/bin/python3
# Generate config/dex/dex_mj.xml from config/dex/robot.yaml.
#
# Device id order MUST match robot.yaml joint_names; each MujocoJoint gets an
# initial_pos (degrees) from the yaml init_position so the robot spawns in the
# training crouch pose like the gazebo ros2_control initial_value spawn.
#
# Dknt / Claude, 2026.8

import math
import os

import yaml

REPO = "/home/dknt/Project/bitbot-ovinf"
ROBOT_YAML = os.path.join(REPO, "config/dex/robot.yaml")
OUT = os.path.join(REPO, "config/dex/dex_mj.xml")
MODEL = "../../models/dex/mjcf/dex_terrain.xml"

SHORT2MJCF = {
    "l_hip_p": "hip_pitch_l_joint", "l_hip_r": "hip_roll_l_joint",
    "l_hip_y": "hip_yaw_l_joint", "l_knee": "knee_pitch_l_joint",
    "l_ankle_p": "ankle_pitch_l_joint", "l_ankle_r": "ankle_roll_l_joint",
    "r_hip_p": "hip_pitch_r_joint", "r_hip_r": "hip_roll_r_joint",
    "r_hip_y": "hip_yaw_r_joint", "r_knee": "knee_pitch_r_joint",
    "r_ankle_p": "ankle_pitch_r_joint", "r_ankle_r": "ankle_roll_r_joint",
    "waist_y": "waist_yaw_joint", "waist_r": "waist_roll_joint",
    "waist_p": "waist_pitch_joint",
    "l_shoulder_p": "shoulder_pitch_l_joint", "l_shoulder_r": "shoulder_roll_l_joint",
    "l_shoulder_y": "shoulder_yaw_l_joint", "l_elbow_p": "elbow_pitch_l_joint",
    "l_elbow_y": "elbow_yaw_l_joint", "l_wrist_p": "wrist_pitch_l_joint",
    "l_wrist_r": "wrist_roll_l_joint",
    "r_shoulder_p": "shoulder_pitch_r_joint", "r_shoulder_r": "shoulder_roll_r_joint",
    "r_shoulder_y": "shoulder_yaw_r_joint", "r_elbow_p": "elbow_pitch_r_joint",
    "r_elbow_y": "elbow_yaw_r_joint", "r_wrist_p": "wrist_pitch_r_joint",
    "r_wrist_r": "wrist_roll_r_joint",
}


def main():
    cfg = yaml.safe_load(open(ROBOT_YAML))["RobotConfig"]
    joints = cfg["joint_names"]
    init = cfg["init_pos"]["init_position"]
    assert len(joints) == cfg["joint_size"] == cfg["motor_size"]

    lines = [
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>',
        '<bitbot>',
        '  <logger path="../../log/" level="debug"/>', '',
        '  <backend port="12888" settings_file="./backend.json"/>', '',
        f'  <mujoco file="{MODEL}"/>', '', '  <bus>',
    ]
    for i, short in enumerate(joints):
        deg = math.degrees(init[short])
        lines.append(f'    <device id="{i}" type="MujocoJoint" name="{SHORT2MJCF[short]}" mode="torque"')
        lines.append(f'            initial_pos="{deg:.6f}"')
        lines.append("            pos_kp='0' pos_kd='0' pos_ki='0' vel_kp='0' vel_kd='0'/>")
    n = len(joints)
    lines += [
        '',
        f'    <device id="{n}" type="MujocoImu" name="imu"',
        '      site="imu" acc="linear-acceleration" gyro="angular-velocity"/>',
        f'    <device id="{n+1}" type="MujocoFramepos" name="imu_pos" />',
        f'    <device id="{n+2}" type="MujocoFramelinvel" name="imu_linvel" />',
        '',
        f'    <device id="{n+3}" type="MujocoDepthCamera" name="depth_camera"',
        '      camera="depth_camera" width="64" height="36" frequency="50.0"',
        '      min_depth="0.0" max_depth="2.5" znear="0.05" zfar="10.0" noise_stddev="0.01"',
        '      crop_top="18" crop_bottom="0" crop_left="16" crop_right="16"',
        '      blur_kernel="3" blur_sigma="1.0" normalize="true"',
        '      debug_vis="false" debug_vis_raw="false"/>',
        '  </bus>', '</bitbot>', '',
    ]
    open(OUT, "w").write("\n".join(lines))
    print(f"wrote {OUT}: {n} joints (crouch spawn) + imu/framepos/linvel/depth")


if __name__ == "__main__":
    main()
