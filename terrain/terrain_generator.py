#!/usr/bin/python3
# MJCF Terrain Generator
#
# Reference: https://github.com/unitreerobotics/unitree_mujoco/blob/main/terrain_tool/terrain_generator.py
# Dknt 2025.5

import xml.etree.ElementTree as xml_et
import numpy as np
import cv2
import noise

ROOT_PATH = "/home/dknt/Project/bitbot-ovinf/models/dex/"
INPUT_SCENE_PATH = ROOT_PATH + "mjcf/dex.xml"
OUTPUT_SCENE_PATH = ROOT_PATH + "mjcf/dex_terrain.xml"
HF_IMAGE_PATH = ROOT_PATH + "hf_image/"
HF_INPUT = HF_IMAGE_PATH + "test.png"


# zyx euler angle to quaternion
def euler_to_quat(roll, pitch, yaw):
    cx = np.cos(roll / 2)
    sx = np.sin(roll / 2)
    cy = np.cos(pitch / 2)
    sy = np.sin(pitch / 2)
    cz = np.cos(yaw / 2)
    sz = np.sin(yaw / 2)

    return np.array(
        [
            cx * cy * cz + sx * sy * sz,
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
        ],
        dtype=np.float64,
    )


# zyx euler angle to rotation matrix
def euler_to_rot(roll, pitch, yaw):
    rot_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ],
        dtype=np.float64,
    )

    rot_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ],
        dtype=np.float64,
    )
    rot_z = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ],
        dtype=np.float64,
    )
    return rot_z @ rot_y @ rot_x


# 2d rotate
def rot2d(x, y, yaw):
    nx = x * np.cos(yaw) - y * np.sin(yaw)
    ny = x * np.sin(yaw) + y * np.cos(yaw)
    return nx, ny


# 3d rotate
def rot3d(pos, euler):
    R = euler_to_rot(euler[0], euler[1], euler[2])
    return R @ pos


def list_to_str(vec):
    return " ".join(str(s) for s in vec)


class TerrainGenerator:

    def __init__(self) -> None:
        self.scene = xml_et.parse(INPUT_SCENE_PATH)
        self.root = self.scene.getroot()
        self.worldbody = self.root.find("worldbody")
        self.asset = self.root.find("asset")

    # Add Box to scene
    def AddBox(
        self, position=[1.0, 0.0, 0.0], euler=[0.0, 0.0, 0.0], size=[0.1, 0.1, 0.1]
    ):
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"] = "box"
        geo.attrib["size"] = list_to_str(
            0.5 * np.array(size)
        )  # half size of box for mujoco
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)
        r = np.random.uniform(0, 1)
        g = np.random.uniform(0, 1 - r)
        b = 1 - r - g
        # Alpha must be 1.0: MuJoCo renders geoms with rgba alpha < 0.995 in
        # the transparent pass without depth-buffer writes, making them
        # invisible to depth cameras (mjr_readPixels).
        geo.attrib["rgba"] = f"{r} {g} {b} 1.0"

    def AddGeometry(
        self,
        position=[1.0, 0.0, 0.0],
        euler=[0.0, 0.0, 0.0],
        size=[0.1, 0.1],
        geo_type="box",
    ):
        """
        Stand geometry generator, supporting "plane", "sphere", "capsule", "ellipsoid", "cylinder", "box"

        :param position: Position
        :param euler: Orientation
        :param size: size
        :param geo_type:
        """
        # geo_type supports "plane", "sphere", "capsule", "ellipsoid", "cylinder", "box"
        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["pos"] = list_to_str(position)
        geo.attrib["type"] = geo_type
        geo.attrib["size"] = list_to_str(
            0.5 * np.array(size)
        )  # half size of box for mujoco
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)

    def AddStairs(
        self,
        init_pos=[1.0, 0.0, 0.0],
        yaw=0.0,
        width=0.2,
        height=0.15,
        length=1.5,
        stair_nums=10,
    ):

        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddBox(
                [x + init_pos[0], y + init_pos[1], local_pos[2]],
                [0.0, 0.0, yaw],
                [width, length, height],
            )

    def AddSuspendStairs(
        self,
        init_pos=[1.0, 0.0, 0.0],
        yaw=1.0,
        width=0.2,
        height=0.15,
        length=1.5,
        gap=0.1,
        stair_nums=10,
    ):

        local_pos = [0.0, 0.0, -0.5 * height]
        for i in range(stair_nums):
            local_pos[0] += width
            local_pos[2] += height
            x, y = rot2d(local_pos[0], local_pos[1], yaw)
            self.AddBox(
                [x + init_pos[0], y + init_pos[1], local_pos[2]],
                [0.0, 0.0, yaw],
                [width, length, abs(height - gap)],
            )

    def AddRoughGround(
        self,
        init_pos=[1.0, 0.0, 0.0],
        euler=[0.0, -0.0, 0.0],
        nums=[10, 10],
        box_size=[0.5, 0.5, 0.5],
        box_euler=[0.0, 0.0, 0.0],
        separation=[0.2, 0.2],
        box_size_rand=[0.05, 0.05, 0.05],
        box_euler_rand=[0.2, 0.2, 0.2],
        separation_rand=[0.05, 0.05],
    ):

        local_pos = [0.0, 0.0, -0.5 * box_size[2]]
        new_separation = np.array(separation) + np.array(
            separation_rand
        ) * np.random.uniform(-1.0, 1.0, 2)
        for i in range(nums[0]):
            local_pos[0] += new_separation[0]
            local_pos[1] = 0.0
            for j in range(nums[1]):
                new_box_size = np.array(box_size) + np.array(
                    box_size_rand
                ) * np.random.uniform(-1.0, 1.0, 3)
                new_box_euler = np.array(box_euler) + np.array(
                    box_euler_rand
                ) * np.random.uniform(-1.0, 1.0, 3)
                new_separation = np.array(separation) + np.array(
                    separation_rand
                ) * np.random.uniform(-1.0, 1.0, 2)

                local_pos[1] += new_separation[1]
                pos = rot3d(local_pos, euler) + np.array(init_pos)
                self.AddBox(pos, new_box_euler, new_box_size)

    def AddPerlinHeighField(
        self,
        position=[1.0, 0.0, 0.0],  # position
        euler=[0.0, -0.0, 0.0],  # attitude
        size=[1.0, 1.0],  # width and length
        height_scale=0.2,  # max height
        negative_height=0.2,  # height in the negative direction of z axis
        image_width=128,  # height field image size
        img_height=128,
        smooth=100.0,  # smooth scale
        perlin_octaves=6,  # perlin noise parameter
        perlin_persistence=0.5,
        perlin_lacunarity=2.0,
        output_hfield_image=HF_IMAGE_PATH + "perlin_hf.png",
    ):

        # Generating height field based on perlin noise
        terrain_image = np.zeros((img_height, image_width), dtype=np.uint8)
        for y in range(image_width):
            for x in range(image_width):
                # Perlin noise
                noise_value = noise.pnoise2(
                    x / smooth,
                    y / smooth,
                    octaves=perlin_octaves,
                    persistence=perlin_persistence,
                    lacunarity=perlin_lacunarity,
                )
                terrain_image[y, x] = int((noise_value + 1) / 2 * 255)

        cv2.imwrite(output_hfield_image, terrain_image)

        hfield = xml_et.SubElement(self.asset, "hfield")
        hfield.attrib["name"] = "perlin_hfield"
        hfield.attrib["size"] = list_to_str(
            [size[0] / 2.0, size[1] / 2.0, height_scale, negative_height]
        )
        hfield.attrib["file"] = output_hfield_image

        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["type"] = "hfield"
        geo.attrib["hfield"] = "perlin_hfield"
        geo.attrib["pos"] = list_to_str(position)
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)

    def AddHeighFieldFromImage(
        self,
        position=[1.0, 0.0, 0.0],  # position
        euler=[0.0, -0.0, 0.0],  # attitude
        size=[2.0, 1.6],  # width and length
        height_scale=0.02,  # max height
        negative_height=0.1,  # height in the negative direction of z axis
        input_img=None,
        output_hfield_image=HF_IMAGE_PATH + "image_hf.png",
        image_scale=[1.0, 1.0],  # reduce image resolution
        invert_gray=False,
    ):

        input_image = cv2.imread(input_img)

        width = int(input_image.shape[1] * image_scale[0])
        height = int(input_image.shape[0] * image_scale[1])
        resized_image = cv2.resize(
            input_image, (width, height), interpolation=cv2.INTER_AREA
        )
        terrain_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        if invert_gray:
            terrain_image = 255 - position
        cv2.imwrite(output_hfield_image, terrain_image)

        hfield = xml_et.SubElement(self.asset, "hfield")
        hfield.attrib["name"] = "image_hfield"
        hfield.attrib["size"] = list_to_str(
            [size[0] / 2.0, size[1] / 2.0, height_scale, negative_height]
        )
        hfield.attrib["file"] = output_hfield_image

        geo = xml_et.SubElement(self.worldbody, "geom")
        geo.attrib["type"] = "hfield"
        geo.attrib["hfield"] = "image_hfield"
        geo.attrib["pos"] = list_to_str(position)
        quat = euler_to_quat(euler[0], euler[1], euler[2])
        geo.attrib["quat"] = list_to_str(quat)

    def Save(self):
        self.scene.write(OUTPUT_SCENE_PATH)


def DiscreteUneven(tg: TerrainGenerator):
    tg.AddRoughGround(
        init_pos=[1.5, -3.0, 0.005],
        euler=[0, 0, 0.0],
        nums=[15, 15],
        box_size=[0.5, 0.5, 0.3],
        box_euler=[0.0, 0.0, 0.0],
        separation=[0.5, 0.5],
        box_size_rand=[0.2, 0.2, 0.01],
        # box_euler_rand=[0.07, 0.07, 0.3],
        box_euler_rand=[0.19, 0.19, 0.3],
        separation_rand=[0.05, 0.05],
    )


def Stair(tg: TerrainGenerator):
    tg.AddStairs(
        init_pos=[0.0, 3.0, 0.0],
        yaw=1.57,
        width=0.3,
        height=0.15,
        length=4.0,
        stair_nums=10,
    )
    tg.AddStairs(
        init_pos=[0.0, 9.3, 0.0],
        yaw=-1.57,
        width=0.3,
        height=0.15,
        length=4.0,
        stair_nums=10,
    )


def Slope(tg: TerrainGenerator):
    tg.AddBox(position=[-1.0, 0.0, 0.0], euler=[0.0, 0.20, 0.0], size=[10.0, 5.0, 0.03])


def Gap(tg: TerrainGenerator):
    """Slope up to a platform, then platforms separated by gaps
    (port of the gazebo terrain generator)."""
    import numpy as np

    init_pos = np.array([1.0, -3.0, 0.0])
    bias_x = 3.0
    platform_l = 1.0
    gap_size = [0.6, 0.6, 0.6, 0.6]
    height = 0.3
    slope_angle = 0.12  # rad (~7 deg)
    slope_width = 2.0
    slope_thickness = 0.005

    slope_length = height / np.sin(slope_angle)
    slope_horiz = slope_length * np.cos(slope_angle)
    slope_x_center = bias_x - slope_horiz / 2

    tg.AddBox(
        position=(init_pos + np.array([slope_x_center, 0.0, height / 2])).tolist(),
        euler=[0.0, -slope_angle, 0.0],
        size=[slope_length, slope_width, slope_thickness],
    )
    box_pos = init_pos + np.array([bias_x + platform_l / 2.0, 0.0, height / 2])
    tg.AddBox(position=box_pos.tolist(), size=[platform_l, 2.0, height])
    for _ in gap_size:
        box_pos = box_pos + np.array([platform_l + 0.6, 0.0, 0.0])
        tg.AddBox(position=box_pos.tolist(), size=[platform_l, 2.0, height])


def Stairs(tg: TerrainGenerator):
    """Pyramid stairs: 10 up, platform, 10 down (gazebo layout)."""
    import numpy as np

    init_pos = np.array([1.0, 2.0, 0.0])
    height = 0.15
    width = 0.4
    length = 4.5
    stair_nums = 10
    platform_length = 1.0

    tg.AddStairs(
        init_pos=init_pos.tolist(),
        yaw=0.0,
        width=width,
        height=height,
        length=length,
        stair_nums=stair_nums,
    )
    tg.AddBox(
        position=[
            init_pos[0] + width * stair_nums + (width + platform_length) / 2,
            init_pos[1],
            height * stair_nums + init_pos[2] - 0.05,
        ],
        euler=[0.0, 0.0, 0.0],
        size=[platform_length, length, 0.1],
    )
    tg.AddStairs(
        init_pos=(
            init_pos
            + np.array([width * (stair_nums * 2 + 1) + platform_length, 0.0, 0.0])
        ).tolist(),
        yaw=3.14,
        width=width,
        height=height,
        length=length,
        stair_nums=stair_nums,
    )


def DexTerrain(tg: TerrainGenerator):
    """Dex perceptive-policy test terrain (gazebo reference layout)."""
    Gap(tg)
    Stairs(tg)
    tg.AddBox(position=[20.0, 0.0, 0.0], euler=[0.0, 0.0, 0.0], size=[1.0, 20.0, 5])
    tg.AddBox(position=[0.0, -10.0, 0.0], euler=[0.0, 0.0, 0.0], size=[20.0, 1.0, 5])
    tg.AddBox(position=[2.0, 7.0, 0.0], euler=[0.0, 0.0, 0.0], size=[2.0, 3.0, 0.6])


if __name__ == "__main__":
    import sys

    # Usage: terrain_generator.py [dex|dex]
    robot = sys.argv[1] if len(sys.argv) > 1 else "dex"
    ROOT_PATH = f"/home/dknt/Project/bitbot-ovinf/models/{robot}/"
    INPUT_SCENE_PATH = ROOT_PATH + f"mjcf/{robot}.xml"
    OUTPUT_SCENE_PATH = ROOT_PATH + f"mjcf/{robot}_terrain.xml"

    tg = TerrainGenerator()

    if robot == "dex":
        DexTerrain(tg)
    else:
        DiscreteUneven(tg)
        Slope(tg)
        Stair(tg)

    tg.Save()
