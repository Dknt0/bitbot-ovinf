#!/usr/bin/python3
# Generate models/dex/mjcf/dex.xml for the bitbot mujoco sim2sim.
#
# Base: tiangong2dex_torq.xml (29 dof tree, meshes, motor classes, excludes)
# Collisions: curated layout of urdf/tiangong2dex_getup.urdf (meshes where
#   available, cylinders for hip_pitch and the feet, none for the rest)
# Sensors: imu site on pelvis (acc/gyro/framepos/linvel) for MujocoImu devices
# Depth camera: mounting of the dex23dof training config
#   (dex23dof_depth_cfg.py: pos (0.092709, -0.07125, 0.039976), rot quat
#   (0.88701, 0, 0.46175, 0) = Ry(55deg) -> euler "0 -(pi/2-55deg) -pi/2",
#   fovx 89.51 / fovy 58.29 on 64x36 -> fovy 58.29)
#
# Dknt / Claude, 2026.8

import math
import os
import xml.etree.ElementTree as ET
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TORQ = os.path.join(ROOT, "tiangong2dex_torq.xml")
OUT = os.path.join(ROOT, "mjcf", "dex.xml")
MESH_DIR = os.path.join(ROOT, "meshes_new")

# Spawn pelvis height (user requirement: 1.05, matching the training init).
# Crouch-pose soles sit 0.9787 m below the pelvis, so the robot drops ~7 cm;
# the waiting-state init-PD hold is active from the first physics step and
# catches the landing.
PELVIS_Z = 1.05


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def qstr(q):
    return " ".join(f"{v:.9g}" for v in q)


# Collision layout from urdf/tiangong2dex_getup.urdf, currently feet-only:
# uncomment MESH_COLLISIONS / CYLINDER_COLLISIONS to re-enable body collisions
# (contact excludes for adjacent + near-connected pairs are already in place).
MESH_COLLISIONS = [
    # "pelvis",
    # "hip_yaw_l_link", "hip_yaw_r_link",
    # "knee_pitch_l_link", "knee_pitch_r_link",
    # "waist_pitch_link",
    # "shoulder_yaw_l_link", "shoulder_yaw_r_link",
    # "wrist_pitch_l_link", "wrist_pitch_r_link",
    # "left_tcp_link", "right_tcp_link",
]

# name: list of (radius, half_length, pos, rpy)
CYLINDER_COLLISIONS = {
    # "hip_pitch_l_link": [(0.06, 0.035, (0, 0.08, -0.02), (0, 1.5708, 0))],
    # "hip_pitch_r_link": [(0.06, 0.035, (0, -0.08, -0.02), (0, 1.5708, 0))],
}

# Non-adjacent link pairs excluded from collision (adjacent pairs come from
# the torq model already)
EXTRA_EXCLUDES = [
    ("pelvis", "hip_yaw_l_link"), ("pelvis", "hip_yaw_r_link"),
    ("hip_pitch_l_link", "knee_pitch_l_link"), ("hip_pitch_r_link", "knee_pitch_r_link"),
    ("knee_pitch_l_link", "ankle_roll_l_link"), ("knee_pitch_r_link", "ankle_roll_r_link"),
    ("hip_yaw_l_link", "hip_yaw_r_link"),
    ("waist_pitch_link", "shoulder_yaw_l_link"), ("waist_pitch_link", "shoulder_yaw_r_link"),
]


def foot_cylinders(sign):
    # sole bars: (radius, half_length, pos(x, y, z), rpy); y offsets from the
    # getup urdf are absolute and identical for both feet
    return [
        (0.012, 0.03, (0.085, -0.03, -0.046), (0, 1.5708, 0)),
        (0.012, 0.03, (0.085, 0.03, -0.046), (0, 1.5708, 0)),
        (0.01, 0.10, (0.045, -0.02, -0.046), (0, -1.5708, 0)),
        (0.01, 0.10, (0.045, 0.02, -0.046), (0, -1.5708, 0)),
        (0.012, 0.11, (0.045, -0.01, -0.046), (0, -1.5708, 0)),
        (0.012, 0.12, (0.045, 0, -0.046), (0, -1.5708, 0)),
        (0.012, 0.11, (0.045, 0.01, -0.046), (0, -1.5708, 0)),
    ]


FEET = {
    "ankle_roll_l_link": foot_cylinders(1),
    "ankle_roll_r_link": foot_cylinders(1),
}


def add_collision_mesh(body, mesh_stl):
    g = ET.SubElement(body, "geom")
    g.set("name", f"{body.get('name')}_collision")
    g.set("type", "mesh")
    g.set("mesh", mesh_stl)
    g.set("class", "collision")
    return g


def add_cylinder(body, radius, half_len, pos, rpy, idx):
    g = ET.SubElement(body, "geom")
    g.set("name", f"{body.get('name')}_collision_{idx}")
    g.set("type", "cylinder")
    g.set("size", f"{radius} {half_len}")
    g.set("pos", " ".join(f"{v:.9g}" for v in pos))
    g.set("quat", qstr(rpy_to_quat(*rpy)))
    g.set("class", "collision")
    return g


def main():
    tree = ET.parse(TORQ)
    root = tree.getroot()

    # --- passive joint dynamics: efc-style light values. The torq motor
    # classes carry damping 0.5-2.0 and frictionloss 0.1-1.0 Nm which the
    # software torque-PD controller does not model (efc mujoco uses 0.01).
    for cls in root.findall("./default/default/default"):
        j = cls.find("joint")
        if j is not None:
            j.set("damping", "0.01")
            j.attrib.pop("frictionloss", None)

    # --- assets: use meshes_new via meshdir, drop explicit file paths ---
    compiler = root.find("compiler")
    compiler.set("meshdir", "../meshes_new")
    # keep the default free camera close to the robot (the terrain walls
    # would otherwise blow up the model extent and the initial view distance)
    stat = ET.Element("statistic", extent="3.0", center="0 0 1.0")
    root.insert(list(root).index(compiler) + 1, stat)
    asset = root.find("asset")
    for mesh in asset.findall("mesh"):
        fname = mesh.get("file").split("/")[-1]
        assert os.path.exists(os.path.join(MESH_DIR, fname)), f"missing mesh {fname}"
        mesh.attrib.pop("file")
        mesh.set("name", fname)
        mesh.set("file", fname)
    # collision material: alpha 0 (invisible) so depth/GUI see visual meshes only
    col_mat = asset.find("material[@name='collision_material']")
    col_mat.set("rgba", "0.5 0.5 0.5 0")
    # floor checker texture/material
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker",
                  width="512", height="512", rgb1=".1 .2 .3", rgb2=".2 .3 .4")
    ET.SubElement(asset, "material", name="grid", texture="grid",
                  texrepeat="1 1", texuniform="true", reflectance=".2")

    # --- simulation options: same as the efc sim2sim setup ---
    opt = ET.Element("option", timestep="0.001", integrator="RK4")
    ET.SubElement(opt, "flag", frictionloss="enable")
    root.insert(list(root).index(asset) + 1, opt)

    worldbody = root.find("worldbody")

    # --- world: lights + floor ---
    floor = worldbody.find("geom[@name='floor']")
    floor.set("material", "grid")
    floor.set("size", "0 0 0.05")
    floor.attrib.pop("rgba", None)
    floor.attrib.pop("friction", None)
    floor.attrib.pop("condim", None)
    ET.SubElement(worldbody, "light", name="main_light", diffuse="2 2 2",
                  specular="0.5 0.5 0.5", pos="0 0 40", dir="0 0 -1", castshadow="true")

    # --- pelvis: spawn height + imu site ---
    pelvis = worldbody.find("body[@name='pelvis']")
    pos = [float(v) for v in pelvis.get("pos").split()]
    pos[2] = PELVIS_Z
    pelvis.set("pos", " ".join(f"{v:.9g}" for v in pos))
    site = pelvis.find("site[@name='pelvis_site']")
    site.set("name", "imu")
    site.set("size", "0.01")

    # --- collisions: drop torq blanket meshes, add the getup urdf layout ---
    for body in worldbody.iter("body"):
        name = body.get("name")
        for geom in list(body.findall("geom")):
            if geom.get("class") == "collision" or "_collision" in geom.get("name", ""):
                body.remove(geom)
        if name in MESH_COLLISIONS:
            add_collision_mesh(body, f"{name}.STL")
        elif name in CYLINDER_COLLISIONS:
            for i, (r, hl, p, rpy) in enumerate(CYLINDER_COLLISIONS[name]):
                add_cylinder(body, r, hl, p, rpy, i)
        elif name in FEET:
            for i, (r, hl, p, rpy) in enumerate(FEET[name]):
                add_cylinder(body, r, hl, p, rpy, i)

    # --- depth camera housing: keep the body (mass) but drop its geoms, the
    # camera mesh sits right in front of the depth camera and occludes it ---
    cam_body = worldbody.find(".//body[@name='camera_body_front_link']")
    for geom in list(cam_body.findall("geom")):
        cam_body.remove(geom)

    # --- depth camera on waist_pitch_link (training mounting) ---
    waist_pitch = worldbody.find(".//body[@name='waist_pitch_link']")
    cam = ET.SubElement(waist_pitch, "camera")
    cam.set("name", "depth_camera")
    cam.set("pos", "0.092709 -0.07125 0.039976")
    cam.set("euler", "0 -0.6108652 -1.5707963")  # Ry(55deg) pitched down, verified
    cam.set("mode", "fixed")
    cam.set("fovy", "58.29")

    # --- weld the head: the real-hardware 29dof urdf has no head joints ---
    for jname in ("head_yaw_joint", "head_pitch_joint"):
        for body in worldbody.iter("body"):
            for j in body.findall(f"joint[@name='{jname}']"):
                body.remove(j)
        actuator = root.find("actuator")
        for a in actuator.findall(f"motor[@name='{jname}_ctrl']"):
            actuator.remove(a)

    # --- actuators named exactly like the joints (MujocoJoint resolves both) ---
    for motor in root.find("actuator").findall("motor"):
        motor.set("name", motor.get("name").removesuffix("_ctrl"))

    # --- sensors for the bus devices (MujocoImu / framepos / framelinvel) ---
    sensor = root.find("sensor")
    sensor.clear()
    ET.SubElement(sensor, "accelerometer", name="linear-acceleration", site="imu")
    ET.SubElement(sensor, "gyro", name="angular-velocity", site="imu")
    ET.SubElement(sensor, "framepos", name="imu_pos", objtype="site", objname="imu")
    ET.SubElement(sensor, "framelinvel", name="imu_linvel", objtype="site", objname="imu")

    # --- contact excludes for near-connected (non-adjacent) links ---
    contact = root.find("contact")
    for b1, b2 in EXTRA_EXCLUDES:
        ET.SubElement(contact, "exclude", body1=b1, body2=b2)

    ET.indent(tree, space="  ")
    tree.write(OUT, encoding="unicode", xml_declaration=True)
    print(f"wrote {OUT}")
    n_joint = len(worldbody.findall(".//joint"))
    n_motor = len(root.find("actuator").findall("motor"))
    print(f"joints: {n_joint}, motors: {n_motor}")

    # --- sanity: joint set must match the real-hardware 29dof urdf ---
    urdf = ET.parse(os.path.join(ROOT, "urdf", "tiangong2dex_29dof.urdf")).getroot()
    urdf_joints = {j.get("name") for j in urdf.findall(".//joint") if j.get("type") != "fixed"}
    mjcf_joints = {j.get("name") for j in worldbody.findall(".//joint")}
    if urdf_joints != mjcf_joints:
        print(f"WARNING joint mismatch\n  only urdf: {sorted(urdf_joints - mjcf_joints)}"
              f"\n  only mjcf: {sorted(mjcf_joints - urdf_joints)}")
    else:
        print("joint set matches urdf/tiangong2dex_29dof.urdf (29 dof hardware)")


if __name__ == "__main__":
    main()
