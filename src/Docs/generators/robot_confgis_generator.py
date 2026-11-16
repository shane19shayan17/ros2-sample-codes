import numpy as np
import xml.etree.ElementTree as ET
import modern_robotics as mr

def rpy_to_R(r, p, y):
    # URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def origin_T(elem):
    if elem is None:
        return np.eye(4)
    xyz = np.array([float(v) for v in elem.get("xyz", "0 0 0").split()])
    rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()]
    return mr.RpToTrans(rpy_to_R(*rpy), xyz)

def urdf_to_screws(urdf_path, chain_joints):
    """chain_joints: joint names ordered from base to end-effector."""
    root = ET.parse(urdf_path).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}

    T = np.eye(4)          # pose of the current joint frame, at zero config
    S_cols = []
    for name in chain_joints:
        j = joints[name]
        T = T @ origin_T(j.find("origin"))     # parent link -> this joint frame

        jtype = j.get("type")
        if jtype in ("revolute", "continuous", "prismatic"):
            a_elem = j.find("axis")
            a = np.array([float(v) for v in
                          (a_elem.get("xyz") if a_elem is not None else "1 0 0").split()])
            a = a / np.linalg.norm(a)
            w = T[:3, :3] @ a                  # axis in base frame
            q = T[:3, 3]                       # a point on the axis
            if jtype == "prismatic":
                S = np.hstack([np.zeros(3), w])
            else:
                S = np.hstack([w, -np.cross(w, q)])
            S_cols.append(S)
        # fixed joints only move the frame; they add no screw

    M = T                                      # end-effector home pose
    Slist = np.array(S_cols).T                 # 6 x n
    Blist = mr.Adjoint(mr.TransInv(M)) @ Slist # body screws
    return Slist, Blist, M

Slist, Blist, M = urdf_to_screws(
    "/home/shayo/coding/learn_ros2/src/robot_description/urdf/arm.urdf",
    ["base_to_link_1", "link_1_link_2", "link_2_link_3", "end_effector"])
print(Blist)