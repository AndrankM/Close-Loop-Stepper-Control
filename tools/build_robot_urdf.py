"""
Convert the flat SolidWorks URDF export (every joint parented to base_link)
into a proper serial-chain URDF for the digital twin viewer.

Chain: base_link -> Axis 1 -> Axis 2 -> Axis 3 -> Axis 4 -> Axis 5 -> Axis 6

- Joint origins in the export are all expressed relative to base_link.
  For a chain, each joint origin must be relative to its PARENT link frame.
  rel_i = inv(M_{i-1}) * M_i, with M_0 = identity (base_link).
- Visual/collision origins stay at 0 (meshes are already in each link frame),
  so the STL files are reused unchanged.
- Joints 2/3/4 have real axes from the export and are kept.
  Joints 1/5/6 were exported as "fixed" with no axis -> best-guess axes,
  flagged for confirmation against the physical arm (or re-export from SW).

Output: led_app/static/robot/robot.urdf  (mesh paths -> meshes/<clean>.STL)
"""
import math
import os

# (link, motor label, joint type, xyz, rpy, axis, limit_lo, limit_hi, mesh)
# axis/type per analysis; GUESS axes marked in comments.
JOINTS = [
    # name, child_link, jtype, xyz, rpy, axis, lo, hi, confirmed
    ("joint1", "Axis 1", "revolute",
        (0.32844, -0.107, -0.060546), (1.5708, 0.0, 0.0),
        (0.0, 0.0, 1.0), -3.14159, 3.14159, False),          # Motor 1 base - GUESS axis
    ("joint2", "Axis 2", "continuous",
        (0.098881, -0.1891, 0.18214), (1.5708, 0.0, -3.139),
        (0.0, 1.0, 0.0), None, None, True),                  # Motor 2
    ("joint3", "Axis 3", "continuous",
        (0.0988814853654335, -0.189096181083927, 0.275171996022754),
        (1.05806181807897, 0.0, -1.56821304671233),
        (0.0, 0.871406321384849, 0.490561946190821), None, None, True),  # Motor 3 PAN
    ("joint4", "Axis 4", "revolute",
        (0.14941, -0.17272, 0.46913), (-0.16557, 0.0, -1.5682),
        (0.0, 1.0, 0.0), -3.14159, 3.14159, True),           # Motor 4 (export said prismatic)
    ("joint5", "Axis 5", "revolute",
        (0.2774, -0.15609, 0.46612), (-0.16557, 0.0076518, -1.5695),
        (1.0, 0.0, 0.0), -2.35619, 2.35619, False),          # Servo 5 TILT - GUESS axis (+-135 deg)
    ("joint6", "Axis 6", "revolute",
        (0.33225, -0.15567, 0.54515), (-3.1323, -0.60803, -0.004021),
        (0.0, 0.0, 1.0), -1.5708, 1.5708, False),            # Servo 6 gripper - GUESS axis
]

MESH_FILE = {
    "base_link": "base_link.STL",
    "Axis 1": "axis1.STL",
    "Axis 2": "axis2.STL",
    "Axis 3": "axis3.STL",
    "Axis 4": "axis4.STL",
    "Axis 5": "axis5.STL",
    "Axis 6": "axis6.STL",
}


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    # R = Rz * Ry * Rx
    def m3(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return m3(Rz, m3(Ry, Rx))


def make_T(xyz, rpy):
    R = rpy_to_R(*rpy)
    return [
        [R[0][0], R[0][1], R[0][2], xyz[0]],
        [R[1][0], R[1][1], R[1][2], xyz[1]],
        [R[2][0], R[2][1], R[2][2], xyz[2]],
        [0, 0, 0, 1],
    ]


def inv_T(T):
    # inverse of a rigid transform
    R = [[T[i][j] for j in range(3)] for i in range(3)]
    p = [T[i][3] for i in range(3)]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]  # transpose
    pinv = [-sum(Rt[i][k] * p[k] for k in range(3)) for i in range(3)]
    return [
        [Rt[0][0], Rt[0][1], Rt[0][2], pinv[0]],
        [Rt[1][0], Rt[1][1], Rt[1][2], pinv[1]],
        [Rt[2][0], Rt[2][1], Rt[2][2], pinv[2]],
        [0, 0, 0, 1],
    ]


def T_to_rpy(T):
    R = [[T[i][j] for j in range(3)] for i in range(3)]
    # ZYX extraction
    sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
    if sy > 1e-9:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:  # gimbal lock
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0.0
    return (roll, pitch, yaw)


def f(x):
    return ("%.9g" % x)


def main():
    # absolute transforms from base for each joint (zero pose)
    abs_T = [make_T(j[3], j[4]) for j in JOINTS]

    parts = []
    parts.append('<?xml version="1.0" encoding="utf-8"?>')
    parts.append('<!-- Corrected serial-chain URDF for digital twin (generated). -->')
    parts.append('<!-- Joints 1/5/6 axes are best-guess (export had them fixed). -->')
    parts.append('<robot name="robotic_arm">')

    # base link
    parts.append('  <link name="base_link">')
    parts.append('    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry>')
    parts.append('      <mesh filename="meshes/%s"/></geometry>' % MESH_FILE["base_link"])
    parts.append('      <material name="metal"><color rgba="0.75 0.75 0.75 1"/></material></visual>')
    parts.append('  </link>')

    prev_T = [[1 if i == j else 0 for j in range(4)] for i in range(4)]  # identity (base)
    for idx, j in enumerate(JOINTS):
        name, child, jtype, xyz, rpy, axis, lo, hi, confirmed = j
        Mi = abs_T[idx]
        rel = matmul(inv_T(prev_T), Mi)
        rxyz = (rel[0][3], rel[1][3], rel[2][3])
        rrpy = T_to_rpy(rel)
        parent = "base_link" if idx == 0 else JOINTS[idx - 1][1]

        # link
        col = "0.5 0.5 0.5 1" if child == "Axis 5" else "0.75 0.75 0.75 1"
        parts.append('  <link name="%s">' % child)
        parts.append('    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry>')
        parts.append('      <mesh filename="meshes/%s"/></geometry>' % MESH_FILE[child])
        parts.append('      <material name=""><color rgba="%s"/></material></visual>' % col)
        parts.append('  </link>')

        # joint
        tag = "" if confirmed else "  <!-- GUESS axis: confirm vs hardware -->"
        parts.append('  <joint name="%s" type="%s">%s' % (name, jtype, tag))
        parts.append('    <origin xyz="%s %s %s" rpy="%s %s %s"/>' % (
            f(rxyz[0]), f(rxyz[1]), f(rxyz[2]), f(rrpy[0]), f(rrpy[1]), f(rrpy[2])))
        parts.append('    <parent link="%s"/>' % parent)
        parts.append('    <child link="%s"/>' % child)
        parts.append('    <axis xyz="%s %s %s"/>' % (f(axis[0]), f(axis[1]), f(axis[2])))
        if jtype == "revolute":
            parts.append('    <limit lower="%s" upper="%s" effort="10" velocity="3"/>' % (f(lo), f(hi)))
        parts.append('  </joint>')

        prev_T = Mi

    parts.append('</robot>')

    out_dir = os.path.join(os.path.dirname(__file__), "..", "led_app", "static", "robot")
    out_dir = os.path.abspath(out_dir)
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "robot.urdf")
    with open(out_path, "w", encoding="utf-8") as fp:
        fp.write("\n".join(parts) + "\n")
    print("Wrote", out_path)


if __name__ == "__main__":
    main()
