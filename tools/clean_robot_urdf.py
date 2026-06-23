"""
Clean a SolidWorks URDF export for the digital-twin viewer.

The second export is already a proper serial chain with real joint axes, so this
just normalizes it:
  - rename joints to joint1..jointN in chain order (matches /joint_states),
  - rewrite mesh paths to relative meshes/<clean>.STL and copy the STL files,
  - unlock revolute joints whose exporter limits are lower==upper (0,0) by
    converting them to 'continuous' so the twin can mirror full motion.

Usage: python tools/clean_robot_urdf.py "<export folder>"
Output: led_app/static/robot/robot.urdf + led_app/static/robot/meshes/*.STL
"""
import os
import sys
import shutil
import xml.etree.ElementTree as ET


def find_export(folder):
    udir = os.path.join(folder, "urdf")
    for f in os.listdir(udir):
        if f.lower().endswith(".urdf"):
            return os.path.join(udir, f), os.path.join(folder, "meshes")
    raise SystemExit("no .urdf found in " + udir)


def mesh_basename(link):
    """Return the STL basename referenced by a link's visual mesh, if any."""
    for mesh in link.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn:
            return fn.replace("\\", "/").split("/")[-1]
    return None


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else \
        r"D:\Git CLones\PI SSH Test\TOP Level assembly robotic\TOP Level assembly robotic"
    urdf_path, mesh_dir = find_export(folder)

    tree = ET.parse(urdf_path)
    robot = tree.getroot()

    links = {l.get("name"): l for l in robot.findall("link")}
    joints = robot.findall("joint")

    # Find chain order: root = link that is never a child.
    child_links = {j.find("child").get("link") for j in joints}
    roots = [n for n in links if n not in child_links]
    if len(roots) != 1:
        print("WARN: expected 1 root, found", roots)
    root = roots[0]

    # Walk the chain from root.
    by_parent = {}
    for j in joints:
        by_parent.setdefault(j.find("parent").get("link"), []).append(j)
    order = [root]
    cur = root
    while cur in by_parent:
        nxt = by_parent[cur][0].find("child").get("link")
        order.append(nxt)
        cur = nxt

    # Clean mesh name per link, in chain order: root -> base_link, rest -> axisN.
    clean_name = {}
    for idx, lname in enumerate(order):
        clean_name[lname] = "base_link" if idx == 0 else "axis%d" % idx

    out_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "led_app", "static", "robot"))
    out_mesh = os.path.join(out_dir, "meshes")
    os.makedirs(out_mesh, exist_ok=True)

    # Copy + rename meshes and rewrite link mesh paths.
    # NOTE: the SolidWorks export put the ENTIRE robot geometry into the base
    # link mesh ("Miro.STL"), which duplicates the whole arm. The first moving
    # link (axis1) already contains the real pedestal, so strip the base link's
    # visual/collision geometry instead of rendering that duplicate.
    for lname, link in links.items():
        if lname == root:
            for tag in ("visual", "collision"):
                for el in link.findall(tag):
                    link.remove(el)
            continue
        src_base = mesh_basename(link)
        if not src_base:
            continue
        dst_base = clean_name[lname] + ".STL"
        src = os.path.join(mesh_dir, src_base)
        if os.path.exists(src):
            shutil.copyfile(src, os.path.join(out_mesh, dst_base))
        else:
            print("WARN missing mesh:", src)
        for mesh in link.iter("mesh"):
            mesh.set("filename", "meshes/" + dst_base)

    # Rename joints joint1.. in chain order and apply travel limits.
    # The base joint (root -> first link) is not used -> make it 'fixed' and
    # renumber only the remaining articulated joints joint1..jointN.
    # Each movable joint becomes 'revolute' with a sensible range (radians) so
    # the arm cannot fold into itself / through itself in the viewer.
    JOINT_LIMITS = {
        1: (-1.5708, 1.5708),   # shoulder  +/-90 deg
        2: (-1.5708, 1.5708),   # elbow     +/-90 deg
        3: (-1.5708, 1.5708),   # forearm   +/-90 deg
        4: (-1.5708, 1.5708),   # wrist     +/-90 deg
        5: (-1.0472, 1.0472),   # gripper   +/-60 deg
    }
    joint_by_child = {j.find("child").get("link"): j for j in joints}
    chain_joints = [joint_by_child[lname] for lname in order[1:]]
    jidx = 0
    for i, j in enumerate(chain_joints):
        if i == 0:
            j.set("type", "fixed")
            j.set("name", "base_fixed")
            for tag in ("limit", "axis"):
                el = j.find(tag)
                if el is not None:
                    j.remove(el)
            continue
        jidx += 1
        j.set("name", "joint%d" % jidx)
        j.set("type", "revolute")
        lo, hi = JOINT_LIMITS.get(jidx, (-3.1416, 3.1416))
        limit = j.find("limit")
        if limit is None:
            limit = ET.SubElement(j, "limit")
        limit.set("lower", "%g" % lo)
        limit.set("upper", "%g" % hi)
        limit.set("effort", "10")
        limit.set("velocity", "3")


    out_urdf = os.path.join(out_dir, "robot.urdf")
    tree.write(out_urdf, encoding="utf-8", xml_declaration=True)
    print("Wrote", out_urdf)
    print("Chain:", " -> ".join(order))
    print("Meshes:", ", ".join(sorted(os.listdir(out_mesh))))


if __name__ == "__main__":
    main()
