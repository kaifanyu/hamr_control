"""Compose two existing caster URDFs for orientation-only history playback."""
import copy
import xml.etree.ElementTree as ET


def dual_caster_description(source):
    caster = ET.parse(source).getroot()
    robot = ET.Element("robot", name="ordinary_caster_history")
    ET.SubElement(robot, "link", name="ordinary_caster_history")
    for wheel, lateral in (("front_left", 0.22), ("front_right", -0.22)):
        prefix = wheel + "_"
        names = {element.attrib["name"] for element in caster.iter() if "name" in element.attrib}
        for child in caster:
            if child.tag == "gazebo":
                continue
            cloned = copy.deepcopy(child)
            for element in cloned.iter():
                for attribute in ("name", "link"):
                    if element.get(attribute) in names:
                        element.set(attribute, prefix + element.get(attribute))
            robot.append(cloned)
        joint = ET.SubElement(robot, "joint", name=prefix + "display_mount", type="fixed")
        ET.SubElement(joint, "parent", link="ordinary_caster_history")
        ET.SubElement(joint, "child", link=prefix + "mount_frame")
        ET.SubElement(joint, "origin", xyz=f"0 {lateral} 0")
    return ET.tostring(robot, encoding="unicode")
