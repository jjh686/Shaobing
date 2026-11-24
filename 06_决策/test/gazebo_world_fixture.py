"""Build the unarmed Gazebo Fortress world used by the navigation-only test."""

from itertools import groupby
from pathlib import Path
import re
from xml.etree import ElementTree

import yaml


def collision_rectangles(blocked, width, height):
    """Merge blocked cells into rectangles on exact grid boundaries."""
    active = {}
    for row in range(height + 1):
        spans = []
        column = 0
        for occupied, cells in groupby(blocked[row * width:(row + 1) * width]):
            end = column + sum(1 for _ in cells)
            if occupied:
                spans.append((column, end))
            column = end
        for first, last in sorted(active.keys() - set(spans)):
            yield first, last, active.pop((first, last)), row
        for span in spans:
            active.setdefault(span, row)


def add_box(link, name, center, size, color):
    for tag in ('collision', 'visual'):
        part = ElementTree.SubElement(link, tag, name=f'{name}_{tag}')
        ElementTree.SubElement(part, 'pose').text = ' '.join(map(str, (*center, 0, 0, 0)))
        geometry = ElementTree.SubElement(part, 'geometry')
        box = ElementTree.SubElement(geometry, 'box')
        ElementTree.SubElement(box, 'size').text = ' '.join(map(str, size))
        if tag == 'visual':
            material = ElementTree.SubElement(part, 'material')
            ElementTree.SubElement(material, 'ambient').text = color
            ElementTree.SubElement(material, 'diffuse').text = color


def mobile_base(spawn_xy):
    x, y = spawn_xy
    model = ElementTree.fromstring(f"""
      <model name="offline_base" canonical_link="chassis">
        <pose>{x} {y} 0.001 0 0 0</pose>
        <link name="chassis">
          <inertial>
            <pose>-0.025 0 0.085 0 0 0</pose>
            <mass>2</mass>
            <inertia><ixx>0.01</ixx><iyy>0.02</iyy><izz>0.03</izz></inertia>
          </inertial>
        </link>
        <link name="caster">
          <pose>-0.14 0 0.035 0 0 0</pose>
          <inertial>
            <mass>0.1</mass>
            <inertia><ixx>0.00005</ixx><iyy>0.00005</iyy><izz>0.00005</izz></inertia>
          </inertial>
          <collision name="contact">
            <geometry><sphere><radius>0.035</radius></sphere></geometry>
          </collision>
          <visual name="body">
            <geometry><sphere><radius>0.035</radius></sphere></geometry>
          </visual>
        </link>
        <joint name="caster_joint" type="ball"><parent>chassis</parent><child>caster</child></joint>
        <plugin filename="ignition-gazebo-diff-drive-system" name="gz::sim::systems::DiffDrive">
          <left_joint>left_wheel_joint</left_joint>
          <right_joint>right_wheel_joint</right_joint>
          <wheel_separation>0.28</wheel_separation>
          <wheel_radius>0.06</wheel_radius>
          <topic>/model/offline_base/cmd_vel</topic>
          <odom_topic>/model/offline_base/odometry</odom_topic>
          <tf_topic>/model/offline_base/tf</tf_topic>
          <frame_id>odom</frame_id>
          <child_frame_id>base_footprint</child_frame_id>
          <odom_publish_frequency>30</odom_publish_frequency>
        </plugin>
      </model>""")
    chassis = model.find("link[@name='chassis']")
    add_box(chassis, 'body', (-0.025, 0, 0.085), (0.33, 0.24, 0.065), '0.15 0.45 0.7 1')
    for side, offset in (('left', 0.14), ('right', -0.14)):
        wheel = f'{side}_wheel'
        model.append(ElementTree.fromstring(f"""
          <link name="{wheel}">
            <pose>0 {offset} 0.06 -1.5707963268 0 0</pose>
            <inertial>
              <mass>0.15</mass>
              <inertia><ixx>0.00015</ixx><iyy>0.00015</iyy><izz>0.00027</izz></inertia>
            </inertial>
            <collision name="contact">
              <geometry><cylinder><radius>0.06</radius><length>0.035</length></cylinder></geometry>
            </collision>
            <visual name="tire">
              <geometry><cylinder><radius>0.06</radius><length>0.035</length></cylinder></geometry>
              <material><diffuse>0.2 0.2 0.2 1</diffuse></material>
            </visual>
          </link>"""))
        model.append(ElementTree.fromstring(f"""
          <joint name="{wheel}_joint" type="revolute">
            <parent>chassis</parent>
            <child>{wheel}</child>
            <axis><xyz>0 0 1</xyz></axis>
          </joint>"""))
    return model


def write_world(map_path: Path, world_path: Path, spawn_xy: tuple[float, float]) -> None:
    """Write a Gazebo world with the same blocked cells as the Nav2 fixture."""
    config = yaml.safe_load(map_path.read_text(encoding='utf-8'))
    image = (map_path.parent / config['image']).read_bytes()
    header = re.match(rb'P5\s+(?:#[^\r\n]*[\r\n]+\s*)*(\d+)\s+(\d+)\s+255(?:\r\n|\s)', image)
    if header is None:
        raise ValueError('Expected an 8-bit binary PGM')
    width, height = (int(value) for value in header.groups())
    pixels = image[header.end():]
    if len(pixels) != width * height:
        raise ValueError('Wrong number of PGM pixels')
    origin_x, origin_y, yaw = config['origin']
    if config['mode'] != 'trinary' or yaw != 0:
        raise ValueError('The fixture requires an axis-aligned trinary map')
    resolution = config['resolution']
    # Unknown cells are also blocked because the test disables Navfn allow_unknown.
    blocked = [(value if config['negate'] else 255 - value) / 255 >= config['free_thresh']
               for value in pixels]

    sdf = ElementTree.fromstring("""
      <sdf version="1.8">
        <world name="offline_3v3">
          <physics name="fixed_step" type="ignored">
            <max_step_size>0.002</max_step_size>
            <real_time_factor>1.0</real_time_factor>
          </physics>
          <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
          <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
          <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
          <light name="sun" type="directional">
            <pose>0 0 8 0 0 0</pose>
            <diffuse>0.85 0.85 0.85 1</diffuse>
            <direction>-0.3 0.2 -0.9</direction>
          </light>
          <model name="map_geometry">
            <static>true</static>
            <link name="field"/>
          </model>
        </world>
      </sdf>""")
    world = sdf.find('world')
    field = world.find("model[@name='map_geometry']/link")
    field_size = width * resolution, height * resolution
    center = origin_x + field_size[0] / 2, origin_y + field_size[1] / 2
    add_box(field, 'floor', (*center, -0.05), (*field_size, 0.1), '0.65 0.65 0.65 1')
    for index, (left, right, top, bottom) in enumerate(collision_rectangles(blocked, width, height)):
        x = origin_x + (left + right) * resolution / 2
        y = origin_y + (height - (top + bottom) / 2) * resolution
        size = (right - left) * resolution, (bottom - top) * resolution, 0.38
        add_box(field, f'wall_{index}', (x, y, 0.19), size, '0.4 0.34 0.31 1')
    world.append(mobile_base(spawn_xy))
    ElementTree.indent(sdf, space='  ')
    ElementTree.ElementTree(sdf).write(world_path, encoding='utf-8', xml_declaration=True)
