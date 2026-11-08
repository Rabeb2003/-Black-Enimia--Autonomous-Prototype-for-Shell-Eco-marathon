#!/usr/bin/env python3
"""
Convert SDF <road> element to static models with boxes for Gazebo Classic.
"""

import xml.etree.ElementTree as ET
import math

def parse_points(road_element):
    """Extract points from road element."""
    points = []
    for point_elem in road_element.findall('point'):
        coords = point_elem.text.strip().split()
        x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
        points.append((x, y, z))
    return points

def calculate_segment_length(p1, p2):
    """Calculate distance between two points."""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2 + (p2[2] - p1[2])**2)

def calculate_segment_angle(p1, p2):
    """Calculate yaw angle for segment."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.atan2(dy, dx)

def generate_static_model(points, width, model_index):
    """Generate static model XML for a road segment."""
    if len(points) < 2:
        return None
    
    p1 = points[0]
    p2 = points[1]
    
    # Calculate segment properties
    length = calculate_segment_length(p1, p2)
    angle = calculate_segment_angle(p1, p2)
    
    # Calculate midpoint
    mid_x = (p1[0] + p2[0]) / 2
    mid_y = (p1[1] + p2[1]) / 2
    mid_z = (p1[2] + p2[2]) / 2
    
    # Generate model XML
    model_xml = f'''    <model name="road_segment_{model_index}">
      <static>true</static>
      <pose>{mid_x} {mid_y} {mid_z + 0.003} 0 0 {angle}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{length} {width} 0.006</size>
            </box>
          </geometry>
          <material>
            <ambient>0.22 0.22 0.22 1</ambient>
            <diffuse>0.22 0.22 0.22 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>{length} {width} 0.006</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.9</mu>
                <mu2>0.9</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>'''
    
    return model_xml

def convert_world(input_file, output_file):
    """Convert world file with road to static models."""
    # Parse input file
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # Find namespace
    namespace = {'sdf': 'http://sdformat.org/sdf'}
    
    # Register namespace if not present
    if root.tag.startswith('{'):
        ns_uri = root.tag.split('{')[1].split('}')[0]
        namespace = {'sdf': ns_uri}
    
    # Find world element
    world_elem = root.find('.//world', namespace) if namespace else root.find('.//world')
    if world_elem is None:
        world_elem = root.find('world')
    
    # Find road element
    road_elem = world_elem.find('road')
    
    if road_elem is None:
        print("No road element found in world file")
        return
    
    # Extract road properties
    width_elem = road_elem.find('width')
    width = float(width_elem.text) if width_elem is not None else 10.0
    
    # Extract points
    points = parse_points(road_elem)
    print(f"Found {len(points)} points in road")
    
    # Remove road element
    world_elem.remove(road_elem)
    
    # Generate static models for each segment
    model_index = 0
    for i in range(len(points) - 1):
        segment_points = [points[i], points[i+1]]
        model_xml = generate_static_model(segment_points, width, model_index)
        
        if model_xml:
            # Parse the model XML and insert into world
            # We'll add it as a string and re-parse
            model_str = f"<model_wrapper>{model_xml}</model_wrapper>"
            model_wrapper = ET.fromstring(model_str)
            model_elem = model_wrapper.find('model')
            
            # Find insertion point (before closing </world>)
            # Insert after ground_plane if exists, otherwise at end
            ground_plane = world_elem.find(".//model[@name='ground_plane']")
            if ground_plane is not None:
                idx = list(world_elem).index(ground_plane) + 1
                world_elem.insert(idx, model_elem)
            else:
                world_elem.append(model_elem)
            
            model_index += 1
    
    print(f"Generated {model_index} road segments")
    
    # Write output file
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"Converted world saved to {output_file}")

if __name__ == '__main__':
    input_file = '/home/rabeb/Downloads/enimia_black/src/enimia_black_description/world/new_silesia_ring.world'
    output_file = '/home/rabeb/Downloads/enimia_black/src/enimia_black_description/world/new_silesia_ring_static.world'
    
    convert_world(input_file, output_file)
