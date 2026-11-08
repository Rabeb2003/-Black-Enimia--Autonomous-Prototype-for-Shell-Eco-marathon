#!/usr/bin/env python3
"""
Convert SDF <road> element to static models with boxes - simplified version.
Groups multiple points into fewer segments for better performance.
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

def generate_static_model(p1, p2, width, model_index):
    """Generate static model XML for a road segment."""
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

def convert_world(input_file, output_file, skip_points=5):
    """Convert world file with road to static models with skipping points."""
    # Parse input file
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # Find world element
    world_elem = root.find('.//world')
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
    
    # Generate static models with skipping points for fewer segments
    model_index = 0
    for i in range(0, len(points) - skip_points, skip_points):
        p1 = points[i]
        p2 = points[i + skip_points]
        
        model_xml = generate_static_model(p1, p2, width, model_index)
        
        # Parse the model XML and insert into world
        model_str = f"<model_wrapper>{model_xml}</model_wrapper>"
        model_wrapper = ET.fromstring(model_str)
        model_elem = model_wrapper.find('model')
        
        # Find insertion point (after ground_plane if exists)
        ground_plane = world_elem.find(".//model[@name='ground_plane']")
        if ground_plane is not None:
            idx = list(world_elem).index(ground_plane) + 1
            world_elem.insert(idx, model_elem)
        else:
            world_elem.append(model_elem)
        
        model_index += 1
    
    print(f"Generated {model_index} road segments (skipping every {skip_points} points)")
    
    # Write output file
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    print(f"Converted world saved to {output_file}")

if __name__ == '__main__':
    input_file = '/home/rabeb/Downloads/enimia_black/src/enimia_black_description/world/new_silesia_ring.world'
    output_file = '/home/rabeb/Downloads/enimia_black/src/enimia_black_description/world/new_silesia_ring_static.world'
    
    # Skip every 5 points to reduce number of segments from 436 to ~87
    convert_world(input_file, output_file, skip_points=5)
