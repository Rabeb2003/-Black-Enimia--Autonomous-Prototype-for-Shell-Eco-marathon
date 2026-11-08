#!/usr/bin/env python3
import argparse
import csv
import math
import xml.etree.ElementTree as ET


GPX_NS = {'gpx': 'http://www.topografix.com/GPX/1/1'}
EARTH_RADIUS_M = 6378137.0


def read_trackpoints(gpx_path):
    root = ET.parse(gpx_path).getroot()
    points = [
        (float(point.attrib['lat']), float(point.attrib['lon']))
        for point in root.findall('.//gpx:trkpt', GPX_NS)
    ]
    if len(points) < 2:
        raise RuntimeError(f'Need at least 2 trackpoints in {gpx_path}')
    return points


def project_points(points, origin_lat=None, origin_lon=None):
    if origin_lat is None:
        origin_lat = points[0][0]
    if origin_lon is None:
        origin_lon = points[0][1]

    origin_lat_rad = math.radians(origin_lat)
    scale_lon = math.cos(origin_lat_rad)
    projected = []

    for lat, lon in points:
        x = math.radians(lon - origin_lon) * EARTH_RADIUS_M * scale_lon
        y = math.radians(lat - origin_lat) * EARTH_RADIUS_M
        projected.append((x, y))

    return projected


def transform_points(points, yaw_offset, scale, translate_x, translate_y):
    cos_yaw = math.cos(yaw_offset)
    sin_yaw = math.sin(yaw_offset)
    transformed = []

    for x, y in points:
        xr = scale * (x * cos_yaw - y * sin_yaw) + translate_x
        yr = scale * (x * sin_yaw + y * cos_yaw) + translate_y
        transformed.append((xr, yr))

    return transformed


def compute_yaws(points):
    yaws = []
    for index, (x, y) in enumerate(points):
        next_x, next_y = points[(index + 1) % len(points)]
        if index == len(points) - 1:
            prev_x, prev_y = points[index - 1]
            dx = x - prev_x
            dy = y - prev_y
        else:
            dx = next_x - x
            dy = next_y - y
        yaws.append(math.atan2(dy, dx))
    return yaws


def write_waypoints(points, output_path, z, velocity):
    yaws = compute_yaws(points)
    with open(output_path, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['x', 'y', 'z', 'yaw', 'velocity', 'change_flag'])
        for (x, y), yaw in zip(points, yaws):
            writer.writerow([
                f'{x:.4f}',
                f'{y:.4f}',
                f'{z:.4f}',
                f'{yaw:.6f}',
                f'{velocity:.4f}',
                0,
            ])


def parse_args():
    parser = argparse.ArgumentParser(
        description='Convert a GPX track into ENIMIA waypoint CSV format.'
    )
    parser.add_argument('input_gpx')
    parser.add_argument('output_csv')
    parser.add_argument('--origin-lat', type=float)
    parser.add_argument('--origin-lon', type=float)
    parser.add_argument('--yaw-offset', type=float, default=0.0)
    parser.add_argument('--scale', type=float, default=1.0)
    parser.add_argument('--translate-x', type=float, default=0.0)
    parser.add_argument('--translate-y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--velocity', type=float, default=3.0)
    return parser.parse_args()


def main():
    args = parse_args()
    gps_points = read_trackpoints(args.input_gpx)
    local_points = project_points(gps_points, args.origin_lat, args.origin_lon)
    gazebo_points = transform_points(
        local_points,
        args.yaw_offset,
        args.scale,
        args.translate_x,
        args.translate_y,
    )
    write_waypoints(gazebo_points, args.output_csv, args.z, args.velocity)


if __name__ == '__main__':
    main()
