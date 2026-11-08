#!/usr/bin/env python3
"""Expand a Xacro file and emit comment-free URDF XML.

gazebo_ros2_control 0.4.x forwards ``robot_description`` through an rcl
``--param`` argument.  Colons in XML comments make that parser reject the
whole URDF.  ElementTree intentionally discards comments while parsing, so
the output remains semantically identical but safe for the plugin.
"""

import sys
import xml.etree.ElementTree as element_tree

import xacro


def parse_mappings(arguments):
    mappings = {}
    for argument in arguments:
        if ':=' not in argument:
            raise ValueError(f'Expected a xacro mapping, got {argument!r}')
        name, value = argument.split(':=', 1)
        mappings[name] = value
    return mappings


def main():
    if len(sys.argv) < 2:
        raise SystemExit(f'Usage: {sys.argv[0]} XACRO_FILE [name:=value ...]')

    document = xacro.process_file(
        sys.argv[1],
        mappings=parse_mappings(sys.argv[2:]),
    )
    root = element_tree.fromstring(document.toxml())
    sys.stdout.write(element_tree.tostring(root, encoding='unicode'))


if __name__ == '__main__':
    main()
