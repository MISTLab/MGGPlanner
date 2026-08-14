#!/usr/bin/env python3
"""
Convert a ROS 1 MGGPlanner config to a ROS 2 parameter file.

Three things have to change.

1. ROS 1's rosparam evaluates ``rad(...)`` and ``deg(...)`` expressions while
   loading YAML, so a config could write ``fov: [rad(2.0*pi), rad(pi/6)]`` and
   the planner would read two doubles. ROS 2's parameter loader has no such
   feature and would either reject the value or hand the node a string. Every
   such expression is evaluated here, so ``rad(2.0*pi)`` becomes
   ``6.283185307179586``.

2. ROS 2 requires the ``<node_name>: ros__parameters:`` wrapper.

3. Nested maps survive: ROS 2 flattens them to dotted parameter names, which
   is what the loader in mgg_ros expects (``PlanningParams.edge_length_max``).

Usage:
    convert_config.py <ros1.yaml> <ros2.yaml> [node_name]
"""
import math
import pathlib
import re
import sys

import yaml

# Only what the shipped configs actually use inside rad()/deg().
_EVAL_ENV = {
    "__builtins__": {},
    "pi": math.pi,
    "e": math.e,
    "sqrt": math.sqrt,
}

_ANGLE_RE = re.compile(r"^\s*(rad|deg)\s*\((.*)\)\s*$", re.S)


def _eval_angle(text):
    """Returns the value in radians, or None if `text` is not an angle call."""
    m = _ANGLE_RE.match(text)
    if not m:
        return None
    kind, expr = m.group(1), m.group(2)
    if not re.fullmatch(r"[0-9eE+\-*/(). \tpisqrt]*", expr):
        raise ValueError(f"refusing to evaluate suspicious expression: {expr!r}")
    value = float(eval(expr, _EVAL_ENV))  # noqa: S307 - guarded above
    return value if kind == "rad" else math.radians(value)


def convert(node):
    """Walks the loaded YAML, replacing angle expressions with numbers."""
    if isinstance(node, dict):
        return {k: convert(v) for k, v in node.items()}
    if isinstance(node, list):
        return [convert(v) for v in node]
    if isinstance(node, str):
        angle = _eval_angle(node)
        return angle if angle is not None else node
    return node


def rename_grid_resolution(params):
    """GridGraphLocal/min_extension is the grid resolution, not a bound
    extension: rrg.cpp:3216 assigns it to grid_graph_res_val_ and the YAML
    marks it only with a "# Resolution" comment. mgg_core's GridGraphParams has
    a real `resolution` field, so rename it on the way across. Leaving the old
    name would preserve an overload that makes zeroing the bound extensions
    silently disable local exploration."""
    grid = params.get("BoundedSpaceParams", {}).get("GridGraphLocal")
    if isinstance(grid, dict) and "min_extension" in grid:
        grid["resolution"] = grid.pop("min_extension")
        print("  GridGraphLocal: min_extension -> resolution")
    return params


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    src, dst = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    node_name = sys.argv[3] if len(sys.argv) > 3 else "mggplanner_node"

    raw = yaml.safe_load(src.read_text())
    converted = rename_grid_resolution(convert(raw))

    out = {node_name: {"ros__parameters": converted}}
    header = (
        f"# Generated from {src.name} by tools/convert_config.py.\n"
        "#\n"
        "# rad()/deg() expressions have been evaluated: ROS 1's rosparam did\n"
        "# that at load time, ROS 2's parameter loader does not.\n"
    )
    dst.write_text(header + yaml.safe_dump(out, default_flow_style=False,
                                           sort_keys=False))
    print(f"{src} -> {dst}  (node: {node_name})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
