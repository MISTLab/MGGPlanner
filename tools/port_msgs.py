#!/usr/bin/env python3
"""
Port planner_msgs + planner_semantic_msgs to ROS 2 as mgg_msgs.

Fixes exactly what rosidl_adapter rejected (verified against the real parser
from ros:jazzy-perception: 15 of 43 definitions parsed, 28 failed):

  1. '-------' (7 dashes) as the service separator -> '---'
  2. a .srv with NO separator at all (pci_to_waypoint.srv). ROS 1 accepted it
     and treated the whole file as the request with an empty response; the
     handler does read req.waypoint (planner_control_interface.cpp:270), so a
     trailing '---' preserves the behaviour. Its "Return best path" comment is
     a copy-paste leftover from another service.
  3. snake_case .srv filenames -> CamelCase (ROS 2 derives the type name from
     the filename, so planner_set_vel.srv yielded 'planner_set_vel_Request')
  4. 'kFooBar' constants -> 'FOO_BAR' (must match ^[A-Z]([A-Z0-9_]?[A-Z0-9]+)*$)
  5. pathFollowerAction.action -> PathFollower.action

Also drops the six vendored copies of Swarm-SLAM's cslam_common_interfaces
messages: they are identical to upstream, referenced by zero lines of code,
and the port takes that dependency properly in mgg_cslam instead.

Usage: port_msgs.py <repo-root> <ros2/src/mgg_msgs>
"""
import pathlib
import re
import sys

CSLAM_VENDORED = {
    "PoseGraph", "PoseGraphValue", "PoseGraphEdge",
    "MultiRobotKey", "InterRobotLoopClosure", "IntraRobotLoopClosure",
}

CONST_RE = re.compile(
    r"^(\s*(?:u?int(?:8|16|32|64)|float(?:32|64)|bool|string)\s+)"
    r"(k[A-Za-z0-9]+)(\s*=)", re.M)


def camel(stem):
    """planner_set_vel -> PlannerSetVel"""
    return "".join(p[:1].upper() + p[1:] for p in stem.split("_") if p)


def const_name(k):
    """kExtendedBound -> EXTENDED_BOUND"""
    body = k[1:] if len(k) > 1 and k[0] == "k" and k[1].isupper() else k
    return re.sub(r"(?<!^)(?=[A-Z])", "_", body).upper()


def fix_body(text):
    renamed = set()

    def sub(m):
        new = const_name(m.group(2))
        renamed.add((m.group(2), new))
        return f"{m.group(1)}{new}{m.group(3)}"

    text = CONST_RE.sub(sub, text)
    text = re.sub(r"^-{3,}[ \t]*$", "---", text, flags=re.M)
    # Bare "Header" must be qualified. rosidl_adapter's *parser* accepts it
    # (so a parse-only check passes), but the type-description generator then
    # resolves it against the current package and dies looking for
    # mgg_msgs/msg/Header.json. ROS 1 special-cased the bare name; ROS 2 does
    # not.
    text = re.sub(r"^(\s*)Header(\s+\w+)", r"\1std_msgs/Header\2", text, flags=re.M)
    # Both source packages merge into mgg_msgs, so same-package references
    # become unqualified.
    text = text.replace("planner_msgs/", "").replace("planner_semantic_msgs/", "")
    return text, renamed


def fix_srv(text):
    text, renamed = fix_body(text)
    if not re.search(r"^---$", text, flags=re.M):
        if not text.endswith("\n"):
            text += "\n"
        text += "---\n"
    return text, renamed


def main():
    src, dst = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    for sub in ("msg", "srv", "action"):
        (dst / sub).mkdir(parents=True, exist_ok=True)

    renamed = set()
    n = {"msg": 0, "srv": 0, "action": 0, "dropped": 0, "sep_added": 0}

    for pkg in ("planner_msgs", "planner_semantic_msgs"):
        for f in sorted((src / pkg / "msg").glob("*.msg")):
            if f.stem in CSLAM_VENDORED:
                n["dropped"] += 1
                continue
            body, r = fix_body(f.read_text())
            renamed |= r
            (dst / "msg" / f.name).write_text(body)
            n["msg"] += 1

        for f in sorted((src / pkg / "srv").glob("*.srv")):
            original = f.read_text()
            body, r = fix_srv(original)
            if not re.search(r"^-{3,}[ \t]*$", original, flags=re.M):
                n["sep_added"] += 1
                print(f"  note: {f.name} had no separator; appended '---'")
            renamed |= r
            (dst / "srv" / f"{camel(f.stem)}.srv").write_text(body)
            n["srv"] += 1

        for f in sorted((src / pkg / "action").glob("*.action")):
            body, r = fix_body(f.read_text())
            renamed |= r
            stem = camel(f.stem)
            if stem.endswith("Action"):
                stem = stem[: -len("Action")]
            (dst / "action" / f"{stem}.action").write_text(body)
            n["action"] += 1

    print(f"\nmsg={n['msg']} srv={n['srv']} action={n['action']} "
          f"dropped_cslam={n['dropped']} separators_added={n['sep_added']}")
    print("constant renames:")
    for old, new in sorted(renamed):
        print(f"  {old:<24} -> {new}")


if __name__ == "__main__":
    main()
