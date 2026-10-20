#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []  # add here (e.g. matplotlib) if tooling grows
# ///
"""Convert a path.jerryio.com LemLib-format export (.txt) into a generated header.

Usage:
    python3 motion-lib/tools/path2h.py paths/Path1.txt include/path-data.h
    python3 motion-lib/tools/path2h.py --list paths/Takes.jerryio.txt
    python3 motion-lib/tools/path2h.py --doc docs/paths.md paths/Takes.jerryio.txt include/path-data.h

Workflow: draw paths at https://path.jerryio.com (LemLib Tarball v0.5),
save the .txt into paths/, run this script, rebuild. Never edit the
generated header by hand.

--list mode: prints every drawing in the export with its generated C
symbol prefix, point count, start pose, and waypoint count, without
writing the header. Handy for discovering which path names to use in
code when one export holds several paths (see docs Tutorial 5).

Companion reference: the convert mode also writes a generated markdown
quick-reference with EVERY path's variable names and a copy-paste
init/follow block, so nobody has to read the header to know what to
call. Placement: next to main.cpp in ../src (VEXcode layout) when that
directory exists, else next to the header; override with --doc <file>.
Never hand-edit the header or the reference.

Start-heading guard: jerryio writes `heading: 0` on end-points that never
had a heading drawn, and a wrong start heading rotates the whole run. The
script compares the drawing's start heading with the path's departure
tangent (first end-point -> first control point); if they disagree by
more than HEADING_DISAGREE_DEG it prints a warning and emits a VERIFY
comment next to the generated `_start_h`.

Waypoint table: every drawn end-point (all segments, joint points deduped)
is emitted as a PathEndpoint table: position, the drawn facing (heading,
-1 + a VERIFY comment when the drawing never set one - jerryio's default
0 is NOT trusted), and pts_index, the matching row in the sample array.
Use it for controls AT the waypoint, e.g. stop there:
    drive.follow(path, path_len, 1.0, path_endpoints[i].pts_index);
"""
import json
import math
import os
import re
import sys

# Warn when the drawing's start heading and the path's departure tangent
# differ by more than this (degrees). 45 tolerates a normal "slight initial
# turn" while catching the 180-degree unset-default bug.
HEADING_DISAGREE_DEG = 45.0


def wrap180(a):
    """Wrap an angle to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def parse(txt):
    blocks = []  # (name, pts) in file order
    json_line = None
    name, pts = None, []
    for line in txt.splitlines():
        line = line.strip()
        if line.startswith("#PATH-POINTS-START"):
            if name is not None:
                blocks.append((name, pts))
            name = line[len("#PATH-POINTS-START"):].strip() or "path%d" % len(blocks)
            pts = []
        elif line == "endData":
            if name is not None:
                blocks.append((name, pts))
                name, pts = None, []
        elif line.startswith("#PATH.JERRYIO-DATA") and json_line is None:
            json_line = line[len("#PATH.JERRYIO-DATA"):].strip()
        elif not line or line.startswith("#"):
            continue
        else:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) == 3:
                try:
                    pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
                except ValueError:
                    pass
    if name is not None:  # file without endData
        blocks.append((name, pts))

    # Per drawing: robot heading at each path start (the end-point's
    # `heading` property), and the exact departure tangent (first end-point
    # -> first control point = the Bezier tangent at t=0). The JSON sits
    # AFTER endData. Compass convention: 0=+Y, 90=+X, clockwise.
    headings = {}
    tangents = {}
    waypoints = {}
    if json_line:
        try:
            for p in json.loads(json_line).get("paths", []):
                pname = p.get("name", "")
                segs = p.get("segments") or []
                # Waypoints: every end-point of every segment, in path order.
                # Segments share their joint end-point (same uid) - emit once.
                # `arrive` = arrival tangent of the segment ENDING at the point
                # (control[-2] -> control[-1]), to sanity-check drawn headings.
                eps, index = [], {}
                for s in segs:
                    cs = s.get("controls") or []
                    for ci, c in enumerate(cs):
                        if c.get("__type") != "end-point":
                            continue
                        key = str(c.get("uid")) if c.get("uid") else \
                            "%.3f,%.3f" % (float(c.get("x", 0.0)), float(c.get("y", 0.0)))
                        arrive = None
                        if ci == len(cs) - 1 and ci >= 1:
                            dx = float(cs[ci].get("x", 0.0)) - float(cs[ci - 1].get("x", 0.0))
                            dy = float(cs[ci].get("y", 0.0)) - float(cs[ci - 1].get("y", 0.0))
                            if dx or dy:
                                arrive = math.degrees(math.atan2(dx, dy))
                        if key in index:
                            if arrive is not None:
                                eps[index[key]]["arrive"] = arrive
                        else:
                            index[key] = len(eps)
                            eps.append({"x": float(c.get("x", 0.0)),
                                        "y": float(c.get("y", 0.0)),
                                        "heading": float(c["heading"]) if "heading" in c else None,
                                        "arrive": arrive})
                if eps:
                    waypoints[pname] = eps
                controls = (segs[0].get("controls") or []) if segs else []
                if not controls:
                    continue
                c0 = controls[0]
                if "heading" in c0:
                    headings[pname] = float(c0["heading"])
                if len(controls) >= 2:
                    c1 = controls[1]
                    dx = float(c1.get("x", c0.get("x", 0.0))) - float(c0.get("x", 0.0))
                    dy = float(c1.get("y", c0.get("y", 0.0))) - float(c0.get("y", 0.0))
                    if dx or dy:
                        tangents[pname] = math.degrees(math.atan2(dx, dy))
        except (ValueError, KeyError, TypeError, IndexError):
            m = re.search(r'"heading"\s*:\s*(-?[\d.]+)', json_line)
            if m and blocks:
                headings[blocks[0][0]] = float(m.group(1))

    return [(n, p, headings.get(n), tangents.get(n), waypoints.get(n) or [])
            for n, p in blocks]


def cname(s):
    n = re.sub(r"[^0-9a-zA-Z_]+", "_", s).strip("_").lower()
    return n or "path"


def unique_cname(name, used):
    """cname(), deduplicated across paths in one file (suffix _)."""
    arr = cname(name)
    while arr in used:
        arr += "_"
    used.add(arr)
    return arr


def dedupe(pts):
    """Drop consecutive duplicate points (segment-boundary artifact)."""
    return [p for i, p in enumerate(pts) if i == 0 or
            abs(p[0] - pts[i - 1][0]) + abs(p[1] - pts[i - 1][1]) > 1e-9]


def departure_tangent(dedup):
    """Compass heading of the sampled path's initial direction, or None."""
    if len(dedup) < 2:
        return None
    dx, dy = dedup[1][0] - dedup[0][0], dedup[1][1] - dedup[0][1]
    if dx or dy:
        return math.degrees(math.atan2(dx, dy))  # compass: 0=+Y, 90=+X
    return None


def endpoint_rows(dedup, waypts):
    """Match each drawn waypoint to its nearest sample row; collect VERIFY notes."""
    rows = []
    for w in waypts:
        pi, best = -1, 1e9
        for i, (sx, sy, _s) in enumerate(dedup):
            d = math.hypot(sx - w["x"], sy - w["y"])
            if d < best:
                best, pi = d, i
        if best > 1.0:
            pi = -1
        note = []
        if w["heading"] is None:
            note.append("VERIFY: heading not drawn")
        elif w.get("arrive") is not None and \
                abs(wrap180(w["heading"] - w["arrive"])) > HEADING_DISAGREE_DEG:
            note.append("VERIFY: drawn heading %g vs arrival tangent %g"
                        % (w["heading"], w["arrive"]))
        if pi < 0:
            note.append("no sample row within 1 in")
        h = "%g" % w["heading"] if w["heading"] is not None else "-1"
        rows.append((w["x"], w["y"], h, pi, "; ".join(note)))
    return rows


def emit(paths):
    out = [
        "#pragma once",
        "// Generated by motion-lib/tools/path2h.py from path.jerryio.com LemLib exports. DO NOT EDIT.",
        "// Units: inches; speed column = motor % (0-100).",
        '#include "MotionLib/Pursuit.hpp"',
        "",
    ]
    used = set()
    for name, pts, start_heading, json_tangent, waypts in paths:
        dedup = dedupe(pts)
        arr = unique_cname(name, used)
        out.append('// Path "%s" - %d points' % (name, len(dedup)))
        out.append("static const PathPoint %s[] = {" % arr)
        for x, y, s in dedup:
            out.append("  {%g, %g, %g}," % (x, y, s))
        out.append("};")
        out.append("static const int %s_len = %d;" % (arr, len(dedup)))
        # Start pose for motion::Drive::init(): first point + robot heading there.
        # Heading = drawn heading from the JSON, else the departure tangent
        # (exact Bezier tangent from the drawing's control points when
        # present, else the first-segment tangent of the sampled points).
        tangent = json_tangent if json_tangent is not None \
            else departure_tangent(dedup)
        if start_heading is None:
            start_heading = tangent
        if len(dedup) >= 2 and start_heading is not None:
            out.append("static const double %s_start_x = %g;" % (arr, dedup[0][0]))
            out.append("static const double %s_start_y = %g;" % (arr, dedup[0][1]))
            out.append("static const double %s_start_h = %g;" % (arr, start_heading))
            # Guard: jerryio defaults unset end-point headings to 0. If the
            # drawing's heading disagrees with where the path actually goes,
            # flag it - a wrong start heading rotates the whole run.
            if tangent is not None:
                delta = abs(wrap180(start_heading - tangent))
                if delta > HEADING_DISAGREE_DEG:
                    out.append("// VERIFY: drawing start heading %g but the path departs"
                               " at %g (%g deg apart)." % (start_heading, tangent, delta))
                    out.append("// A wrong start heading rotates the whole run - confirm"
                               " the robot's physical start facing (AGENT.md frame rules).")
                    print('WARNING: path "%s": start heading %g disagrees with the'
                          " departure tangent %g by %g deg"
                          % (name, start_heading, tangent, delta), file=sys.stderr)
                    print("  jerryio defaults unset end-point headings to 0 - if that"
                          " is what happened here, draw the true heading in jerryio"
                          " or pass it to drive.init() explicitly.", file=sys.stderr)
        # Waypoints (drawn end-points): poses to arrive at / steer actions by.
        # heading: the drawn facing (same compass frame); -1 + VERIFY when the
        # drawing never set one (jerryio's default 0 is NOT trusted here).
        # pts_index: matching row in the sample array above (within 1 in -
        # waypoints between lattice rows link to the nearest row; stopping
        # precision stays the follower's usual ~2 in),
        # so follow(pts, len, scale, pts_index) finishes at this waypoint;
        # -1 when no row matches.
        if waypts:
            out.append("// Waypoints (drawn end-points) - where the robot should BE at")
            out.append("// the ends of meaningful segments; steer endpoint actions by these.")
            out.append("// pts_index: row in %s[] above; pass it to follow(..., last_pt)" % arr)
            out.append("//   to finish at a waypoint, act, then follow the next stretch.")
            out.append("static const PathEndpoint %s_endpoints[] = {" % arr)
            for x, y, h, pi, note in endpoint_rows(dedup, waypts):
                row = "  {%g, %g, %s, %d}," % (x, y, h, pi)
                if note:
                    row += "  // " + note
                out.append(row)
            out.append("};")
            out.append("static const int %s_endpoints_len = %d;" % (arr, len(waypts)))
        out.append("")
    return "\n".join(out)


def doc_md(paths, source, header_name):
    """Generated quick-reference for the header (written alongside it)."""
    used = set()
    infos = []
    for name, pts, start_heading, json_tangent, waypts in paths:
        dedup = dedupe(pts)
        arr = unique_cname(name, used)
        tangent = json_tangent if json_tangent is not None \
            else departure_tangent(dedup)
        h = start_heading if start_heading is not None else tangent
        infos.append((name, arr, dedup, h, tangent, waypts))
    out = [
        "# Path quick reference",
        "",
        "Generated by `motion-lib/tools/path2h.py` from `%s` — DO NOT EDIT." % source,
        "Regenerate: `python3 motion-lib/tools/path2h.py %s %s`" % (source, header_name),
        "",
        "**Copy the block for the path you want into your code.** Place the"
        " robot at that path's start pose first — it runs right after download.",
        "",
    ]
    for name, arr, dedup, h, tangent, waypts in infos:
        start = ""
        if dedup and h is not None:
            start = " — start at (%g, %g) facing %s" % (dedup[0][0], dedup[0][1], "%g" % h)
        out.append("### %s — `%s`%s" % (name, arr, start))
        out.append("")
        out.append("```cpp")
        out.append("drive.init(%s_start_x, %s_start_y, %s_start_h);" % (arr, arr, arr))
        out.append("drive.follow(%s, %s_len, 1.0);   // 1.0 = full speed, 0.5 = half" % (arr, arr))
        out.append("```")
        if h is not None and tangent is not None and \
                abs(wrap180(h - tangent)) > HEADING_DISAGREE_DEG:
            out.append("")
            out.append("VERIFY: start heading disagrees with the path's departure"
                       " direction (%g) — check the robot's facing before running." % tangent)
        out.append("")
    out += ["---", "", "## All generated variable names", "",
            "| Drawing | Prefix | Variables |", "| --- | --- | --- |"]
    for name, arr, dedup, h, tangent, waypts in infos:
        fam = ", ".join("`%s`" % v for v in (
            arr, arr + "_len", arr + "_start_x", arr + "_start_y",
            arr + "_start_h", arr + "_endpoints", arr + "_endpoints_len"))
        out.append("| %s | `%s` | %s |" % (name, arr, fam))
    out += ["", "## Waypoints", ""]
    for name, arr, dedup, h, tangent, waypts in infos:
        if not waypts:
            continue
        out.append("**%s** — `%s_endpoints[]` (stop at a waypoint with"
                   " `drive.follow(%s, %s_len, 1.0, %s_endpoints[i].pts_index)`):"
                   % (name, arr, arr, arr, arr))
        out.append("")
        out.append("| i | x | y | heading | pts_index | notes |")
        out.append("| --- | --- | --- | --- | --- | --- |")
        for i, (x, y, hh, pi, note) in enumerate(endpoint_rows(dedup, waypts)):
            out.append("| %d | %g | %g | %s | %d | %s |" % (i, x, y, hh, pi, note))
        out.append("")
    return "\n".join(out)


def list_paths(paths, source):
    """Print a symbol-prefix table for every drawing (no header written)."""
    print("paths in %s:\n" % source)
    print("%-28s %4s  %-26s %4s  %s" %
          ("C symbol prefix", "pts", "start (x, y) @ heading", "wps", "drawing"))
    used = set()
    for name, pts, start_heading, json_tangent, waypts in paths:
        dedup = dedupe(pts)
        arr = unique_cname(name, used)
        tangent = json_tangent if json_tangent is not None \
            else departure_tangent(dedup)
        h = start_heading if start_heading is not None else tangent
        if not dedup:
            print("%-28s %4d  %-26s %4d  \"%s\"  (no points)"
                  % (arr, 0, "-", 0, name))
            continue
        start = "(%g, %g) @ %s" % (dedup[0][0], dedup[0][1],
                                   ("%g" % h) if h is not None else "?")
        note = ""
        if h is not None and tangent is not None and \
                abs(wrap180(h - tangent)) > HEADING_DISAGREE_DEG:
            note = "   <- VERIFY start heading (departs at %g)" % tangent
        print("%-28s %4d  %-26s %4d  \"%s\"%s"
              % (arr, len(dedup), start, len(waypts), name, note))
    print("""

Every path generates this symbol family (prefix = C symbol above):
  <prefix>[]              sampled PathPoint array
  <prefix>_len            row count
  <prefix>_start_x/_y/_h  start pose for drive.init()
  <prefix>_endpoints[]    waypoint table (see header comments)
  <prefix>_endpoints_len

e.g. in code:
  drive.init(<prefix>_start_x, <prefix>_start_y, <prefix>_start_h);
  drive.follow(<prefix>, <prefix>_len, 0.5);
""")


if __name__ == "__main__":
    argv = sys.argv[1:]
    md = None
    if len(argv) == 2 and argv[0] == "--list":
        with open(argv[1]) as f:
            txt = f.read()
        list_paths(parse(txt), argv[1])
        sys.exit(0)
    if len(argv) == 3 and argv[0] == "--doc":
        md, argv = argv[1], argv[2:]
    if len(argv) != 2:
        print(__doc__)
        sys.exit(1)
    with open(argv[0]) as f:
        txt = f.read()
    paths = parse(txt)
    with open(argv[1], "w") as f:
        f.write(emit(paths))
    print("wrote %s" % argv[1])
    # Reference file: default location is next to main.cpp in ../src
    # (VEXcode layout - that is where students look), falling back to
    # next to the header. --doc <file> overrides.
    if md is None:
        src_dir = os.path.join(os.path.dirname(os.path.abspath(argv[1])),
                               os.pardir, "src")
        if os.path.isdir(src_dir):
            md = os.path.join(src_dir, "path-data.md")
        else:
            md = (argv[1][:-2] if argv[1].endswith(".h") else argv[1]) + ".md"
    with open(md, "w") as f:
        f.write(doc_md(paths, argv[0], argv[1]))
    print("wrote %s (reference)" % md)
