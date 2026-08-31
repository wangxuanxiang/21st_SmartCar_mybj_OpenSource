import ast
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle


FIELD_W = 320.0
FIELD_H = 240.0
SAFE_MARGIN = 13.0
OBSTACLE_R = 16.0
OBJECT_SIZE = {
    "T": (4.0, 4.0),
    "S": (3.0, 3.0),
    "E": (3.0, 3.0),
    "B": (2.0, 2.0),
    "W": (2.0, 2.0),
}
OBJECT_COLORS = {
    "T": "limegreen",
    "S": "red",
    "E": "royalblue",
    "B": "saddlebrown",
    "W": "white",
}


def parse_config(path):
    text = path.read_text(encoding="utf-8", errors="ignore")
    data = {}
    for name in ("cube_obstacles", "circle"):
        match = re.search(rf"^\s*{name}\s*=\s*\"([^\"]*)\"", text, re.M)
        if not match:
            data[name] = []
            continue
        value = match.group(1).strip()
        if not value:
            data[name] = []
            continue
        parsed = ast.literal_eval(f"[{value}]")
        data[name] = [tuple(item) for item in parsed]
    return data


def parse_wire_text(text):
    obj_match = re.search(r"(?:^|\n)\s*(1?\{.*?\}|\[.*?\])\s*(?:target|$)", text, re.S)
    target_match = re.search(r"target(\[.*?\])\s*(?:path|score|$)", text, re.S)
    path_match = re.search(r"path(\[.*?\])\s*(?:score|$)", text, re.S)
    score_match = re.search(r"score(\[.*?\])\s*$", text, re.S)
    if not obj_match:
        raise ValueError("没有找到 1{...} 物体数据")
    object_text = obj_match.group(1)
    if object_text.startswith("1{"):
        object_text = object_text[1:]
    raw_objects = ast.literal_eval(object_text)
    if isinstance(raw_objects, dict):
        objects = raw_objects
    elif isinstance(raw_objects, list):
        objects = {kind: [] for kind in OBJECT_SIZE}
        for item in raw_objects:
            if not isinstance(item, (list, tuple)) or len(item) != 3:
                raise ValueError("each object must be (kind, x, y)")
            kind, x, y = item
            if kind not in objects:
                raise ValueError(f"unsupported object kind: {kind}")
            objects[kind].append((float(x), float(y)))
    else:
        raise ValueError("object data must be a dict or a list")
    targets = ast.literal_eval(target_match.group(1)) if target_match else []
    path = ast.literal_eval(path_match.group(1)) if path_match else []
    scores = ast.literal_eval(score_match.group(1)) if score_match else []
    if scores and len(scores) != len(targets):
        raise ValueError("score count must match target count")
    return objects, targets, path, scores


def make_rect(cx, cy, half_w, half_h):
    return [
        (cx - half_w, cy - half_h),
        (cx + half_w, cy - half_h),
        (cx + half_w, cy + half_h),
        (cx - half_w, cy + half_h),
    ]


def create_expanded_cube(cx, cy, width, height):
    return make_rect(cx, cy, width / 2.0 + SAFE_MARGIN, height / 2.0 + SAFE_MARGIN)


def forward_right(direction):
    direction = int(direction)
    if direction == 0:
        return (0.0, 1.0), (1.0, 0.0)
    if direction == 90:
        return (1.0, 0.0), (0.0, -1.0)
    if direction == 180:
        return (0.0, -1.0), (-1.0, 0.0)
    if direction == -90:
        return (-1.0, 0.0), (0.0, 1.0)
    raise ValueError("direction must be one of -90, 0, 90, 180")


def swell_rect(rect, swell_angle, direction=None):
    swell_size = 10.0 if swell_angle in (1, -1) else 20.0
    out = []
    if swell_angle in (1, -1):
        if direction is None:
            direction = -90
        _, right = forward_right(direction)
        cx = sum(p[0] for p in rect) / len(rect)
        cy = sum(p[1] for p in rect) / len(rect)
    for p in rect:
        x, y = float(p[0]), float(p[1])
        if swell_angle == -90:
            if x < rect[0][0] + 0.001:
                x -= swell_size
        elif swell_angle == 0:
            if y > rect[0][1] + 0.001:
                y += swell_size
        elif swell_angle == 90:
            if x > rect[0][0] + 0.001:
                x += swell_size
        elif swell_angle == 180:
            if y < rect[2][1] - 0.001:
                y -= swell_size
        elif swell_angle in (1, -1):
            side = (x - cx) * right[0] + (y - cy) * right[1]
            if side > 0.001:
                x += right[0] * swell_size
                y += right[1] * swell_size
            elif side < -0.001:
                x -= right[0] * swell_size
                y -= right[1] * swell_size
        out.append((x, y))
    return out


def object_rects(objects):
    rects = []
    for kind, points in objects.items():
        size = OBJECT_SIZE.get(kind)
        if not size:
            continue
        for x, y in points:
            rects.append((kind, make_rect(x, y, size[0] / 2.0 + SAFE_MARGIN, size[1] / 2.0 + SAFE_MARGIN)))
    return rects


def circle_rects(circles):
    return [("circle", make_rect(c[0], c[1], OBSTACLE_R + SAFE_MARGIN, OBSTACLE_R + SAFE_MARGIN)) for c in circles]


def cube_rects(cubes):
    return [("cube", create_expanded_cube(c[0], c[1], c[2], c[3])) for c in cubes]


def push_boundary_point(kind, point):
    x, y = float(point[0]), float(point[1])
    if kind == "T":
        return [x, FIELD_H]
    if kind in ("S", "E"):
        return [0.0, y]
    if kind in ("B", "W"):
        return [FIELD_W, y]
    return [x, y]


def is_boundary_point(kind, point):
    x, y = float(point[0]), float(point[1])
    eps = 0.001
    if kind == "T":
        return abs(y - FIELD_H) < eps
    if kind in ("S", "E"):
        return abs(x - 0.0) < eps
    if kind in ("B", "W"):
        return abs(x - FIELD_W) < eps
    return False


def target_push_path(target, path_point):
    start = [float(target[2]), float(target[3])]
    mid = [float(path_point[0]), float(path_point[1])]
    if is_boundary_point(target[1], mid):
        return [start, mid]
    return [start, mid, push_boundary_point(target[1], mid)]


def draw_scene(objects, targets, path, scores, config, output, swell_angle=None, direction=-90):
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-10, FIELD_W + 10)
    ax.set_ylim(-10, FIELD_H + 10)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.25)
    ax.add_patch(Rectangle((0, 0), FIELD_W, FIELD_H, fill=False, edgecolor="black", linewidth=1.5))

    for cube in config["cube_obstacles"]:
        ax.add_patch(Rectangle((cube[0] - cube[2] / 2.0, cube[1] - cube[3] / 2.0), cube[2], cube[3],
                               fill=False, edgecolor="black", linestyle="--", linewidth=1.2))
    for circ in config["circle"]:
        ax.add_patch(Circle((circ[0], circ[1]), OBSTACLE_R, fill=False, edgecolor="black", linestyle="--", linewidth=1.2))

    if swell_angle is not None:
        rect_items = object_rects(objects) + circle_rects(config["circle"]) + cube_rects(config["cube_obstacles"])
        for label, rect in rect_items:
            swelled = swell_rect(rect, swell_angle, direction)
            ax.add_patch(Polygon(swelled, closed=True, fill=True, alpha=0.12, edgecolor="orange", facecolor="orange"))

    for kind, points in objects.items():
        color = OBJECT_COLORS.get(kind, "gray")
        for x, y in points:
            edge = "black" if kind == "W" else color
            ax.scatter([x], [y], s=70, c=color, edgecolors=edge, zorder=4)
            ax.text(x + 2, y + 2, f"{kind}({x:.1f},{y:.1f})", fontsize=8, color="black")

    for idx, target in enumerate(targets):
        if len(target) >= 4:
            ax.scatter([target[2]], [target[3]], s=120, c="black", marker="x", linewidths=2.5, zorder=5)
            label = f"target {target[1]}#{target[0]}"
            if scores:
                label += f" score={scores[idx]:.1f}"
            ax.text(target[2] + 3, target[3] - 5, label, fontsize=9, color="black")

    if path:
        if targets and scores:
            # A score above 10000 means this target has no generated path point.
            path_targets = [target for target, score in zip(targets, scores) if score <= 10000]
        else:
            path_targets = targets

        if path_targets and len(path) == len(path_targets):
            for idx, (target, path_point) in enumerate(zip(path_targets, path)):
                route = target_push_path(target, path_point)
                xs = [p[0] for p in route]
                ys = [p[1] for p in route]
                label = "push path" if idx == 0 else None
                ax.plot(xs, ys, color="magenta", linewidth=2.0, marker="o", label=label, zorder=3)
                ax.text(path_point[0] + 2, path_point[1] + 2, f"{target[1]} p1", fontsize=8, color="magenta")
        else:
            xs = [p[0] for p in path]
            ys = [p[1] for p in path]
            ax.plot(xs, ys, color="magenta", linewidth=2.2, marker="o", label="path", zorder=3)
            for idx, p in enumerate(path):
                ax.text(p[0] + 2, p[1] + 2, f"p{idx}", fontsize=8, color="magenta")

    title = "Boundary Debug"
    if swell_angle is not None:
        title += f" | swell={swell_angle}, direction={direction}"
    ax.set_title(title)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output, dpi=180)
    plt.close(fig)


def read_multiline():
    print("Paste wireless serial data. Submit an empty line to finish.")
    print("Enter -100 on an empty input cycle to return to mode selection, or -1 to exit.")
    lines = []
    while True:
        try:
            line = input("> " if not lines else "")
        except EOFError:
            return "-1"
        if not line.strip():
            break
        if line.strip().startswith("Paste wireless serial data"):
            continue
        lines.append(line)
        if line.strip().startswith("score"):
            try:
                parse_wire_text("\n".join(lines))
                break
            except Exception:
                pass
    return "\n".join(lines)


def read_scene_data():
    """Read one wireless payload and keep asking until it is valid or cancelled."""
    while True:
        text = read_multiline().strip()
        if text in ("-100", "-1"):
            return text, None
        if not text:
            print("No wireless data received. Please try again.")
            continue
        try:
            return None, parse_wire_text(text)
        except (SyntaxError, ValueError, TypeError) as exc:
            print(f"Invalid wireless data: {exc}")
            print("Please paste the complete objects/target/path data again.")


def select_swell_angle():
    while True:
        raw = input("Enter swell type (-90/90/0/180/-1/1), -100 for mode selection, or -1 to exit: ").strip()
        if raw in ("-100", "-1"):
            return raw, None, None
        try:
            swell_angle = int(raw)
        except ValueError:
            print("Please enter an integer.")
            continue
        if swell_angle not in (-90, 90, 0, 180, -1, 1):
            print("Supported swell types: -90, 90, 0, 180, -1, 1.")
            continue

        direction = -90
        if swell_angle in (-1, 1):
            direction_raw = input("Enter push direction (-90/0/90/180), Enter for -90, -100 for mode selection, or -1 to exit: ").strip()
            if direction_raw in ("-100", "-1"):
                return direction_raw, None, None
            if direction_raw:
                try:
                    direction = int(direction_raw)
                except ValueError:
                    print("Push direction must be an integer.")
                    continue
                if direction not in (-90, 0, 90, 180):
                    print("Supported push directions: -90, 0, 90, 180.")
                    continue
        return None, swell_angle, direction


def run_swell_mode(config, out_dir):
    command, scene_data = read_scene_data()
    if command:
        return command

    objects, targets, path, scores = scene_data
    base_output = out_dir / "scene_base.png"
    draw_scene(objects, targets, path, scores, config, base_output)
    print(f"Base image generated: {base_output}")

    while True:
        command, swell_angle, direction = select_swell_angle()
        if command:
            return command
        output = out_dir / f"scene_swell_{swell_angle}_dir_{direction}.png"
        draw_scene(objects, targets, path, scores, config, output, swell_angle=swell_angle, direction=direction)
        print(f"Swell image generated: {output}")


def run_continuous_base_mode(config, out_dir):
    print("Continuous base display mode.")
    while True:
        command, scene_data = read_scene_data()
        if command:
            return command
        objects, targets, path, scores = scene_data
        base_output = out_dir / "scene_base.png"
        draw_scene(objects, targets, path, scores, config, base_output)
        print(f"Base image generated: {base_output}")


def main():
    root = Path(__file__).resolve().parents[1]
    config_path = root / "main_car" / "main_config.txt"
    out_dir = root / "boundary_debug_outputs"
    out_dir.mkdir(exist_ok=True)

    config = parse_config(config_path)

    while True:
        mode = input("Select mode: 0 = swell mode, 1 = continuous base display, -1 = exit: ").strip()
        if mode == "-1":
            break
        if mode == "0":
            command = run_swell_mode(config, out_dir)
        elif mode == "1":
            command = run_continuous_base_mode(config, out_dir)
        else:
            print("Please enter 0, 1, or -1.")
            continue
        if command == "-1":
            break


if __name__ == "__main__":
    main()
