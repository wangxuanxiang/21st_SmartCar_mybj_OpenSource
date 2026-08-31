import argparse

from boundary_path_plan import BoundaryPathPlanner


def draw_result(planner, path, objects, circles, out_file):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_xlim(-10, planner.field_x + 10)
    ax.set_ylim(-10, planner.field_y + 10)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Boundary Path Plan")

    ax.add_patch(Rectangle((0, 0), planner.field_x, planner.field_y,
                           fill=False, edgecolor="black", linewidth=2))

    rects = planner._build_rects(objects, circles)
    for rect in rects:
        xs = [p[0] for p in rect]
        ys = [p[1] for p in rect]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        ax.add_patch(Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                               facecolor="#f4a261", edgecolor="#b45309",
                               alpha=0.35, linewidth=1.5))

    for obj in objects:
        cx, cy, length, width = obj
        ax.add_patch(Rectangle((cx - length / 2.0, cy - width / 2.0),
                               length, width, fill=False,
                               edgecolor="#7c2d12", linestyle="--"))

    for c in circles:
        cx, cy = c
        ax.scatter([cx], [cy], color="#dc2626", s=40)

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, "-o", color="#2563eb", linewidth=2.5)
        ax.scatter([xs[0]], [ys[0]], color="#16a34a", s=80, label="start")
        ax.scatter([xs[-1]], [ys[-1]], color="#2563eb", s=80, label="end")
        for i, p in enumerate(path):
            ax.text(p[0] + 2, p[1] + 2, str(i), color="#1d4ed8")
    else:
        ax.text(20, 20, "No valid path", color="red", fontsize=16)

    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    print("path =", path)
    print("saved =", out_file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=40.0)
    parser.add_argument("--y", type=float, default=80.0)
    parser.add_argument("--dir", type=int, default=0,
                        choices=[-90, 0, 90, 180])
    parser.add_argument("--avoid-dir", type=int, default=1,
                        choices=[-1, 1])
    parser.add_argument("--out", default="boundary_path_plan_result.png")
    args = parser.parse_args()

    objects = [
        (110.0, 120.0, 35.0, 35.0),
        (170.0, 210.0, 45.0, 28.0),
        (100.0, 170.0, 15.0, 35.0),
        (70.0, 50.0, 25.0, 8.0),
    ]
    circles = [
        (80.0, 170.0),
    ]

    planner = BoundaryPathPlanner()
    path = planner.plan_one_turn(args.x, args.y, args.dir, args.avoid_dir,
                                 objects=objects, circles=circles)
    draw_result(planner, path, objects, circles, args.out)


if __name__ == "__main__":
    main()
