import argparse
import random


OBJECT_GROUPS = (("T",), ("E", "S"), ("B", "W"))


def generate_random_grid(object_count, seed=None):
    """Return [(slot, kind), ...] with at most one object per slot."""
    if not 0 <= object_count <= 9:
        raise ValueError("object_count must be between 0 and 9")

    rng = random.Random(seed)
    slots = rng.sample(range(1, 10), object_count)
    groups = [rng.randrange(len(OBJECT_GROUPS)) for _ in range(object_count)]
    if object_count >= 2 and len(set(groups)) == 1:
        # Ensure a test layout is not all bears, all tennis balls, or all bags.
        groups[-1] = (groups[-1] + rng.randrange(1, len(OBJECT_GROUPS))) % len(OBJECT_GROUPS)

    assignments = []
    for slot, group_idx in zip(sorted(slots), groups):
        assignments.append((slot, rng.choice(OBJECT_GROUPS[group_idx])))
    return assignments


def format_grid(assignments):
    by_slot = {slot: kind for slot, kind in assignments}
    rows = []
    for row in range(3):
        cells = []
        for col in range(3):
            slot = row * 3 + col + 1
            kind = by_slot.get(slot, ".")
            cells.append("{}:{}".format(slot, kind))
        rows.append("  ".join(cells))
    return "\n".join(rows)


def main():
    parser = argparse.ArgumentParser(
        description="Randomly place up to nine objects in a 3x3 numbered grid."
    )
    parser.add_argument("n", nargs="?", type=int,
                        help="number of objects, from 0 to 9")
    parser.add_argument("--seed", type=int, help="optional seed for repeatable output")
    args = parser.parse_args()

    object_count = args.n
    if object_count is None:
        try:
            object_count = int(input("Object count (0-9): "))
        except ValueError:
            parser.error("n must be an integer from 0 to 9")
    if not 0 <= object_count <= 9:
        parser.error("n must be between 0 and 9")

    assignments = generate_random_grid(object_count, args.seed)
    print("assignments:", assignments)
    print(format_grid(assignments))


if __name__ == "__main__":
    main()
