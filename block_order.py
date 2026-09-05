from functools import lru_cache

def solve(columns_top_to_bottom):
    """
    columns_top_to_bottom: list of columns, each a list of colors
                            written TOP -> BOTTOM (as you'd read the picture).
    Returns (num_trips, plan) where plan is a list of
        (color, {column_index: how_many_placed})
    in the order you should do them.
    """
    # Because of gravity, blocks must be placed bottom-first.
    fill_order = [list(reversed(col)) for col in columns_top_to_bottom]
    n = len(fill_order)
    lengths = tuple(len(c) for c in fill_order)
    target = lengths

    def frontier(ptrs):
        colors = set()
        for i in range(n):
            if ptrs[i] < lengths[i]:
                colors.add(fill_order[i][ptrs[i]])
        return colors

    def advance_color(ptrs, color):
        ptrs = list(ptrs)
        changed = True
        while changed:
            changed = False
            for i in range(n):
                if ptrs[i] < lengths[i] and fill_order[i][ptrs[i]] == color:
                    ptrs[i] += 1
                    changed = True
        return tuple(ptrs)

    memo = {}
    choice = {}

    def dp(ptrs):
        if ptrs == target:
            return 0
        if ptrs in memo:
            return memo[ptrs]
        best, best_color = None, None
        for color in frontier(ptrs):
            new_ptrs = advance_color(ptrs, color)
            cost = 1 + dp(new_ptrs)
            if best is None or cost < best:
                best, best_color = cost, color
        memo[ptrs] = best
        choice[ptrs] = best_color
        return best

    start = tuple([0] * n)
    total_trips = dp(start)

    # reconstruct the actual plan
    plan = []
    ptrs = start
    while ptrs != target:
        color = choice[ptrs]
        new_ptrs = advance_color(ptrs, color)
        placed = {i: new_ptrs[i] - ptrs[i] for i in range(n) if new_ptrs[i] != ptrs[i]}
        plan.append((color, placed))
        ptrs = new_ptrs

    return total_trips, plan


def print_plan(plan):
    for trip_num, (color, placed) in enumerate(plan, start=1):
        parts = [f"Column {i+1} x{cnt}" for i, cnt in placed.items()]
        print(f"Trip {trip_num}: get {color} blocks -> place into: {', '.join(parts)}")


if __name__ == "__main__":
    # Example from the prompt (top -> bottom as drawn):
    columns = [
        ["Blue", "White", "Yellow", "Blue"],    # column 1
        ["Green", "Blue", "Green", "Yellow"],   # column 2
        ["Yellow", "Green", "Green", "Yellow"], # column 3
    ]

    trips, plan = solve(columns)
    print(f"Minimum number of trips (color-location switches): {trips}\n")
    print_plan(plan)
