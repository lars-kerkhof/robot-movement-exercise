# DO NOT MODIFY IMPORT
from robot_simulator import create_simulator, GameState
from time import sleep
from collections import deque

# Persistent belief about the world, accumulated across choose_action() calls.
# The simulator regenerates noisy perception every tick, so we keep our own
# locked-in knowledge here.
_belief = {
    "obstacles":      set(),  # (x, y) tiles ever read 1.0 -> definitely walls
    "free":           set(),  # (x, y) tiles ever stood on / read 0.0 / known goal
    "goal":           None,   # (x, y) once spotted (perception value 2.0)
    # 0.5 readings are ignored (they're both "ambiguous" and the default for
    # cells outside the sensor cone, so we can't distinguish the two).
    "low_reads":      {},     # (x, y) -> count of 0.25 readings  (evidence: free)
    "high_reads":     {},     # (x, y) -> count of 0.75 readings  (evidence: wall)
    "soft_obstacles": set(),  # cells we've given up on after repeated re-rolls
    "stuck_target":   None,   # cell we've been spinning at
    "stuck_count":    0,      # consecutive re-rolls on stuck_target
    "shape":          None,   # used to detect a sim reset
    "last_pos":       None,   # used to detect a sim reset (position teleport)
}

_ORIENT_TO_LETTER = {"NORTH": "N", "SOUTH": "S", "EAST": "E", "WEST": "W"}
_DELTA_TO_LETTER = {(0, -1): "N", (0, 1): "S", (1, 0): "E", (-1, 0): "W"}
_NEIGHBOR_DELTAS = [(0, -1), (0, 1), (1, 0), (-1, 0)]


def _reset_belief(shape):
    _belief["obstacles"] = set()
    _belief["free"] = set()
    _belief["goal"] = None
    _belief["low_reads"] = {}
    _belief["high_reads"] = {}
    _belief["soft_obstacles"] = set()
    _belief["stuck_target"] = None
    _belief["stuck_count"] = 0
    _belief["shape"] = shape
    _belief["last_pos"] = None


def _maybe_reset(game_state):
    shape = game_state.perception_matrix.shape
    pos = game_state.current_position
    if _belief["shape"] != shape:
        _reset_belief(shape)
        return
    last = _belief["last_pos"]
    if last is not None:
        if abs(last[0] - pos[0]) + abs(last[1] - pos[1]) > 1:
            _reset_belief(shape)


def _update_belief(game_state):
    pm = game_state.perception_matrix
    h, w = pm.shape
    cx, cy = game_state.current_position
    _belief["free"].add((cx, cy))
    for y in range(h):
        for x in range(w):
            v = pm[y, x]
            cell = (x, y)
            if v == 1.0:
                _belief["obstacles"].add(cell)
            elif v == 0.0:
                _belief["free"].add(cell)
            elif v == 2.0:
                _belief["goal"] = cell
                _belief["free"].add(cell)
            elif v == 0.25:
                _belief["low_reads"][cell] = _belief["low_reads"].get(cell, 0) + 1
            elif v == 0.75:
                _belief["high_reads"][cell] = _belief["high_reads"].get(cell, 0) + 1
    _belief["last_pos"] = (cx, cy)


def _bfs(start, is_target, passable, w, h):
    """Shortest path from start to any cell satisfying is_target.
    Returns list [start, ..., target] or None if unreachable."""
    parent = {start: None}
    queue = deque([start])
    while queue:
        cur = queue.popleft()
        if cur != start and is_target(cur):
            path = [cur]
            while parent[path[-1]] is not None:
                path.append(parent[path[-1]])
            return list(reversed(path))
        cx, cy = cur
        for dx, dy in _NEIGHBOR_DELTAS:
            nb = (cx + dx, cy + dy)
            if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                continue
            if nb in parent or not passable(nb):
                continue
            parent[nb] = cur
            queue.append(nb)
    # Also accept start itself if it satisfies the target
    if is_target(start):
        return [start]
    return None


# DO NOT MODIFY FUNCTION DECLARATION
def choose_action(game_state: GameState) -> str:
    """
    Belief-map navigator.
      1. Perception: read the (noisy) perception matrix.
      2. Processing: lock cells we now know for sure into our persistent belief.
      3. Planning:   BFS to the goal if seen, otherwise to the nearest
                     'frontier' (a known-free tile next to an unknown one),
                     treating UNKNOWN tiles as optimistically passable.
      4. Actuation:  turn toward the next step, or move forward (with a
                     look-before-you-leap safety check).
    """
    _maybe_reset(game_state)
    _update_belief(game_state)

    pm = game_state.perception_matrix
    h, w = pm.shape
    pos = game_state.current_position
    orient_letter = _ORIENT_TO_LETTER[game_state.current_orientation.name]

    obstacles      = _belief["obstacles"]
    free           = _belief["free"]
    goal           = _belief["goal"]
    low_reads      = _belief["low_reads"]
    high_reads     = _belief["high_reads"]
    soft_obstacles = _belief["soft_obstacles"]

    # A cell that's been seen as 0.75 many times without compensating low
    # readings is probably a wall the planner should route around early
    # (instead of marching us into it and then re-rolling).
    def strong_suspect(cell):
        h_ = high_reads.get(cell, 0)
        l_ = low_reads.get(cell, 0)
        return h_ >= 3 and h_ > 2 * l_

    def strict_passable(cell):
        return (cell not in obstacles
                and cell not in soft_obstacles
                and not strong_suspect(cell))

    # Fallback: only respect hard-locked obstacles.
    def loose_passable(cell):
        return cell not in obstacles

    def find_path(passable_fn):
        if goal is not None:
            p = _bfs(pos, lambda c: c == goal, passable_fn, w, h)
            if p is not None:
                return p
        # Frontier: locked-free with an unclassified neighbor.
        def is_frontier(c):
            if c not in free:
                return False
            cx, cy = c
            for dx, dy in _NEIGHBOR_DELTAS:
                nb = (cx + dx, cy + dy)
                if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                    continue
                if nb in free or nb in obstacles:
                    continue
                return True
            return False
        p = _bfs(pos, is_frontier, passable_fn, w, h)
        if p is not None:
            return p
        return _bfs(pos, lambda c: c not in free and c not in obstacles,
                    passable_fn, w, h)

    path = find_path(strict_passable)
    if path is None:
        # Strict view blocked us in -- forget soft strikes and try again.
        soft_obstacles.clear()
        path = find_path(loose_passable)

    if path is None or len(path) < 2:
        return 'E' if orient_letter == 'N' else 'N'

    next_cell = path[1]
    needed = _DELTA_TO_LETTER.get((next_cell[0] - pos[0], next_cell[1] - pos[1]))
    if needed is None:
        return 'N'

    if orient_letter != needed:
        return needed

    # Look-before-you-leap, using *accumulated* evidence rather than the last
    # single reading. A real obstacle very rarely sustains low > high without
    # ever hitting 1.0, so this is much safer than trusting one 0.25.
    if next_cell not in free:
        low = low_reads.get(next_cell, 0)
        high = high_reads.get(next_cell, 0)
        if not (low >= 2 and low > high):
            if _belief["stuck_target"] != next_cell:
                _belief["stuck_target"] = next_cell
                _belief["stuck_count"] = 0
            _belief["stuck_count"] += 1
            if _belief["stuck_count"] >= 5:
                # Spun too long; give up on this cell and let BFS reroute.
                soft_obstacles.add(next_cell)
                _belief["stuck_target"] = None
                _belief["stuck_count"] = 0
            # Re-roll perception by turning perpendicular; the target cell
            # stays in our sensor cone as a side reading next tick.
            return 'E' if needed in ('N', 'S') else 'N'

    _belief["stuck_target"] = None
    _belief["stuck_count"] = 0
    return 'M'




#DO NOT MODIFY CODE BEYOND THIS POINT (except the 2nd sleep func)
def main():
    simulator = create_simulator()
    game_state = simulator.get_game_state()
    last_moves = -1  # Track the previous moves_made count
    
    while True:
        if simulator.attempt_quit():
            print("Quitting 3/3 ...")
            quit()
            
        # Detect reset by checking if moves_made decreased
        if simulator.moves_made < last_moves:
            sleep(0.8)  # Small delay to ensure clean state
            game_state = simulator.get_game_state()
            
        last_moves = simulator.moves_made
            
        if game_state.simulation_state.name == "RUNNING":
            action = choose_action(game_state)
            game_state = simulator.step(action)
            
        sleep(0.5)  # You may modify this if you want to speed up your simulation
        simulator.root.update()  # Ensure Tkinter processes events even when game is not running

if __name__ == "__main__":
    main()