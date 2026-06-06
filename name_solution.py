# DO NOT MODIFY IMPORT
from robot_simulator import create_simulator, GameState
from time import sleep
from collections import deque

# Persistent belief about the world, accumulated across choose_action() calls.
# The simulator regenerates noisy perception every tick, so we keep our own
# locked-in knowledge here.
_belief = {
    "obstacles": set(),  # (x, y) tiles ever read as 1.0 -> definitely walls
    "free":      set(),  # (x, y) tiles ever stood on / read 0.0 / known goal
    "goal":      None,   # (x, y) once spotted (perception value 2.0)
    "shape":     None,   # (h, w) of last seen map, used to detect a sim reset
    "last_pos":  None,   # used to detect a sim reset (position teleport)
}

_ORIENT_TO_LETTER = {"NORTH": "N", "SOUTH": "S", "EAST": "E", "WEST": "W"}
_DELTA_TO_LETTER = {(0, -1): "N", (0, 1): "S", (1, 0): "E", (-1, 0): "W"}
_NEIGHBOR_DELTAS = [(0, -1), (0, 1), (1, 0), (-1, 0)]


def _reset_belief(shape):
    _belief["obstacles"] = set()
    _belief["free"] = set()
    _belief["goal"] = None
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

    obstacles = _belief["obstacles"]
    free = _belief["free"]
    goal = _belief["goal"]

    # Optimistic passability: anything not locked as obstacle is fair game.
    def passable(cell):
        return cell not in obstacles

    # 1) If we've spotted the goal, plan straight to it.
    path = None
    if goal is not None:
        path = _bfs(pos, lambda c: c == goal, passable, w, h)

    # 2) Otherwise, walk to the nearest frontier tile (known-free next to unknown).
    if path is None:
        def is_frontier(c):
            if c not in free:
                return False
            cx, cy = c
            for dx, dy in _NEIGHBOR_DELTAS:
                nb = (cx + dx, cy + dy)
                if not (0 <= nb[0] < w and 0 <= nb[1] < h):
                    continue
                if nb not in free and nb not in obstacles:
                    return True
            return False
        path = _bfs(pos, is_frontier, passable, w, h)

    # 3) Last resort: head to any reachable unknown cell.
    if path is None:
        path = _bfs(pos, lambda c: c not in free and c not in obstacles,
                    passable, w, h)

    # Nothing reachable -> spin in place to keep re-sensing.
    if path is None or len(path) < 2:
        return 'E' if orient_letter == 'N' else 'N'

    next_cell = path[1]
    needed = _DELTA_TO_LETTER.get((next_cell[0] - pos[0], next_cell[1] - pos[1]))
    if needed is None:
        return 'N'

    # Turn first if not facing the right way.
    if orient_letter != needed:
        return needed

    # Look-before-you-leap. We're facing next_cell, so it's in our front cone
    # and the current reading is fresh.
    if next_cell not in free:
        nx, ny = next_cell
        reading = pm[ny, nx]
        # Anything firmer than "low confidence free" -> don't step yet.
        # 0.0/2.0 would have been locked into `free` above, so this guards
        # against 0.5/0.75 (and a paranoid bound on 1.0 which is unreachable
        # here because BFS rejects locked obstacles).
        if reading > 0.25 and reading != 2.0:
            # Turn perpendicular to re-roll perception next tick. The same
            # cell will remain in our sensor cone as a side reading.
            return 'E' if needed in ('N', 'S') else 'N'

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
            
        sleep(0.05)  # You may modify this if you want to speed up your simulation
        simulator.root.update()  # Ensure Tkinter processes events even when game is not running

if __name__ == "__main__":
    main()
