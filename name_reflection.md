# Reflection — Team Polar Autonomous Challenge

## How I tackled it

I read the brief and saw the structure mapped onto PPPA almost one-to-one: a noisy sensor (Perception), a state to maintain (Processing), a path to plan (Planning), a single action to return (Actuation). So I wrote `choose_action` as exactly those four stages, in that order, every tick.

The first thing I built was a persistent "belief map" the function carries between calls. The simulator regenerates the noisy perception matrix every tick, but the brief gives one anchor I could rely on: a `1.0` reading is permanent. So I lock cells as obstacles the moment I ever see `1.0`, lock them as free on `0.0`, and lock the goal on `2.0` (which the simulator emits even though the brief doesn't mention it — I caught that reading `robot_simulator.py`). Noisy in-between readings (`0.25`, `0.5`, `0.75`) don't lock anything by themselves; they just contribute to a per-cell evidence counter.

With that map, planning was the easy part. I run BFS from the current tile every single tick — no commitment to a stale plan. If the goal has been spotted, BFS targets it; if not, BFS targets the nearest "frontier" tile (known-free with an unknown neighbor) so the robot naturally explores outward until the goal enters its sensor cone. BFS treats unknown cells as passable, so the robot doesn't get stuck waiting to know everything before moving.

Actuation is then a six-line function: compare orientation to the direction of the next path cell, return `'N/S/E/W'` to turn or `'M'` to step.

## The fix that actually mattered

My first version solved 9/10 of my test maps but crashed on the 10th. Diagnosing it: I had been trusting a single low reading on the tile in front of me before stepping. The simulator's noise model means a real wall produces a `0.25` reading 10% of the time — over a long path, that's near-certain death.

The fix was to switch the step-permission rule from "current reading is low" to "accumulated evidence is low": require at least two `0.25` readings across the journey and more lows than highs. I also taught BFS to route around cells with repeated `0.75` readings before they get hard-locked, and added a 5-strike stuck breaker that soft-blocks a cell if I've re-rolled on it too many times. Re-running the suite: 10/10, zero crashes, and average step count down ~17%.

## Difficulty

The maze itself is simple — it's the sensor model that makes it interesting. The hard part wasn't path-finding (BFS on a 14×14 grid is trivial); it was reasoning about *asymmetric* cost: a false-positive obstacle reading just means a detour, but a false-negative is a crash. That asymmetry is what motivates basically every safety choice in the code. I found it a satisfying problem because the right move isn't to be smarter — it's to be more honest about what you don't know.

Two of the simulator's quirks (perception indexed `[y][x]` while position is `(x, y)`; orientation as `Orientation.NORTH` not `"NORTH"`) were called out in the brief and easy to handle. The undocumented `2.0` for the goal tile was a nice find.

## Resources

I used Claude as a thinking partner — mostly to talk through the PPPA-style decomposition, brainstorm fusion rules, and sanity-check the probability math behind the crash diagnosis. Code is mine; the design conversation was assisted. Searched briefly for "frontier exploration" to confirm I was reinventing Yamauchi's 1997 approach and not something dumber.

## One thing worth flagging

The step counts vary a lot (roughly 15–290 across my test maps). That's mostly intrinsic — bigger maps mean longer paths, and each cell costs ~2-4 extra ticks of sensor gathering on top of the turn+move base. If step count became important, the next move would probably be a short scan-in-place at the start (4 turns = 4 perception cone shifts, much better initial belief map), or relaxing the step rule and leaning harder on the BFS soft-obstacle gate. Solving reliably was the priority, so I stopped tuning once that was solid.
