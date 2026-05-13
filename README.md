# MovHex

Final exam project for **Algorithms and Data Structures** (Algoritmi e Strutture Dati) at Politecnico di Milano, A.Y. 2024–2025.

**Grade achieved: 30 cum laude / 30**.

---

## Problem overview

MovHex is a fictional freight company managing a fleet of vehicles across a large geographic area. The program models the territory as a rectangular grid of hexagonal tiles and answers shortest-path queries on it.

Each hexagon has an integer *exit cost* (1–100). A cost of 0 means the hex is impassable (cannot be left, but can be a destination). In addition to the standard six adjacencies, hexes can be connected by directed *air routes* with their own costs. The goal is to process a stream of commands efficiently, since `travel_cost` queries vastly outnumber map-modification commands.

## Commands

| Command | Description |
|---|---|
| `init <cols> <rows>` | (Re)initialise the map; all costs set to 1, no air routes |
| `change_cost <x> <y> <v> <radius>` | Apply a radial cost delta centred on `(x, y)`; also updates outgoing air route costs |
| `toggle_air_route <x1> <y1> <x2> <y2>` | Add or remove a directed air route from `(x1,y1)` to `(x2,y2)` |
| `travel_cost <xp> <yp> <xd> <yd>` | Return the minimum-cost path from source to destination (`-1` if unreachable) |

Full specification: [`specifica_progetto_api_2024_2025.pdf`](specifica_progetto_api_2024_2025.pdf)

## Implementation highlights

- **Dijkstra with a binary min-heap** for shortest-path queries; both ground adjacency and air routes are explored in a single pass.
- **Timestamp-based lazy reset**: `dist[]` and `visited[]` arrays are never zeroed between queries — a monotonically incrementing counter makes stale entries invisible in O(1).
- **Open-addressing hash map** for hexes that have air routes, using linear probing and tombstone deletion. Kept at load factor ≤ 0.5.
- **Travel-cost result cache**: a separate open-addressing hash table (Fibonacci hashing, load factor ≤ 0.7) memoises recent `travel_cost` results. A version counter invalidates the entire cache on any map modification without clearing memory.
- **Cube coordinate system** for `change_cost`: offset coordinates are converted to cube coordinates to enumerate all hexes within a given hex-distance radius.

## Performance

Results on the official Polimi grader (30L threshold):

| Metric | Value |
|---|---|
| Execution time | **9.911 s** (limit: 10.000 s) |
| Memory used | **8.29 MiB** (limit: 26.0 MiB) |

> **Note:** these figures depend on the grader's hardware. Running the binary on a different machine will produce different timings.

## Build & run

Requires a C11-compatible compiler (GCC or Clang). No external libraries beyond the C standard library.

```bash
gcc -std=c11 -O2 -o movhex movhex.c -lm
./movhex < input.txt
```

Input is read from `stdin`; output is written to `stdout`.

## Repository layout

```
movhex.c                              # source code
specifica_progetto_api_2024_2025.pdf  # official problem specification
test_cases/                           # sample I/O provided by the course
```

## License

MIT — see [`LICENSE`](LICENSE).
