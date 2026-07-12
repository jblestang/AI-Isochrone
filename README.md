# AI-Isochrone — SOTA Multi-Criteria Isochrone Routing

State-of-the-art isochrone routing for sailing vessels in Rust, with **Egui** visualization (native desktop + **WebAssembly**).

## Features

### Multi-criteria objective function

All weights are configurable in the GUI (or via `ObjectiveWeights` in code):

```
J = ETA + λ₁·wave_risk + λ₂·comfort + λ₃·manoeuvre_penalty + λ₄·safety_margin
```

| Term | Meaning |
|------|---------|
| **ETA** | Elapsed time (seconds) |
| **wave_risk** | Significant wave height, period, wind coupling |
| **comfort** | Head/b beam seas relative to heading |
| **manoeuvre_penalty** | Heading change cost (tacks/gybes) |
| **safety_margin** | Wave steepness + adverse current |

### Environmental data (GRIB / BUFR)

- `GribProvider` trait: wind, current, **sea state** (Hs, period, direction)
- `BufrGribGridProvider`: spatial grid with IDW interpolation (synthetic Mediterranean grid included)
- `SimpleGribProvider`: constant values for quick tests
- `CachedGribProvider`: lookup cache wrapper

### Boat & sea-state polars

- `Polar` trait + `SimplePolar` (bilinear table interpolation)
- `SeaStatePolarModifier` tweaks polar speed by wave conditions
- `SeaStateAdjustedPolar` composite wrapper
- `tweak_polar_table()` for batch polar adjustment

### Arrival envelopes

For a destination point, visualize **bands of points** that reach the target with ETAs separated by **x minutes** (`envelope_step_minutes`):

- `build_arrival_envelopes()` — boundary polygons per time band
- Toggle **Isochrones / Envelopes / Both** in the GUI

### Hard routing constraints

- `RoutingConstraints`: max true wind, max significant wave height, optional min depth
- Nodes violating limits are pruned before cost evaluation (`constraints` module)
- Optimistic ETA lower bound for destination-cone pruning

### Route reconstruction & export

- `backtrack_route()` — parent-map backtracking from best arrival
- `build_route_legs()` — tack detection, leg metadata (wind, sea state)
- `route_to_gpx()` — GPX export (native GUI)

### Weather scenarios & ensemble routing

- `WeatherScenario` presets: baseline, front early/late, conservative (P90 wind)
- `ScenarioGribProvider` — time-shifted / scaled GRIB for divergent scenarios
- `EnsembleGribProvider` — multi-member wind spread
- `eta_percentiles_from_results()` — P10 / P50 / P90 ETA from scenario fan

### Opponent routing (dual isochrones)

- `calculate_dual_routing()` — parallel routing for you vs opponent
- `ScaledPolar` — opponent speed factor vs reference polar
- `compute_cover_headings()` — mark-centric cover cone headings
- `tack_decision_eta()` — optimistic tack comparison helper
- GUI **Dual** button: opponent isochrones (magenta), ETA delta, scenario route fan

### Coastal land handling

- One land hop allowed from sea (or from start) so harbor departures reach open water
- Prevents spurious empty isochrones with spatial GRIB grids near the coast

### Land avoidance

- **Native**: `roaring-landmask` (GSHHG, full accuracy)
- **Wasm**: lightweight coastal stub (no native GEOS dependency)

---

## Build

```bash
# Core library + CLI (default)
cargo build --release

# Native GUI (requires desktop display + libgeos, libssl, python3-dev)
cargo build --release --features gui,native-landmask
cargo run --release --bin ai-isochrone-gui --features gui,native-landmask

# WebAssembly
rustup target add wasm32-unknown-unknown
cargo install trunk --locked
trunk build --features web --no-default-features --release
# Output in web/dist/ — serve with any static server
trunk serve --features web --no-default-features
```

### System dependencies (native)

```bash
sudo apt install libssl-dev libgeos-dev python3-dev pkg-config
```

---

## Usage

### CLI (classic isochrones)

```bash
cargo run --release -- \
  --start-lat 47.55 --start-lon -3.48 \
  --dest-lat 43.12 --dest-lon 5.93 \
  --time-limit-hours 12
```

### SOTA routing (library)

```rust
use ai_isochrone::*;

let config = SotaRoutingConfig::default();
let weights = ObjectiveWeights {
    lambda_wave_risk: 1.0,
    lambda_comfort: 0.5,
    lambda_manoeuvre: 1.5,
    lambda_safety: 2.0,
    ..Default::default()
};

let result = calculate_sota_routing(
    config,
    weights,
    Landmask::new()?,
    Box::new(SimplePolar::default_voilier()),
    Box::new(BufrGribGridProvider::synthetic_mediterranean(43.0, 48.0, -5.0, 8.0, 0.5)),
    chrono::Utc::now(),
);

// result.isochrones — forward reachable fronts
// result.arrival_envelopes — ETA bands at destination
// result.best_eta_hours, result.best_cost
```

### GUI controls

- **Compute** — solo SOTA routing
- **Dual** — opponent + multi-scenario route fan
- **Opponent** position / polar scale controls
- **Scenario** checkboxes (baseline, front early/late, conservative)
- **Constraints** panel (max wind, max Hs)
- **P10 / P50 / P90** ETA display after dual run
- **Export GPX** (native) for best route
- **λ₁–λ₄ sliders** — objective weights
- **Envelope band (min)** — minutes between arrival envelope rings
- **Use GRIB/BUFR grid** — synthetic spatial wind/current/wave grid
- **Optimize composite cost J** — cost-based vs time-only wavefront

---

## Architecture

### Grid-based isochrones

Isochrones are computed on a **regular lat/lon grid** aligned with GRIB spacing when available:

1. Precompute sea-only grid cells (land excluded)
2. Wavefront expansion at **exact simulation coordinates** — arrival time recorded at each precise point
3. Bin each sea point into its containing grid cell; keep **one best (earliest) arrival** per cell
4. Extract the **outward envelope** (farthest point per bearing sector from start)

Configure via `IsochroneConfig.grid_step_deg` (optional) and `envelope_sector_deg` (default 10°).

| Module | Role |
|--------|------|
| `grid` | GRIB-aligned routing grid, per-cell best ETA, envelope builder |
| `objective` | J function components & weights |
| `sea_state` | Polar modification for waves |
| `sota_isochrone` | Multi-criteria wavefront router |
| `envelope` | Destination arrival envelope builder |
| `constraints` | Hard wind/wave limits & optimistic ETA |
| `route` | Route backtrack, legs, GPX export |
| `scenario` | Divergent weather scenario GRIB wrapper |
| `ensemble` | Multi-member GRIB & ETA percentiles |
| `opponent` | Dual-boat routing & cover headings |
| `grib` | Wind / current / sea-state providers |
| `gui` | Egui + walkers map (native + wasm) |
| `web` | Wasm `WebHandle` entry point |

## Tests

```bash
cargo test
```

## License

MIT
