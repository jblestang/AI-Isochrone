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
  --start-lat 47.75 --start-lon -3.37 \
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
    lambda_manoeuvre: 0.3,
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

- **λ₁–λ₄ sliders** — objective weights
- **Envelope band (min)** — minutes between arrival envelope rings
- **Use GRIB/BUFR grid** — synthetic spatial wind/current/wave grid
- **Optimize composite cost J** — cost-based vs time-only wavefront
- **Compute** — runs routing in background (native thread / wasm async)

---

## Architecture

| Module | Role |
|--------|------|
| `objective` | J function components & weights |
| `sea_state` | Polar modification for waves |
| `sota_isochrone` | Multi-criteria wavefront router |
| `envelope` | Destination arrival envelope builder |
| `grib` | Wind / current / sea-state providers |
| `gui` | Egui + walkers map (native + wasm) |
| `web` | Wasm `WebHandle` entry point |

## Tests

```bash
cargo test
```

## License

MIT
