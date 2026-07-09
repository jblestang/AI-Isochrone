use crate::envelope::extract_outward_envelope;
use crate::grib::GribProvider;
use crate::landmask::Landmask;
use crate::types::{Isochrone, Point};

/// Regular lat/lon grid specification (typically aligned with GRIB data).
#[derive(Debug, Clone, Copy)]
pub struct GridSpec {
    pub min_lat: f64,
    pub max_lat: f64,
    pub min_lon: f64,
    pub max_lon: f64,
    pub step_deg: f64,
}

impl GridSpec {
    pub fn n_lat(&self) -> i32 {
        ((self.max_lat - self.min_lat) / self.step_deg).round() as i32 + 1
    }

    pub fn n_lon(&self) -> i32 {
        ((self.max_lon - self.min_lon) / self.step_deg).round() as i32 + 1
    }

    /// Bounding box covering start and destination with padding in degrees.
    pub fn from_route(start: Point, destination: Option<Point>, step_deg: f64, padding_deg: f64) -> Self {
        let mut min_lat = start.lat;
        let mut max_lat = start.lat;
        let mut min_lon = start.lon;
        let mut max_lon = start.lon;

        if let Some(dest) = destination {
            min_lat = min_lat.min(dest.lat);
            max_lat = max_lat.max(dest.lat);
            min_lon = min_lon.min(dest.lon);
            max_lon = max_lon.max(dest.lon);
        }

        Self {
            min_lat: (min_lat - padding_deg).floor(),
            max_lat: (max_lat + padding_deg).ceil(),
            min_lon: (min_lon - padding_deg).floor(),
            max_lon: (max_lon + padding_deg).ceil(),
            step_deg,
        }
    }
}

/// Cell index on a regular grid.
pub type CellKey = (i32, i32);

/// Regular routing grid aligned with GRIB spacing.
#[derive(Debug, Clone)]
pub struct RoutingGrid {
    spec: GridSpec,
    sea_cells: std::collections::HashSet<CellKey>,
}

impl RoutingGrid {
    pub fn from_spec(spec: GridSpec, landmask: &Landmask) -> Self {
        let mut sea_cells = std::collections::HashSet::new();
        let n_lat = spec.n_lat();
        let n_lon = spec.n_lon();

        for i in 0..=n_lat {
            for j in 0..=n_lon {
                let center = Self::cell_center_with_spec(&spec, (i, j));
                if landmask.is_sea(&center) {
                    sea_cells.insert((i, j));
                }
            }
        }

        Self { spec, sea_cells }
    }

    pub fn spec(&self) -> &GridSpec {
        &self.spec
    }

    pub fn is_sea_cell(&self, key: CellKey) -> bool {
        self.sea_cells.contains(&key)
    }

    pub fn cell_key(&self, point: &Point) -> CellKey {
        let i = ((point.lat - self.spec.min_lat) / self.spec.step_deg).round() as i32;
        let j = ((point.lon - self.spec.min_lon) / self.spec.step_deg).round() as i32;
        (i, j)
    }

    /// Nearest sea cell for a point (searches local neighborhood if rounded cell is land).
    pub fn nearest_sea_cell(&self, point: &Point) -> Option<CellKey> {
        let base = self.cell_key(point);
        if self.is_sea_cell(base) {
            return Some(base);
        }

        let max_radius = 4i32;
        for radius in 1..=max_radius {
            for di in -radius..=radius {
                for dj in -radius..=radius {
                    if di.abs() != radius && dj.abs() != radius {
                        continue;
                    }
                    let key = (base.0 + di, base.1 + dj);
                    if self.is_sea_cell(key) {
                        return Some(key);
                    }
                }
            }
        }
        None
    }

    pub fn cell_center(&self, key: CellKey) -> Point {
        Self::cell_center_with_spec(&self.spec, key)
    }

    fn cell_center_with_spec(spec: &GridSpec, key: CellKey) -> Point {
        Point::new(
            spec.min_lat + key.0 as f64 * spec.step_deg,
            spec.min_lon + key.1 as f64 * spec.step_deg,
        )
    }

    pub fn snap_to_cell(&self, point: &Point) -> (CellKey, Point) {
        let key = self.cell_key(point);
        (key, self.cell_center(key))
    }
}

/// Per-cell best arrival tracking for grid-based isochrones.
#[derive(Debug, Clone)]
pub struct GridBestTracker {
    grid: RoutingGrid,
    best_time: std::collections::HashMap<CellKey, f64>,
    best_point: std::collections::HashMap<CellKey, Point>,
}

impl GridBestTracker {
    pub fn new(grid: RoutingGrid) -> Self {
        Self {
            grid,
            best_time: std::collections::HashMap::new(),
            best_point: std::collections::HashMap::new(),
        }
    }

    pub fn grid(&self) -> &RoutingGrid {
        &self.grid
    }

    pub fn best_time(&self, key: CellKey) -> f64 {
        self.best_time.get(&key).copied().unwrap_or(f64::INFINITY)
    }

    pub fn best_point(&self, key: CellKey) -> Option<Point> {
        self.best_point.get(&key).copied()
    }

    /// Register arrival at the nearest sea grid cell. Returns the cell key if updated.
    pub fn try_update(&mut self, point: Point, time: f64) -> Option<CellKey> {
        let key = self.grid.nearest_sea_cell(&point)?;
        if time + 1e-6 >= self.best_time(key) {
            return None;
        }

        self.best_time.insert(key, time);
        self.best_point.insert(key, point);
        Some(key)
    }

    /// Cells whose best arrival falls within the isochrone time band.
    pub fn points_in_time_band(&self, target_seconds: f64, band_seconds: f64) -> Vec<Point> {
        let min_t = target_seconds - band_seconds;
        let max_t = target_seconds + band_seconds * 0.5;

        self.best_time
            .iter()
            .filter(|(_, &t)| t >= min_t && t <= max_t)
            .filter_map(|(key, _)| self.best_point.get(key).copied())
            .collect()
    }

    /// Build an isochrone ring at `target_seconds` using per-cell bests and outward envelope.
    pub fn build_isochrone_envelope(
        &self,
        target_seconds: f64,
        band_seconds: f64,
        center: Point,
        sector_deg: f64,
    ) -> Isochrone {
        let raw = self.points_in_time_band(target_seconds, band_seconds);
        let envelope = extract_outward_envelope(&raw, &center, sector_deg);

        Isochrone {
            time_hours: target_seconds / 3600.0,
            points: envelope,
        }
    }
}

/// Resolve routing grid specification from GRIB provider and route config.
pub fn resolve_grid_spec(
    grib: &dyn GribProvider,
    start: Point,
    destination: Option<Point>,
    grid_step_deg: Option<f64>,
) -> GridSpec {
    if let Some(step) = grid_step_deg {
        return GridSpec::from_route(start, destination, step, 2.0);
    }

    if let Some(grib_spec) = grib.grid_spec() {
        return GridSpec {
            min_lat: grib_spec.min_lat,
            max_lat: grib_spec.max_lat,
            min_lon: grib_spec.min_lon,
            max_lon: grib_spec.max_lon,
            step_deg: grib_spec.step_deg,
        };
    }

    GridSpec::from_route(start, destination, 0.05, 2.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cell_snapping_rounds_to_nearest() {
        let spec = GridSpec {
            min_lat: 47.0,
            max_lat: 48.0,
            min_lon: -4.0,
            max_lon: -3.0,
            step_deg: 0.5,
        };
        let landmask = Landmask::new().unwrap();
        let grid = RoutingGrid::from_spec(spec, &landmask);
        let (key, center) = grid.snap_to_cell(&Point::new(47.74, -3.36));
        assert_eq!(key, (1, 1));
        assert!((center.lat - 47.5).abs() < 1e-9);
        assert!((center.lon - (-3.5)).abs() < 1e-9);
    }

    #[test]
    fn tracker_keeps_best_arrival_per_cell() {
        let spec = GridSpec::from_route(
            Point::new(47.75, -3.37),
            Some(Point::new(43.12, 5.93)),
            0.5,
            1.0,
        );
        let landmask = Landmask::new().unwrap();
        let grid = RoutingGrid::from_spec(spec, &landmask);
        let mut tracker = GridBestTracker::new(grid);

        let p = Point::new(47.74, -3.36);
        if tracker.grid().is_sea_cell(tracker.grid().cell_key(&p)) {
            tracker.try_update(p, 3600.0);
            tracker.try_update(p, 7200.0);
            let key = tracker.grid().cell_key(&p);
            assert_eq!(tracker.best_time(key), 3600.0);
        }
    }
}
