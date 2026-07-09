use crate::types::Point;

/// Default route: Lorient → Ushuaia (overridable via CLI or env).
pub const DEFAULT_FROM_LAT: f64 = 47.55;
pub const DEFAULT_FROM_LON: f64 = -3.48;
pub const DEFAULT_TO_LAT: f64 = -54.80;
pub const DEFAULT_TO_LON: f64 = -68.30;
pub const DEFAULT_TIME_LIMIT_HOURS: f64 = 3500.0;

#[derive(Debug, Clone, Copy)]
pub struct RouteEndpoints {
    pub start: Point,
    pub dest: Point,
}

impl RouteEndpoints {
    pub fn new(start: Point, dest: Point) -> Self {
        Self { start, dest }
    }

    pub fn defaults() -> Self {
        Self {
            start: Point::new(DEFAULT_FROM_LAT, DEFAULT_FROM_LON),
            dest: Point::new(DEFAULT_TO_LAT, DEFAULT_TO_LON),
        }
    }

    pub fn from_coords(from_lat: f64, from_lon: f64, to_lat: f64, to_lon: f64) -> Self {
        Self {
            start: Point::new(from_lat, from_lon),
            dest: Point::new(to_lat, to_lon),
        }
    }

    pub fn from_env() -> Self {
        Self::from_coords(
            env_f64("AI_ISOCHRONE_FROM_LAT", DEFAULT_FROM_LAT),
            env_f64("AI_ISOCHRONE_FROM_LON", DEFAULT_FROM_LON),
            env_f64("AI_ISOCHRONE_TO_LAT", DEFAULT_TO_LAT),
            env_f64("AI_ISOCHRONE_TO_LON", DEFAULT_TO_LON),
        )
    }

    pub fn time_limit_hours_from_env() -> f64 {
        env_f64("AI_ISOCHRONE_TIME_LIMIT_HOURS", DEFAULT_TIME_LIMIT_HOURS)
    }

    /// Short label for logs and map title bar.
    pub fn label(&self) -> String {
        format!(
            "({:.2}, {:.2}) → ({:.2}, {:.2})",
            self.start.lat, self.start.lon, self.dest.lat, self.dest.lon
        )
    }

    pub fn snapshot_slug(&self) -> String {
        format!(
            "route_{:.0}_{:.0}_to_{:.0}_{:.0}",
            self.start.lat, self.start.lon, self.dest.lat, self.dest.lon
        )
        .replace('.', "p")
        .replace('-', "m")
    }
}

fn env_f64(key: &str, default: f64) -> f64 {
    std::env::var(key)
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
}
