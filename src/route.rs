use crate::polar::is_tack_or_gybe;
use crate::types::{Point, RouteLeg, Wind, SeaState};

const METERS_PER_NM: f64 = 1852.0;

/// Backtrack from arrival cell key through parent map to start.
pub fn backtrack_route<S1, S2>(
    _start: Point,
    arrival_key: (i32, i32),
    parent_map: &std::collections::HashMap<(i32, i32), (i32, i32), S1>,
    point_map: &std::collections::HashMap<(i32, i32), Point, S2>,
) -> Vec<Point>
where
    S1: std::hash::BuildHasher,
    S2: std::hash::BuildHasher,
{
    let mut keys = vec![arrival_key];
    let mut current = arrival_key;
    let mut guard = 0usize;
    while let Some(&parent) = parent_map.get(&current) {
        keys.push(parent);
        current = parent;
        guard += 1;
        if guard > 50_000 {
            break;
        }
    }
    keys.reverse();
    keys.iter()
        .filter_map(|k| point_map.get(k).copied())
        .collect::<Vec<_>>()
}

/// Build route legs with tack detection from a point sequence.
pub fn build_route_legs(
    points: &[Point],
    headings: &[f64],
    step_hours: f64,
    wind_samples: &[(Wind, SeaState)],
) -> Vec<RouteLeg> {
    if points.len() < 2 {
        return Vec::new();
    }
    let mut legs = Vec::new();
    for i in 0..points.len() - 1 {
        let from = points[i];
        let to = points[i + 1];
        let bearing = from.bearing_to(&to);
        let dist_nm = from.distance_to(&to) / METERS_PER_NM;
        // Cell heading is stored on the arrival point (heading used on this leg).
        let boat_heading = headings.get(i + 1).copied().unwrap_or(bearing);
        let next_leg_heading = headings.get(i + 2).copied().unwrap_or(boat_heading);
        let (wind, sea) = wind_samples
            .get(i + 1)
            .copied()
            .or_else(|| wind_samples.get(i).copied())
            .unwrap_or((Wind::new(0.0, 0.0), SeaState::default()));
        let is_tack = is_tack_or_gybe(boat_heading, next_leg_heading, wind.direction);
        legs.push(RouteLeg {
            from,
            to,
            bearing_deg: bearing,
            boat_heading_deg: boat_heading,
            distance_nm: dist_nm,
            duration_hours: step_hours,
            is_tack,
            wind,
            sea_state: sea,
        });
    }
    legs
}

/// Export route to GPX 1.1 format.
pub fn route_to_gpx(name: &str, points: &[Point]) -> String {
    let mut gpx = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="ai-isochrone">
  <trk>
    <name>{}</name>
    <trkseg>
"#,
        name
    );
    for p in points {
        gpx.push_str(&format!(
            "      <trkpt lat=\"{:.6}\" lon=\"{:.6}\"/>\n",
            p.lat, p.lon
        ));
    }
    gpx.push_str("    </trkseg>\n  </trk>\n</gpx>\n");
    gpx
}

/// Simplify route by keeping every N-th point and endpoints.
pub fn simplify_route(points: &[Point], max_points: usize) -> Vec<Point> {
    if points.len() <= max_points {
        return points.to_vec();
    }
    let step = (points.len() - 1) / (max_points - 1).max(1);
    let mut out = vec![points[0]];
    let mut i = step;
    while i < points.len() - 1 {
        out.push(points[i]);
        i += step;
    }
    out.push(*points.last().unwrap());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backtrack_reaches_start() {
        let start = Point::new(47.0, -3.0);
        let mid = Point::new(46.0, -2.0);
        let end = Point::new(45.0, -1.0);
        let mut parent = std::collections::HashMap::new();
        let mut points = std::collections::HashMap::new();
        points.insert((0, 0), start);
        points.insert((1, 1), mid);
        points.insert((2, 2), end);
        parent.insert((1, 1), (0, 0));
        parent.insert((2, 2), (1, 1));
        let route = backtrack_route(start, (2, 2), &parent, &points);
        assert_eq!(route.len(), 3);
        assert_eq!(route[0], start);
    }

    #[test]
    fn routed_legs_respect_no_go_zone() {
        use crate::{
            calculate_sota_routing, polar, simulation_grib, IsochroneConfig, Landmask,
            ObjectiveWeights, Polar, SimplePolar, SotaRoutingConfig,
        };
        use chrono::Utc;

        let start = Point::new(47.55, -3.48);
        let dest = Point::new(43.12, 5.93);
        let config = SotaRoutingConfig::route_only(IsochroneConfig {
            start,
            destination: Some(dest),
            time_limit_hours: 200.0,
            ..Default::default()
        });
        let r = calculate_sota_routing(
            config,
            ObjectiveWeights::default(),
            Landmask::new().unwrap(),
            Box::new(SimplePolar::default_voilier()),
            Box::new(simulation_grib(42).with_epoch(Utc::now())),
            Utc::now(),
        );
        for (i, leg) in r.route_legs.iter().enumerate().take(30) {
            let twa_boat = polar::angle_au_vent(leg.boat_heading_deg, leg.wind.direction);
            let twa_track = polar::angle_au_vent(leg.bearing_deg, leg.wind.direction);
            let spd = SimplePolar::default_voilier().speed_ms(twa_boat, leg.wind.speed);
            assert!(
                spd >= 0.05,
                "leg {i} boat TWA {twa_boat} track TWA {twa_track} hdg={} wind={} -> {spd} m/s",
                leg.boat_heading_deg,
                leg.wind.direction
            );
            assert!(
                twa_boat + 1e-6 >= polar::MIN_ANGLE_AU_VENT_DEG
                    && twa_track + 1e-6 >= polar::MIN_ANGLE_AU_VENT_DEG,
                "leg {i} no-go: boat TWA {twa_boat} track TWA {twa_track}"
            );
        }
    }
}
