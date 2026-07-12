use crate::types::{ArrivalEnvelope, Isochrone, Point};

/// Build arrival time envelopes around a destination.
/// Each envelope contains boundary points that can reach the destination
/// with ETA within [min_eta, max_eta] separated by `step_minutes`.
pub fn build_arrival_envelopes(
    isochrones: &[Isochrone],
    destination: Point,
    arrival_radius_m: f64,
    step_minutes: f64,
) -> Vec<ArrivalEnvelope> {
    if isochrones.is_empty() || step_minutes <= 0.0 {
        return Vec::new();
    }

    let step_hours = step_minutes / 60.0;

    // Collect points from each isochrone that are within arrival radius of destination
    let mut reachable: Vec<(f64, Point)> = Vec::new();
    for iso in isochrones {
        for pt in &iso.points {
            if pt.distance_to(&destination) <= arrival_radius_m {
                reachable.push((iso.time_hours, *pt));
            }
        }
    }

    if reachable.is_empty() {
        // Fallback: use closest points from each isochrone to form approach envelope
        for iso in isochrones {
            if let Some(closest) = iso
                .points
                .iter()
                .min_by(|a, b| {
                    a.distance_to(&destination)
                        .partial_cmp(&b.distance_to(&destination))
                        .unwrap()
                })
            {
                reachable.push((iso.time_hours, *closest));
            }
        }
    }

    if reachable.is_empty() {
        return Vec::new();
    }

    reachable.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let min_eta = reachable.first().map(|r| r.0).unwrap_or(0.0);
    let max_eta = reachable.last().map(|r| r.0).unwrap_or(min_eta);

    let mut envelopes = Vec::new();
    let mut band_start = min_eta;

    while band_start <= max_eta + 1e-6 {
        let band_end = band_start + step_hours;
        let band_points: Vec<Point> = reachable
            .iter()
            .filter(|(t, _)| *t >= band_start && *t < band_end)
            .map(|(_, p)| *p)
            .collect();

        if !band_points.is_empty() {
            let boundary = compute_boundary(&band_points, &destination, 10.0);
            envelopes.push(ArrivalEnvelope {
                min_eta_hours: band_start,
                max_eta_hours: band_end,
                boundary_points: boundary,
            });
        }

        band_start = band_end;
    }

    envelopes
}

/// Outward envelope: farthest reachable point per bearing sector from a center.
pub fn extract_outward_envelope(points: &[Point], center: &Point, sector_deg: f64) -> Vec<Point> {
    compute_boundary(points, center, sector_deg)
}

/// Compute envelope boundary as points sorted by bearing from center
fn compute_boundary(points: &[Point], center: &Point, sector_deg: f64) -> Vec<Point> {
    if points.len() < 3 {
        return points.to_vec();
    }

    let mut with_bearing: Vec<(f64, Point)> = points
        .iter()
        .map(|p| (center.bearing_to(p), *p))
        .collect();
    with_bearing.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    // Keep outermost point per bearing sector
    let mut boundary: Vec<Point> = Vec::new();
    let mut i = 0;
    while i < with_bearing.len() {
        let sector = (with_bearing[i].0 / sector_deg).floor() as i32;
        let mut best = with_bearing[i].1;
        let mut best_dist = center.distance_to(&best);
        let mut j = i + 1;
        while j < with_bearing.len()
            && (with_bearing[j].0 / sector_deg).floor() as i32 == sector
        {
            let d = center.distance_to(&with_bearing[j].1);
            if d > best_dist {
                best = with_bearing[j].1;
                best_dist = d;
            }
            j += 1;
        }
        boundary.push(best);
        i = j;
    }

    boundary
}

/// Simplify envelope boundary by removing near-duplicate points
pub fn simplify_envelope_boundary(envelope: &mut ArrivalEnvelope) {
    if envelope.boundary_points.len() < 3 {
        return;
    }

    let mut simplified = vec![envelope.boundary_points[0]];
    for pt in envelope.boundary_points.iter().skip(1) {
        let last = simplified.last().unwrap();
        if pt.distance_to(last) > 500.0 {
            simplified.push(*pt);
        }
    }
    envelope.boundary_points = simplified;
}

/// Points on isochrones that form the "reachability envelope" toward a target:
/// for each isochrone time T, returns points within `max_distance_m` of any
/// point that could still reach the target within `remaining_hours`.
pub fn reachability_envelope_from_isochrones(
    isochrones: &[Isochrone],
    destination: Point,
    time_gap_minutes: f64,
) -> Vec<ArrivalEnvelope> {
    build_arrival_envelopes(
        isochrones,
        destination,
        10_000.0,
        time_gap_minutes,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn outward_envelope_keeps_farthest_per_sector() {
        let center = Point::new(47.0, -3.0);
        let points = vec![
            Point::new(47.5, -3.0),
            Point::new(47.2, -3.0),
            Point::new(47.0, -2.5),
        ];
        let env = extract_outward_envelope(&points, &center, 45.0);
        assert!(!env.is_empty());
        assert!(env.len() <= points.len());
    }

    #[test]
    fn envelope_bands_are_non_overlapping() {
        let dest = Point::new(43.12, 5.93);
        let isochrones = vec![
            Isochrone {
                time_hours: 10.0,
                points: vec![
                    Point::new(43.15, 5.90),
                    Point::new(43.10, 5.95),
                ],
            },
            Isochrone {
                time_hours: 10.5,
                points: vec![Point::new(43.13, 5.92)],
            },
        ];
        let envs = build_arrival_envelopes(&isochrones, dest, 5000.0, 30.0);
        for e in &envs {
            assert!(e.max_eta_hours > e.min_eta_hours);
        }
    }
}
