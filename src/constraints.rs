use crate::types::{RoutingConstraints, SeaState, Wind};

/// Returns true if environment violates hard routing constraints.
pub fn violates_constraints(
    constraints: &RoutingConstraints,
    wind: &Wind,
    sea_state: &SeaState,
) -> bool {
    if let Some(max_wind) = constraints.max_true_wind_ms {
        if wind.speed > max_wind {
            return true;
        }
    }
    if let Some(max_hs) = constraints.max_significant_wave_m {
        if sea_state.significant_wave_height_m > max_hs {
            return true;
        }
    }
    let _ = constraints.min_depth_m;
    false
}

/// Optimistic lower-bound ETA (hours) from point to destination at max boat speed.
pub fn optimistic_eta_hours(
    from: &crate::types::Point,
    to: &crate::types::Point,
    max_speed_ms: f64,
) -> f64 {
    let dist = from.distance_to(to);
    if max_speed_ms <= 0.0 {
        return f64::INFINITY;
    }
    dist / max_speed_ms / 3600.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::SeaState;

    #[test]
    fn wind_limit_blocks() {
        let c = RoutingConstraints {
            max_true_wind_ms: Some(15.0),
            ..RoutingConstraints::default()
        };
        assert!(violates_constraints(
            &c,
            &Wind::new(270.0, 20.0),
            &SeaState::default()
        ));
    }
}
