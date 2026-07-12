use crate::ensemble::eta_percentiles_from_results;
use crate::grib::{BufrGribGridProvider, GribProvider};
use crate::landmask::Landmask;
use crate::objective::ObjectiveWeights;
use crate::polar::{default_routing_polar, Polar, ScaledPolar, MultiSailPolar};
use crate::scenario::ScenarioGribProvider;
use crate::sota_isochrone::calculate_sota_routing;
use crate::types::*;
use chrono::{DateTime, Duration, Utc};
use rayon::prelude::*;

fn med_grid() -> BufrGribGridProvider {
    BufrGribGridProvider::synthetic_mediterranean(43.0, 49.0, -6.0, 9.0, 0.5)
}

/// Run dual-boat opponent routing with multi-scenario route fan.
pub fn calculate_dual_routing(
    config: SotaRoutingConfig,
    weights: ObjectiveWeights,
    landmask: Landmask,
    grib: Box<dyn GribProvider + Send + Sync>,
    opponent: OpponentState,
    start_time: DateTime<Utc>,
    scenarios: &[WeatherScenario],
) -> DualRoutingResult {
    let dest = config.base.destination;

    let scenario_results: Vec<SotaRoutingResult> = scenarios
        .par_iter()
        .map(|scenario| {
            let grib_for_scenario: Box<dyn GribProvider + Send + Sync> =
                Box::new(ScenarioGribProvider::new(med_grid(), scenario.clone()));

            let mut r = calculate_sota_routing(
                config.clone(),
                weights.clone(),
                landmask.clone(),
                default_routing_polar(),
                grib_for_scenario,
                start_time,
            );
            r.scenario_id = Some(scenario.id.clone());
            r
        })
        .collect();

    let mut mine = calculate_sota_routing(
        config.clone(),
        weights.clone(),
        landmask.clone(),
        default_routing_polar(),
        grib,
        start_time,
    );

    let mut opp_config = config.clone();
    opp_config.base.start = opponent.position;

    let opp_polar: Box<dyn Polar + Send + Sync> = Box::new(ScaledPolar::new(
        MultiSailPolar::default_voilier(),
        opponent.polar_scale,
    ));
    let opp_start =
        start_time + Duration::seconds((opponent.start_time_offset_hours * 3600.0) as i64);

    let opponent_result = calculate_sota_routing(
        opp_config,
        weights,
        landmask,
        opp_polar,
        Box::new(med_grid()),
        opp_start,
    );

    let eta_delta_hours = match (mine.best_eta_hours, opponent_result.best_eta_hours) {
        (Some(a), Some(b)) => Some(a - b),
        _ => None,
    };

    let cover_headings_deg = compute_cover_headings(config.base.start, opponent.position, dest);

    let etas: Vec<f64> = scenario_results
        .iter()
        .filter_map(|r| r.best_eta_hours)
        .collect();
    let combined_eta_percentiles = if etas.is_empty() {
        None
    } else {
        Some(eta_percentiles_from_results(&etas))
    };
    mine.eta_percentiles = combined_eta_percentiles;

    DualRoutingResult {
        mine,
        opponent: opponent_result,
        eta_delta_hours,
        cover_headings_deg,
        scenario_results,
        combined_eta_percentiles,
    }
}

/// Headings from `me` that keep `mark` between self and opponent (cover geometry).
pub fn compute_cover_headings(me: Point, opponent: Point, mark: Option<Point>) -> Vec<f64> {
    let Some(mark) = mark else {
        return Vec::new();
    };
    let bearing_to_mark = me.bearing_to(&mark);
    let spread = 15.0;
    vec![
        (bearing_to_mark - spread + 360.0) % 360.0,
        bearing_to_mark,
        (bearing_to_mark + spread) % 360.0,
        (me.bearing_to(&opponent) + 180.0) % 360.0,
    ]
}

/// Tack vs hold: optimistic ETA hours to destination.
pub fn tack_decision_eta(from: Point, to: Point, boat_speed_ms: f64) -> (f64, f64) {
    let dist = from.distance_to(&to);
    let speed = boat_speed_ms.max(0.5);
    let eta = dist / speed / 3600.0;
    (eta, eta * 1.05)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grib::SimpleGribProvider;

    #[test]
    fn cover_headings_generated() {
        let h = compute_cover_headings(
            Point::new(47.0, -3.0),
            Point::new(47.5, -2.5),
            Some(Point::new(43.0, 5.0)),
        );
        assert!(!h.is_empty());
    }

    #[test]
    fn dual_routing_runs() {
        let config = SotaRoutingConfig {
            base: IsochroneConfig {
                time_limit_hours: 2.0,
                ..IsochroneConfig::default()
            },
            enable_destination_prune: false,
            ..SotaRoutingConfig::default()
        };
        let landmask = Landmask::new().unwrap();
        let opponent = OpponentState {
            position: config.base.start,
            polar_scale: 1.0,
            ..OpponentState::default()
        };
        let result = calculate_dual_routing(
            config,
            ObjectiveWeights::default(),
            landmask,
            Box::new(SimpleGribProvider::default()),
            opponent,
            Utc::now(),
            &[WeatherScenario::baseline()],
        );
        assert!(!result.mine.isochrones.is_empty());
        assert!(!result.opponent.isochrones.is_empty());
        assert!(!result.scenario_results.is_empty());
    }

    #[test]
    fn med_grid_produces_isochrones() {
        use crate::sota_isochrone::calculate_sota_routing;
        let config = SotaRoutingConfig {
            base: IsochroneConfig {
                time_limit_hours: 2.0,
                ..IsochroneConfig::default()
            },
            enable_destination_prune: false,
            ..SotaRoutingConfig::default()
        };
        let landmask = Landmask::new().unwrap();
        let result = calculate_sota_routing(
            config,
            ObjectiveWeights::default(),
            landmask,
            default_routing_polar(),
            Box::new(med_grid()),
            Utc::now(),
        );
        assert!(
            !result.isochrones.is_empty(),
            "med_grid routing produced no isochrones"
        );
    }
}
