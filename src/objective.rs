use crate::geometry::angle_difference;
use crate::types::{Current, SeaState, Wind};
use serde::{Deserialize, Serialize};

/// Configurable weights for the multi-criteria objective function:
/// J = ETA + λ1·wave_risk + λ2·comfort + λ3·manoeuvre_penalty + λ4·safety_margin
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObjectiveWeights {
    pub lambda_wave_risk: f64,
    pub lambda_comfort: f64,
    pub lambda_manoeuvre: f64,
    pub lambda_safety: f64,
}

impl Default for ObjectiveWeights {
    fn default() -> Self {
        Self {
            lambda_wave_risk: 1.0,
            lambda_comfort: 0.5,
            lambda_manoeuvre: 0.3,
            lambda_safety: 2.0,
        }
    }
}

/// Accumulated cost components along a route segment (all in seconds-equivalent units)
#[derive(Debug, Clone, Copy, Default)]
pub struct CostComponents {
    pub eta_seconds: f64,
    pub wave_risk: f64,
    pub comfort: f64,
    pub manoeuvre_penalty: f64,
    pub safety_margin: f64,
}

impl CostComponents {
    pub fn total(&self, weights: &ObjectiveWeights) -> f64 {
        self.eta_seconds
            + weights.lambda_wave_risk * self.wave_risk
            + weights.lambda_comfort * self.comfort
            + weights.lambda_manoeuvre * self.manoeuvre_penalty
            + weights.lambda_safety * self.safety_margin
    }

    pub fn add(&self, other: &CostComponents) -> CostComponents {
        CostComponents {
            eta_seconds: self.eta_seconds + other.eta_seconds,
            wave_risk: self.wave_risk + other.wave_risk,
            comfort: self.comfort + other.comfort,
            manoeuvre_penalty: self.manoeuvre_penalty + other.manoeuvre_penalty,
            safety_margin: self.safety_margin + other.safety_margin,
        }
    }
}

/// Computes incremental cost for a simulation step.
pub fn step_cost(
    step_seconds: f64,
    prev_heading: Option<f64>,
    new_heading: f64,
    wind: &Wind,
    current: &Current,
    sea_state: &SeaState,
    weights: &ObjectiveWeights,
) -> CostComponents {
    let _ = weights;
    let wave_risk = wave_risk_score(sea_state, wind) * step_seconds;
    let comfort = comfort_penalty(sea_state, wind, new_heading) * step_seconds;
    let manoeuvre = manoeuvre_penalty(prev_heading, new_heading) * step_seconds;
    let safety = safety_margin_penalty(sea_state, current, wind) * step_seconds;

    CostComponents {
        eta_seconds: step_seconds,
        wave_risk,
        comfort,
        manoeuvre_penalty: manoeuvre,
        safety_margin: safety,
    }
}

/// Wave risk: higher significant wave height and steepness increase risk.
fn wave_risk_score(sea_state: &SeaState, wind: &Wind) -> f64 {
    let hs_factor = (sea_state.significant_wave_height_m / 4.0).clamp(0.0, 2.0);
    let period_factor = if sea_state.wave_period_s > 0.0 {
        (8.0 / sea_state.wave_period_s).clamp(0.5, 2.0)
    } else {
        1.0
    };
    let wind_factor = (wind.speed / 15.0).clamp(0.0, 2.0);
    hs_factor * period_factor * (0.5 + 0.5 * wind_factor)
}

/// Comfort penalty: seas on the beam and head seas reduce comfort.
fn comfort_penalty(sea_state: &SeaState, wind: &Wind, heading: f64) -> f64 {
    let wave_dir = if sea_state.wave_direction_deg.is_finite() {
        sea_state.wave_direction_deg
    } else {
        wind.direction
    };
    let relative = angle_difference(wave_dir, heading).abs();
    let beam_factor = 1.0 - (relative - 90.0).abs() / 90.0;
    let head_sea = (relative / 45.0).min(1.0);
    let hs = sea_state.significant_wave_height_m;
    hs * (0.3 * head_sea + 0.7 * beam_factor.max(0.0))
}

/// Manoeuvre penalty: penalize large heading changes (tacks/gybes).
fn manoeuvre_penalty(prev_heading: Option<f64>, new_heading: f64) -> f64 {
    match prev_heading {
        None => 0.0,
        Some(prev) => {
            let delta = angle_difference(prev, new_heading).abs();
            if delta < 15.0 {
                0.0
            } else if delta < 45.0 {
                0.01 * delta
            } else {
                0.02 * delta
            }
        }
    }
}

/// Safety margin: shallow water proxy via wave steepness + adverse current vs wind.
fn safety_margin_penalty(sea_state: &SeaState, current: &Current, wind: &Wind) -> f64 {
    let steepness = if sea_state.wave_period_s > 0.0 {
        sea_state.significant_wave_height_m / sea_state.wave_period_s
    } else {
        0.0
    };
    let steep_penalty = (steepness - 0.5).max(0.0) * 2.0;

    let wind_current_angle = angle_difference(wind.direction, current.direction).abs();
    let adverse_current = if wind_current_angle > 120.0 {
        current.speed * 2.0
    } else {
        0.0
    };

    steep_penalty + adverse_current
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{Current, SeaState, Wind};

    #[test]
    fn total_cost_increases_with_wave_height() {
        let weights = ObjectiveWeights::default();
        let calm = step_cost(
            300.0,
            Some(90.0),
            95.0,
            &Wind::new(270.0, 8.0),
            &Current::new(90.0, 0.3),
            &SeaState {
                significant_wave_height_m: 0.5,
                wave_period_s: 8.0,
                wave_direction_deg: 270.0,
            },
            &weights,
        );
        let rough = step_cost(
            300.0,
            Some(90.0),
            95.0,
            &Wind::new(270.0, 15.0),
            &Current::new(90.0, 0.3),
            &SeaState {
                significant_wave_height_m: 3.0,
                wave_period_s: 6.0,
                wave_direction_deg: 270.0,
            },
            &weights,
        );
        assert!(rough.total(&weights) > calm.total(&weights));
    }
}
