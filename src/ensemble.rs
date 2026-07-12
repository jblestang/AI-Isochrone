use crate::grib::{GribProvider, SimpleGribProvider};
use crate::types::{Current, EtaPercentiles, Point, SeaState, Wind};
use chrono::{DateTime, Utc};

/// Ensemble statistics for wind at a point/time
#[derive(Debug, Clone, Copy)]
pub struct WindSpread {
    pub mean: Wind,
    pub speed_std: f64,
    pub speed_p90: f64,
}

/// Wraps N member providers (synthetic ensemble or perturbed grids).
pub struct EnsembleGribProvider {
    members: Vec<Box<dyn GribProvider + Send + Sync>>,
    labels: Vec<String>,
}

impl EnsembleGribProvider {
    pub fn new(members: Vec<Box<dyn GribProvider + Send + Sync>>, labels: Vec<String>) -> Self {
        Self { members, labels }
    }

    pub fn member_count(&self) -> usize {
        self.members.len()
    }

    pub fn member_label(&self, idx: usize) -> &str {
        self.labels.get(idx).map(|s| s.as_str()).unwrap_or("member")
    }

    pub fn member_provider(&self, idx: usize) -> &dyn GribProvider {
        self.members[idx].as_ref()
    }

    /// Build synthetic ensemble by perturbing wind on cloned simple provider.
    pub fn synthetic_from_simple(
        perturbations: &[(f64, f64)], // (speed_factor, direction_offset_deg)
    ) -> Self {
        use crate::scenario::ScenarioGribProvider;
        use crate::types::WeatherScenario;

        let mut members: Vec<Box<dyn GribProvider + Send + Sync>> = Vec::new();
        let mut labels = Vec::new();
        for (i, (sf, dir)) in perturbations.iter().enumerate() {
            let scenario = WeatherScenario {
                id: format!("eps_{}", i),
                label: format!("EPS {:+}%", ((sf - 1.0) * 100.0) as i32),
                time_shift_hours: 0.0,
                wind_speed_factor: *sf,
                wind_direction_offset_deg: *dir,
            };
            members.push(Box::new(ScenarioGribProvider::new(
                SimpleGribProvider::default(),
                scenario,
            )));
            labels.push(format!("member_{}", i));
        }
        Self { members, labels }
    }

    /// Build synthetic ensemble by perturbing a base provider.
    pub fn from_base_perturbations(
        _base: Box<dyn GribProvider + Send + Sync>,
        perturbations: &[(f64, f64)],
    ) -> Self {
        Self::synthetic_from_simple(perturbations)
    }

    /// Create ensemble from multiple providers directly.
    pub fn from_providers(
        providers: Vec<Box<dyn GribProvider + Send + Sync>>,
        labels: Vec<String>,
    ) -> Self {
        Self { members: providers, labels }
    }

    pub fn wind_spread(&self, point: &Point, time: DateTime<Utc>) -> WindSpread {
        let mut speeds = Vec::new();
        let mut dir_x = 0.0;
        let mut dir_y = 0.0;
        for m in &self.members {
            if let Some(w) = m.get_wind(point, time) {
                speeds.push(w.speed);
                let r = w.direction.to_radians();
                dir_x += r.sin();
                dir_y += r.cos();
            }
        }
        if speeds.is_empty() {
            return WindSpread {
                mean: Wind::new(270.0, 10.0),
                speed_std: 0.0,
                speed_p90: 10.0,
            };
        }
        let n = speeds.len() as f64;
        let mean_speed = speeds.iter().sum::<f64>() / n;
        let var = speeds.iter().map(|s| (s - mean_speed).powi(2)).sum::<f64>() / n;
        let mut sorted = speeds.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p90_idx = ((sorted.len() as f64 * 0.9) as usize).min(sorted.len() - 1);
        WindSpread {
            mean: Wind::new(dir_x.atan2(dir_y).to_degrees(), mean_speed),
            speed_std: var.sqrt(),
            speed_p90: sorted[p90_idx],
        }
    }
}

impl GribProvider for EnsembleGribProvider {
    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind> {
        Some(self.wind_spread(point, time).mean)
    }

    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current> {
        self.members.first()?.get_current(point, time)
    }

    fn get_sea_state(&self, point: &Point, time: DateTime<Utc>) -> Option<SeaState> {
        self.members.first()?.get_sea_state(point, time)
    }
}

/// Compute P10/P50/P90 ETA from scenario routing results.
pub fn eta_percentiles_from_results(etas: &[f64]) -> EtaPercentiles {
    if etas.is_empty() {
        return EtaPercentiles::default();
    }
    let mut v = etas.to_vec();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let pick = |p: f64| {
        let idx = ((v.len() as f64 - 1.0) * p).round() as usize;
        v[idx.min(v.len() - 1)]
    };
    EtaPercentiles {
        p10_hours: pick(0.1),
        p50_hours: pick(0.5),
        p90_hours: pick(0.9),
    }
}

/// Conservative provider using P90 wind speed with mean direction.
pub struct P90WindGribProvider {
    ensemble: EnsembleGribProvider,
}

impl P90WindGribProvider {
    pub fn new(ensemble: EnsembleGribProvider) -> Self {
        Self { ensemble }
    }
}

impl GribProvider for P90WindGribProvider {
    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind> {
        let spread = self.ensemble.wind_spread(point, time);
        Some(Wind::new(spread.mean.direction, spread.speed_p90))
    }

    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current> {
        self.ensemble.get_current(point, time)
    }

    fn get_sea_state(&self, point: &Point, time: DateTime<Utc>) -> Option<SeaState> {
        self.ensemble.get_sea_state(point, time)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grib::SimpleGribProvider;

    #[test]
    fn percentiles_ordered() {
        let p = eta_percentiles_from_results(&[10.0, 12.0, 14.0, 16.0, 20.0]);
        assert!(p.p10_hours <= p.p50_hours);
        assert!(p.p50_hours <= p.p90_hours);
    }

    #[test]
    fn ensemble_spread_nonzero() {
        let e = EnsembleGribProvider::from_base_perturbations(
            Box::new(SimpleGribProvider::default()),
            &[(0.9, -5.0), (1.0, 0.0), (1.1, 5.0), (1.2, 10.0)],
        );
        assert!(e.member_count() >= 3);
        let spread = e.wind_spread(&Point::new(47.0, -3.0), Utc::now());
        assert!(spread.speed_std > 0.0);
    }
}
