use crate::grib::GribProvider;
use crate::types::{Current, Point, SeaState, WeatherScenario, Wind};
use chrono::{DateTime, Duration, Utc};

/// Applies divergent scenario transforms on top of a base GRIB provider.
pub struct ScenarioGribProvider<P: GribProvider> {
    base: P,
    scenario: WeatherScenario,
}

impl<P: GribProvider> ScenarioGribProvider<P> {
    pub fn new(base: P, scenario: WeatherScenario) -> Self {
        Self { base, scenario }
    }

    fn shift_time(&self, time: DateTime<Utc>) -> DateTime<Utc> {
        time + Duration::seconds((self.scenario.time_shift_hours * 3600.0) as i64)
    }

    fn transform_wind(&self, wind: Wind) -> Wind {
        Wind::new(
            wind.direction + self.scenario.wind_direction_offset_deg,
            wind.speed * self.scenario.wind_speed_factor,
        )
    }
}

impl<P: GribProvider> GribProvider for ScenarioGribProvider<P> {
    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind> {
        self.base
            .get_wind(point, self.shift_time(time))
            .map(|w| self.transform_wind(w))
    }

    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current> {
        self.base.get_current(point, self.shift_time(time))
    }

    fn get_sea_state(&self, point: &Point, time: DateTime<Utc>) -> Option<SeaState> {
        self.base.get_sea_state(point, self.shift_time(time)).map(|mut s| {
            s.significant_wave_height_m *= self.scenario.wind_speed_factor;
            s
        })
    }
}

/// Standard divergent scenario presets for UI / routing.
pub fn default_scenarios() -> Vec<WeatherScenario> {
    vec![
        WeatherScenario::baseline(),
        WeatherScenario::front_early(),
        WeatherScenario::front_late(),
        WeatherScenario::conservative(),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grib::SimpleGribProvider;

    #[test]
    fn conservative_increases_wind() {
        let base = SimpleGribProvider::default();
        let w_base = base.get_wind(&Point::new(47.0, -3.0), Utc::now()).unwrap();
        let scenario = ScenarioGribProvider::new(base, WeatherScenario::conservative());
        let w = scenario
            .get_wind(&Point::new(47.0, -3.0), Utc::now())
            .unwrap();
        assert!(w.speed > w_base.speed);
    }
}
