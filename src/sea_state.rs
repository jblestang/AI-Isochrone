use crate::polar::Polar;
use crate::types::SeaState;

/// Modifies boat polar performance based on sea state.
pub trait SeaStatePolarModifier: Send + Sync {
    /// Returns speed factor in [0, 1] applied to base polar speed.
    fn speed_factor(&self, angle_au_vent: f64, sea_state: &SeaState) -> f64;

    /// Optional polar tweak: returns adjusted speed in knots.
    fn adjusted_speed_knots(
        &self,
        base_polar: &dyn Polar,
        angle_au_vent: f64,
        wind_speed_ms: f64,
        sea_state: &SeaState,
    ) -> f64 {
        let base = base_polar.speed_knots(angle_au_vent, wind_speed_ms);
        base * self.speed_factor(angle_au_vent, sea_state)
    }
}

/// Default sea-state modifier: reduces speed in head/b beam seas.
#[derive(Debug, Clone, Default)]
pub struct DefaultSeaStateModifier {
    /// Maximum speed reduction fraction (0.0 = no reduction, 0.5 = 50% slower)
    pub max_reduction: f64,
}

impl DefaultSeaStateModifier {
    pub fn new(max_reduction: f64) -> Self {
        Self { max_reduction }
    }
}

impl SeaStatePolarModifier for DefaultSeaStateModifier {
    fn speed_factor(&self, angle_au_vent: f64, sea_state: &SeaState) -> f64 {
        let hs = sea_state.significant_wave_height_m;
        if hs < 0.1 {
            return 1.0;
        }

        // Head seas (low angle) hurt more than following seas
        let head_factor = 1.0 - (angle_au_vent / 180.0);
        let wave_stress = (hs / 5.0).clamp(0.0, 1.0);
        let reduction = self.max_reduction * wave_stress * (0.4 + 0.6 * head_factor);
        (1.0 - reduction).clamp(0.3, 1.0)
    }
}

/// Composite polar wrapping base polar with sea-state adjustment.
pub struct SeaStateAdjustedPolar<P: Polar, M: SeaStatePolarModifier> {
    base: P,
    modifier: M,
    sea_state: SeaState,
}

impl<P: Polar + Clone, M: SeaStatePolarModifier + Clone> SeaStateAdjustedPolar<P, M> {
    pub fn new(base: P, modifier: M, sea_state: SeaState) -> Self {
        Self {
            base,
            modifier,
            sea_state,
        }
    }

    pub fn with_sea_state(&self, sea_state: SeaState) -> Self {
        Self {
            base: self.base.clone(),
            modifier: self.modifier.clone(),
            sea_state,
        }
    }

    pub fn set_sea_state(&mut self, sea_state: SeaState) {
        self.sea_state = sea_state;
    }
}

impl<P: Polar + Clone, M: SeaStatePolarModifier + Clone> Polar for SeaStateAdjustedPolar<P, M> {
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        self.modifier.adjusted_speed_knots(
            &self.base,
            angle_au_vent,
            wind_speed_ms,
            &self.sea_state,
        )
    }
}

/// Tweaks polar table values for a given sea state (returns new SimplePolar-compatible table).
pub fn tweak_polar_table(
    base_speeds: &[Vec<f64>],
    angles: &[f64],
    sea_state: &SeaState,
    modifier: &dyn SeaStatePolarModifier,
) -> Vec<Vec<f64>> {
    base_speeds
        .iter()
        .map(|row| {
            row.iter()
                .enumerate()
                .map(|(i, &speed)| {
                    let angle = angles.get(i).copied().unwrap_or(90.0);
                    speed * modifier.speed_factor(angle, sea_state)
                })
                .collect()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polar::SimplePolar;

    #[test]
    fn head_sea_reduces_speed_more_than_following() {
        let modifier = DefaultSeaStateModifier::new(0.4);
        let sea = SeaState {
            significant_wave_height_m: 2.5,
            wave_period_s: 7.0,
            wave_direction_deg: 0.0,
        };
        let head = modifier.speed_factor(30.0, &sea);
        let following = modifier.speed_factor(150.0, &sea);
        assert!(head < following);
    }

    #[test]
    fn adjusted_polar_slower_in_rough_seas() {
        let base = SimplePolar::default_voilier();
        let modifier = DefaultSeaStateModifier::new(0.35);
        let calm = SeaState {
            significant_wave_height_m: 0.2,
            wave_period_s: 10.0,
            wave_direction_deg: 270.0,
        };
        let rough = SeaState {
            significant_wave_height_m: 3.5,
            wave_period_s: 6.0,
            wave_direction_deg: 270.0,
        };
        let polar_calm = SeaStateAdjustedPolar::new(base.clone(), modifier.clone(), calm);
        let polar_rough = SeaStateAdjustedPolar::new(base, modifier, rough);
        assert!(
            polar_rough.speed_knots(60.0, 10.0) < polar_calm.speed_knots(60.0, 10.0)
        );
    }
}
