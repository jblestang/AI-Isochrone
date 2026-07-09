/// Polaire de bateau : vitesse en fonction de l'angle au vent et de la force du vent
/// 
/// La polaire définit la vitesse du bateau (en nœuds) en fonction de:
/// - L'angle au vent (angle entre le cap du bateau et la direction d'où vient le vent)
/// - La force du vent (en m/s ou Beaufort)
pub trait Polar {
    /// Retourne la vitesse du bateau en nœuds pour un angle au vent et une force de vent donnés
    /// angle_au_vent: angle entre le cap du bateau et la direction du vent (0° = vent de face, 180° = vent arrière)
    /// wind_speed_ms: vitesse du vent en m/s
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64;
    
    /// Retourne la vitesse en m/s
    fn speed_ms(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        self.speed_knots(angle_au_vent, wind_speed_ms) / 1.944
    }
}

/// Polaire simple basée sur des tables de valeurs
/// Utilise une interpolation bilinéaire
#[derive(Clone)]
pub struct SimplePolar {
    /// Angles au vent en degrés (clés de la table)
    angles: Vec<f64>,
    /// Forces de vent en m/s (clés de la table)
    wind_speeds: Vec<f64>,
    /// Table [angle_index][wind_index] -> vitesse en nœuds
    speed_table: Vec<Vec<f64>>,
}

impl SimplePolar {
    /// Crée une nouvelle polaire simple avec une table de valeurs
    pub fn new(angles: Vec<f64>, wind_speeds: Vec<f64>, speed_table: Vec<Vec<f64>>) -> Self {
        Self {
            angles,
            wind_speeds,
            speed_table,
        }
    }

    /// Crée une polaire par défaut pour un voilier type (exemple)
    /// Cette polaire est un exemple simplifié et devrait être remplacée par des données réelles
    pub fn default_voilier() -> Self {
        // Angles au vent: 30° (no-go limit), 45°, 60°, 90°, 120°, 135°, 150°, 180° (vent arrière)
        let angles = vec![30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 180.0];
        
        // Forces de vent en m/s: 2.5, 5, 7.5, 10, 12.5, 15 m/s (~5, 10, 15, 20, 25, 30 nœuds)
        let wind_speeds = vec![2.5, 5.0, 7.5, 10.0, 12.5, 15.0];
        
        // Table de vitesses en nœuds [wind][angle]
        // IMPORTANT: Le nombre de lignes doit correspondre exactement au nombre d'éléments dans wind_speeds
        // Exemple simplifié: plus rapide au largue, limité au près et vent arrière
        let speed_table = vec![
            // 30° 45°  60°   90°  120° 135° 150° 180°
            vec![0.0, 4.0, 5.0, 6.0, 7.0, 6.5, 5.5, 4.0], // 2.5 m/s (index 0)
            vec![0.0, 5.5, 6.5, 8.0, 9.5, 9.0, 7.5, 5.5], // 5.0 m/s (index 1)
            vec![0.0, 6.5, 8.0, 9.5, 11.0, 10.5, 9.0, 6.5], // 7.5 m/s (index 2)
            vec![0.0, 7.5, 9.0, 10.5, 12.0, 11.5, 10.0, 7.0], // 10.0 m/s (index 3)
            vec![0.0, 8.0, 9.5, 11.0, 12.5, 12.0, 10.5, 7.5], // 12.5 m/s (index 4)
            vec![0.0, 8.5, 10.0, 11.5, 13.0, 12.5, 11.0, 8.0], // 15.0 m/s (index 5)
        ];
        
        Self::new(angles, wind_speeds, speed_table)
    }
}

/// Polar scaled by a factor (opponent slower/faster boat).
#[derive(Clone)]
pub struct ScaledPolar<P: Polar + Clone> {
    inner: P,
    scale: f64,
}

impl<P: Polar + Clone> ScaledPolar<P> {
    pub fn new(inner: P, scale: f64) -> Self {
        Self { inner, scale }
    }
}

impl<P: Polar + Clone> Polar for ScaledPolar<P> {
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        self.inner.speed_knots(angle_au_vent, wind_speed_ms) * self.scale
    }
}

impl SimplePolar {
    fn interpolate_1d(&self, x: f64, x_values: &[f64], y_values: &[f64]) -> f64 {
        if x <= x_values[0] {
            return y_values[0];
        }
        if x >= x_values[x_values.len() - 1] {
            return y_values[y_values.len() - 1];
        }

        for i in 0..x_values.len() - 1 {
            if x >= x_values[i] && x <= x_values[i + 1] {
                let t = (x - x_values[i]) / (x_values[i + 1] - x_values[i]);
                return y_values[i] * (1.0 - t) + y_values[i + 1] * t;
            }
        }
        y_values[y_values.len() - 1]
    }
}

impl Polar for SimplePolar {
    fn speed_knots(&self, angle_au_vent: f64, wind_speed_ms: f64) -> f64 {
        let angle = angle_au_vent.min(180.0).max(0.0);
        if angle < MIN_ANGLE_AU_VENT_DEG {
            return 0.0;
        }
        
        // Vérifier que la table a le bon nombre de lignes
        if self.speed_table.len() < self.wind_speeds.len() {
            eprintln!("⚠️  Erreur polaire: speed_table a {} lignes mais wind_speeds a {} valeurs", 
                     self.speed_table.len(), self.wind_speeds.len());
            // Fallback: retourner une vitesse minimale
            return 2.0;
        }
        
        // Interpolation bilinéaire
        // 1. Interpoler pour chaque angle au niveau des vitesses de vent
        let mut speeds_at_winds = Vec::new();
        for wind_idx in 0..self.wind_speeds.len() {
            if wind_idx >= self.speed_table.len() {
                eprintln!("⚠️  Erreur polaire: wind_idx {} >= speed_table.len() {}", wind_idx, self.speed_table.len());
                // Utiliser la dernière ligne disponible
                if !self.speed_table.is_empty() {
                    let last_idx = self.speed_table.len() - 1;
                    let mut speeds_at_angle = Vec::new();
                    for angle_idx in 0..self.angles.len() {
                        if angle_idx < self.speed_table[last_idx].len() {
                            speeds_at_angle.push(self.speed_table[last_idx][angle_idx]);
                        } else {
                            speeds_at_angle.push(0.0);
                        }
                    }
                    let speed = self.interpolate_1d(angle, &self.angles, &speeds_at_angle);
                    speeds_at_winds.push(speed);
                } else {
                    speeds_at_winds.push(2.0); // Fallback
                }
                continue;
            }
            
            let mut speeds_at_angle = Vec::new();
            for angle_idx in 0..self.angles.len() {
                if angle_idx < self.speed_table[wind_idx].len() {
                    speeds_at_angle.push(self.speed_table[wind_idx][angle_idx]);
                } else {
                    eprintln!("⚠️  Erreur polaire: angle_idx {} >= speed_table[{}].len() {}", 
                             angle_idx, wind_idx, self.speed_table[wind_idx].len());
                    speeds_at_angle.push(0.0);
                }
            }
            let speed = self.interpolate_1d(angle, &self.angles, &speeds_at_angle);
            speeds_at_winds.push(speed);
        }
        
        // 2. Interpoler selon la vitesse du vent
        let result = self.interpolate_1d(wind_speed_ms, &self.wind_speeds, &speeds_at_winds);

        result.max(0.0)
    }
}

/// Minimum angle au vent (no-go zone). Polars are zero for TWA strictly below 30°.
pub const MIN_ANGLE_AU_VENT_DEG: f64 = 30.0;

/// Calcule l'angle au vent à partir du cap du bateau et de la direction du vent.
/// Retourne l'angle au vent en degrés (0-180).
pub fn angle_au_vent(boat_heading: f64, wind_direction: f64) -> f64 {
    let diff = (boat_heading - wind_direction).abs() % 360.0;
    diff.min(360.0 - diff).min(180.0)
}

/// Signed angle from wind direction to boat heading in (-180, 180].
/// Positive = wind on starboard, negative = wind on port.
pub fn signed_wind_side(boat_heading: f64, wind_direction: f64) -> f64 {
    let diff = (boat_heading - wind_direction).rem_euclid(360.0);
    if diff > 180.0 {
        diff - 360.0
    } else {
        diff
    }
}

/// Minimum heading change to count as a manoeuvre (tack or gybe).
pub const TACK_GYBE_MIN_HEADING_DELTA_DEG: f64 = 35.0;

/// True when a significant heading change crosses the wind (tack or gybe).
pub fn is_tack_or_gybe(prev_heading: f64, new_heading: f64, wind_direction: f64) -> bool {
    use crate::geometry::angle_difference;

    if angle_difference(prev_heading, new_heading).abs() < TACK_GYBE_MIN_HEADING_DELTA_DEG {
        return false;
    }
    let prev_side = signed_wind_side(prev_heading, wind_direction);
    let next_side = signed_wind_side(new_heading, wind_direction);
    prev_side.signum() != next_side.signum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_angle_au_vent() {
        // Vent de 0° (Nord), bateau au 90° (Est) -> angle au vent = 90°
        assert!((angle_au_vent(90.0, 0.0) - 90.0).abs() < 1e-10);
        
        // Vent de 0°, bateau au 180° (Sud) -> angle au vent = 180° (vent arrière)
        assert!((angle_au_vent(180.0, 0.0) - 180.0).abs() < 1e-10);
        
        // Vent de 0°, bateau au 0° -> angle au vent = 0° (vent de face)
        assert!((angle_au_vent(0.0, 0.0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn no_go_zone_returns_zero_speed() {
        let polar = SimplePolar::default_voilier();
        assert_eq!(polar.speed_knots(20.0, 10.0), 0.0);
        assert_eq!(polar.speed_knots(29.0, 10.0), 0.0);
        assert!(polar.speed_ms(35.0, 10.0) > 0.05);
    }

    #[test]
    fn test_polar_interpolation() {
        let polar = SimplePolar::default_voilier();
        let speed = polar.speed_knots(90.0, 5.0);
        assert!(speed > 0.0);
        assert!(speed < 20.0); // Vérification raisonnable
    }

    #[test]
    fn tack_crosses_wind_side() {
        // West wind, tack from NW (starboard) to SW (port)
        assert!(is_tack_or_gybe(315.0, 225.0, 270.0));
    }

    #[test]
    fn bear_away_on_same_tack_is_not_a_manoeuvre() {
        // West wind, bear away from NW toward N on starboard tack
        assert!(!is_tack_or_gybe(315.0, 340.0, 270.0));
    }

    #[test]
    fn small_heading_adjustment_is_not_a_manoeuvre() {
        assert!(!is_tack_or_gybe(315.0, 330.0, 270.0));
    }

    #[test]
    fn gybe_crosses_wind_side_downwind() {
        // North wind, gybe from SE to SW
        assert!(is_tack_or_gybe(135.0, 225.0, 0.0));
    }
}
