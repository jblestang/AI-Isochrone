use crate::types::{Point, Wind, Current, SeaState};
use chrono::{DateTime, Utc};
use std::collections::HashMap;

/// Regular grid metadata shared with routing (from GRIB lattice).
#[derive(Debug, Clone, Copy)]
pub struct GribGridSpec {
    pub min_lat: f64,
    pub max_lat: f64,
    pub min_lon: f64,
    pub max_lon: f64,
    pub step_deg: f64,
}

/// Provider pour obtenir les données météorologiques et océaniques
/// Interface abstraite pour charger les données depuis des fichiers GRIB ou autres sources
pub trait GribProvider {
    /// Obtient le vent à un point donné et à un temps donné
    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind>;
    
    /// Obtient le courant à un point donné et à un temps donné
    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current>;

    /// Obtient l'état de la mer (vagues) à un point et temps donnés
    fn get_sea_state(&self, point: &Point, time: DateTime<Utc>) -> Option<SeaState> {
        let _ = (point, time);
        None
    }

    /// Obtient vent, courant et état de mer simultanément
    fn get_environment(
        &self,
        point: &Point,
        time: DateTime<Utc>,
    ) -> (Option<Wind>, Option<Current>, Option<SeaState>) {
        (
            self.get_wind(point, time),
            self.get_current(point, time),
            self.get_sea_state(point, time),
        )
    }
    
    /// Optional regular grid metadata when the provider is lattice-based.
    fn grid_spec(&self) -> Option<GribGridSpec> {
        None
    }
    
    /// Obtient le vent et le courant simultanément (optimisation)
    fn get_wind_and_current(&self, point: &Point, time: DateTime<Utc>) -> (Option<Wind>, Option<Current>) {
        (self.get_wind(point, time), self.get_current(point, time))
    }
}

/// Provider simple qui retourne des valeurs constantes (pour tests et développement)
#[derive(Clone)]
pub struct SimpleGribProvider {
    default_wind: Wind,
    default_current: Current,
    default_sea_state: SeaState,
}

impl SimpleGribProvider {
    pub fn new(default_wind: Wind, default_current: Current, default_sea_state: SeaState) -> Self {
        Self {
            default_wind,
            default_current,
            default_sea_state,
        }
    }

    /// Crée un provider avec des valeurs par défaut réalistes pour la zone Lorient-Toulon
    pub fn default() -> Self {
        // Vent moyen de secteur Ouest-Nord-Ouest (250-290°), 10 m/s (~20 nœuds)
        let wind = Wind::new(270.0, 10.0);
        
        // Courant faible méditerranéen, secteur Est, 0.5 m/s (~1 nœud)
        let current = Current::new(90.0, 0.5);

        let sea_state = SeaState::new(1.2, 7.5, 270.0);
        
        Self::new(wind, current, sea_state)
    }
}

impl GribProvider for SimpleGribProvider {
    fn get_wind(&self, _point: &Point, _time: DateTime<Utc>) -> Option<Wind> {
        Some(self.default_wind)
    }

    fn get_current(&self, _point: &Point, _time: DateTime<Utc>) -> Option<Current> {
        Some(self.default_current)
    }

    fn get_sea_state(&self, _point: &Point, _time: DateTime<Utc>) -> Option<SeaState> {
        Some(self.default_sea_state)
    }
}

/// Provider basé sur une grille spatiale (interpolation bilinéaire)
/// Utilise une table de valeurs pour différentes zones
pub struct GridGribProvider {
    // Grille de points avec leurs valeurs de vent et courant
    grid_points: Vec<(Point, Wind, Current)>,
}

impl GridGribProvider {
    pub fn new(grid_points: Vec<(Point, Wind, Current)>) -> Self {
        Self { grid_points }
    }

    /// Interpole les valeurs entre les points de grille les plus proches
    fn interpolate(&self, point: &Point, _time: DateTime<Utc>) -> (Option<Wind>, Option<Current>) {
        if self.grid_points.is_empty() {
            return (None, None);
        }

        // Trouver les points les plus proches (simplifié: prendre le plus proche)
        // En production, utiliser une interpolation bilinéaire sur une grille régulière
        let mut closest = None;
        let mut min_dist = f64::INFINITY;

        for (grid_point, wind, current) in &self.grid_points {
            let dist = point.distance_to(grid_point);
            if dist < min_dist {
                min_dist = dist;
                closest = Some((wind, current));
            }
        }

        if let Some((wind, current)) = closest {
            (Some(*wind), Some(*current))
        } else {
            (None, None)
        }
    }
}

impl GribProvider for GridGribProvider {
    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind> {
        self.interpolate(point, time).0
    }

    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current> {
        self.interpolate(point, time).1
    }
}

/// Provider avec cache pour améliorer les performances
pub struct CachedGribProvider<P: GribProvider> {
    provider: P,
    wind_cache: HashMap<(i32, i32, i64), Wind>, // (lat_rounded, lon_rounded, time_rounded) -> Wind
    current_cache: HashMap<(i32, i32, i64), Current>,
    cache_precision: f64, // Précision du cache en degrés
}

impl<P: GribProvider> CachedGribProvider<P> {
    pub fn new(provider: P, cache_precision: f64) -> Self {
        Self {
            provider,
            wind_cache: HashMap::new(),
            current_cache: HashMap::new(),
            cache_precision,
        }
    }

    fn cache_key(&self, point: &Point, time: DateTime<Utc>) -> (i32, i32, i64) {
        let lat_rounded = (point.lat / self.cache_precision).round() as i32;
        let lon_rounded = (point.lon / self.cache_precision).round() as i32;
        let time_rounded = time.timestamp() / 3600; // Arrondi à l'heure
        (lat_rounded, lon_rounded, time_rounded)
    }
}

impl<P: GribProvider> GribProvider for CachedGribProvider<P> {
    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind> {
        let key = self.cache_key(point, time);
        if let Some(wind) = self.wind_cache.get(&key) {
            return Some(*wind);
        }
        // Note: dans une vraie implémentation, on devrait muter le cache
        // Pour l'instant, on appelle directement le provider sous-jacent
        self.provider.get_wind(point, time)
    }

    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current> {
        let key = self.cache_key(point, time);
        if let Some(current) = self.current_cache.get(&key) {
            return Some(*current);
        }
        self.provider.get_current(point, time)
    }
}

/// Helper pour charger des données GRIB depuis un fichier
/// Note: Cette implémentation est un placeholder. 
/// Pour une vraie implémentation, utiliser une bibliothèque comme `grib-rs` ou `eccodes`
pub struct FileGribProvider {
    // À implémenter avec une vraie bibliothèque GRIB
    _placeholder: (),
}

impl FileGribProvider {
    pub fn from_file(_path: &str) -> Result<Self, String> {
        // TODO: Charger un vrai fichier GRIB
        // Pour l'instant, retourner un placeholder
        Ok(Self { _placeholder: () })
    }
}

impl GribProvider for FileGribProvider {
    fn get_wind(&self, _point: &Point, _time: DateTime<Utc>) -> Option<Wind> {
        // TODO: Lire depuis le fichier GRIB
        None
    }

    fn get_current(&self, _point: &Point, _time: DateTime<Utc>) -> Option<Current> {
        // TODO: Lire depuis le fichier GRIB
        None
    }
}

/// Grille spatio-temporelle simulant GRIB/BUFR avec interpolation bilinéaire
#[derive(Debug, Clone)]
pub struct GribGridCell {
    pub point: Point,
    pub wind: Wind,
    pub current: Current,
    pub sea_state: SeaState,
}

/// Provider GRIB/BUFR basé sur une grille régulière avec interpolation
#[derive(Clone)]
pub struct BufrGribGridProvider {
    cells: Vec<GribGridCell>,
    default_wind: Wind,
    default_current: Current,
    default_sea_state: SeaState,
    /// Reference epoch for time interpolation
    epoch: DateTime<Utc>,
    /// Hours between time slices in synthetic temporal variation
    time_step_hours: f64,
}

impl BufrGribGridProvider {
    pub fn new(cells: Vec<GribGridCell>) -> Self {
        Self {
            cells,
            default_wind: Wind::new(270.0, 10.0),
            default_current: Current::new(90.0, 0.5),
            default_sea_state: SeaState::new(1.0, 8.0, 270.0),
            epoch: Utc::now(),
            time_step_hours: 3.0,
        }
    }

    /// Génère une grille synthétique pour une zone bounding box
    pub fn synthetic_mediterranean(
        min_lat: f64,
        max_lat: f64,
        min_lon: f64,
        max_lon: f64,
        grid_step_deg: f64,
    ) -> Self {
        let mut cells = Vec::new();
        let mut lat = min_lat;
        while lat <= max_lat {
            let mut lon = min_lon;
            while lon <= max_lon {
                let wind_dir = 260.0 + (lat - 45.0) * 2.0;
                let wind_speed = 8.0 + (lon + 3.0).abs() * 0.3;
                let hs = 0.8 + wind_speed * 0.08;
                cells.push(GribGridCell {
                    point: Point::new(lat, lon),
                    wind: Wind::new(wind_dir, wind_speed),
                    current: Current::new(90.0, 0.3 + (lat - 44.0).abs() * 0.05),
                    sea_state: SeaState::new(hs, 7.0 + hs, wind_dir),
                });
                lon += grid_step_deg;
            }
            lat += grid_step_deg;
        }
        Self::new(cells)
    }

    /// Infer regular grid metadata from synthetic cell layout.
    pub fn grid_spec(&self) -> Option<GribGridSpec> {
        if self.cells.is_empty() {
            return None;
        }

        let mut min_lat = f64::INFINITY;
        let mut max_lat = f64::NEG_INFINITY;
        let mut min_lon = f64::INFINITY;
        let mut max_lon = f64::NEG_INFINITY;
        let mut lats: Vec<f64> = Vec::new();
        let mut lons: Vec<f64> = Vec::new();

        for cell in &self.cells {
            min_lat = min_lat.min(cell.point.lat);
            max_lat = max_lat.max(cell.point.lat);
            min_lon = min_lon.min(cell.point.lon);
            max_lon = max_lon.max(cell.point.lon);
            if !lats.iter().any(|&v| (v - cell.point.lat).abs() < 1e-6) {
                lats.push(cell.point.lat);
            }
            if !lons.iter().any(|&v| (v - cell.point.lon).abs() < 1e-6) {
                lons.push(cell.point.lon);
            }
        }

        lats.sort_by(|a, b| a.partial_cmp(b).unwrap());
        lons.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let step_lat = if lats.len() >= 2 {
            lats[1] - lats[0]
        } else {
            0.5
        };
        let step_lon = if lons.len() >= 2 {
            lons[1] - lons[0]
        } else {
            step_lat
        };

        Some(GribGridSpec {
            min_lat,
            max_lat,
            min_lon,
            max_lon,
            step_deg: step_lat.max(step_lon),
        })
    }

    fn interpolate_env(&self, point: &Point, time: DateTime<Utc>) -> (Wind, Current, SeaState) {
        let (mut wind, current, mut sea) = self.interpolate_env_static(point);
        // Temporal: wind speed varies ±10% over 24h cycle
        let hours = (time - self.epoch).num_seconds() as f64 / 3600.0;
        let phase = (hours / self.time_step_hours).sin();
        wind.speed *= 1.0 + 0.1 * phase;
        sea.significant_wave_height_m *= 1.0 + 0.08 * phase;
        (wind, current, sea)
    }

    fn interpolate_env_static(&self, point: &Point) -> (Wind, Current, SeaState) {
        if self.cells.is_empty() {
            return (
                self.default_wind,
                self.default_current,
                self.default_sea_state,
            );
        }

        // Inverse-distance weighted interpolation from 4 nearest cells
        let mut weights: Vec<(f64, &GribGridCell)> = self
            .cells
            .iter()
            .map(|c| {
                let d = point.distance_to(&c.point).max(100.0);
                (1.0 / d, c)
            })
            .collect();
        weights.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        weights.truncate(4);

        let w_sum: f64 = weights.iter().map(|(w, _)| w).sum();
        if w_sum <= 0.0 {
            return (
                self.default_wind,
                self.default_current,
                self.default_sea_state,
            );
        }

        let mut wind_dir_x = 0.0;
        let mut wind_dir_y = 0.0;
        let mut wind_speed = 0.0;
        let mut cur_dir_x = 0.0;
        let mut cur_dir_y = 0.0;
        let mut cur_speed = 0.0;
        let mut hs = 0.0;
        let mut period = 0.0;
        let mut wave_dir_x = 0.0;
        let mut wave_dir_y = 0.0;

        for (w, cell) in &weights {
            let nw = w / w_sum;
            let wr = cell.wind.direction.to_radians();
            wind_dir_x += nw * wr.sin();
            wind_dir_y += nw * wr.cos();
            wind_speed += nw * cell.wind.speed;

            let cr = cell.current.direction.to_radians();
            cur_dir_x += nw * cr.sin();
            cur_dir_y += nw * cr.cos();
            cur_speed += nw * cell.current.speed;

            hs += nw * cell.sea_state.significant_wave_height_m;
            period += nw * cell.sea_state.wave_period_s;
            let wdr = cell.sea_state.wave_direction_deg.to_radians();
            wave_dir_x += nw * wdr.sin();
            wave_dir_y += nw * wdr.cos();
        }

        (
            Wind::new(wind_dir_x.atan2(wind_dir_y).to_degrees(), wind_speed),
            Current::new(cur_dir_x.atan2(cur_dir_y).to_degrees(), cur_speed),
            SeaState::new(hs, period, wave_dir_x.atan2(wave_dir_y).to_degrees()),
        )
    }
}

impl GribProvider for BufrGribGridProvider {
    fn grid_spec(&self) -> Option<GribGridSpec> {
        BufrGribGridProvider::grid_spec(self)
    }

    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind> {
        Some(self.interpolate_env(point, time).0)
    }

    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current> {
        Some(self.interpolate_env(point, time).1)
    }

    fn get_sea_state(&self, point: &Point, time: DateTime<Utc>) -> Option<SeaState> {
        Some(self.interpolate_env(point, time).2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraints::violates_constraints;
    use crate::polar::{angle_au_vent, Polar, SimplePolar};
    use crate::types::RoutingConstraints;

    #[test]
    fn synthetic_mediterranean_supports_routing_headings() {
        let grid = BufrGribGridProvider::synthetic_mediterranean(43.0, 49.0, -6.0, 9.0, 0.5);
        let point = Point::new(47.55, -3.48);
        let (wind, _current, sea) = grid.get_environment(&point, Utc::now());
        let wind = wind.unwrap();
        let sea = sea.unwrap();
        assert!(wind.speed.is_finite() && wind.speed > 0.0);
        assert!(sea.significant_wave_height_m.is_finite());
        assert!(!violates_constraints(
            &RoutingConstraints::default(),
            &wind,
            &sea
        ));

        let polar = SimplePolar::default_voilier();
        let viable = (0..16)
            .filter(|&i| {
                let heading = i as f64 * 22.5;
                let angle = angle_au_vent(heading, wind.direction);
                polar.speed_ms(angle, wind.speed) >= 0.05
            })
            .count();
        assert!(
            viable > 0,
            "expected at least one viable heading, got wind {wind:?}"
        );
    }

    #[test]
    fn synthetic_mediterranean_first_hops_reach_sea() {
        use crate::geometry::{calculate_effective_velocity, move_from_point};
        use crate::landmask::Landmask;
        use crate::sea_state::{DefaultSeaStateModifier, SeaStatePolarModifier};
        use crate::types::IsochroneConfig;

        let grid = BufrGribGridProvider::synthetic_mediterranean(43.0, 49.0, -6.0, 9.0, 0.5);
        let simple = SimpleGribProvider::default();
        let landmask = Landmask::new().unwrap();
        let polar = SimplePolar::default_voilier();
        let modifier = DefaultSeaStateModifier::new(0.35);
        let start = IsochroneConfig::default().start;
        let step_seconds = 300.0;
        let now = Utc::now();

        for (label, grib) in [("med", &grid as &dyn GribProvider), ("simple", &simple)] {
            let (wind, current, sea) = grib.get_environment(&start, now);
            let wind = wind.unwrap();
            let current = current.unwrap();
            let sea = sea.unwrap();
            let mut sea_hops = 0;
            let mut sea_two_hops = 0;
            for i in 0..16 {
                let heading = i as f64 * 22.5;
                let angle = angle_au_vent(heading, wind.direction);
                let boat_speed = polar.speed_ms(angle, wind.speed)
                    * modifier.speed_factor(angle, &sea);
                if boat_speed < 0.05 {
                    continue;
                }
                let (eff_speed, eff_dir) =
                    calculate_effective_velocity(boat_speed, heading, &current);
                let hop1 = move_from_point(&start, eff_dir, eff_speed * step_seconds);
                if landmask.is_sea(&hop1) {
                    sea_hops += 1;
                    continue;
                }
                for j in 0..16 {
                    let h2 = j as f64 * 22.5;
                    let a2 = angle_au_vent(h2, wind.direction);
                    let spd = polar.speed_ms(a2, wind.speed) * modifier.speed_factor(a2, &sea);
                    if spd < 0.05 {
                        continue;
                    }
                    let (es2, ed2) = calculate_effective_velocity(spd, h2, &current);
                    let hop2 = move_from_point(&hop1, ed2, es2 * step_seconds);
                    if landmask.is_sea(&hop2) {
                        sea_two_hops += 1;
                    }
                }
            }
            assert!(
                sea_hops + sea_two_hops > 0,
                "{label} provider produced no sea within two hops (wind {wind:?})"
            );
        }
    }
}
