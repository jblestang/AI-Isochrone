use crate::types::{Point, Wind, Current, SeaState};
use chrono::{DateTime, Utc};
use std::collections::HashMap;

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
    
    /// Obtient le vent et le courant simultanément (optimisation)
    fn get_wind_and_current(&self, point: &Point, time: DateTime<Utc>) -> (Option<Wind>, Option<Current>) {
        (self.get_wind(point, time), self.get_current(point, time))
    }
}

/// Provider simple qui retourne des valeurs constantes (pour tests et développement)
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
pub struct BufrGribGridProvider {
    cells: Vec<GribGridCell>,
    default_wind: Wind,
    default_current: Current,
    default_sea_state: SeaState,
}

impl BufrGribGridProvider {
    pub fn new(cells: Vec<GribGridCell>) -> Self {
        Self {
            cells,
            default_wind: Wind::new(270.0, 10.0),
            default_current: Current::new(90.0, 0.5),
            default_sea_state: SeaState::new(1.0, 8.0, 270.0),
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

    fn interpolate_env(&self, point: &Point) -> (Wind, Current, SeaState) {
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
    fn get_wind(&self, point: &Point, _time: DateTime<Utc>) -> Option<Wind> {
        Some(self.interpolate_env(point).0)
    }

    fn get_current(&self, point: &Point, _time: DateTime<Utc>) -> Option<Current> {
        Some(self.interpolate_env(point).1)
    }

    fn get_sea_state(&self, point: &Point, _time: DateTime<Utc>) -> Option<SeaState> {
        Some(self.interpolate_env(point).2)
    }
}
