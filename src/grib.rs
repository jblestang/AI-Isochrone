use crate::types::{Point, Wind, Current};
use chrono::{DateTime, Utc};
use std::collections::HashMap;

/// Provider pour obtenir les données météorologiques et océaniques
/// Interface abstraite pour charger les données depuis des fichiers GRIB ou autres sources
pub trait GribProvider {
    /// Obtient le vent à un point donné et à un temps donné
    fn get_wind(&self, point: &Point, time: DateTime<Utc>) -> Option<Wind>;
    
    /// Obtient le courant à un point donné et à un temps donné
    fn get_current(&self, point: &Point, time: DateTime<Utc>) -> Option<Current>;
    
    /// Obtient le vent et le courant simultanément (optimisation)
    fn get_wind_and_current(&self, point: &Point, time: DateTime<Utc>) -> (Option<Wind>, Option<Current>) {
        (self.get_wind(point, time), self.get_current(point, time))
    }
}

/// Provider simple qui retourne des valeurs constantes (pour tests et développement)
pub struct SimpleGribProvider {
    default_wind: Wind,
    default_current: Current,
}

impl SimpleGribProvider {
    pub fn new(default_wind: Wind, default_current: Current) -> Self {
        Self {
            default_wind,
            default_current,
        }
    }

    /// Crée un provider avec des valeurs par défaut réalistes pour la zone Lorient-Toulon
    pub fn default() -> Self {
        // Vent moyen de secteur Ouest-Nord-Ouest (250-290°), 10 m/s (~20 nœuds)
        let wind = Wind::new(270.0, 10.0);
        
        // Courant faible méditerranéen, secteur Est, 0.5 m/s (~1 nœud)
        let current = Current::new(90.0, 0.5);
        
        Self::new(wind, current)
    }
}

impl GribProvider for SimpleGribProvider {
    fn get_wind(&self, _point: &Point, _time: DateTime<Utc>) -> Option<Wind> {
        Some(self.default_wind)
    }

    fn get_current(&self, _point: &Point, _time: DateTime<Utc>) -> Option<Current> {
        Some(self.default_current)
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
