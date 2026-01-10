use ordered_float::OrderedFloat;

/// Point géographique avec latitude et longitude
#[derive(Debug, Clone, Copy)]
pub struct Point {
    pub lat: f64,
    pub lon: f64,
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        OrderedFloat(self.lat) == OrderedFloat(other.lat) &&
        OrderedFloat(self.lon) == OrderedFloat(other.lon)
    }
}

impl Eq for Point {}

impl std::hash::Hash for Point {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        OrderedFloat(self.lat).hash(state);
        OrderedFloat(self.lon).hash(state);
    }
}

impl Point {
    pub fn new(lat: f64, lon: f64) -> Self {
        Self { lat, lon }
    }

    /// Distance en mètres entre deux points (formule de Haversine)
    pub fn distance_to(&self, other: &Point) -> f64 {
        const R: f64 = 6371000.0; // Rayon de la Terre en mètres
        
        let dlat = (other.lat - self.lat).to_radians();
        let dlon = (other.lon - self.lon).to_radians();
        
        let a = (dlat / 2.0).sin().powi(2) +
            self.lat.to_radians().cos() *
            other.lat.to_radians().cos() *
            (dlon / 2.0).sin().powi(2);
        
        let c = 2.0 * a.sqrt().asin();
        R * c
    }

    /// Bearing initial en degrés (0-360) de self vers other
    pub fn bearing_to(&self, other: &Point) -> f64 {
        let lat1 = self.lat.to_radians();
        let lat2 = other.lat.to_radians();
        let dlon = (other.lon - self.lon).to_radians();

        let y = dlon.sin() * lat2.cos();
        let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
        
        let bearing = y.atan2(x).to_degrees();
        (bearing + 360.0) % 360.0
    }
}

/// Direction du vent en degrés (0-360, 0 = Nord)
#[derive(Debug, Clone, Copy)]
pub struct Wind {
    pub direction: f64, // Direction d'où vient le vent (0-360)
    pub speed: f64,     // Vitesse du vent en m/s
}

impl Wind {
    pub fn new(direction: f64, speed: f64) -> Self {
        Self {
            direction: direction % 360.0,
            speed,
        }
    }
}

/// Courant marin
#[derive(Debug, Clone, Copy)]
pub struct Current {
    pub direction: f64, // Direction du courant en degrés (0-360, 0 = Nord)
    pub speed: f64,     // Vitesse du courant en m/s
}

impl Current {
    pub fn new(direction: f64, speed: f64) -> Self {
        Self {
            direction: direction % 360.0,
            speed,
        }
    }

    /// Convertit la vitesse en m/s vers nœuds
    pub fn speed_knots(&self) -> f64 {
        self.speed * 1.944
    }
}

/// État d'un nœud dans le graphe d'exploration
#[derive(Debug, Clone)]
pub struct NodeState {
    pub point: Point,
    pub time: f64, // Temps écoulé depuis le départ en secondes
    pub distance: f64, // Distance parcourue en mètres
}

/// Isochrone : ensemble de points atteignables à un temps donné
#[derive(Debug, Clone)]
pub struct Isochrone {
    pub time_hours: f64,
    pub points: Vec<Point>,
}

/// Configuration pour le calcul d'isochrone
#[derive(Debug, Clone)]
pub struct IsochroneConfig {
    pub start: Point,
    pub destination: Option<Point>, // Optionnel pour isochrone simple
    pub time_limit_hours: f64,
    pub isochrone_step_hours: f64, // Intervalle entre isochrones (1h)
    pub simulation_step_minutes: f64, // Pas de simulation (5 min)
    pub max_distance_meters: f64, // Distance maximale pour un pas de 5 min
    pub num_directions: usize, // Nombre de directions explorées (ex: 16 pour 22.5°)
}

impl Default for IsochroneConfig {
    fn default() -> Self {
        Self {
            start: Point::new(47.75, -3.37), // Lorient par défaut
            destination: Some(Point::new(43.12, 5.93)), // Toulon
            time_limit_hours: 24.0,
            isochrone_step_hours: 1.0,
            simulation_step_minutes: 5.0,
            max_distance_meters: 50000.0, // ~50 km max par pas (27 nœuds max)
            num_directions: 16, // 16 directions = 22.5° entre chaque
        }
    }
}

impl IsochroneConfig {
    pub fn simulation_step_seconds(&self) -> f64 {
        self.simulation_step_minutes * 60.0
    }

    pub fn direction_step_degrees(&self) -> f64 {
        360.0 / (self.num_directions as f64)
    }
}
