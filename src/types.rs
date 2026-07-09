use ordered_float::OrderedFloat;
use serde::{Deserialize, Serialize};

fn normalize_angle_deg(angle: f64) -> f64 {
    let mut normalized = angle % 360.0;
    if normalized < 0.0 {
        normalized += 360.0;
    }
    normalized
}

/// Point géographique avec latitude et longitude
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Wind {
    pub direction: f64, // Direction d'où vient le vent (0-360)
    pub speed: f64,     // Vitesse du vent en m/s
}

impl Wind {
    pub fn new(direction: f64, speed: f64) -> Self {
        Self {
            direction: normalize_angle_deg(direction),
            speed,
        }
    }
}

/// Courant marin
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Current {
    pub direction: f64, // Direction du courant en degrés (0-360, 0 = Nord)
    pub speed: f64,     // Vitesse du courant en m/s
}

impl Current {
    pub fn new(direction: f64, speed: f64) -> Self {
        Self {
            direction: normalize_angle_deg(direction),
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
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Isochrone {
    pub time_hours: f64,
    pub points: Vec<Point>,
}

/// Sea state from GRIB/BUFR wave models
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct SeaState {
    pub significant_wave_height_m: f64,
    pub wave_period_s: f64,
    /// Direction waves are coming FROM (meteorological convention)
    pub wave_direction_deg: f64,
}

impl SeaState {
    pub fn new(hs: f64, period: f64, direction: f64) -> Self {
        Self {
            significant_wave_height_m: hs,
            wave_period_s: period,
            wave_direction_deg: normalize_angle_deg(direction),
        }
    }
}

/// Hard routing limits (pruned before cost evaluation)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RoutingConstraints {
    pub max_true_wind_ms: Option<f64>,
    pub max_significant_wave_m: Option<f64>,
    pub min_depth_m: Option<f64>,
}

impl Default for RoutingConstraints {
    fn default() -> Self {
        Self {
            max_true_wind_ms: Some(25.0),
            max_significant_wave_m: Some(4.0),
            min_depth_m: None,
        }
    }
}

impl RoutingConstraints {
    pub fn unlimited() -> Self {
        Self {
            max_true_wind_ms: None,
            max_significant_wave_m: None,
            min_depth_m: None,
        }
    }
}

/// Opponent / rival boat for dual routing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OpponentState {
    pub label: String,
    pub position: Point,
    /// Speed factor vs reference polar (1.0 = same boat, 0.85 = slower)
    pub polar_scale: f64,
    pub start_time_offset_hours: f64,
}

impl Default for OpponentState {
    fn default() -> Self {
        Self {
            label: "Opponent".into(),
            position: Point::new(47.85, -3.20),
            polar_scale: 0.92,
            start_time_offset_hours: 0.0,
        }
    }
}

/// One leg of a reconstructed route
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteLeg {
    pub from: Point,
    pub to: Point,
    pub bearing_deg: f64,
    pub distance_nm: f64,
    pub duration_hours: f64,
    pub is_tack: bool,
    pub wind: Wind,
    pub sea_state: SeaState,
}

/// Divergent weather scenario (ensemble member or shifted front)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeatherScenario {
    pub id: String,
    pub label: String,
    /// Time shift applied to GRIB validity (hours)
    pub time_shift_hours: f64,
    /// Wind speed multiplier
    pub wind_speed_factor: f64,
    /// Wind direction offset (degrees)
    pub wind_direction_offset_deg: f64,
}

impl WeatherScenario {
    pub fn baseline() -> Self {
        Self {
            id: "baseline".into(),
            label: "Baseline GRIB".into(),
            time_shift_hours: 0.0,
            wind_speed_factor: 1.0,
            wind_direction_offset_deg: 0.0,
        }
    }

    pub fn front_early() -> Self {
        Self {
            id: "front_early".into(),
            label: "Front 6h early".into(),
            time_shift_hours: -6.0,
            wind_speed_factor: 1.1,
            wind_direction_offset_deg: 15.0,
        }
    }

    pub fn front_late() -> Self {
        Self {
            id: "front_late".into(),
            label: "Front 6h late".into(),
            time_shift_hours: 6.0,
            wind_speed_factor: 0.9,
            wind_direction_offset_deg: -10.0,
        }
    }

    pub fn conservative() -> Self {
        Self {
            id: "p90_wind".into(),
            label: "P90 wind (conservative)".into(),
            time_shift_hours: 0.0,
            wind_speed_factor: 1.15,
            wind_direction_offset_deg: 0.0,
        }
    }
}

/// ETA percentiles from multi-scenario / ensemble runs
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct EtaPercentiles {
    pub p10_hours: f64,
    pub p50_hours: f64,
    pub p90_hours: f64,
}

/// Result of SOTA isochrone computation
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SotaRoutingResult {
    pub isochrones: Vec<Isochrone>,
    pub arrival_envelopes: Vec<ArrivalEnvelope>,
    pub best_route: Option<Vec<Point>>,
    pub route_legs: Vec<RouteLeg>,
    pub best_eta_hours: Option<f64>,
    pub best_cost: Option<f64>,
    pub scenario_id: Option<String>,
    pub eta_percentiles: Option<EtaPercentiles>,
}

/// Dual-boat routing result (opponent routing)
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct DualRoutingResult {
    pub mine: SotaRoutingResult,
    pub opponent: SotaRoutingResult,
    /// My ETA minus opponent ETA at mark (negative = I'm faster)
    pub eta_delta_hours: Option<f64>,
    /// Suggested headings that keep mark between us and opponent
    pub cover_headings_deg: Vec<f64>,
    pub scenario_results: Vec<SotaRoutingResult>,
    pub combined_eta_percentiles: Option<EtaPercentiles>,
}

/// Extended configuration for SOTA multi-criteria isochrone routing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SotaRoutingConfig {
    pub base: IsochroneConfig,
    /// Arrival envelope time band width in minutes
    pub envelope_step_minutes: f64,
    /// Radius around destination to consider "arrived" (meters)
    pub arrival_radius_m: f64,
    /// Use composite cost J instead of pure time for pruning
    pub optimize_cost: bool,
    pub constraints: RoutingConstraints,
    /// Prune nodes whose optimistic ETA exceeds best known + slack (hours)
    pub destination_prune_slack_hours: f64,
    pub enable_destination_prune: bool,
}

impl Default for SotaRoutingConfig {
    fn default() -> Self {
        Self {
            base: IsochroneConfig::default(),
            envelope_step_minutes: 30.0,
            arrival_radius_m: 5000.0,
            optimize_cost: true,
            constraints: RoutingConstraints::default(),
            destination_prune_slack_hours: 4.0,
            enable_destination_prune: true,
        }
    }
}

/// Arrival envelope band: points reachable at destination within a time window
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArrivalEnvelope {
    pub min_eta_hours: f64,
    pub max_eta_hours: f64,
    /// Upwind / departure-side boundary points forming the envelope
    pub boundary_points: Vec<Point>,
}

/// Configuration pour le calcul d'isochrone
#[derive(Debug, Clone, Serialize, Deserialize)]
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
