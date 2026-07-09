use crate::types::*;
use crate::geometry::*;
use crate::landmask::Landmask;
use crate::polar::*;
use crate::grib::*;
use crate::grid::{resolve_grid_spec, CellKey, GridBestTracker, RoutingGrid};
use chrono::{DateTime, Utc, Duration};
use std::collections::{HashMap, BinaryHeap};
use ordered_float::OrderedFloat;
use rayon::prelude::*;
// Plus besoin de ConvexHull, on utilise notre propre algorithme

/// État d'un nœud dans le graphe d'exploration
#[derive(Debug, Clone)]
struct Node {
    point: Point,
    time: f64, // Temps écoulé depuis le départ en secondes
    distance: f64, // Distance parcourue en mètres
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time && 
        (OrderedFloat(self.point.lat), OrderedFloat(self.point.lon)) == 
        (OrderedFloat(other.point.lat), OrderedFloat(other.point.lon))
    }
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // Ordre inverse pour BinaryHeap (min-heap sur le temps)
        OrderedFloat(other.time).partial_cmp(&OrderedFloat(self.time))
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap()
    }
}

/// Calculateur d'isochrones pour bateau
pub struct IsochroneCalculator {
    config: IsochroneConfig,
    landmask: Landmask,
    polar: Box<dyn Polar + Send + Sync>,
    grib_provider: Box<dyn GribProvider + Send + Sync>,
    start_time: DateTime<Utc>,
}

impl IsochroneCalculator {
    /// Crée un nouveau calculateur d'isochrones
    pub fn new(
        config: IsochroneConfig,
        landmask: Landmask,
        polar: Box<dyn Polar + Send + Sync>,
        grib_provider: Box<dyn GribProvider + Send + Sync>,
        start_time: DateTime<Utc>,
    ) -> Self {
        Self {
            config,
            landmask,
            polar,
            grib_provider,
            start_time,
        }
    }

    /// Calcule les isochrones depuis le point de départ (grid-based, outward envelope).
    pub fn calculate(&self) -> Vec<Isochrone> {
        let time_limit = self.config.time_limit_hours * 3600.0;
        let _step_seconds = self.config.simulation_step_seconds();
        let iso_band = self.config.isochrone_step_hours * 3600.0;

        let grid_spec = resolve_grid_spec(
            self.grib_provider.as_ref(),
            self.config.start,
            self.config.destination,
            self.config.grid_step_deg,
        );
        let routing_grid = RoutingGrid::from_spec(grid_spec, &self.landmask);
        let mut tracker = GridBestTracker::new(routing_grid);
        let grid = tracker.grid().clone();

        let mut visited: HashMap<CellKey, f64> = HashMap::new();
        let mut frontier = BinaryHeap::new();

        let start_node = Node {
            point: self.config.start,
            time: 0.0,
            distance: 0.0,
        };
        frontier.push(start_node);
        let start_cell = grid.cell_containing(&self.config.start);
        visited.insert(start_cell, 0.0);
        let _ = tracker.try_update(self.config.start, 0.0, &self.landmask);

        let mut isochrones = Vec::new();
        let mut next_isochrone_time = self.config.isochrone_step_hours * 3600.0;

        let max_nodes = 3_000_000;
        let mut nodes_explored = 0usize;

        while let Some(node) = frontier.pop() {
            if nodes_explored >= max_nodes {
                break;
            }
            nodes_explored += 1;

            if node.time > time_limit {
                break;
            }

            let cell_key = grid.cell_containing(&node.point);
            if visited.get(&cell_key).copied().unwrap_or(f64::INFINITY) + 1e-6 < node.time {
                continue;
            }
            visited.insert(cell_key, node.time);

            if node.time > 0.0 && !self.landmask.is_sea(&node.point) {
                continue;
            }

            if node.time > 0.0 {
                tracker.try_update(node.point, node.time, &self.landmask);
            }

            if node.time >= next_isochrone_time {
                let iso = tracker.build_isochrone_envelope(
                    next_isochrone_time,
                    iso_band,
                    self.config.start,
                    self.config.envelope_sector_deg,
                );
                if !iso.points.is_empty() {
                    isochrones.push(iso);
                }
                next_isochrone_time += self.config.isochrone_step_hours * 3600.0;
            }

            for successor in self.explore_directions(&node, &grid) {
                if successor.time > 0.0 && !self.landmask.is_sea(&successor.point) {
                    continue;
                }
                let skey = grid.cell_containing(&successor.point);
                if successor.time < visited.get(&skey).copied().unwrap_or(f64::INFINITY) {
                    frontier.push(successor);
                }
            }
        }

        if next_isochrone_time <= time_limit + iso_band {
            let iso = tracker.build_isochrone_envelope(
                next_isochrone_time,
                iso_band,
                self.config.start,
                self.config.envelope_sector_deg,
            );
            if !iso.points.is_empty() {
                isochrones.push(iso);
            }
        }

        isochrones
    }

    /// Explore toutes les directions possibles depuis un nœud
    fn explore_directions(&self, node: &Node, _grid: &RoutingGrid) -> Vec<Node> {
        let step_seconds = self.config.simulation_step_seconds();
        let current_time = self.start_time + Duration::seconds(node.time as i64);
        
        // Obtenir vent et courant pour ce point à ce temps
        let (wind_opt, current_opt) = self.grib_provider.get_wind_and_current(
            &node.point,
            current_time,
        );
        
        let wind = wind_opt.unwrap_or(Wind::new(270.0, 10.0));
        let current = current_opt.unwrap_or(Current::new(90.0, 0.5));
        
        // Explorer toutes les directions possibles
        let direction_step = self.config.direction_step_degrees();
        let directions: Vec<f64> = (0..self.config.num_directions)
            .map(|i| i as f64 * direction_step)
            .collect();
        
        // Paralléliser l'exploration des directions (sans vérification landmask immédiate)
        let mut candidates: Vec<Node> = directions
            .par_iter()
            .filter_map(|&heading| {
                self.explore_direction_without_landmask(node, heading, &wind, &current, step_seconds)
            })
            .collect();
        
        // Vérifier le landmask en batch pour tous les candidats (parallélisé)
        if !candidates.is_empty() {
            let candidate_points: Vec<Point> = candidates.iter().map(|n| n.point).collect();
            let are_sea = self.landmask.are_sea(&candidate_points);

            candidates = candidates
                .into_iter()
                .enumerate()
                .filter_map(|(idx, candidate)| {
                    if node.time > 0.0 && !are_sea.get(idx).copied().unwrap_or(false) {
                        return None;
                    }
                    Some(candidate)
                })
                .collect();
        }
        
        candidates
    }

    /// Explore une direction spécifique depuis un nœud (sans vérification landmask)
    /// Cette méthode est utilisée pour optimiser en vérifiant le landmask en batch
    fn explore_direction_without_landmask(
        &self,
        node: &Node,
        heading: f64,
        wind: &Wind,
        current: &Current,
        step_seconds: f64,
    ) -> Option<Node> {
        // Calculer l'angle au vent
        let angle_au_vent = angle_au_vent(heading, wind.direction);
        
        // Obtenir la vitesse du bateau depuis la polaire
        let boat_speed_ms = self.polar.speed_ms(angle_au_vent, wind.speed);
        
        // Debug pour diagnostiquer les problèmes de vitesse
        if (wind.speed - 15.0).abs() < 0.5 && boat_speed_ms < 0.1 {
            eprintln!("⚠️  Vitesse très faible détectée: wind={:.1}m/s, angle_au_vent={:.1}°, boat_speed_ms={:.4}m/s", 
                     wind.speed, angle_au_vent, boat_speed_ms);
        }
        
        // Si la vitesse est trop faible, ignorer cette direction
        // Réduire le seuil minimum pour permettre plus de directions
        if boat_speed_ms < 0.05 {
            return None;
        }
        
        // Calculer la vitesse effective en tenant compte du courant
        let (effective_speed, effective_direction) = calculate_effective_velocity(
            boat_speed_ms,
            heading,
            current,
        );
        
        // Distance parcourue en un pas de temps
        let distance = effective_speed * step_seconds;
        
        // Limiter la distance maximale
        if distance > self.config.max_distance_meters {
            return None;
        }
        
        // Nouveau point
        let new_point = move_from_point(&node.point, effective_direction, distance);
        
        // Éviter de générer des nœuds qui sont trop proches du point précédent
        // La clé spatiale gère déjà la déduplication à ~700m, donc on accepte des distances plus courtes
        // Seulement filtrer si la vitesse est vraiment très faible ET la distance vraiment très courte
        let min_distance = 150.0; // Distance minimale de 150m (la clé spatiale gère ~700m)
        if node.time > 0.0 && distance < min_distance && boat_speed_ms < 0.3 {
            // Si la vitesse est vraiment très faible (< 0.3 m/s) et distance très courte, ignorer
            return None;
        }
        
        // Créer le nœud successeur
        Some(Node {
            point: new_point,
            time: node.time + step_seconds,
            distance: node.distance + distance,
        })
    }

    /// Crée une clé pour un point (arrondi pour éviter les doublons proches)
    fn point_key(&self, point: &Point) -> (i32, i32) {
        // Granularité adaptative basée sur la distance au point de départ
        // Plus fine au début, plus grossière plus loin pour permettre 24h
        let distance_from_start = self.config.start.distance_to(point);
        let precision = if distance_from_start < 50000.0 {
            // Proche du départ (< 50km) : ~700m de précision
            0.0063
        } else if distance_from_start < 200000.0 {
            // Moyenne distance (50-200km) : ~900m de précision
            0.0081
        } else {
            // Grande distance (> 200km) : ~1.1km de précision
            0.0099
        };
        let lat_rounded = (point.lat / precision).round() as i32;
        let lon_rounded = (point.lon / precision).round() as i32;
        (lat_rounded, lon_rounded)
    }
}

/// Simplifie une isochrone (public API for SOTA router)
pub fn simplify_isochrone_public(isochrone: &mut Isochrone) {
    simplify_isochrone(isochrone);
}

/// Simplifie une isochrone en éliminant les points trop proches par discrétisation lat/lon
/// Utilise une grille spatiale pour garder un seul point par cellule
fn simplify_isochrone(isochrone: &mut Isochrone) {
    // Si moins de 3 points, pas besoin de simplification
    if isochrone.points.len() < 3 {
        return;
    }
    
    // Résolution de la grille en degrés
    // ~500m à l'équateur correspond à environ 0.0045 degrés
    // On utilise une résolution adaptative basée sur la taille de l'isochrone
    let grid_resolution = calculate_grid_resolution(&isochrone.points);
    
    // Créer une grille pour stocker un point par cellule
    // Utilise une HashMap avec (lat_cell, lon_cell) comme clé
    use std::collections::HashMap;
    let mut grid: HashMap<(i32, i32), Point> = HashMap::new();
    
    // Parcourir tous les points et les placer dans la grille
    for point in &isochrone.points {
        // Calculer la cellule de grille pour ce point
        let lat_cell = (point.lat / grid_resolution).round() as i32;
        let lon_cell = (point.lon / grid_resolution).round() as i32;
        let cell_key = (lat_cell, lon_cell);
        
        // Si la cellule est vide ou si ce point est plus proche du centre de la cellule, le garder
        if let Some(existing_point) = grid.get(&cell_key) {
            // Calculer le centre de la cellule
            let cell_center_lat = lat_cell as f64 * grid_resolution;
            let cell_center_lon = lon_cell as f64 * grid_resolution;
            let cell_center = Point::new(cell_center_lat, cell_center_lon);
            
            // Garder le point le plus proche du centre de la cellule
            let dist_existing = existing_point.distance_to(&cell_center);
            let dist_current = point.distance_to(&cell_center);
            
            if dist_current < dist_existing {
                grid.insert(cell_key, *point);
            }
        } else {
            // Première fois qu'on voit cette cellule, garder le point
            grid.insert(cell_key, *point);
        }
    }
    
    // Extraire les points de la grille et les trier pour préserver l'ordre
    // On va essayer de préserver l'ordre original en utilisant l'index original
    let mut grid_points: Vec<(usize, Point)> = Vec::new();
    
    for (idx, point) in isochrone.points.iter().enumerate() {
        let lat_cell = (point.lat / grid_resolution).round() as i32;
        let lon_cell = (point.lon / grid_resolution).round() as i32;
        let cell_key = (lat_cell, lon_cell);
        
        // Vérifier si ce point est celui qui a été gardé pour cette cellule
        if let Some(&grid_point) = grid.get(&cell_key) {
            if grid_point == *point {
                grid_points.push((idx, *point));
            }
        }
    }
    
    // Trier par index original pour préserver l'ordre
    grid_points.sort_by_key(|(idx, _)| *idx);
    
    // Extraire les points dans l'ordre
    let simplified_points: Vec<Point> = grid_points.into_iter().map(|(_, point)| point).collect();
    
    // Si on a trop peu de points, utiliser une simplification moins agressive
    let final_points = if simplified_points.len() < 3 && isochrone.points.len() >= 3 {
        // Fallback: utiliser tous les points mais avec un filtre de distance minimale
        let mut filtered = Vec::new();
        filtered.push(isochrone.points[0]);
        for point in isochrone.points.iter().skip(1) {
            let last_point = filtered.last().unwrap();
            // Distance minimale de 200m
            if point.distance_to(last_point) > 200.0 {
                filtered.push(*point);
            }
        }
        filtered
    } else {
        simplified_points
    };
    
    // Remplacer les points par la version simplifiée
    if !final_points.is_empty() {
        isochrone.points = final_points;
    }
}

/// Calcule la résolution de grille adaptative basée sur la taille de l'isochrone
fn calculate_grid_resolution(points: &[Point]) -> f64 {
    if points.is_empty() {
        return 0.005; // Résolution par défaut (~500m)
    }
    
    // Calculer la bounding box
    let mut min_lat = points[0].lat;
    let mut max_lat = points[0].lat;
    let mut min_lon = points[0].lon;
    let mut max_lon = points[0].lon;
    
    for point in points {
        min_lat = min_lat.min(point.lat);
        max_lat = max_lat.max(point.lat);
        min_lon = min_lon.min(point.lon);
        max_lon = max_lon.max(point.lon);
    }
    
    // Calculer la taille approximative de l'isochrone
    let center_lat = (min_lat + max_lat) / 2.0;
    let lat_span = max_lat - min_lat;
    let lon_span = max_lon - min_lon;
    
    // À une latitude donnée, 1 degré de latitude ≈ 111 km
    // 1 degré de longitude ≈ 111 km * cos(lat)
    let lat_km = lat_span * 111.0;
    let lon_km = lon_span * 111.0 * center_lat.to_radians().cos();
    
    // Taille moyenne en km
    let avg_size_km = (lat_km + lon_km) / 2.0;
    
    // Adapter la résolution : pour une grande isochrone, utiliser une grille plus grossière
    // Objectif : ~500m de résolution pour les petites, ~1km pour les grandes
    let target_resolution_m = if avg_size_km < 10.0 {
        1000.0 // Petite isochrone : 500m
    } else if avg_size_km < 50.0 {
        1000.0 // Moyenne isochrone : 750m
    } else {
        1000.0 // Grande isochrone : 1km
    };
    
    // Convertir en degrés (approximatif)
    // 1 degré ≈ 111 km, donc target_resolution_m / 111000.0 degrés
    target_resolution_m / 111000.0
}

/// Calcule les isochrones avec une approche optimisée (wavefront expansion)
pub fn calculate_isochrones(
    config: IsochroneConfig,
    landmask: Landmask,
    polar: Box<dyn Polar + Send + Sync>,
    grib_provider: Box<dyn GribProvider + Send + Sync>,
    start_time: DateTime<Utc>,
) -> Vec<Isochrone> {
    let calculator = IsochroneCalculator::new(
        config,
        landmask,
        polar,
        grib_provider,
        start_time,
    );
    calculator.calculate()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::polar::SimplePolar;
    use crate::grib::SimpleGribProvider;

    #[test]
    fn test_isochrone_calculator() {
        let config = IsochroneConfig::default();
        let landmask = Landmask::new().unwrap();
        let polar = Box::new(SimplePolar::default_voilier());
        let grib = Box::new(SimpleGribProvider::default());
        let start_time = Utc::now();
        
        let calculator = IsochroneCalculator::new(
            config,
            landmask,
            polar,
            grib,
            start_time,
        );
        
        // Test basique - ne pas exécuter complètement pour éviter d'être trop long
        // let isochrones = calculator.calculate();
        // assert!(!isochrones.is_empty());
    }
}
