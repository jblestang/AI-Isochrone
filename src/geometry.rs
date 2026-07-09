use crate::types::*;

/// Utilitaires géométriques pour la navigation

/// Normalise un angle en degrés dans [0, 360)
pub fn normalize_angle(angle: f64) -> f64 {
    let mut normalized = angle % 360.0;
    if normalized < 0.0 {
        normalized += 360.0;
    }
    normalized
}

/// Calcule l'angle relatif entre deux directions (en degrés)
/// Retourne l'angle le plus court entre from et to dans [-180, 180]
pub fn angle_difference(from: f64, to: f64) -> f64 {
    let diff = normalize_angle(to - from);
    if diff > 180.0 {
        diff - 360.0
    } else {
        diff
    }
}

/// Fast equirectangular move for short routing hops (< ~50 km).
/// ~5× faster than full spherical `move_from_point`; error < 0.1% at 10-min sail steps.
#[inline]
pub fn move_from_point_fast(start: &Point, bearing_degrees: f64, distance_meters: f64) -> Point {
    const M_PER_DEG_LAT: f64 = 111_320.0;
    let lat_rad = start.lat.to_radians();
    let m_per_deg_lon = M_PER_DEG_LAT * lat_rad.cos();
    let bearing = bearing_degrees.to_radians();
    Point::new(
        start.lat + (distance_meters * bearing.cos()) / M_PER_DEG_LAT,
        start.lon + (distance_meters * bearing.sin()) / m_per_deg_lon,
    )
}

/// Calcule un nouveau point en se déplaçant depuis un point donné
/// sur une distance et un cap donnés
pub fn move_from_point(start: &Point, bearing_degrees: f64, distance_meters: f64) -> Point {
    const R: f64 = 6371000.0; // Rayon de la Terre en mètres
    
    let lat1 = start.lat.to_radians();
    let lon1 = start.lon.to_radians();
    let bearing = bearing_degrees.to_radians();
    let d = distance_meters / R;
    
    let lat2 = (lat1.sin() * d.cos() + lat1.cos() * d.sin() * bearing.cos()).asin();
    let lon2 = lon1 + (bearing.sin() * d.sin()).atan2(
        lat1.cos() * d.cos() - lat1.sin() * d.sin() * bearing.cos()
    );
    
    Point::new(lat2.to_degrees(), lon2.to_degrees())
}

/// Calcule la vitesse effective du bateau en tenant compte du courant
/// Retourne la vitesse résultante et sa direction
pub fn calculate_effective_velocity(
    boat_speed: f64, // vitesse du bateau en m/s
    boat_heading: f64, // cap du bateau en degrés
    current: &Current, // courant
) -> (f64, f64) {
    // Conversion en radians
    let boat_heading_rad = boat_heading.to_radians();
    let current_dir_rad = current.direction.to_radians();
    
    // Composantes du bateau
    let boat_vx = boat_speed * boat_heading_rad.sin();
    let boat_vy = boat_speed * boat_heading_rad.cos();
    
    // Composantes du courant
    let current_vx = current.speed * current_dir_rad.sin();
    let current_vy = current.speed * current_dir_rad.cos();
    
    // Vitesse effective
    let effective_vx = boat_vx + current_vx;
    let effective_vy = boat_vy + current_vy;
    
    // Magnitude et direction de la vitesse effective
    let effective_speed = (effective_vx.powi(2) + effective_vy.powi(2)).sqrt();
    let effective_direction = effective_vx.atan2(effective_vy).to_degrees();
    let effective_direction = normalize_angle(effective_direction);
    
    (effective_speed, effective_direction)
}

/// Calcule la vitesse du bateau sur le fond (SOG - Speed Over Ground)
/// en tenant compte du courant
pub fn speed_over_ground(
    boat_speed: f64,
    boat_heading: f64,
    current: &Current,
) -> f64 {
    let (sog, _) = calculate_effective_velocity(boat_speed, boat_heading, current);
    sog
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(370.0) - 10.0).abs() < 1e-10);
        assert!((normalize_angle(-10.0) - 350.0).abs() < 1e-10);
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_move_from_point() {
        let start = Point::new(47.55, -3.48);
        let end = move_from_point(&start, 90.0, 1000.0); // 1km vers l'Est
        assert!(end.lon > start.lon);
    }

    #[test]
    fn test_move_from_point_fast_matches_spherical_at_short_hop() {
        let start = Point::new(47.55, -3.48);
        let end = move_from_point(&start, 90.0, 2500.0);
        let fast = move_from_point_fast(&start, 90.0, 2500.0);
        assert!(end.distance_to(&fast) < 5.0);
    }
}
