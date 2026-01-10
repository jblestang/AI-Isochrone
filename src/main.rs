use ai_isochrone::*;
use chrono::Utc;
use clap::Parser;
use std::time::Instant;

#[derive(Parser, Debug)]
#[command(name = "ai-isochrone")]
#[command(about = "Calcul d'isochrones pour bateau avec GRIBS, courants et polaire")]
struct Args {
    /// Latitude du point de départ
    #[arg(long, default_value_t = 47.75)]
    start_lat: f64,

    /// Longitude du point de départ
    #[arg(long, default_value_t = -3.37)]
    start_lon: f64,

    /// Latitude du point d'arrivée (optionnel)
    #[arg(long, default_value_t = 43.12)]
    dest_lat: f64,

    /// Longitude du point d'arrivée (optionnel)
    #[arg(long, default_value_t = 5.93)]
    dest_lon: f64,

    /// Temps limite en heures
    #[arg(long, default_value_t = 96.0)]
    time_limit_hours: f64,

    /// Pas d'isochrone en heures
    #[arg(long, default_value_t = 1.0)]
    isochrone_step_hours: f64,

    /// Pas de simulation en minutes
    #[arg(long, default_value_t = 5.0)]
    simulation_step_minutes: f64,

    /// Nombre de directions à explorer
    #[arg(long, default_value_t = 16)]
    num_directions: usize,

    /// Fichier de sortie pour les résultats (JSON)
    #[arg(long)]
    output: Option<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    println!("🚢 Calculateur d'isochrones pour bateau");
    println!("==========================================\n");

    // Configuration
    let config = IsochroneConfig {
        start: Point::new(args.start_lat, args.start_lon),
        destination: Some(Point::new(args.dest_lat, args.dest_lon)),
        time_limit_hours: args.time_limit_hours,
        isochrone_step_hours: args.isochrone_step_hours,
        simulation_step_minutes: args.simulation_step_minutes,
        max_distance_meters: 50000.0, // ~50 km max par pas (27 nœuds max)
        num_directions: args.num_directions,
    };

    println!("📍 Point de départ: ({:.2}°, {:.2}°)", config.start.lat, config.start.lon);
    if let Some(dest) = config.destination {
        println!("🎯 Point d'arrivée: ({:.2}°, {:.2}°)", dest.lat, dest.lon);
        let distance = config.start.distance_to(&dest) / 1000.0; // en km
        println!("📏 Distance directe: {:.2} km\n", distance);
    }

    println!("⚙️  Configuration:");
    println!("   - Temps limite: {:.1} h", config.time_limit_hours);
    println!("   - Pas d'isochrone: {:.1} h", config.isochrone_step_hours);
    println!("   - Pas de simulation: {:.1} min", config.simulation_step_minutes);
    println!("   - Directions explorées: {}\n", config.num_directions);

    // Initialisation des composants
    println!("🔧 Initialisation...");
    let start_time = Instant::now();

    println!("   - Chargement du landmask...");
    let landmask = Landmask::new()
        .map_err(|e| format!("Erreur lors du chargement du landmask: {}", e))?;

    println!("   - Configuration de la polaire...");
    let polar: Box<dyn Polar + Send + Sync> = Box::new(polar::SimplePolar::default_voilier());

    println!("   - Configuration du provider GRIB...");
    let grib_provider: Box<dyn GribProvider + Send + Sync> = 
        Box::new(grib::SimpleGribProvider::default());

    let init_time = start_time.elapsed();
    println!("   ✓ Initialisation terminée en {:.2?}\n", init_time);

    // Calcul des isochrones
    println!("🧮 Calcul des isochrones...");
    let calc_start = Instant::now();
    let start_datetime = Utc::now();

    let isochrones = calculate_isochrones(
        config.clone(),
        landmask,
        polar,
        grib_provider,
        start_datetime,
    );

    let calc_time = calc_start.elapsed();
    println!("   ✓ Calcul terminé en {:.2?}\n", calc_time);

    // Affichage des résultats
    println!("📊 Résultats:");
    println!("   Nombre d'isochrones calculées: {}\n", isochrones.len());

    for isochrone in &isochrones {
        println!("   ⏰ Isochrone {:.1}h: {} points", 
                 isochrone.time_hours, 
                 isochrone.points.len());
        
        // Distance maximale depuis le départ
        if let Some(furthest) = isochrone.points.iter()
            .max_by(|a, b| {
                let dist_a = config.start.distance_to(a);
                let dist_b = config.start.distance_to(b);
                dist_a.partial_cmp(&dist_b).unwrap()
            }) {
            let distance = config.start.distance_to(furthest) / 1000.0; // en km
            println!("      → Distance max depuis départ: {:.2} km ({:.2}°, {:.2}°)", 
                     distance, furthest.lat, furthest.lon);
        }
        
        // Distance minimale à l'objectif d'arrivée (si destination spécifiée)
        if let Some(dest) = config.destination {
            if let Some(closest_to_dest) = isochrone.points.iter()
                .min_by(|a, b| {
                    let dist_a = dest.distance_to(a);
                    let dist_b = dest.distance_to(b);
                    dist_a.partial_cmp(&dist_b).unwrap()
                }) {
                let min_distance_to_dest = dest.distance_to(closest_to_dest) / 1000.0; // en km
                let distance_from_start = config.start.distance_to(closest_to_dest) / 1000.0; // en km
                println!("      → Distance min à l'objectif: {:.2} km ({:.2}°, {:.2}°), dist depuis départ: {:.2} km", 
                         min_distance_to_dest, closest_to_dest.lat, closest_to_dest.lon, distance_from_start);
            }
        }
    }

    // Vérifier si la destination est atteinte
    if let Some(dest) = config.destination {
        println!("\n🎯 Vérification de la destination:");
        let mut reached = false;
        let mut reached_time = 0.0;
        
        for isochrone in &isochrones {
            if isochrone.points.iter().any(|p| {
                p.distance_to(&dest) < 5000.0 // 5 km de tolérance
            }) {
                reached = true;
                reached_time = isochrone.time_hours;
                break;
            }
        }
        
        if reached {
            println!("   ✓ Destination atteinte en {:.1}h", reached_time);
        } else {
            println!("   ✗ Destination non atteinte dans la limite de temps");
        }
    }

    // Sauvegarder les résultats si demandé
    if let Some(output_path) = args.output {
        println!("\n💾 Sauvegarde des résultats dans {}...", output_path);
        save_isochrones_json(&isochrones, &output_path)?;
        println!("   ✓ Résultats sauvegardés");
    }

    let total_time = start_time.elapsed();
    println!("\n⏱️  Temps total d'exécution: {:.2?}", total_time);

    Ok(())
}

/// Sauvegarde les isochrones au format JSON
fn save_isochrones_json(isochrones: &[Isochrone], path: &str) -> Result<(), Box<dyn std::error::Error>> {
    use std::fs::File;
    use std::io::Write;
    
    let mut file = File::create(path)?;
    writeln!(file, "{{")?;
    writeln!(file, "  \"isochrones\": [")?;
    
    for (i, isochrone) in isochrones.iter().enumerate() {
        writeln!(file, "    {{")?;
        writeln!(file, "      \"time_hours\": {},", isochrone.time_hours)?;
        writeln!(file, "      \"num_points\": {},", isochrone.points.len())?;
        writeln!(file, "      \"points\": [")?;
        
        for (j, point) in isochrone.points.iter().enumerate() {
            write!(file, "        {{\"lat\": {}, \"lon\": {}}}", point.lat, point.lon)?;
            if j < isochrone.points.len() - 1 {
                writeln!(file, ",")?;
            } else {
                writeln!(file)?;
            }
        }
        
        write!(file, "      ]")?;
        if i < isochrones.len() - 1 {
            writeln!(file, ",")?;
        } else {
            writeln!(file)?;
        }
        writeln!(file, "    }}")?;
    }
    
    writeln!(file, "  ]")?;
    writeln!(file, "}}")?;
    
    Ok(())
}
