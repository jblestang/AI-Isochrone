use ai_isochrone::*;
use eframe::egui;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "ai-isochrone-gui")]
#[command(about = "Interface graphique pour visualiser les isochrones")]
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
    #[arg(long, default_value_t = 24.0 * 7.0)]
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

    /// Fichier JSON avec les isochrones (optionnel, sinon calcul)
    #[arg(long)]
    input: Option<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let (isochrones, start, destination) = if let Some(input_file) = args.input {
        // Charger depuis un fichier JSON
        let json_content = std::fs::read_to_string(input_file)?;
        let json: serde_json::Value = serde_json::from_str(&json_content)?;
        
        let mut loaded_isochrones = Vec::new();
        if let Some(isochrones_array) = json.get("isochrones").and_then(|v| v.as_array()) {
            for iso_json in isochrones_array {
                if let (Some(time_hours), Some(points_array)) = (
                    iso_json.get("time_hours").and_then(|v| v.as_f64()),
                    iso_json.get("points").and_then(|v| v.as_array()),
                ) {
                    let points: Vec<Point> = points_array
                        .iter()
                        .filter_map(|p| {
                            if let (Some(lat), Some(lon)) = (
                                p.get("lat").and_then(|v| v.as_f64()),
                                p.get("lon").and_then(|v| v.as_f64()),
                            ) {
                                Some(Point::new(lat, lon))
                            } else {
                                None
                            }
                        })
                        .collect();
                    
                    loaded_isochrones.push(Isochrone {
                        time_hours,
                        points,
                    });
                }
            }
        }
        
        let start = Point::new(args.start_lat, args.start_lon);
        let destination = Some(Point::new(args.dest_lat, args.dest_lon));
        
        (loaded_isochrones, start, destination)
    } else {
        // Calculer les isochrones
        println!("🔧 Calcul des isochrones...");
        
        let config = IsochroneConfig {
            start: Point::new(args.start_lat, args.start_lon),
            destination: Some(Point::new(args.dest_lat, args.dest_lon)),
            time_limit_hours: args.time_limit_hours,
            isochrone_step_hours: args.isochrone_step_hours,
            simulation_step_minutes: args.simulation_step_minutes,
            max_distance_meters: 50000.0,
            num_directions: args.num_directions,
        };

        let landmask = Landmask::new()
            .map_err(|e| format!("Erreur lors du chargement du landmask: {}", e))?;
        let polar: Box<dyn Polar + Send + Sync> = Box::new(polar::SimplePolar::default_voilier());
        let grib_provider: Box<dyn GribProvider + Send + Sync> = 
            Box::new(grib::SimpleGribProvider::default());

        let start_datetime = chrono::Utc::now();
        let isochrones = calculate_isochrones(
            config.clone(),
            landmask,
            polar,
            grib_provider,
            start_datetime,
        );
        
        println!("✓ {} isochrones calculées", isochrones.len());
        
        (isochrones, config.start, config.destination)
    };

    // Lancer l'interface graphique
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("Visualisation des Isochrones"),
        ..Default::default()
    };

    eframe::run_native(
        "AI-Isochrone GUI",
        options,
        Box::new(move |_cc| {
            Ok(Box::new(gui::IsochroneApp::new(isochrones, start, destination)))
        }),
    )?;

    Ok(())
}
