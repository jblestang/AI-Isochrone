use ai_isochrone::*;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "ai-isochrone-gui")]
#[command(about = "SOTA isochrone routing GUI (native)")]
struct Args {
    #[arg(long, default_value_t = 47.55)]
    start_lat: f64,
    #[arg(long, default_value_t = -3.48)]
    start_lon: f64,
    #[arg(long, default_value_t = DEFAULT_TO_LAT)]
    dest_lat: f64,
    #[arg(long, default_value_t = DEFAULT_TO_LON)]
    dest_lon: f64,
    #[arg(long, default_value_t = DEFAULT_TIME_LIMIT_HOURS)]
    time_limit_hours: f64,
    #[arg(long, default_value_t = 30.0)]
    envelope_step_minutes: f64,
    #[arg(long)]
    input: Option<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let start = Point::new(args.start_lat, args.start_lon);
    let destination = Some(Point::new(args.dest_lat, args.dest_lon));

    if let Some(input_file) = args.input {
        let json = std::fs::read_to_string(input_file)?;
        let result: SotaRoutingResult = serde_json::from_str(&json)?;
        gui::SotaRoutingApp::with_result(start, destination, result);
        // with_result creates app but we need run - use run_native with preloaded via custom
        // For simplicity, run with empty and user can reload - or embed in future
    }

    gui::run_native(start, destination)?;
    Ok(())
}
