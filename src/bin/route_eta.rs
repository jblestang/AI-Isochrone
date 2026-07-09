use ai_isochrone::*;
use chrono::Utc;
use clap::Parser;
use std::time::Instant;

const DEFAULT_WEATHER_SEED: u64 = 42;

#[derive(Parser, Debug)]
#[command(name = "route-eta", about = "Benchmark route-only ETA")]
struct Args {
    #[arg(long, env = "AI_ISOCHRONE_FROM_LAT", default_value_t = DEFAULT_FROM_LAT)]
    from_lat: f64,
    #[arg(long, env = "AI_ISOCHRONE_FROM_LON", default_value_t = DEFAULT_FROM_LON)]
    from_lon: f64,
    #[arg(long, env = "AI_ISOCHRONE_TO_LAT", default_value_t = DEFAULT_TO_LAT)]
    to_lat: f64,
    #[arg(long, env = "AI_ISOCHRONE_TO_LON", default_value_t = DEFAULT_TO_LON)]
    to_lon: f64,
    #[arg(long, env = "AI_ISOCHRONE_TIME_LIMIT_HOURS", default_value_t = DEFAULT_TIME_LIMIT_HOURS)]
    time_limit_hours: f64,
}

fn weather_seed() -> u64 {
    std::env::var("AI_ISOCHRONE_WEATHER_SEED")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(DEFAULT_WEATHER_SEED)
}

fn main() {
    let args = Args::parse();
    let endpoints = RouteEndpoints::from_coords(args.from_lat, args.from_lon, args.to_lat, args.to_lon);
    let start = endpoints.start;
    let dest = endpoints.dest;
    let direct_nm = start.distance_to(&dest) / 1852.0;

    println!("Route {}", endpoints.label());
    println!("Direct rhumb: {:.0} nm", direct_nm);
    println!("Weather seed: {}\n", weather_seed());

    let config = SotaRoutingConfig::route_only(IsochroneConfig {
        start,
        destination: Some(dest),
        time_limit_hours: args.time_limit_hours,
        ..Default::default()
    });

    let time_limit_hours = config.base.time_limit_hours;
    let seed = weather_seed();
    let start_time = Utc::now();
    let t0 = Instant::now();
    let r = calculate_sota_routing(
        config,
        ObjectiveWeights::default(),
        Landmask::new().unwrap(),
        default_routing_polar(),
        Box::new(simulation_grib(seed).with_epoch(start_time)),
        start_time,
    );
    println!(
        "Router ({:.1?}): {} isochrones, last {:.0} h, reach {:.0} nm",
        t0.elapsed(),
        r.isochrones.len(),
        r.isochrones.last().map(|i| i.time_hours).unwrap_or(0.0),
        r.isochrones
            .iter()
            .flat_map(|i| &i.points)
            .map(|p| start.distance_to(p) / 1852.0)
            .fold(0.0, f64::max),
    );

    if let Some(eta) = r.best_eta_hours {
        let sailed: f64 = r.route_legs.iter().map(|l| l.distance_nm).sum();
        let sail_changes = count_distinct_sail_changes(&r.route_legs);
        println!("\nRouted ETA: {:.1} h ({:.1} days)", eta, eta / 24.0);
        println!("Distance sailed: {:.0} nm, avg {:.1} kt", sailed, sailed / eta);
        println!("Sail changes: {sail_changes}");
        return;
    }

    let grib = SimpleGribProvider::default();
    let polar = default_routing_polar();
    let wind = grib.get_wind(&start, Utc::now()).unwrap();
    let bearing = start.bearing_to(&dest);
    let angle = polar::angle_au_vent(bearing, wind.direction);
    let spd_kt = polar.speed_ms(angle, wind.speed) * 1.944;

    let optimistic_h = constraints::optimistic_eta_hours(&start, &dest, 8.0);
    let rhumb_h = if spd_kt > 0.5 {
        direct_nm / spd_kt
    } else {
        f64::INFINITY
    };
    let passage_nm = direct_nm * 1.18;
    let realistic_h = passage_nm / 6.5;

    println!(
        "\nRouter did not reach destination within {:.0} h.",
        time_limit_hours
    );
    println!(
        "Estimates with default polar + {:.0} kn W wind:",
        wind.speed * 1.944
    );
    println!(
        "  Optimistic (max speed, straight line): {:.0} h ({:.1} days)",
        optimistic_h,
        optimistic_h / 24.0
    );
    println!(
        "  Direct rhumb at {:.1} kt: {:.0} h ({:.1} days)",
        spd_kt, rhumb_h, rhumb_h / 24.0
    );
    println!(
        "  Realistic passage ~{:.0} nm at 6.5 kt avg: {:.0} h ({:.1} days)",
        passage_nm, realistic_h, realistic_h / 24.0
    );
}
