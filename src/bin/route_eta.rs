use ai_isochrone::*;
use chrono::Utc;
use std::time::Instant;

fn main() {
    let start = Point::new(47.55, -3.48);
    let dest = Point::new(43.12, 5.93);
    let direct_nm = start.distance_to(&dest) / 1852.0;

    println!("Lorient (south of Groix) → Toulon");
    println!("Direct rhumb: {:.0} nm\n", direct_nm);

    let config = SotaRoutingConfig::route_only(IsochroneConfig {
        start,
        destination: Some(dest),
        time_limit_hours: 200.0,
        ..Default::default()
    });

    let time_limit_hours = config.base.time_limit_hours;
    let t0 = Instant::now();
    let r = calculate_sota_routing(
        config,
        ObjectiveWeights::default(),
        Landmask::new().unwrap(),
        Box::new(SimplePolar::default_voilier()),
        Box::new(SimpleGribProvider::default()),
        Utc::now(),
    );
    println!("Router ({:.1?}): {} isochrones, last {:.0} h, reach {:.0} nm",
        t0.elapsed(),
        r.isochrones.len(),
        r.isochrones.last().map(|i| i.time_hours).unwrap_or(0.0),
        r.isochrones.iter().flat_map(|i| &i.points).map(|p| start.distance_to(p)/1852.0).fold(0.0, f64::max),
    );

    if let Some(eta) = r.best_eta_hours {
        let sailed: f64 = r.route_legs.iter().map(|l| l.distance_nm).sum();
        println!("\nRouted ETA: {:.1} h ({:.1} days)", eta, eta / 24.0);
        println!("Distance sailed: {:.0} nm, avg {:.1} kt", sailed, sailed / eta);
        return;
    }

    // Fallback: polar + default wind toward destination bearing
    let grib = SimpleGribProvider::default();
    let polar = SimplePolar::default_voilier();
    let wind = grib.get_wind(&start, Utc::now()).unwrap();
    let bearing = start.bearing_to(&dest);
    let angle = polar::angle_au_vent(bearing, wind.direction);
    let spd_kt = polar.speed_ms(angle, wind.speed) * 1.944;

    let optimistic_h = constraints::optimistic_eta_hours(&start, &dest, 8.0);
    let rhumb_h = if spd_kt > 0.5 { direct_nm / spd_kt } else { f64::INFINITY };
    // Real passage ~15% longer than rhumb (around Spain)
    let passage_nm = direct_nm * 1.18;
    let realistic_h = passage_nm / 6.5;

    println!("\nRouter did not reach Toulon in {:.0} h.", time_limit_hours);
    println!("Estimates with default polar + {:.0} kn W wind:", wind.speed * 1.944);
    println!("  Optimistic (max speed, straight line): {:.0} h ({:.1} days)", optimistic_h, optimistic_h / 24.0);
    println!("  Direct rhumb at {:.1} kt: {:.0} h ({:.1} days)", spd_kt, rhumb_h, rhumb_h / 24.0);
    println!("  Realistic passage ~{:.0} nm at 6.5 kt avg: {:.0} h ({:.1} days)", passage_nm, realistic_h, realistic_h / 24.0);
}
