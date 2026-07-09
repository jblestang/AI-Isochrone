use ai_isochrone::*;
use chrono::{DateTime, Utc};
use image::{ImageBuffer, Rgba, RgbaImage};
use rayon::prelude::*;
use std::path::PathBuf;
use std::time::Instant;

const WIDTH: u32 = 1600;
const HEIGHT: u32 = 1200;
const ISOCHRONE_STEP_HOURS: f64 = 12.0;
const DEFAULT_WEATHER_SEED: u64 = 42;
const TITLE_BAR_H: u32 = 64;

fn weather_seed() -> u64 {
    std::env::var("AI_ISOCHRONE_WEATHER_SEED")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(DEFAULT_WEATHER_SEED)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let start = Point::new(47.55, -3.48);
    let dest = Point::new(43.12, 5.93);

    let config = SotaRoutingConfig {
        base: IsochroneConfig {
            start,
            destination: Some(dest),
            time_limit_hours: 200.0,
            isochrone_step_hours: ISOCHRONE_STEP_HOURS,
            ..Default::default()
        },
        build_isochrones: true,
        build_arrival_envelopes: false,
        stop_on_arrival: true,
        optimize_cost: false,
        ..Default::default()
    };

    println!(
        "Computing Lorient → Toulon route ({} h isochrones, weather seed {})...",
        ISOCHRONE_STEP_HOURS,
        weather_seed()
    );
    let t0 = Instant::now();
    let landmask = Landmask::new()?;
    let start_time = Utc::now();
    let seed = weather_seed();
    let grib = simulation_grib(seed).with_epoch(start_time);
    let step_hours = config.base.simulation_step_seconds() / 3600.0;
    let result = calculate_sota_routing(
        config,
        ObjectiveWeights::default(),
        landmask.clone(),
        Box::new(SimplePolar::default_voilier()),
        Box::new(grib.clone()),
        start_time,
    );
    let compute_time = t0.elapsed();

    let out_path = snapshot_path();
    render_snapshot(
        &result,
        &landmask,
        &grib,
        start_time,
        seed,
        start,
        dest,
        step_hours,
        &out_path,
    )?;

    let sailed: f64 = result.route_legs.iter().map(|l| l.distance_nm).sum();
    println!("Saved snapshot: {}", out_path.display());
    println!("Compute time: {:.1?} ({:.0} s)", compute_time, compute_time.as_secs_f64());
    if let Some(eta) = result.best_eta_hours {
        println!("Routed ETA: {:.1} h ({:.1} days)", eta, eta / 24.0);
        println!("Distance sailed: {:.0} nm, avg {:.1} kt", sailed, sailed / eta);
    }

    Ok(())
}

fn snapshot_path() -> PathBuf {
    let artifacts = PathBuf::from("/opt/cursor/artifacts");
    if artifacts.exists() {
        artifacts.join("lorient-toulon-route.png")
    } else {
        PathBuf::from("lorient-toulon-route.png")
    }
}

#[derive(Clone, Copy)]
struct Viewport {
    min_lat: f64,
    max_lat: f64,
    min_lon: f64,
    max_lon: f64,
}

impl Viewport {
    fn from_points(points: &[Point], padding_deg: f64) -> Self {
        let mut min_lat = points[0].lat;
        let mut max_lat = points[0].lat;
        let mut min_lon = points[0].lon;
        let mut max_lon = points[0].lon;
        for p in points {
            min_lat = min_lat.min(p.lat);
            max_lat = max_lat.max(p.lat);
            min_lon = min_lon.min(p.lon);
            max_lon = max_lon.max(p.lon);
        }
        Self {
            min_lat: min_lat - padding_deg,
            max_lat: max_lat + padding_deg,
            min_lon: min_lon - padding_deg,
            max_lon: max_lon + padding_deg,
        }
    }

    fn project(&self, point: &Point) -> (i32, i32) {
        let x = ((point.lon - self.min_lon) / (self.max_lon - self.min_lon)) * (WIDTH - 1) as f64;
        let y = ((self.max_lat - point.lat) / (self.max_lat - self.min_lat)) * (HEIGHT - 1) as f64;
        (x.round() as i32, y.round() as i32)
    }

    fn unproject(&self, x: u32, y: u32) -> Point {
        let lon = self.min_lon
            + (x as f64 / (WIDTH - 1) as f64) * (self.max_lon - self.min_lon);
        let lat = self.max_lat
            - (y as f64 / (HEIGHT - 1) as f64) * (self.max_lat - self.min_lat);
        Point::new(lat, lon)
    }
}

fn render_snapshot(
    result: &SotaRoutingResult,
    landmask: &Landmask,
    grib: &SeededWindGribProvider,
    start_time: DateTime<Utc>,
    seed: u64,
    start: Point,
    dest: Point,
    step_hours: f64,
    path: &PathBuf,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut points = vec![start, dest];
    if let Some(route) = &result.best_route {
        points.extend(route.iter().copied());
    }
    for iso in &result.isochrones {
        points.extend(iso.points.iter().copied());
    }
    if let Some(route) = &result.best_route {
        points.extend(route.iter().copied());
    }

    let vp = Viewport::from_points(&points, 1.5);
    let sea = Rgba([20, 60, 110, 255]);
    let land = Rgba([180, 190, 150, 255]);
    let mut img: RgbaImage = ImageBuffer::from_pixel(WIDTH, HEIGHT, sea);

    let land_rows: Vec<(u32, u32)> = (0..HEIGHT)
        .into_par_iter()
        .flat_map(|y| {
            let vp = vp;
            (0..WIDTH)
                .filter_map(move |x| {
                    let p = vp.unproject(x, y);
                    landmask.is_land(&p).then_some((x, y))
                })
                .collect::<Vec<_>>()
        })
        .collect();
    for (x, y) in land_rows {
        img.put_pixel(x, y, land);
    }

    // Isochrone rings (12 h steps)
    if !result.isochrones.is_empty() {
        let iso_colors = [
            Rgba([255, 90, 90, 200]),
            Rgba([255, 180, 70, 200]),
            Rgba([255, 240, 90, 200]),
            Rgba([130, 230, 90, 200]),
            Rgba([90, 190, 255, 200]),
            Rgba([180, 130, 255, 200]),
        ];
        for (idx, iso) in result.isochrones.iter().enumerate() {
            let color = iso_colors[idx % iso_colors.len()];
            for pt in &iso.points {
                let (x, y) = vp.project(pt);
                draw_dot(&mut img, x, y, 2, color);
            }
            if let Some(label_pt) = iso
                .points
                .iter()
                .max_by(|a, b| start.distance_to(a).partial_cmp(&start.distance_to(b)).unwrap())
            {
                let (lx, ly) = vp.project(label_pt);
                let label = format!("{:.0}h", iso.time_hours);
                draw_text(&mut img, lx + 6, ly - 4, &label, Rgba([255, 255, 255, 255]));
            }
        }
    }

    // Wind at simulation time when the wavefront reached each location
    draw_wind_field(
        &mut img,
        &vp,
        landmask,
        grib,
        start_time,
        &result.isochrones,
        result.best_route.as_deref(),
        step_hours,
    );

    // Optimal route
    if let Some(route) = &result.best_route {
        let route_color = Rgba([255, 40, 40, 255]);
        for w in route.windows(2) {
            let a = vp.project(&w[0]);
            let b = vp.project(&w[1]);
            draw_line(&mut img, a, b, route_color);
        }
    }

    // Start / destination markers
    draw_marker(&mut img, vp.project(&start), Rgba([50, 255, 100, 255]), 8);
    draw_marker(&mut img, vp.project(&dest), Rgba([255, 50, 50, 255]), 10);

    // Title bar + legend
    fill_rect(&mut img, 0, 0, WIDTH, TITLE_BAR_H, Rgba([15, 25, 40, 230]));
    draw_label_bar(&mut img, result, grib, start_time, seed, start);

    img.save(path)?;
    Ok(())
}

fn draw_wind_field(
    img: &mut RgbaImage,
    vp: &Viewport,
    landmask: &Landmask,
    grib: &SeededWindGribProvider,
    start_time: DateTime<Utc>,
    isochrones: &[Isochrone],
    route: Option<&[Point]>,
    step_hours: f64,
) {
    let step = 44u32;
    let mut y = TITLE_BAR_H + step / 2;
    while y < HEIGHT {
        let mut x = step / 2;
        while x < WIDTH {
            let point = vp.unproject(x, y);
            if landmask.is_land(&point) {
                x += step;
                continue;
            }
            let Some(sim_hours) = simulation_hours_at(&point, isochrones, route, step_hours) else {
                x += step;
                continue;
            };
            let sample_time =
                start_time + chrono::Duration::seconds((sim_hours * 3600.0).round() as i64);
            let wind = grib
                .get_wind(&point, sample_time)
                .unwrap_or(Wind::new(270.0, 10.0));
            draw_wind_arrow(img, x as i32, y as i32, &wind);
            x += step;
        }
        y += step;
    }
}

/// Simulation time (hours) when routing first reached `point` (from isochrones / route).
fn simulation_hours_at(
    point: &Point,
    isochrones: &[Isochrone],
    route: Option<&[Point]>,
    step_hours: f64,
) -> Option<f64> {
    const NEAR_ISO_M: f64 = 50_000.0;
    const NEAR_ROUTE_M: f64 = 35_000.0;

    let mut from_iso = None;
    let mut sorted: Vec<&Isochrone> = isochrones.iter().filter(|i| !i.points.is_empty()).collect();
    sorted.sort_by(|a, b| {
        a.time_hours
            .partial_cmp(&b.time_hours)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for iso in sorted {
        if iso
            .points
            .iter()
            .any(|p| point.distance_to(p) <= NEAR_ISO_M)
        {
            from_iso = Some(iso.time_hours);
            break;
        }
    }

    let from_route = route.and_then(|r| time_hours_near_route(point, r, step_hours, NEAR_ROUTE_M));

    match (from_iso, from_route) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (Some(a), None) => Some(a),
        (None, Some(b)) => Some(b),
        (None, None) => None,
    }
}

fn time_hours_near_route(
    point: &Point,
    route: &[Point],
    step_hours: f64,
    radius_m: f64,
) -> Option<f64> {
    if route.len() < 2 {
        return None;
    }
    let mut best_time = None;
    let mut best_dist = f64::INFINITY;
    let mut cum_hours = 0.0;
    for seg in route.windows(2) {
        let (d, frac) = point_to_segment(point, &seg[0], &seg[1]);
        if d <= radius_m && d < best_dist {
            best_dist = d;
            best_time = Some(cum_hours + frac * step_hours);
        }
        cum_hours += step_hours;
    }
    best_time
}

fn point_to_segment(p: &Point, a: &Point, b: &Point) -> (f64, f64) {
    let ax = a.lon.to_radians();
    let ay = a.lat.to_radians();
    let bx = b.lon.to_radians();
    let by = b.lat.to_radians();
    let px = p.lon.to_radians();
    let py = p.lat.to_radians();

    let abx = bx - ax;
    let aby = by - ay;
    let apx = px - ax;
    let apy = py - ay;
    let ab2 = abx * abx + aby * aby;
    let frac = if ab2 < 1e-18 {
        0.0
    } else {
        (apx * abx + apy * aby).clamp(0.0, 1.0) / ab2
    };
    let proj_x = ax + frac * abx;
    let proj_y = ay + frac * aby;
    let dlat = py - proj_y;
    let dlon = px - proj_x;
    let dist_m = ((dlat * dlat + dlon * dlon).sqrt()) * 6371000.0;
    (dist_m, frac)
}

/// Draw arrow pointing where wind blows (meteorological FROM → TO = dir + 180°).
fn draw_wind_arrow(img: &mut RgbaImage, cx: i32, cy: i32, wind: &Wind) {
    let to_deg = (wind.direction + 180.0).rem_euclid(360.0);
    let len = (wind.speed * 3.0).clamp(18.0, 42.0) as i32;
    let rad = to_deg.to_radians();
    let ex = cx + (rad.sin() * len as f64).round() as i32;
    let ey = cy - (rad.cos() * len as f64).round() as i32;

    let outline = Rgba([8, 25, 55, 255]);
    let shaft = Rgba([80, 210, 255, 255]);
    let head = Rgba([160, 245, 255, 255]);

    // Dark outline for contrast on blue sea
    for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)] {
        draw_thick_line(
            img,
            (cx + dx, cy + dy),
            (ex + dx, ey + dy),
            outline,
            2,
        );
    }
    draw_thick_line(img, (cx, cy), (ex, ey), shaft, 2);

    for sign in [-1.0_f64, 1.0] {
        let hr = (to_deg + 180.0 + sign * 24.0).to_radians();
        let hx = ex + (hr.sin() * 9.0).round() as i32;
        let hy = ey - (hr.cos() * 9.0).round() as i32;
        draw_thick_line(img, (ex, ey), (hx, hy), head, 2);
    }
    put_pixel_opaque(img, cx, cy, shaft);
}

fn draw_label_bar(
    img: &mut RgbaImage,
    result: &SotaRoutingResult,
    grib: &SeededWindGribProvider,
    start_time: DateTime<Utc>,
    seed: u64,
    start: Point,
) {
    let eta = result
        .best_eta_hours
        .map(|h| format!("ETA {:.0}h ({:.1}d)", h, h / 24.0))
        .unwrap_or_else(|| "No arrival".into());
    let sailed: f64 = result.route_legs.iter().map(|l| l.distance_nm).sum();
    let subtitle = format!(
        "Lorient > Toulon | {} | {:.0} nm | {:.0}h isochrones x{} | optimal route",
        eta,
        sailed,
        ISOCHRONE_STEP_HOURS,
        result.isochrones.len()
    );

    let w0 = grib.get_wind(&start, start_time).unwrap();
    let mid_hours = result.best_eta_hours.unwrap_or(96.0) / 2.0;
    let mid_time = start_time + chrono::Duration::seconds((mid_hours * 3600.0) as i64);
    let mid_pt = result
        .best_route
        .as_ref()
        .and_then(|r| r.get(r.len() / 2))
        .copied()
        .unwrap_or(start);
    let w_mid = grib.get_wind(&mid_pt, mid_time).unwrap();
    let wind_line = format!(
        "Wind @ sim time | t0: {:.0}deg {:.1}kt | route mid: {:.0}deg {:.1}kt | seed {}",
        w0.direction,
        w0.speed * 1.944,
        w_mid.direction,
        w_mid.speed * 1.944,
        seed
    );

    draw_text(img, 16, 8, "AI Isochrone Routing", Rgba([240, 240, 255, 255]));
    draw_text(img, 16, 24, &subtitle, Rgba([180, 200, 230, 255]));
    draw_text(img, 16, 42, &wind_line, Rgba([150, 220, 255, 255]));
    draw_text(
        img,
        WIDTH as i32 - 220,
        42,
        "arrows = TWS @ sim time",
        Rgba([150, 220, 255, 255]),
    );
}

fn draw_text(img: &mut RgbaImage, mut x: i32, y: i32, text: &str, color: Rgba<u8>) {
    for ch in text.chars() {
        draw_char(img, x, y, ch, color);
        x += 8;
    }
}

fn draw_char(img: &mut RgbaImage, x: i32, y: i32, ch: char, color: Rgba<u8>) {
    let glyph = glyph_5x7(ch);
    for (row, bits) in glyph.iter().enumerate() {
        for col in 0..5 {
            if (bits >> (4 - col)) & 1 == 1 {
                let px = x + col as i32;
                let py = y + row as i32;
                if px >= 0 && py >= 0 && (px as u32) < WIDTH && (py as u32) < HEIGHT {
                    img.put_pixel(px as u32, py as u32, color);
                }
            }
        }
    }
}

fn glyph_5x7(ch: char) -> [u8; 7] {
    match ch {
        'A' => [0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'B' => [0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E],
        'C' => [0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E],
        'D' => [0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E],
        'E' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F],
        'G' => [0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E],
        'H' => [0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'I' => [0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F],
        'M' => [0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11],
        'N' => [0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11],
        'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'R' => [0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11],
        'S' => [0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E],
        'T' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04],
        'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A],
        'Y' => [0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04],
        'k' => [0x00, 0x04, 0x04, 0x0E, 0x12, 0x12, 0x0E],
        't' => [0x04, 0x0E, 0x04, 0x04, 0x04, 0x04, 0x06],
        'd' => [0x02, 0x02, 0x0E, 0x12, 0x12, 0x12, 0x0E],
        'g' => [0x00, 0x00, 0x0E, 0x10, 0x12, 0x12, 0x0E],
        'h' => [0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x11],
        'm' => [0x00, 0x00, 0x1A, 0x15, 0x15, 0x11, 0x11],
        'n' => [0x00, 0x00, 0x16, 0x19, 0x11, 0x11, 0x11],
        'o' => [0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E],
        'r' => [0x00, 0x00, 0x16, 0x18, 0x10, 0x10, 0x10],
        's' => [0x00, 0x00, 0x0E, 0x10, 0x0E, 0x01, 0x1E],
        'u' => [0x00, 0x00, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'w' => [0x00, 0x00, 0x11, 0x11, 0x15, 0x15, 0x0A],
        'x' => [0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11],
        '0'..='9' => digit_glyph(ch),
        '.' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C],
        '+' => [0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00],
        '-' => [0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00],
        ' ' => [0x00; 7],
        '→' | '>' => [0x04, 0x02, 0x1F, 0x02, 0x04, 0x00, 0x00],
        '|' => [0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        _ => [0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F],
    }
}

fn digit_glyph(ch: char) -> [u8; 7] {
    match ch {
        '0' => [0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E],
        '1' => [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
        '2' => [0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F],
        '3' => [0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E],
        '4' => [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
        '5' => [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
        '6' => [0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E],
        '7' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
        '9' => [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C],
        _ => [0x00; 7],
    }
}

fn fill_rect(img: &mut RgbaImage, x0: u32, y0: u32, w: u32, h: u32, color: Rgba<u8>) {
    for y in y0..y0.saturating_add(h).min(HEIGHT) {
        for x in x0..x0.saturating_add(w).min(WIDTH) {
            img.put_pixel(x, y, color);
        }
    }
}

fn draw_dot(img: &mut RgbaImage, cx: i32, cy: i32, r: i32, color: Rgba<u8>) {
    for dy in -r..=r {
        for dx in -r..=r {
            if dx * dx + dy * dy <= r * r {
                let x = cx + dx;
                let y = cy + dy;
                if x >= 0 && y >= 0 && (x as u32) < WIDTH && (y as u32) < HEIGHT {
                    blend_pixel(img, x as u32, y as u32, color);
                }
            }
        }
    }
}

fn draw_marker(img: &mut RgbaImage, (cx, cy): (i32, i32), color: Rgba<u8>, r: i32) {
    draw_dot(img, cx, cy, r, color);
    draw_dot(img, cx, cy, r + 2, Rgba([255, 255, 255, 255]));
    draw_dot(img, cx, cy, r, color);
}

fn draw_line(img: &mut RgbaImage, (x0, y0): (i32, i32), (x1, y1): (i32, i32), color: Rgba<u8>) {
    draw_thick_line(img, (x0, y0), (x1, y1), color, 1);
}

fn draw_thick_line(
    img: &mut RgbaImage,
    (x0, y0): (i32, i32),
    (x1, y1): (i32, i32),
    color: Rgba<u8>,
    half_width: i32,
) {
    let mut x = x0;
    let mut y = y0;
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;

    loop {
        for t in -half_width..=half_width {
            for s in -half_width..=half_width {
                let px = x + s;
                let py = y + t;
                if px >= 0 && py >= 0 && (px as u32) < WIDTH && (py as u32) < HEIGHT {
                    put_pixel_opaque(img, px, py, color);
                }
            }
        }
        if x == x1 && y == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            err += dx;
            y += sy;
        }
    }
}

fn put_pixel_opaque(img: &mut RgbaImage, x: i32, y: i32, color: Rgba<u8>) {
    if x >= 0 && y >= 0 && (x as u32) < WIDTH && (y as u32) < HEIGHT {
        img.put_pixel(x as u32, y as u32, color);
    }
}

fn blend_pixel(img: &mut RgbaImage, x: u32, y: u32, color: Rgba<u8>) {
    let base = img.get_pixel(x, y);
    let alpha = color[3] as f32 / 255.0;
    let inv = 1.0 - alpha;
    let blended = Rgba([
        (color[0] as f32 * alpha + base[0] as f32 * inv) as u8,
        (color[1] as f32 * alpha + base[1] as f32 * inv) as u8,
        (color[2] as f32 * alpha + base[2] as f32 * inv) as u8,
        255,
    ]);
    img.put_pixel(x, y, blended);
}
