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
const ROUTE_WIND_STEP_HOURS: f64 = 12.0;

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

    // Optimal route
    if let Some(route) = &result.best_route {
        let route_color = Rgba([255, 40, 40, 255]);
        for w in route.windows(2) {
            let a = vp.project(&w[0]);
            let b = vp.project(&w[1]);
            draw_line(&mut img, a, b, route_color);
        }

        // True wind along route at simulation time for each 12 h leg
        let eta = result.best_eta_hours.unwrap_or(200.0);
        let polar = SimplePolar::default_voilier();
        draw_route_wind(
            &mut img,
            &vp,
            route,
            &result.best_route_headings,
            &result.route_legs,
            &polar,
            grib,
            start_time,
            eta,
        );
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

fn draw_route_wind(
    img: &mut RgbaImage,
    vp: &Viewport,
    route: &[Point],
    headings: &[f64],
    legs: &[RouteLeg],
    polar: &SimplePolar,
    grib: &SeededWindGribProvider,
    start_time: DateTime<Utc>,
    eta_hours: f64,
) {
    let samples = if legs.is_empty() {
        samples_along_route_with_heading(route, headings, eta_hours, ROUTE_WIND_STEP_HOURS)
            .into_iter()
            .map(|(pt, hdg, h)| (pt, hdg, h, None))
            .collect::<Vec<_>>()
    } else {
        samples_from_route_legs(legs, eta_hours, ROUTE_WIND_STEP_HOURS)
    };

    for (point, heading, sim_hours, leg_wind) in samples {
        let wind = if let Some(w) = leg_wind {
            w
        } else {
            let sample_time =
                start_time + chrono::Duration::seconds((sim_hours * 3600.0).round() as i64);
            grib
                .get_wind(&point, sample_time)
                .unwrap_or(Wind::new(270.0, 10.0))
        };
        let twa = polar::angle_au_vent(heading, wind.direction);
        let boat_kt = polar.speed_ms(twa, wind.speed) * 1.944;
        let color = wind_color_for_time(sim_hours, eta_hours);
        let (mut x, mut y) = vp.project(&point);
        x += 14;
        y -= 10;
        draw_wind_arrow(img, x, y, &wind, color);
        let label = format!(
            "{:.0}h TWA {:.0}° boat {:.0}kt",
            sim_hours, twa, boat_kt
        );
        draw_text(
            img,
            x + 16,
            y - 6,
            &label,
            Rgba([255, 255, 255, 255]),
        );
    }
}

fn samples_from_route_legs(
    legs: &[RouteLeg],
    eta_hours: f64,
    step_hours: f64,
) -> Vec<(Point, f64, f64, Option<Wind>)> {
    if legs.is_empty() || eta_hours <= 0.0 {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut h = 0.0;
    while h <= eta_hours + 1e-3 {
        if let Some((pt, heading, wind)) = point_on_legs_at_hour(legs, eta_hours, h) {
            out.push((pt, heading, h, Some(wind)));
        }
        h += step_hours;
    }
    if out.last().map(|(_, _, th, _)| (*th - eta_hours).abs()) > Some(1.0) {
        if let Some((pt, heading, wind)) = point_on_legs_at_hour(legs, eta_hours, eta_hours) {
            out.push((pt, heading, eta_hours, Some(wind)));
        }
    }
    out
}

fn point_on_legs_at_hour(legs: &[RouteLeg], eta_hours: f64, hour: f64) -> Option<(Point, f64, Wind)> {
    let target = (hour / eta_hours).clamp(0.0, 1.0) * eta_hours;
    let mut cum = 0.0;
    for leg in legs {
        let end = cum + leg.duration_hours;
        if target <= end + 1e-6 {
            let frac = if leg.duration_hours > 0.0 {
                ((target - cum) / leg.duration_hours).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let pt = interpolate_point(&leg.from, &leg.to, frac);
            return Some((pt, leg.boat_heading_deg, leg.wind));
        }
        cum = end;
    }
    legs.last().map(|leg| (leg.to, leg.boat_heading_deg, leg.wind))
}

fn samples_along_route_with_heading(
    route: &[Point],
    headings: &[f64],
    eta_hours: f64,
    step_hours: f64,
) -> Vec<(Point, f64, f64)> {
    if route.is_empty() || eta_hours <= 0.0 {
        return Vec::new();
    }
    let mut out = Vec::new();
    let mut h = 0.0;
    while h <= eta_hours + 1e-3 {
        if let Some((pt, heading)) = point_and_heading_on_route_at_hour(route, headings, eta_hours, h)
        {
            out.push((pt, heading, h));
        }
        h += step_hours;
    }
    if out.last().map(|(_, _, th)| (*th - eta_hours).abs()) > Some(1.0) {
        if let Some((pt, heading)) =
            point_and_heading_on_route_at_hour(route, headings, eta_hours, eta_hours)
        {
            out.push((pt, heading, eta_hours));
        }
    }
    out
}

fn point_and_heading_on_route_at_hour(
    route: &[Point],
    headings: &[f64],
    eta_hours: f64,
    hour: f64,
) -> Option<(Point, f64)> {
    let point = point_on_route_at_hour(route, eta_hours, hour)?;
    if route.len() == 1 {
        return Some((point, headings.first().copied().unwrap_or(0.0)));
    }
    let total_dist: f64 = route
        .windows(2)
        .map(|seg| seg[0].distance_to(&seg[1]))
        .sum();
    if total_dist <= 0.0 {
        return Some((point, headings.first().copied().unwrap_or(0.0)));
    }
    let target_dist = (hour / eta_hours).clamp(0.0, 1.0) * total_dist;
    let mut cum = 0.0;
    for (i, seg) in route.windows(2).enumerate() {
        let seg_len = seg[0].distance_to(&seg[1]);
        if cum + seg_len >= target_dist - 1e-6 {
            let heading = headings
                .get(i + 1)
                .copied()
                .unwrap_or_else(|| seg[0].bearing_to(&seg[1]));
            return Some((point, heading));
        }
        cum += seg_len;
    }
    let last = route.len() - 1;
    Some((
        point,
        headings
            .last()
            .copied()
            .unwrap_or_else(|| route[last - 1].bearing_to(&route[last])),
    ))
}

fn point_on_route_at_hour(route: &[Point], eta_hours: f64, hour: f64) -> Option<Point> {
    if route.is_empty() {
        return None;
    }
    if route.len() == 1 {
        return Some(route[0]);
    }
    let total_dist: f64 = route
        .windows(2)
        .map(|seg| seg[0].distance_to(&seg[1]))
        .sum();
    if total_dist <= 0.0 {
        return Some(route[0]);
    }
    let target_dist = (hour / eta_hours).clamp(0.0, 1.0) * total_dist;
    let mut cum = 0.0;
    for seg in route.windows(2) {
        let seg_len = seg[0].distance_to(&seg[1]);
        if cum + seg_len >= target_dist - 1e-6 {
            let frac = if seg_len > 0.0 {
                ((target_dist - cum) / seg_len).clamp(0.0, 1.0)
            } else {
                0.0
            };
            return Some(interpolate_point(&seg[0], &seg[1], frac));
        }
        cum += seg_len;
    }
    route.last().copied()
}

fn interpolate_point(a: &Point, b: &Point, frac: f64) -> Point {
    Point::new(
        a.lat + frac * (b.lat - a.lat),
        a.lon + frac * (b.lon - a.lon),
    )
}

/// Arrow tint: blue (departure) → yellow → red (arrival) by simulation hour.
fn wind_color_for_time(sim_hours: f64, eta_hours: f64) -> Rgba<u8> {
    let t = (sim_hours / eta_hours.max(1.0)).clamp(0.0, 1.0);
    Rgba([
        (60.0 + 195.0 * t) as u8,
        (220.0 - 80.0 * t) as u8,
        (255.0 - 215.0 * t) as u8,
        255,
    ])
}

/// Draw arrow pointing where wind blows (meteorological FROM → TO = dir + 180°).
fn draw_wind_arrow(img: &mut RgbaImage, cx: i32, cy: i32, wind: &Wind, color: Rgba<u8>) {
    let to_deg = (wind.direction + 180.0).rem_euclid(360.0);
    let len = (wind.speed * 3.0).clamp(18.0, 42.0) as i32;
    let rad = to_deg.to_radians();
    let ex = cx + (rad.sin() * len as f64).round() as i32;
    let ey = cy - (rad.cos() * len as f64).round() as i32;

    let outline = Rgba([8, 25, 55, 255]);
    let head = Rgba([
        color[0].saturating_add(40),
        color[1].saturating_add(20),
        color[2],
        255,
    ]);

    for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)] {
        draw_thick_line(
            img,
            (cx + dx, cy + dy),
            (ex + dx, ey + dy),
            outline,
            2,
        );
    }
    draw_thick_line(img, (cx, cy), (ex, ey), color, 2);

    for sign in [-1.0_f64, 1.0] {
        let hr = (to_deg + 180.0 + sign * 24.0).to_radians();
        let hx = ex + (hr.sin() * 9.0).round() as i32;
        let hy = ey - (hr.cos() * 9.0).round() as i32;
        draw_thick_line(img, (ex, ey), (hx, hy), head, 2);
    }
    put_pixel_opaque(img, cx, cy, color);
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
        "Lorient > Toulon | {} | {:.0} nm | {:.0}h isochrones | wind every {:.0}h on route",
        eta,
        sailed,
        ISOCHRONE_STEP_HOURS,
        ROUTE_WIND_STEP_HOURS,
    );

    let w0 = grib.get_wind(&start, start_time).unwrap();
    let eta_h = result.best_eta_hours.unwrap_or(200.0);
    let end_time = start_time + chrono::Duration::seconds((eta_h * 3600.0) as i64);
    let end_pt = result
        .best_route
        .as_ref()
        .and_then(|r| r.last().copied())
        .unwrap_or(start);
    let w_end = grib.get_wind(&end_pt, end_time).unwrap();
    let wind_line = format!(
        "Route TWS | start: {:.0}deg {:.1}kt | finish: {:.0}deg {:.1}kt | seed {}",
        w0.direction,
        w0.speed * 1.944,
        w_end.direction,
        w_end.speed * 1.944,
        seed
    );

    draw_text(img, 16, 8, "AI Isochrone Routing", Rgba([240, 240, 255, 255]));
    draw_text(img, 16, 24, &subtitle, Rgba([180, 200, 230, 255]));
    draw_text(img, 16, 42, &wind_line, Rgba([150, 220, 255, 255]));
    draw_text(
        img,
        WIDTH as i32 - 280,
        42,
        "arrow color 0h -> ETA",
        Rgba([150, 220, 255, 255]),
    );
    // Mini legend gradient
    for i in 0..120 {
        let c = wind_color_for_time(i as f64 / 119.0 * result.best_eta_hours.unwrap_or(180.0), result.best_eta_hours.unwrap_or(180.0));
        fill_rect(img, (WIDTH - 280 + i) as u32, 52, 1, 6, c);
    }
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
