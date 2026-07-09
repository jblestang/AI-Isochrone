use ai_isochrone::{MultiSailPolar, Polar, MIN_ANGLE_AU_VENT_DEG};
use clap::Parser;
use image::{Rgba, RgbaImage};
use std::path::PathBuf;

const WIDTH: u32 = 1200;
const HEIGHT: u32 = 1000;

#[derive(Parser, Debug)]
#[command(name = "polar-snapshot", about = "Render boat polar diagram PNG")]
struct Args {
    #[arg(long, default_value = "/opt/cursor/artifacts/boat_polar.png")]
    output: PathBuf,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let polar = MultiSailPolar::default_voilier();
    let wind_speeds_ms = [2.5, 5.0, 7.5, 10.0, 12.5, 15.0];
    let colors = [
        Rgba([120, 180, 255, 255]),
        Rgba([80, 200, 160, 255]),
        Rgba([255, 210, 80, 255]),
        Rgba([255, 140, 80, 255]),
        Rgba([255, 90, 120, 255]),
        Rgba([200, 120, 255, 255]),
    ];
    let sail_colors = [
        Rgba([140, 190, 255, 140]),
        Rgba([255, 180, 100, 140]),
        Rgba([255, 100, 100, 140]),
        Rgba([180, 255, 140, 140]),
        Rgba([220, 140, 255, 140]),
    ];

    let mut img = RgbaImage::from_pixel(WIDTH, HEIGHT, Rgba([18, 24, 38, 255]));

    let cx = WIDTH as f64 / 2.0;
    let cy = HEIGHT as f64 * 0.52;
    let max_kt = 15.0_f64;
    let scale = 320.0 / max_kt;

    draw_title(&mut img);
    draw_grid(&mut img, cx, cy, scale, max_kt);
    draw_nogo_zone(&mut img, cx, cy, scale * max_kt, MIN_ANGLE_AU_VENT_DEG);

    // Individual sail curves at 10 m/s (reference breeze)
    for (i, sail) in polar.sails().iter().enumerate() {
        let c = sail_colors[i % sail_colors.len()];
        draw_sail_curve(&mut img, sail, 10.0, cx, cy, scale, c);
    }

    // Composite envelope at several wind speeds
    for (i, &wind_ms) in wind_speeds_ms.iter().enumerate() {
        draw_polar_curve(&mut img, &polar, wind_ms, cx, cy, scale, colors[i]);
    }

    draw_wind_arrow(&mut img, cx as i32, 48);
    draw_sail_legend(&mut img, polar.sails(), &sail_colors);
    draw_wind_legend(&mut img, &wind_speeds_ms, &colors);
    draw_axis_labels(&mut img, cx, cy, scale * max_kt);

    std::fs::create_dir_all(
        args.output
            .parent()
            .unwrap_or_else(|| std::path::Path::new(".")),
    )?;
    img.save(&args.output)?;
    println!("Saved polar diagram: {}", args.output.display());
    Ok(())
}

fn draw_title(img: &mut RgbaImage) {
    draw_text(
        img,
        24,
        16,
        "Multi-sail polar — default voilier",
        Rgba([240, 245, 255, 255]),
    );
    draw_text(
        img,
        24,
        34,
        "Dashed: each sail at 10 m/s | Solid: composite best",
        Rgba([150, 170, 200, 255]),
    );
}

fn draw_grid(img: &mut RgbaImage, cx: f64, cy: f64, scale: f64, max_kt: f64) {
    let grid = Rgba([45, 55, 75, 255]);
    for kt in (2..=max_kt as i32).step_by(2) {
        let r = kt as f64 * scale;
        draw_circle(img, cx, cy, r, grid, false);
        let lx = cx as i32 + (r * 0.55).sin() as i32;
        let ly = cy as i32 - (r * 0.55).cos() as i32;
        draw_text(img, lx + 4, ly - 4, &format!("{kt}"), Rgba([110, 130, 160, 255]));
    }

    for twa in [0_f64, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0, 150.0, 180.0] {
        let rad = twa.to_radians();
        let r = max_kt * scale;
        let x = cx + r * rad.sin();
        let y = cy - r * rad.cos();
        draw_line(img, (cx as i32, cy as i32), (x as i32, y as i32), grid);
        let lx = cx + (r + 18.0) * rad.sin();
        let ly = cy - (r + 18.0) * rad.cos();
        draw_text(
            img,
            lx as i32 - 12,
            ly as i32 - 4,
            &format!("{twa}"),
            Rgba([110, 130, 160, 255]),
        );
    }
}

fn draw_nogo_zone(img: &mut RgbaImage, cx: f64, cy: f64, max_r: f64, limit_deg: f64) {
    let fill = Rgba([120, 40, 40, 70]);
    let n = 32;
    for side in [-1.0, 1.0] {
        for i in 0..n {
            let a0 = (i as f64 / n as f64) * limit_deg;
            let a1 = ((i + 1) as f64 / n as f64) * limit_deg;
            let p0 = polar_point(cx, cy, 0.0, side * a0);
            let p1 = polar_point(cx, cy, max_r, side * a1);
            let p2 = polar_point(cx, cy, max_r, side * a0);
            fill_triangle(img, p0, (cx as i32, cy as i32), p1, fill);
            fill_triangle(img, p0, p1, p2, fill);
        }
    }
    draw_text(
        img,
        (cx - 40.0) as i32,
        (cy - max_r - 28.0) as i32,
        "NO-GO",
        Rgba([255, 120, 120, 200]),
    );
}

fn draw_sail_curve(
    img: &mut RgbaImage,
    sail: &ai_isochrone::SailConfig,
    wind_ms: f64,
    cx: f64,
    cy: f64,
    scale: f64,
    color: Rgba<u8>,
) {
    if !sail.is_active(MIN_ANGLE_AU_VENT_DEG, wind_ms) {
        return;
    }
    draw_half_curve(
        img,
        |twa| {
            if sail.is_active(twa, wind_ms) {
                sail.polar.speed_knots(twa, wind_ms)
            } else {
                0.0
            }
        },
        cx,
        cy,
        scale,
        color,
        true,
    );
}

fn draw_polar_curve(
    img: &mut RgbaImage,
    polar: &dyn Polar,
    wind_ms: f64,
    cx: f64,
    cy: f64,
    scale: f64,
    color: Rgba<u8>,
) {
    draw_half_curve(
        img,
        |twa| polar.speed_knots(twa, wind_ms),
        cx,
        cy,
        scale,
        color,
        false,
    );
}

fn draw_half_curve(
    img: &mut RgbaImage,
    speed_at: impl Fn(f64) -> f64,
    cx: f64,
    cy: f64,
    scale: f64,
    color: Rgba<u8>,
    dashed: bool,
) {
    for side in [-1.0, 1.0] {
        let mut prev: Option<(i32, i32)> = None;
        for i in 0..=151 {
            let twa = i as f64;
            let kt = speed_at(twa);
            if kt < 0.01 {
                prev = None;
                continue;
            }
            let pt = polar_point(cx, cy, kt * scale, side * twa);
            if let Some(p) = prev {
                if dashed && (i % 3 != 0) {
                    prev = Some(pt);
                    continue;
                }
                draw_thick_line(img, p, pt, color, if dashed { 1 } else { 2 });
            }
            prev = Some(pt);
        }
    }
}

fn polar_point(cx: f64, cy: f64, r: f64, twa_deg: f64) -> (i32, i32) {
    let rad = twa_deg.to_radians();
    (
        (cx + r * rad.sin()) as i32,
        (cy - r * rad.cos()) as i32,
    )
}

fn draw_wind_arrow(img: &mut RgbaImage, cx: i32, y: i32) {
    draw_text(img, cx - 28, y, "WIND", Rgba([180, 220, 255, 255]));
    draw_thick_line(img, (cx, y + 16), (cx, y + 56), Rgba([180, 220, 255, 255]), 3);
    draw_thick_line(img, (cx, y + 56), (cx - 10, y + 40), Rgba([180, 220, 255, 255]), 3);
    draw_thick_line(img, (cx, y + 56), (cx + 10, y + 40), Rgba([180, 220, 255, 255]), 3);
}

fn draw_sail_legend(img: &mut RgbaImage, sails: &[ai_isochrone::SailConfig], colors: &[Rgba<u8>]) {
    let x0 = 24;
    let mut y = HEIGHT as i32 - 120;
    draw_text(img, x0, y, "Sails @ 10 m/s", Rgba([200, 210, 230, 255]));
    y += 18;
    for (i, sail) in sails.iter().enumerate() {
        let c = colors[i % colors.len()];
        fill_rect(img, (x0 - 4) as u32, y as u32, 20, 4, c);
        draw_text(
            img,
            x0 + 24,
            y - 4,
            &format!(
                "{} ({}-{} deg, {}-{} m/s)",
                sail.name,
                sail.min_twa_deg as i32,
                sail.max_twa_deg as i32,
                sail.min_wind_ms as i32,
                sail.max_wind_ms as i32
            ),
            c,
        );
        y += 16;
    }
}

fn draw_wind_legend(img: &mut RgbaImage, wind_ms: &[f64; 6], colors: &[Rgba<u8>; 6]) {
    let x0 = WIDTH as i32 - 220;
    let mut y = 80;
    draw_text(img, x0, y, "Composite wind", Rgba([200, 210, 230, 255]));
    y += 20;
    for (i, &ws) in wind_ms.iter().enumerate() {
        fill_rect(img, (x0 - 4) as u32, y as u32, 20, 4, colors[i]);
        let kn = ws * 1.944;
        draw_text(
            img,
            x0 + 24,
            y - 4,
            &format!("{ws:.1} m/s ({kn:.0} kn)"),
            colors[i],
        );
        y += 18;
    }
}

fn draw_axis_labels(img: &mut RgbaImage, cx: f64, cy: f64, max_r: f64) {
    draw_text(
        img,
        (cx - 18.0) as i32,
        (cy - max_r - 52.0) as i32,
        "0",
        Rgba([180, 190, 210, 255]),
    );
    draw_text(
        img,
        (cx + max_r + 8.0) as i32,
        cy as i32 - 4,
        "90 starboard",
        Rgba([180, 190, 210, 255]),
    );
    draw_text(
        img,
        (cx - max_r - 72.0) as i32,
        cy as i32 - 4,
        "90 port",
        Rgba([180, 190, 210, 255]),
    );
    draw_text(
        img,
        (cx - 28.0) as i32,
        (cy + max_r + 12.0) as i32,
        "180",
        Rgba([180, 190, 210, 255]),
    );
}

fn draw_circle(img: &mut RgbaImage, cx: f64, cy: f64, r: f64, color: Rgba<u8>, fill: bool) {
    let steps = 360;
    for i in 0..steps {
        let a0 = (i as f64 / steps as f64) * std::f64::consts::TAU;
        let a1 = ((i + 1) as f64 / steps as f64) * std::f64::consts::TAU;
        let p0 = (cx + r * a0.cos(), cy + r * a0.sin());
        let p1 = (cx + r * a1.cos(), cy + r * a1.sin());
        if fill {
            fill_triangle(
                img,
                (cx as i32, cy as i32),
                (p0.0 as i32, p0.1 as i32),
                (p1.0 as i32, p1.1 as i32),
                color,
            );
        } else {
            draw_line(
                img,
                (p0.0 as i32, p0.1 as i32),
                (p1.0 as i32, p1.1 as i32),
                color,
            );
        }
    }
}

fn fill_triangle(
    img: &mut RgbaImage,
    a: (i32, i32),
    b: (i32, i32),
    c: (i32, i32),
    color: Rgba<u8>,
) {
    let min_x = a.0.min(b.0).min(c.0).max(0);
    let max_x = a.0.max(b.0).max(c.0).min(WIDTH as i32 - 1);
    let min_y = a.1.min(b.1).min(c.1).max(0);
    let max_y = a.1.max(b.1).max(c.1).min(HEIGHT as i32 - 1);
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            if point_in_tri((x, y), a, b, c) {
                blend_pixel(img, x as u32, y as u32, color);
            }
        }
    }
}

fn point_in_tri(p: (i32, i32), a: (i32, i32), b: (i32, i32), c: (i32, i32)) -> bool {
    fn sign(p1: (i32, i32), p2: (i32, i32), p3: (i32, i32)) -> i64 {
        (p1.0 - p3.0) as i64 * (p2.1 - p3.1) as i64 - (p2.0 - p3.0) as i64 * (p1.1 - p3.1) as i64
    }
    let d1 = sign(p, a, b);
    let d2 = sign(p, b, c);
    let d3 = sign(p, c, a);
    let has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    let has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    !(has_neg && has_pos)
}

fn blend_pixel(img: &mut RgbaImage, x: u32, y: u32, color: Rgba<u8>) {
    let bg = *img.get_pixel(x, y);
    let a = color[3] as f32 / 255.0;
    let inv = 1.0 - a;
    let blended = Rgba([
        (color[0] as f32 * a + bg[0] as f32 * inv) as u8,
        (color[1] as f32 * a + bg[1] as f32 * inv) as u8,
        (color[2] as f32 * a + bg[2] as f32 * inv) as u8,
        255,
    ]);
    img.put_pixel(x, y, blended);
}

fn fill_rect(img: &mut RgbaImage, x: u32, y: u32, w: u32, h: u32, color: Rgba<u8>) {
    for py in y..y + h {
        for px in x..x + w {
            if px < WIDTH && py < HEIGHT {
                img.put_pixel(px, py, color);
            }
        }
    }
}

fn draw_line(img: &mut RgbaImage, a: (i32, i32), b: (i32, i32), color: Rgba<u8>) {
    draw_thick_line(img, a, b, color, 1);
}

fn draw_thick_line(
    img: &mut RgbaImage,
    (x0, y0): (i32, i32),
    (x1, y1): (i32, i32),
    color: Rgba<u8>,
    thickness: i32,
) {
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx - dy;
    let mut x = x0;
    let mut y = y0;
    loop {
        for tx in -thickness..=thickness {
            for ty in -thickness..=thickness {
                put_pixel(img, x + tx, y + ty, color);
            }
        }
        if x == x1 && y == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            x += sx;
        }
        if e2 < dx {
            err += dx;
            y += sy;
        }
    }
}

fn put_pixel(img: &mut RgbaImage, x: i32, y: i32, color: Rgba<u8>) {
    if x >= 0 && y >= 0 && (x as u32) < WIDTH && (y as u32) < HEIGHT {
        img.put_pixel(x as u32, y as u32, color);
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
                put_pixel(img, x + col as i32, y + row as i32, color);
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
        'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F],
        'M' => [0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11],
        'N' => [0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11],
        'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'P' => [0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10],
        'R' => [0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11],
        'S' => [0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E],
        'T' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04],
        'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A],
        'Y' => [0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04],
        'Z' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F],
        'a' => [0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F],
        'b' => [0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x1E],
        'd' => [0x02, 0x02, 0x0E, 0x12, 0x12, 0x12, 0x0E],
        'e' => [0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0E],
        'g' => [0x00, 0x00, 0x0E, 0x11, 0x0F, 0x01, 0x0E],
        'h' => [0x10, 0x10, 0x16, 0x19, 0x11, 0x11, 0x11],
        'i' => [0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E],
        'k' => [0x10, 0x10, 0x12, 0x14, 0x18, 0x14, 0x12],
        'l' => [0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E],
        'm' => [0x00, 0x00, 0x1A, 0x15, 0x15, 0x11, 0x11],
        'n' => [0x00, 0x00, 0x16, 0x19, 0x11, 0x11, 0x11],
        'o' => [0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E],
        'p' => [0x00, 0x00, 0x1E, 0x11, 0x1E, 0x10, 0x10],
        'r' => [0x00, 0x00, 0x16, 0x18, 0x10, 0x10, 0x10],
        's' => [0x00, 0x00, 0x0E, 0x10, 0x0E, 0x01, 0x1E],
        't' => [0x04, 0x0E, 0x04, 0x04, 0x04, 0x04, 0x06],
        'u' => [0x00, 0x00, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'v' => [0x00, 0x00, 0x11, 0x11, 0x11, 0x0A, 0x04],
        'w' => [0x00, 0x00, 0x11, 0x11, 0x15, 0x15, 0x0A],
        'x' => [0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11],
        '0'..='9' => digit_glyph(ch),
        '.' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C],
        '-' => [0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00],
        '(' => [0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02],
        ')' => [0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08],
        '/' => [0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10],
        ' ' => [0x00; 7],
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
        '5' => [0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E],
        '6' => [0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E],
        '7' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
        '9' => [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E],
        _ => [0x00; 7],
    }
}
