//! Warps a source image into a metrically-correct birdseye canvas given an
//! image->world homography, using inverse mapping (canvas pixel -> world ->
//! image pixel) with bilinear sampling. Parallelized over output rows with
//! rayon so it stays interactive as points are edited.

use image::{Rgba, RgbaImage};
use nalgebra::{Matrix3, Vector3};
use rayon::prelude::*;

pub struct WorldExtent {
    pub x_min: f64,
    pub x_max: f64,
    pub y_min: f64,
    pub y_max: f64,
}

pub struct BirdseyeParams {
    pub pixels_per_meter: f64,
    pub margin_m: f64,
    pub max_canvas_dim: u32,
}

impl Default for BirdseyeParams {
    fn default() -> Self {
        Self { pixels_per_meter: 25.0, margin_m: 4.0, max_canvas_dim: 4000 }
    }
}

/// Projects the four corners of `(src_w, src_h)` through `h_img_to_world` to
/// find the world-space bounding box the warped output should cover.
pub fn compute_extent(src_w: u32, src_h: u32, h_img_to_world: &Matrix3<f64>) -> WorldExtent {
    let corners = [
        (0.0, 0.0),
        (src_w as f64, 0.0),
        (src_w as f64, src_h as f64),
        (0.0, src_h as f64),
    ];
    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;
    for (u, v) in corners {
        let p = h_img_to_world * Vector3::new(u, v, 1.0);
        let (x, y) = (p.x / p.z, p.y / p.z);
        x_min = x_min.min(x);
        x_max = x_max.max(x);
        y_min = y_min.min(y);
        y_max = y_max.max(y);
    }
    WorldExtent { x_min, x_max, y_min, y_max }
}

/// Result of a warp: the output image, the world extent it covers, and the
/// effective pixels-per-metre actually used (may be lower than requested if
/// clamped to `max_canvas_dim`).
pub struct BirdseyeOutput {
    pub image: RgbaImage,
    pub extent: WorldExtent,
    pub effective_ppm: f64,
}

/// Builds the world(metres)->canvas(pixels) similarity transform, composes it
/// with `h_world_to_img` to get canvas->image directly, and warps the full
/// source image via parallel inverse mapping + bilinear sampling.
pub fn warp_to_birdseye(
    src: &RgbaImage,
    h_world_to_img: &Matrix3<f64>,
    extent_unmargined: &WorldExtent,
    params: &BirdseyeParams,
) -> BirdseyeOutput {
    let x_min = extent_unmargined.x_min - params.margin_m;
    let x_max = extent_unmargined.x_max + params.margin_m;
    let y_min = extent_unmargined.y_min - params.margin_m;
    let y_max = extent_unmargined.y_max + params.margin_m;

    let mut canvas_w = ((x_max - x_min) * params.pixels_per_meter).round() as i64;
    let mut canvas_h = ((y_max - y_min) * params.pixels_per_meter).round() as i64;
    let mut effective_ppm = params.pixels_per_meter;

    if canvas_w > params.max_canvas_dim as i64
        || canvas_h > params.max_canvas_dim as i64
        || canvas_w <= 0
        || canvas_h <= 0
    {
        let scale_fix = params.max_canvas_dim as f64 / (canvas_w.max(canvas_h).max(1) as f64);
        canvas_w = (canvas_w as f64 * scale_fix).max(1.0) as i64;
        canvas_h = (canvas_h as f64 * scale_fix).max(1.0) as i64;
        effective_ppm *= scale_fix;
    }
    let canvas_w = canvas_w as u32;
    let canvas_h = canvas_h as u32;

    // world -> canvas: cx = (X - x_min)*ppm ; cy = (y_max - Y)*ppm  (flip Y so
    // +Y in world, "up the straight", points toward the top of the canvas)
    let s = Matrix3::new(
        effective_ppm, 0.0, -x_min * effective_ppm,
        0.0, -effective_ppm, y_max * effective_ppm,
        0.0, 0.0, 1.0,
    );
    let s_inv = s.try_inverse().expect("similarity transform is always invertible");

    // canvas pixel -> image pixel, in one composed matrix.
    let canvas_to_img = h_world_to_img * s_inv;

    let (src_w, src_h) = src.dimensions();
    let mut out = RgbaImage::new(canvas_w, canvas_h);

    out.enumerate_rows_mut().par_bridge().for_each(|(_y, row)| {
        for (cx, cy, pixel) in row {
            let p = canvas_to_img * Vector3::new(cx as f64 + 0.5, cy as f64 + 0.5, 1.0);
            let (u, v) = (p.x / p.z, p.y / p.z);
            *pixel = bilinear_sample(src, src_w, src_h, u, v);
        }
    });

    BirdseyeOutput {
        image: out,
        extent: WorldExtent { x_min, x_max, y_min, y_max },
        effective_ppm,
    }
}

#[inline]
fn bilinear_sample(src: &RgbaImage, w: u32, h: u32, u: f64, v: f64) -> Rgba<u8> {
    if u < 0.0 || v < 0.0 || u >= w as f64 - 1.0 || v >= h as f64 - 1.0 {
        return Rgba([0, 0, 0, 0]); // transparent outside source bounds
    }
    let x0 = u.floor() as u32;
    let y0 = v.floor() as u32;
    let fx = u - x0 as f64;
    let fy = v - y0 as f64;

    let p00 = src.get_pixel(x0, y0).0;
    let p10 = src.get_pixel(x0 + 1, y0).0;
    let p01 = src.get_pixel(x0, y0 + 1).0;
    let p11 = src.get_pixel(x0 + 1, y0 + 1).0;

    let mut out = [0u8; 4];
    for c in 0..4 {
        let top = p00[c] as f64 * (1.0 - fx) + p10[c] as f64 * fx;
        let bot = p01[c] as f64 * (1.0 - fx) + p11[c] as f64 * fx;
        out[c] = (top * (1.0 - fy) + bot * fy).round() as u8;
    }
    Rgba(out)
}

/// Bakes a 1m grid + an origin cross-marker directly into a warped canvas.
/// Only meant for export, not the interactive preview: recomputing +
/// blending this on every point edit would be wasted work when the live
/// preview panel already draws its own overlay for free via the painter.
///
/// Because canvas space is a plain axis-aligned scale+flip of world space
/// (the warp already removed all perspective), grid lines land on exact
/// canvas rows/columns -- no line-rasterization library needed, just pixel
/// blending along full rows/columns.
pub fn draw_grid_overlay(image: &mut RgbaImage, extent: &WorldExtent, effective_ppm: f64) {
    let (w, h) = image.dimensions();
    let grid_color = Rgba([255, 255, 255, 90]);
    let axis_color = Rgba([230, 60, 60, 255]);

    let world_x_to_col = |x: f64| -> i64 { ((x - extent.x_min) * effective_ppm).round() as i64 };
    let world_y_to_row = |y: f64| -> i64 { ((extent.y_max - y) * effective_ppm).round() as i64 };

    let mut gx = extent.x_min.ceil() as i64;
    while (gx as f64) < extent.x_max {
        let col = world_x_to_col(gx as f64);
        if col >= 0 && (col as u32) < w {
            for row in 0..h {
                blend_pixel(image, col as u32, row, grid_color);
            }
        }
        gx += 1;
    }

    let mut gy = extent.y_min.ceil() as i64;
    while (gy as f64) < extent.y_max {
        let row = world_y_to_row(gy as f64);
        if row >= 0 && (row as u32) < h {
            for col in 0..w {
                blend_pixel(image, col, row as u32, grid_color);
            }
        }
        gy += 1;
    }

    // Origin cross-marker (world (0,0)), a few pixels wide either side.
    let ox = world_x_to_col(0.0);
    let oy = world_y_to_row(0.0);
    for d in -10i64..=10 {
        let x = ox + d;
        if x >= 0 && (x as u32) < w && oy >= 0 && (oy as u32) < h {
            blend_pixel(image, x as u32, oy as u32, axis_color);
        }
        let y = oy + d;
        if ox >= 0 && (ox as u32) < w && y >= 0 && (y as u32) < h {
            blend_pixel(image, ox as u32, y as u32, axis_color);
        }
    }
}

#[inline]
fn blend_pixel(image: &mut RgbaImage, x: u32, y: u32, color: Rgba<u8>) {
    let bg = *image.get_pixel(x, y);
    let a = color.0[3] as f64 / 255.0;
    let mut out = [0u8; 4];
    for c in 0..3 {
        out[c] = (color.0[c] as f64 * a + bg.0[c] as f64 * (1.0 - a)).round() as u8;
    }
    out[3] = 255;
    image.put_pixel(x, y, Rgba(out));
}
