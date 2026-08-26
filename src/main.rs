#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod homography;
mod project;
mod warp;

use std::path::PathBuf;

fn print_usage() {
    eprintln!(
        "HomoKuŽel - interactive drone-photo -> metric birdseye rectifier\n\
         \n\
         USAGE:\n\
         \x20   homokuzel [image.jpg]\n\
         \x20       Launch the GUI, optionally pre-loading an image.\n\
         \n\
         \x20   homokuzel export <project.json> [output.png] [--grid]\n\
         \x20       Headless mode: recompute the homography from a saved project\n\
         \x20       and write the birdseye PNG. No window is opened. Intended for\n\
         \x20       CI pipelines that regenerate maps whenever a project file or\n\
         \x20       drone photo is updated. Pass --grid to bake in the 1m grid +\n\
         \x20       origin marker, same as the GUI's export checkbox.\n"
    );
}

fn run_export(project_path: &str, output_path: &str, with_grid: bool) -> anyhow::Result<()> {
    let project = project::Project::load(std::path::Path::new(project_path))?;
    let src = image::open(&project.image_path)?.into_rgba8();

    let correspondences: Vec<homography::Correspondence> = project
        .points
        .iter()
        .map(|p| homography::Correspondence { pixel: (p.pixel_u, p.pixel_v), world: (p.world_x, p.world_y) })
        .collect();

    let result = homography::solve_homography(&correspondences)?;
    let h_world_to_img = result
        .h_world_to_img
        .ok_or_else(|| anyhow::anyhow!("homography is singular"))?;

    let (w, h) = src.dimensions();
    let extent = warp::compute_extent(w, h, &result.h_img_to_world);
    let params = warp::BirdseyeParams {
        pixels_per_meter: project.pixels_per_meter,
        margin_m: project.margin_m,
        max_canvas_dim: 8000,
    };
    let mut output = warp::warp_to_birdseye(&src, &h_world_to_img, &extent, &params);
    if with_grid {
        warp::draw_grid_overlay(&mut output.image, &output.extent, output.effective_ppm);
    }
    output.image.save(output_path)?;

    println!(
        "Wrote {output_path}  ({}x{} px @ {:.2} px/m)  RMS error = {:.1} cm, max = {:.1} cm",
        output.image.width(),
        output.image.height(),
        output.effective_ppm,
        result.rms_error * 100.0,
        result.max_error * 100.0,
    );
    Ok(())
}

const LOGO_BYTES: &[u8] = include_bytes!("../assets/app_logo.jpg");

fn load_icon() -> Option<egui::IconData> {
    let img = image::load_from_memory(LOGO_BYTES).ok()?.into_rgba8();
    let (width, height) = img.dimensions();
    Some(egui::IconData {
        rgba: img.into_raw(),
        width,
        height,
    })
}

fn main() -> anyhow::Result<()> {
    let mut args: Vec<String> = std::env::args().collect();

    let with_grid = args.iter().any(|a| a == "--grid");
    args.retain(|a| a != "--grid");

    if args.len() >= 2 && args[1] == "export" {
        if args.len() < 3 {
            print_usage();
            std::process::exit(1);
        }
        let project_path = &args[2];
        let output_path = args.get(3).map(String::as_str).unwrap_or("birdseye_output.png");
        return run_export(project_path, output_path, with_grid);
    }

    if args.len() >= 2 && (args[1] == "-h" || args[1] == "--help") {
        print_usage();
        return Ok(());
    }

    let initial_image = args.get(1).map(PathBuf::from);

    let mut viewport = egui::ViewportBuilder::default()
        .with_inner_size([1400.0, 900.0])
        .with_title("HomoKuŽel");

    if let Some(icon) = load_icon() {
        viewport = viewport.with_icon(icon);
    }

    let native_options = eframe::NativeOptions {
        viewport,
        ..Default::default()
    };

    eframe::run_native(
        "HomoKuŽel",
        native_options,
        Box::new(|cc| Box::new(app::BirdseyeApp::new(cc, initial_image))),
    )
    .map_err(|e| anyhow::anyhow!("eframe error: {e}"))
}
