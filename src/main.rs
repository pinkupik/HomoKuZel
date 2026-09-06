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
    let proj_dir = std::path::Path::new(project_path).parent();

    if project.tabs.is_empty() {
        anyhow::bail!("Project has no image tabs");
    }

    let mut loaded_images = Vec::new();
    let mut homographies = Vec::new();
    let mut extents = Vec::new();
    let mut valid_tabs = Vec::new();

    for tab in &project.tabs {
        let Some(img_path) = &tab.image_path else {
            continue;
        };
        let resolved_path = if img_path.is_absolute() || img_path.exists() {
            img_path.clone()
        } else if let Some(parent) = proj_dir {
            let candidate = parent.join(img_path);
            if candidate.exists() {
                candidate
            } else {
                img_path.clone()
            }
        } else {
            img_path.clone()
        };

        let src = match image::open(&resolved_path) {
            Ok(img) => img.into_rgba8(),
            Err(e) => {
                eprintln!("Warning: failed to open {}: {e}", resolved_path.display());
                continue;
            }
        };

        let correspondences: Vec<homography::Correspondence> = tab
            .points
            .iter()
            .map(|p| homography::Correspondence {
                pixel: (p.pixel_u, p.pixel_v),
                world: (p.world_x, p.world_y),
            })
            .collect();

        if correspondences.len() < 4 {
            continue;
        }

        let result = match homography::solve_homography(&correspondences) {
            Ok(r) => r,
            Err(e) => {
                eprintln!("Warning: homography failed for {}: {e}", tab.name);
                continue;
            }
        };

        let Some(h_world_to_img) = result.h_world_to_img else {
            continue;
        };
        let (w, h) = src.dimensions();
        let extent = warp::compute_extent(w, h, &result.h_img_to_world);

        loaded_images.push(src);
        homographies.push(h_world_to_img);
        extents.push(extent);
        valid_tabs.push(tab);
    }

    if loaded_images.is_empty() {
        anyhow::bail!("No calibrated layers available to export (each needs >= 4 points)");
    }

    let layer_inputs: Vec<warp::MergeLayerInput<'_>> = (0..loaded_images.len())
        .map(|i| warp::MergeLayerInput {
            src: &loaded_images[i],
            h_world_to_img: &homographies[i],
            extent_unmargined: &extents[i],
            adjustment: &valid_tabs[i].adjustment,
        })
        .collect();

    let params = warp::BirdseyeParams {
        pixels_per_meter: project.pixels_per_meter,
        margin_m: project.margin_m,
        max_canvas_dim: 8000,
    };

    let mut output = warp::composite_merged_map(&layer_inputs, &params)
        .ok_or_else(|| anyhow::anyhow!("Failed to composite map"))?;

    if with_grid {
        warp::draw_grid_overlay(&mut output.image, &output.extent, output.effective_ppm);
    }

    output.image.save(output_path)?;

    println!(
        "Wrote {output_path} ({}x{} px @ {:.2} px/m) from {} layers",
        output.image.width(),
        output.image.height(),
        output.effective_ppm,
        layer_inputs.len(),
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
        let output_path = args.get(3).map(String::as_str).unwrap_or("homokuzel_output.png");
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
