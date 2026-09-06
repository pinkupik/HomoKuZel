use eframe::egui;
use egui::{Color32, ColorImage, DragValue, Pos2, Rect, Sense, Slider, TextureHandle, TextureOptions, Vec2};
use image::RgbaImage;
use std::path::{Path, PathBuf};

use crate::homography::{self, Correspondence, HomographyResult};
use crate::project::{ImageTabRecord, LayerAdjustment, PointRecord, Project};
use crate::warp::{self, BirdseyeParams, MergeLayerInput, WorldExtent};

const HANDLE_RADIUS: f32 = 6.0;
const LOGO_BYTES: &[u8] = include_bytes!("../assets/app_logo.jpg");

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum ActiveTab {
    Image(usize),
    Merge,
}

pub struct ImageTab {
    pub id: usize,
    pub name: String,
    pub image_path: Option<PathBuf>,
    pub src_image: Option<RgbaImage>,
    pub src_texture: Option<TextureHandle>,

    pub points: Vec<PointRecord>,
    pub next_label: usize,
    pub pending_pixel: Option<(f64, f64)>,
    pub pending_x_str: String,
    pub pending_y_str: String,
    pub focus_pending_input: bool,
    pub dragging: Option<usize>,

    pub zoom: f32,
    pub pan: Vec2,

    pub birdseye_zoom: f32,
    pub birdseye_pan: Vec2,

    pub result: Option<HomographyResult>,
    pub birdseye_texture: Option<TextureHandle>,
    pub effective_ppm: f64,
    pub extent: Option<WorldExtent>,
    pub dirty: bool,

    pub adjustment: LayerAdjustment,
}

impl ImageTab {
    pub fn new(id: usize, name: String) -> Self {
        Self {
            id,
            name,
            image_path: None,
            src_image: None,
            src_texture: None,
            points: Vec::new(),
            next_label: 1,
            pending_pixel: None,
            pending_x_str: String::new(),
            pending_y_str: String::new(),
            focus_pending_input: false,
            dragging: None,
            zoom: 1.0,
            pan: Vec2::ZERO,
            birdseye_zoom: 1.0,
            birdseye_pan: Vec2::ZERO,
            result: None,
            birdseye_texture: None,
            effective_ppm: 25.0,
            extent: None,
            dirty: false,
            adjustment: LayerAdjustment::default(),
        }
    }

    pub fn load_image(&mut self, ctx: &egui::Context, path: &Path) -> anyhow::Result<()> {
        let img = image::open(path)?.into_rgba8();
        let (w, h) = img.dimensions();
        let color_image = ColorImage::from_rgba_unmultiplied([w as usize, h as usize], img.as_raw());
        let texture_name = format!("src_image_{}", self.id);
        let texture = ctx.load_texture(texture_name, color_image, TextureOptions::LINEAR);

        self.src_texture = Some(texture);
        self.src_image = Some(img);
        self.image_path = Some(path.to_path_buf());
        self.name = path
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| format!("Image {}", self.id));
        self.points.clear();
        self.next_label = 1;
        self.result = None;
        self.birdseye_texture = None;
        self.extent = None;
        self.zoom = 1.0;
        self.pan = Vec2::ZERO;
        self.birdseye_zoom = 1.0;
        self.birdseye_pan = Vec2::ZERO;
        self.dirty = false;
        Ok(())
    }

    pub fn recompute(&mut self, ctx: &egui::Context, ppm: f64, margin_m: f64) -> Option<String> {
        self.dirty = false;
        let Some(src) = &self.src_image else {
            return None;
        };

        if self.points.len() < 4 {
            self.result = None;
            self.birdseye_texture = None;
            self.extent = None;
            return Some(format!("Need at least 4 points ({} so far).", self.points.len()));
        }

        let correspondences: Vec<Correspondence> = self
            .points
            .iter()
            .map(|p| Correspondence {
                pixel: (p.pixel_u, p.pixel_v),
                world: (p.world_x, p.world_y),
            })
            .collect();

        let result = match homography::solve_homography(&correspondences) {
            Ok(r) => r,
            Err(e) => {
                self.result = None;
                self.birdseye_texture = None;
                self.extent = None;
                return Some(format!("Homography failed: {e}"));
            }
        };

        let Some(h_world_to_img) = result.h_world_to_img else {
            self.result = Some(result);
            self.birdseye_texture = None;
            self.extent = None;
            return Some("Homography is singular (points may be collinear or duplicated).".to_string());
        };

        let (w, h) = src.dimensions();
        let extent = warp::compute_extent(w, h, &result.h_img_to_world);
        let params = BirdseyeParams {
            pixels_per_meter: ppm,
            margin_m,
            max_canvas_dim: 3000,
        };
        let output = warp::warp_to_birdseye(src, &h_world_to_img, &extent, &params);

        let (cw, ch) = output.image.dimensions();
        let color_image = ColorImage::from_rgba_unmultiplied([cw as usize, ch as usize], output.image.as_raw());
        let texture_name = format!("birdseye_{}", self.id);
        let texture = ctx.load_texture(texture_name, color_image, TextureOptions::LINEAR);

        let status = format!(
            "RMS error: {:.1} cm (max {:.1} cm) | {:.2} px/m | canvas {cw}x{ch}",
            result.rms_error * 100.0,
            result.max_error * 100.0,
            output.effective_ppm,
        );
        self.effective_ppm = output.effective_ppm;
        self.birdseye_texture = Some(texture);
        self.result = Some(result);
        self.extent = Some(extent);

        Some(status)
    }

    pub fn add_point(&mut self, pixel: (f64, f64), world: (f64, f64)) {
        let label = format!("P{}", self.next_label);
        self.next_label += 1;
        self.points.push(PointRecord {
            label,
            pixel_u: pixel.0,
            pixel_v: pixel.1,
            world_x: world.0,
            world_y: world.1,
        });
        self.dirty = true;
    }

    pub fn export_birdseye(&self, path: &Path, ppm: f64, margin_m: f64, with_grid: bool) -> anyhow::Result<()> {
        let Some(result) = &self.result else { anyhow::bail!("no homography computed yet (need >= 4 points)") };
        let Some(h_world_to_img) = result.h_world_to_img else { anyhow::bail!("homography is singular") };
        let Some(src) = &self.src_image else { anyhow::bail!("no source image loaded") };
        let (w, h) = src.dimensions();
        let extent = warp::compute_extent(w, h, &result.h_img_to_world);
        let params = BirdseyeParams {
            pixels_per_meter: ppm,
            margin_m,
            max_canvas_dim: 8000,
        };
        let mut output = warp::warp_to_birdseye(src, &h_world_to_img, &extent, &params);
        if with_grid {
            warp::draw_grid_overlay(&mut output.image, &output.extent, output.effective_ppm);
        }
        output.image.save(path)?;
        Ok(())
    }
}

pub struct BirdseyeApp {
    logo_texture: Option<TextureHandle>,

    tabs: Vec<ImageTab>,
    next_tab_id: usize,
    active_tab: ActiveTab,

    // Global calibration settings
    pixels_per_meter: f64,
    margin_m: f64,
    export_with_grid: bool,

    // Merged view state
    merged_texture: Option<TextureHandle>,
    merged_extent: Option<WorldExtent>,
    merged_ppm: f64,
    merged_zoom: f32,
    merged_pan: Vec2,
    merged_dirty: bool,
    selected_layer_idx: usize,

    // Project & session state
    project_path: Option<PathBuf>,
    status: String,
}

impl Default for BirdseyeApp {
    fn default() -> Self {
        let initial_tab = ImageTab::new(1, "Image 1".to_string());
        Self {
            logo_texture: None,
            tabs: vec![initial_tab],
            next_tab_id: 2,
            active_tab: ActiveTab::Image(0),
            pixels_per_meter: 25.0,
            margin_m: 4.0,
            export_with_grid: true,
            merged_texture: None,
            merged_extent: None,
            merged_ppm: 25.0,
            merged_zoom: 1.0,
            merged_pan: Vec2::ZERO,
            merged_dirty: false,
            selected_layer_idx: 0,
            project_path: None,
            status: "Load an image or project to begin.".to_string(),
        }
    }
}

impl BirdseyeApp {
    pub fn new(cc: &eframe::CreationContext<'_>, initial_file: Option<PathBuf>) -> Self {
        let mut app = Self::default();

        if let Ok(img) = image::load_from_memory(LOGO_BYTES) {
            let rgba = img.into_rgba8();
            let (w, h) = rgba.dimensions();
            let color_image = ColorImage::from_rgba_unmultiplied([w as usize, h as usize], rgba.as_raw());
            app.logo_texture = Some(cc.egui_ctx.load_texture("app_logo", color_image, TextureOptions::LINEAR));
        }

        if let Some(path) = initial_file {
            let is_json = path
                .extension()
                .and_then(|e| e.to_str())
                .map(|e| e.eq_ignore_ascii_case("json"))
                .unwrap_or(false);
            if is_json {
                if let Err(e) = app.load_project(&cc.egui_ctx, &path) {
                    app.status = format!("Failed to load project {}: {e}", path.display());
                }
            } else if let Err(e) = app.tabs[0].load_image(&cc.egui_ctx, &path) {
                app.status = format!("Failed to load {}: {e}", path.display());
            } else {
                let name = app.tabs[0].name.clone();
                app.status = format!("Loaded {name}. Click the photo to add reference points.");
            }
        }
        app
    }

    fn add_image_tab(&mut self, ctx: &egui::Context, path: Option<&Path>) {
        let id = self.next_tab_id;
        self.next_tab_id += 1;
        let mut tab = ImageTab::new(id, format!("Image {id}"));
        if let Some(p) = path {
            if let Err(e) = tab.load_image(ctx, p) {
                self.status = format!("Failed to load {}: {e}", p.display());
            }
        }
        self.tabs.push(tab);
        let new_idx = self.tabs.len() - 1;
        self.active_tab = ActiveTab::Image(new_idx);
        self.selected_layer_idx = new_idx;
        self.merged_dirty = true;
    }

    fn close_tab(&mut self, idx: usize) {
        if self.tabs.len() <= 1 {
            return;
        }
        self.tabs.remove(idx);
        match self.active_tab {
            ActiveTab::Image(current) => {
                if current == idx {
                    self.active_tab = ActiveTab::Image(idx.saturating_sub(1));
                } else if current > idx {
                    self.active_tab = ActiveTab::Image(current - 1);
                }
            }
            ActiveTab::Merge => {}
        }
        if self.selected_layer_idx >= self.tabs.len() {
            self.selected_layer_idx = self.tabs.len().saturating_sub(1);
        }
        self.merged_dirty = true;
    }

    fn recompute_merged(&mut self, ctx: &egui::Context) {
        self.merged_dirty = false;

        let mut layer_inputs = Vec::new();
        for tab in &self.tabs {
            if let (Some(src), Some(result), Some(extent)) = (&tab.src_image, &tab.result, &tab.extent) {
                if let Some(h) = &result.h_world_to_img {
                    layer_inputs.push(MergeLayerInput {
                        src,
                        h_world_to_img: h,
                        extent_unmargined: extent,
                        adjustment: &tab.adjustment,
                    });
                }
            }
        }

        if layer_inputs.is_empty() {
            self.merged_texture = None;
            self.merged_extent = None;
            return;
        }

        let params = BirdseyeParams {
            pixels_per_meter: self.pixels_per_meter,
            margin_m: self.margin_m,
            max_canvas_dim: 3000,
        };

        if let Some(output) = warp::composite_merged_map(&layer_inputs, &params) {
            let (cw, ch) = output.image.dimensions();
            let color_image = ColorImage::from_rgba_unmultiplied([cw as usize, ch as usize], output.image.as_raw());
            let texture = ctx.load_texture("merged_map", color_image, TextureOptions::LINEAR);
            self.merged_texture = Some(texture);
            self.merged_extent = Some(output.extent);
            self.merged_ppm = output.effective_ppm;

            if self.active_tab == ActiveTab::Merge {
                self.status = format!(
                    "Merged {} layers | {:.2} px/m | canvas {cw}x{ch}",
                    layer_inputs.len(),
                    output.effective_ppm
                );
            }
        }
    }

    fn export_merged_map(&self, path: &Path) -> anyhow::Result<()> {
        let mut layer_inputs = Vec::new();
        for tab in &self.tabs {
            if let (Some(src), Some(result), Some(extent)) = (&tab.src_image, &tab.result, &tab.extent) {
                if let Some(h) = &result.h_world_to_img {
                    layer_inputs.push(MergeLayerInput {
                        src,
                        h_world_to_img: h,
                        extent_unmargined: extent,
                        adjustment: &tab.adjustment,
                    });
                }
            }
        }

        if layer_inputs.is_empty() {
            anyhow::bail!("No calibrated layers available to merge (each needs >= 4 points)");
        }

        let params = BirdseyeParams {
            pixels_per_meter: self.pixels_per_meter,
            margin_m: self.margin_m,
            max_canvas_dim: 8000,
        };

        let mut output = warp::composite_merged_map(&layer_inputs, &params)
            .ok_or_else(|| anyhow::anyhow!("Failed to composite merged map"))?;

        if self.export_with_grid {
            warp::draw_grid_overlay(&mut output.image, &output.extent, output.effective_ppm);
        }

        output.image.save(path)?;
        Ok(())
    }

    fn save_project(&self, path: &Path) -> anyhow::Result<()> {
        let tab_records: Vec<ImageTabRecord> = self
            .tabs
            .iter()
            .map(|t| ImageTabRecord {
                name: t.name.clone(),
                image_path: t.image_path.clone(),
                points: t.points.clone(),
                adjustment: t.adjustment.clone(),
            })
            .collect();

        let project = Project {
            tabs: tab_records,
            pixels_per_meter: self.pixels_per_meter,
            margin_m: self.margin_m,
            image_path: None,
            points: None,
        };
        project.save(path)
    }

    fn load_project(&mut self, ctx: &egui::Context, path: &Path) -> anyhow::Result<()> {
        let project = Project::load(path)?;
        self.tabs.clear();
        self.next_tab_id = 1;

        let proj_dir = path.parent();

        for tab_record in project.tabs {
            let mut tab = ImageTab::new(self.next_tab_id, tab_record.name);
            self.next_tab_id += 1;
            tab.adjustment = tab_record.adjustment;
            tab.points = tab_record.points;
            tab.next_label = tab.points.len() + 1;

            if let Some(img_path) = tab_record.image_path {
                let resolved_path = if img_path.is_absolute() || img_path.exists() {
                    img_path
                } else if let Some(parent) = proj_dir {
                    let candidate = parent.join(&img_path);
                    if candidate.exists() {
                        candidate
                    } else {
                        img_path
                    }
                } else {
                    img_path
                };

                if let Err(e) = tab.load_image(ctx, &resolved_path) {
                    eprintln!("Warning: failed to load image {}: {e}", resolved_path.display());
                }
            }

            if tab.src_image.is_some() && tab.points.len() >= 4 {
                tab.recompute(ctx, project.pixels_per_meter, project.margin_m);
            }

            self.tabs.push(tab);
        }

        if self.tabs.is_empty() {
            self.tabs.push(ImageTab::new(1, "Image 1".to_string()));
            self.next_tab_id = 2;
        }

        self.pixels_per_meter = project.pixels_per_meter;
        self.margin_m = project.margin_m;
        self.project_path = Some(path.to_path_buf());
        self.active_tab = ActiveTab::Image(0);
        self.selected_layer_idx = 0;
        self.merged_dirty = true;

        let proj_name = path
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| path.display().to_string());
        self.status = format!("Loaded project {proj_name} with {} image tabs.", self.tabs.len());
        Ok(())
    }

    fn top_panel(&mut self, ctx: &egui::Context) {
        let mut tab_to_close: Option<usize> = None;
        let mut add_image_clicked = false;

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.add_space(2.0);

            // Row 1: Actions & Settings
            ui.horizontal_wrapped(|ui| {
                ui.label(egui::RichText::new("HomoKuŽel").strong().color(Color32::from_rgb(80, 160, 255)));
                ui.separator();

                // Open Image button
                if ui.button("Open Image...").on_hover_text("Open drone photo (PNG, JPG, etc.) into current tab").clicked() {
                    let mut dialog = rfd::FileDialog::new()
                        .set_title("Open Drone Photo")
                        .add_filter("Image Files", &["png", "jpg", "jpeg", "bmp", "webp", "tiff"]);

                    if let ActiveTab::Image(curr) = self.active_tab {
                        if let Some(ref p) = self.tabs[curr].image_path {
                            if let Some(parent) = p.parent() {
                                dialog = dialog.set_directory(parent);
                            }
                        }
                    }
                    if let Some(path) = dialog.pick_file() {
                        if let ActiveTab::Image(curr) = self.active_tab {
                            if let Err(e) = self.tabs[curr].load_image(ctx, &path) {
                                self.status = format!("Failed to load {}: {e}", path.display());
                            } else {
                                let name = self.tabs[curr].name.clone();
                                self.status = format!("Loaded {name}. Click the photo to add reference points.");
                                self.merged_dirty = true;
                            }
                        } else {
                            self.add_image_tab(ctx, Some(&path));
                        }
                    }
                }

                ui.separator();

                // Save Project button
                if ui.button("Save Project...").on_hover_text("Save all tabs, points, and alignment adjustments to JSON").clicked() {
                    let mut dialog = rfd::FileDialog::new()
                        .set_title("Save Project")
                        .add_filter("HomoKuŽel Project (*.json)", &["json"])
                        .set_file_name(
                            self.project_path
                                .as_ref()
                                .and_then(|p| p.file_name())
                                .map(|n| n.to_string_lossy().to_string())
                                .unwrap_or_else(|| "project.json".to_string()),
                        );
                    if let Some(ref p) = self.project_path {
                        if let Some(parent) = p.parent() {
                            dialog = dialog.set_directory(parent);
                        }
                    }
                    if let Some(path) = dialog.save_file() {
                        if let Err(e) = self.save_project(&path) {
                            self.status = format!("Save failed: {e}");
                        } else {
                            let name = path
                                .file_name()
                                .map(|n| n.to_string_lossy().to_string())
                                .unwrap_or_else(|| path.display().to_string());
                            self.status = format!("Saved project to {name}");
                            self.project_path = Some(path);
                        }
                    }
                }

                // Load Project button
                if ui.button("Load Project...").on_hover_text("Load a saved calibration project (.json)").clicked() {
                    let mut dialog = rfd::FileDialog::new()
                        .set_title("Load Project")
                        .add_filter("HomoKuŽel Project (*.json)", &["json"]);
                    if let Some(ref p) = self.project_path {
                        if let Some(parent) = p.parent() {
                            dialog = dialog.set_directory(parent);
                        }
                    }
                    if let Some(path) = dialog.pick_file() {
                        if let Err(e) = self.load_project(ctx, &path) {
                            self.status = format!("Load failed: {e}");
                        }
                    }
                }

                ui.separator();

                // Global Settings: px/m and margin
                ui.label("px/m:");
                if ui.add(DragValue::new(&mut self.pixels_per_meter).clamp_range(1.0..=500.0).speed(1.0))
                    .on_hover_text("Resolution (pixels per meter)").changed()
                {
                    for tab in &mut self.tabs {
                        tab.dirty = true;
                    }
                    self.merged_dirty = true;
                }

                ui.label("margin (m):");
                if ui.add(DragValue::new(&mut self.margin_m).clamp_range(0.0..=50.0).speed(0.1))
                    .on_hover_text("Border padding around area in meters").changed()
                {
                    for tab in &mut self.tabs {
                        tab.dirty = true;
                    }
                    self.merged_dirty = true;
                }

                ui.separator();

                // Export buttons
                ui.checkbox(&mut self.export_with_grid, "1m grid")
                    .on_hover_text("Bake 1m grid and origin marker into exported PNG");

                match self.active_tab {
                    ActiveTab::Image(curr) => {
                        let export_enabled = self.tabs[curr].result.is_some() && self.tabs[curr].src_image.is_some();
                        ui.add_enabled_ui(export_enabled, |ui| {
                            if ui.button("Export Birdseye...").on_hover_text("Export full-resolution rectified PNG of current image").clicked() {
                                let mut dialog = rfd::FileDialog::new()
                                    .set_title("Export Birdseye Image")
                                    .add_filter("PNG Image (*.png)", &["png"])
                                    .set_file_name("homokuzel_output.png");
                                if let Some(ref p) = self.tabs[curr].image_path {
                                    if let Some(parent) = p.parent() {
                                        dialog = dialog.set_directory(parent);
                                    }
                                }
                                if let Some(path) = dialog.save_file() {
                                    match self.tabs[curr].export_birdseye(&path, self.pixels_per_meter, self.margin_m, self.export_with_grid) {
                                        Ok(()) => {
                                            let name = path
                                                .file_name()
                                                .map(|n| n.to_string_lossy().to_string())
                                                .unwrap_or_else(|| path.display().to_string());
                                            self.status = format!("Exported birdseye to {name}");
                                        }
                                        Err(e) => self.status = format!("Export failed: {e}"),
                                    }
                                }
                            }
                        });
                    }
                    ActiveTab::Merge => {
                        let can_export = self.tabs.iter().any(|t| t.result.is_some() && t.adjustment.enabled);
                        ui.add_enabled_ui(can_export, |ui| {
                            if ui.button("Export Merged Map...").on_hover_text("Export full-resolution merged composite PNG").clicked() {
                                let dialog = rfd::FileDialog::new()
                                    .set_title("Export Merged Map")
                                    .add_filter("PNG Image (*.png)", &["png"])
                                    .set_file_name("homokuzel_merged.png");
                                if let Some(path) = dialog.save_file() {
                                    match self.export_merged_map(&path) {
                                        Ok(()) => {
                                            let name = path
                                                .file_name()
                                                .map(|n| n.to_string_lossy().to_string())
                                                .unwrap_or_else(|| path.display().to_string());
                                            self.status = format!("Exported merged map to {name}");
                                        }
                                        Err(e) => self.status = format!("Export failed: {e}"),
                                    }
                                }
                            }
                        });
                    }
                }
            });

            ui.add_space(3.0);
            ui.separator();
            ui.add_space(2.0);

            // Row 2: Tab Bar
            ui.horizontal(|ui| {
                for (i, tab) in self.tabs.iter().enumerate() {
                    let is_active = self.active_tab == ActiveTab::Image(i);
                    let tab_title = if tab.name.len() > 18 {
                        format!("{}. {}...", i + 1, &tab.name[..15])
                    } else {
                        format!("{}. {}", i + 1, &tab.name)
                    };

                    ui.horizontal(|ui| {
                        ui.spacing_mut().item_spacing.x = 2.0;
                        if ui.selectable_label(is_active, &tab_title)
                            .on_hover_text(format!("Image: {}\nPoints: {}\nCalibrated: {}", tab.name, tab.points.len(), tab.result.is_some()))
                            .clicked()
                        {
                            self.active_tab = ActiveTab::Image(i);
                            self.selected_layer_idx = i;
                        }

                        if self.tabs.len() > 1 {
                            if ui.small_button("x").on_hover_text("Close tab").clicked() {
                                tab_to_close = Some(i);
                            }
                        }
                    });
                }

                if ui.button("+ Add Image").on_hover_text("Add another drone photo to this project").clicked() {
                    add_image_clicked = true;
                }

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let is_merge_active = self.active_tab == ActiveTab::Merge;
                    let calibrated_count = self.tabs.iter().filter(|t| t.result.is_some() && t.adjustment.enabled).count();
                    let merge_label = format!("Merged Map ({}/{})", calibrated_count, self.tabs.len());

                    if ui.selectable_label(is_merge_active, egui::RichText::new(merge_label).strong()).clicked() {
                        self.active_tab = ActiveTab::Merge;
                        self.merged_dirty = true;
                    }
                });
            });

            ui.add_space(2.0);
            ui.separator();
            ui.add_space(2.0);

            // Row 3: Status bar
            ui.horizontal(|ui| {
                let status_color = if self.status.starts_with("Failed")
                    || self.status.starts_with("Homography failed")
                    || self.status.starts_with("Export failed")
                    || self.status.starts_with("Save failed")
                {
                    Color32::from_rgb(230, 80, 80)
                } else if self.status.starts_with("RMS") || self.status.starts_with("Merged") {
                    Color32::from_rgb(180, 220, 255)
                } else if self.status.starts_with("Exported") || self.status.starts_with("Saved") {
                    Color32::from_rgb(80, 220, 100)
                } else {
                    Color32::from_gray(200)
                };
                ui.label(egui::RichText::new(&self.status).color(status_color));
            });
            ui.add_space(2.0);
        });

        if let Some(close_idx) = tab_to_close {
            self.close_tab(close_idx);
        }

        if add_image_clicked {
            let dialog = rfd::FileDialog::new()
                .set_title("Open Additional Drone Photo")
                .add_filter("Image Files", &["png", "jpg", "jpeg", "bmp", "webp", "tiff"]);
            if let Some(path) = dialog.pick_file() {
                self.add_image_tab(ctx, Some(&path));
            } else {
                self.add_image_tab(ctx, None);
            }
        }
    }

    fn image_panel(&mut self, tab_idx: usize, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Left click: add point. Drag a marker: reposition it. Scroll: zoom.");
            if ui.button("Fit").clicked() {
                self.tabs[tab_idx].zoom = 1.0;
                self.tabs[tab_idx].pan = Vec2::ZERO;
            }
        });

        let Some(texture) = self.tabs[tab_idx].src_texture.clone() else {
            ui.centered_and_justified(|ui| {
                ui.vertical_centered(|ui| {
                    ui.add_space(20.0);
                    if let Some(ref logo) = self.logo_texture {
                        ui.image((logo.id(), egui::vec2(180.0, 180.0)));
                    }
                    ui.add_space(10.0);
                    ui.heading(egui::RichText::new("HomoKuŽel").size(26.0).strong().color(Color32::from_rgb(80, 160, 255)));
                    ui.add_space(16.0);

                    ui.horizontal(|ui| {
                        if ui.button(egui::RichText::new("Open Image").size(15.0)).clicked() {
                            let dialog = rfd::FileDialog::new()
                                .set_title("Open Drone Photo")
                                .add_filter("Image Files", &["png", "jpg", "jpeg", "bmp", "webp", "tiff"]);
                            if let Some(path) = dialog.pick_file() {
                                if let Err(e) = self.tabs[tab_idx].load_image(ui.ctx(), &path) {
                                    self.status = format!("Failed to load {}: {e}", path.display());
                                } else {
                                    let name = self.tabs[tab_idx].name.clone();
                                    self.status = format!("Loaded {name}. Click the photo to add reference points.");
                                    self.merged_dirty = true;
                                }
                            }
                        }
                        if ui.button(egui::RichText::new("Load Project").size(15.0)).clicked() {
                            let dialog = rfd::FileDialog::new()
                                .set_title("Load Project")
                                .add_filter("HomoKuŽel Project (*.json)", &["json"]);
                            if let Some(path) = dialog.pick_file() {
                                if let Err(e) = self.load_project(ui.ctx(), &path) {
                                    self.status = format!("Load failed: {e}");
                                }
                            }
                        }
                    });

                    ui.add_space(20.0);
                    ui.group(|ui| {
                        ui.set_max_width(440.0);
                        ui.label(egui::RichText::new("Quick Start:").strong().color(Color32::from_rgb(255, 200, 80)));
                        ui.label("1. Drag & drop a drone photo here (or click Open).");
                        ui.label("2. Left-click 4+ points with known real-world positions.");
                        ui.label("3. Type real-world coords in metres (Tab to move, Enter to add).");
                        ui.label("4. Switch to Merged Map to combine multiple photos!");
                    });
                });
            });
            return;
        };

        let avail = ui.available_size();
        let img_size = texture.size_vec2();
        let base_scale = (avail.x / img_size.x).min(2.0).max(0.05);
        let scale = base_scale * self.tabs[tab_idx].zoom;
        let display_size = img_size * scale;

        let (rect, response) = ui.allocate_exact_size(avail, Sense::click_and_drag());

        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.0 {
                self.tabs[tab_idx].zoom = (self.tabs[tab_idx].zoom * (1.0 + scroll * 0.001)).clamp(0.1, 20.0);
            }
        }

        let origin = rect.min + self.tabs[tab_idx].pan;
        let image_rect = Rect::from_min_size(origin, display_size);

        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_gray(30));
        painter.image(
            texture.id(),
            image_rect,
            Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
            Color32::WHITE,
        );

        let to_screen = |px: (f64, f64)| -> Pos2 {
            image_rect.min + Vec2::new(px.0 as f32 * scale, px.1 as f32 * scale)
        };
        let to_pixel = |screen: Pos2| -> (f64, f64) {
            let local = screen - image_rect.min;
            ((local.x / scale) as f64, (local.y / scale) as f64)
        };

        // Draw points
        for (i, p) in self.tabs[tab_idx].points.iter().enumerate() {
            let center = to_screen((p.pixel_u, p.pixel_v));
            let color = match &self.tabs[tab_idx].result {
                Some(r) => {
                    let err = r.reproj_errors.get(i).copied().unwrap_or(0.0);
                    if err < 0.05 {
                        Color32::from_rgb(80, 220, 100)
                    } else if err < 0.2 {
                        Color32::from_rgb(230, 200, 60)
                    } else {
                        Color32::from_rgb(230, 70, 70)
                    }
                }
                None => Color32::from_rgb(90, 160, 230),
            };
            painter.circle_stroke(center, HANDLE_RADIUS, (2.0, color));
            painter.circle_filled(center, 2.0, color);
            painter.text(
                center + Vec2::new(8.0, -8.0),
                egui::Align2::LEFT_BOTTOM,
                format!("{} ({:.2},{:.2})", p.label, p.world_x, p.world_y),
                egui::FontId::proportional(12.0),
                color,
            );
        }

        if response.drag_started() {
            if let Some(pos) = response.interact_pointer_pos() {
                let mut best: Option<(usize, f32)> = None;
                for (i, p) in self.tabs[tab_idx].points.iter().enumerate() {
                    let d = to_screen((p.pixel_u, p.pixel_v)).distance(pos);
                    if d <= HANDLE_RADIUS * 2.0 && best.map_or(true, |(_, bd)| d < bd) {
                        best = Some((i, d));
                    }
                }
                self.tabs[tab_idx].dragging = best.map(|(i, _)| i);
            }
        }

        if response.dragged() {
            if let Some(i) = self.tabs[tab_idx].dragging {
                if let Some(pos) = response.interact_pointer_pos() {
                    let (u, v) = to_pixel(pos);
                    self.tabs[tab_idx].points[i].pixel_u = u;
                    self.tabs[tab_idx].points[i].pixel_v = v;
                    self.tabs[tab_idx].dirty = true;
                }
            } else {
                self.tabs[tab_idx].pan += response.drag_delta();
            }
        }

        if response.drag_stopped() {
            self.tabs[tab_idx].dragging = None;
        }

        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let near_existing = self.tabs[tab_idx]
                    .points
                    .iter()
                    .any(|p| to_screen((p.pixel_u, p.pixel_v)).distance(pos) <= HANDLE_RADIUS * 2.0);
                if !near_existing {
                    self.tabs[tab_idx].pending_pixel = Some(to_pixel(pos));
                    self.tabs[tab_idx].pending_x_str.clear();
                    self.tabs[tab_idx].pending_y_str.clear();
                    self.tabs[tab_idx].focus_pending_input = true;
                }
            }
        }
    }

    fn pending_point_window(&mut self, tab_idx: usize, ctx: &egui::Context) {
        let Some(pixel) = self.tabs[tab_idx].pending_pixel else { return };
        let mut open = true;
        let mut confirmed = false;
        let mut cancelled = false;

        egui::Window::new("New reference point")
            .collapsible(false)
            .resizable(false)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .open(&mut open)
            .show(ctx, |ui| {
                ui.label(format!("Pixel: ({:.1}, {:.1})", pixel.0, pixel.1));
                ui.add_space(4.0);

                ui.horizontal(|ui| {
                    ui.label("World X (m):");
                    let x_edit = ui.add(
                        egui::TextEdit::singleline(&mut self.tabs[tab_idx].pending_x_str)
                            .hint_text("0.0")
                            .desired_width(100.0),
                    );
                    if self.tabs[tab_idx].focus_pending_input {
                        x_edit.request_focus();
                        self.tabs[tab_idx].focus_pending_input = false;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("World Y (m):");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.tabs[tab_idx].pending_y_str)
                            .hint_text("0.0")
                            .desired_width(100.0),
                    );
                });

                ui.add_space(4.0);
                ui.horizontal(|ui| {
                    if ui.button("Add point [Enter]").clicked() {
                        confirmed = true;
                    }
                    if ui.button("Cancel [Esc]").clicked() {
                        cancelled = true;
                    }
                });

                if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    confirmed = true;
                }
                if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    cancelled = true;
                }
            });

        if confirmed {
            let x = self.tabs[tab_idx].pending_x_str.trim().replace(',', ".").parse::<f64>().unwrap_or(0.0);
            let y = self.tabs[tab_idx].pending_y_str.trim().replace(',', ".").parse::<f64>().unwrap_or(0.0);
            self.tabs[tab_idx].add_point(pixel, (x, y));
            self.tabs[tab_idx].pending_pixel = None;
        } else if cancelled || !open {
            self.tabs[tab_idx].pending_pixel = None;
        }
    }

    fn table_panel(&mut self, tab_idx: usize, ui: &mut egui::Ui) {
        let tab = &mut self.tabs[tab_idx];
        ui.horizontal(|ui| {
            ui.label(format!("{} reference points", tab.points.len()));
            if !tab.points.is_empty()
                && ui.button("Clear all").on_hover_text("Delete all reference points for this tab").clicked()
            {
                tab.points.clear();
                tab.next_label = 1;
                tab.dirty = true;
            }
        });

        let reproj_errors = tab.result.as_ref().map(|r| r.reproj_errors.clone());
        let mut point_changed = false;
        let mut to_delete: Option<usize> = None;

        egui::ScrollArea::vertical().max_height(220.0).show(ui, |ui| {
            egui::Grid::new("points_table").striped(true).num_columns(6).show(ui, |ui| {
                ui.strong("Label");
                ui.strong("Pixel u,v");
                ui.strong("World X");
                ui.strong("World Y");
                ui.strong("Error");
                ui.strong("");
                ui.end_row();

                for (i, p) in tab.points.iter_mut().enumerate() {
                    ui.add(egui::TextEdit::singleline(&mut p.label).desired_width(40.0));
                    ui.label(format!("{:.1}, {:.1}", p.pixel_u, p.pixel_v));
                    if ui.add(DragValue::new(&mut p.world_x).speed(0.02)).changed() {
                        point_changed = true;
                    }
                    if ui.add(DragValue::new(&mut p.world_y).speed(0.02)).changed() {
                        point_changed = true;
                    }
                    match &reproj_errors {
                        Some(errs) => match errs.get(i) {
                            Some(e) => {
                                let err_cm = e * 100.0;
                                let color = if *e < 0.05 {
                                    Color32::from_rgb(80, 220, 100)
                                } else if *e < 0.20 {
                                    Color32::from_rgb(230, 200, 60)
                                } else {
                                    Color32::from_rgb(230, 70, 70)
                                };
                                ui.label(egui::RichText::new(format!("{err_cm:.1} cm")).color(color));
                            }
                            None => {
                                ui.label("—");
                            }
                        },
                        None => {
                            ui.label("—");
                        }
                    };
                    if ui.small_button("Delete").clicked() {
                        to_delete = Some(i);
                    }
                    ui.end_row();
                }
            });
        });

        if point_changed {
            tab.dirty = true;
        }
        if let Some(i) = to_delete {
            tab.points.remove(i);
            tab.dirty = true;
        }
    }

    fn birdseye_panel(&mut self, tab_idx: usize, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Live birdseye preview (scroll to zoom, drag to pan)");
            if ui.button("Fit").clicked() {
                self.tabs[tab_idx].birdseye_zoom = 1.0;
                self.tabs[tab_idx].birdseye_pan = Vec2::ZERO;
            }
        });

        let Some(texture) = self.tabs[tab_idx].birdseye_texture.clone() else {
            ui.centered_and_justified(|ui| ui.label("Add >= 4 points to compute the homography."));
            return;
        };

        let avail = ui.available_size();
        let img_size = texture.size_vec2();
        let base_scale = (avail.x / img_size.x).min(avail.y / img_size.y).min(2.0).max(0.01);
        let scale = base_scale * self.tabs[tab_idx].birdseye_zoom;
        let display_size = img_size * scale;

        let (rect, response) = ui.allocate_exact_size(avail, Sense::click_and_drag());

        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.0 {
                self.tabs[tab_idx].birdseye_zoom = (self.tabs[tab_idx].birdseye_zoom * (1.0 + scroll * 0.001)).clamp(0.1, 20.0);
            }
        }
        if response.dragged() {
            self.tabs[tab_idx].birdseye_pan += response.drag_delta();
        }

        let image_rect = Rect::from_min_size(rect.center() - display_size / 2.0 + self.tabs[tab_idx].birdseye_pan, display_size);
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_gray(20));
        painter.image(texture.id(), image_rect, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
    }

    fn merge_viewport(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Merged Map").strong());
            if let Some(ext) = self.merged_extent {
                let span_x = ext.x_max - ext.x_min;
                let span_y = ext.y_max - ext.y_min;
                ui.label(format!("Dimensions: {:.1}m x {:.1}m @ {:.1} px/m", span_x, span_y, self.merged_ppm));
            }
            if ui.button("Fit").clicked() {
                self.merged_zoom = 1.0;
                self.merged_pan = Vec2::ZERO;
            }
            ui.label("(scroll to zoom, drag to pan)");
        });

        let avail = ui.available_size();
        let Some(texture) = self.merged_texture.clone() else {
            ui.centered_and_justified(|ui| {
                ui.label("No calibrated layers to merge. Add at least 4 points to one or more image tabs.");
            });
            return;
        };

        let img_size = texture.size_vec2();
        let base_scale = (avail.x / img_size.x).min(avail.y / img_size.y).min(2.0).max(0.005);
        let scale = base_scale * self.merged_zoom;
        let display_size = img_size * scale;

        let (rect, response) = ui.allocate_exact_size(avail, Sense::click_and_drag());

        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.0 {
                self.merged_zoom = (self.merged_zoom * (1.0 + scroll * 0.001)).clamp(0.05, 30.0);
            }
        }
        if response.dragged() {
            self.merged_pan += response.drag_delta();
        }

        let image_rect = Rect::from_min_size(rect.center() - display_size / 2.0 + self.merged_pan, display_size);
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_gray(20));
        painter.image(texture.id(), image_rect, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
    }

    fn merge_controls_panel(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.heading("Merge Adjustments");
            ui.add_space(4.0);

            // Section 1: Layer stack
            ui.label(egui::RichText::new("Layers in merge:").strong());
            let num_tabs = self.tabs.len();
            let mut layer_changed = false;

            egui::ScrollArea::vertical().max_height(180.0).show(ui, |ui| {
                for i in 0..num_tabs {
                    let is_calibrated = self.tabs[i].result.is_some();
                    let is_selected = self.selected_layer_idx == i;

                    ui.horizontal(|ui| {
                        if ui.checkbox(&mut self.tabs[i].adjustment.enabled, "").on_hover_text("Enable/disable this layer in the merge").changed() {
                            layer_changed = true;
                        }

                        let label_text = format!("{}. {}", i + 1, self.tabs[i].name);
                        if ui.selectable_label(is_selected, &label_text).clicked() {
                            self.selected_layer_idx = i;
                        }

                        if !is_calibrated {
                            ui.label(egui::RichText::new("(uncalibrated)").color(Color32::GRAY));
                        }
                    });
                }
            });

            if layer_changed {
                self.merged_dirty = true;
            }

            ui.add_space(8.0);
            ui.separator();
            ui.add_space(4.0);

            // Section 2: Fine-tuning sliders for selected layer
            if self.selected_layer_idx < self.tabs.len() {
                let sel = self.selected_layer_idx;
                let tab_name = self.tabs[sel].name.clone();
                ui.label(egui::RichText::new(format!("Adjust Layer {}: {}", sel + 1, tab_name)).strong().color(Color32::from_rgb(180, 220, 255)));

                let adj = &mut self.tabs[sel].adjustment;
                let mut adj_changed = false;

                ui.horizontal(|ui| {
                    ui.label("Offset X (m):");
                    if ui.add(DragValue::new(&mut adj.offset_x).speed(0.02).clamp_range(-50.0..=50.0)).changed() {
                        adj_changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Offset Y (m):");
                    if ui.add(DragValue::new(&mut adj.offset_y).speed(0.02).clamp_range(-50.0..=50.0)).changed() {
                        adj_changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Rotation (deg):");
                    if ui.add(DragValue::new(&mut adj.rotation_deg).speed(0.1).clamp_range(-45.0..=45.0)).changed() {
                        adj_changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Scale:");
                    if ui.add(DragValue::new(&mut adj.scale).speed(0.002).clamp_range(0.5..=2.0)).changed() {
                        adj_changed = true;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("Opacity:");
                    if ui.add(Slider::new(&mut adj.opacity, 0.0..=1.0)).changed() {
                        adj_changed = true;
                    }
                });

                if ui.button("Reset Alignment").on_hover_text("Reset translation, rotation, and scale for this layer").clicked() {
                    adj.offset_x = 0.0;
                    adj.offset_y = 0.0;
                    adj.rotation_deg = 0.0;
                    adj.scale = 1.0;
                    adj.opacity = 1.0;
                    adj_changed = true;
                }

                if adj_changed {
                    self.merged_dirty = true;
                }
            }

            ui.add_space(8.0);
            ui.separator();
            ui.add_space(4.0);

            // Section 3: Merge Export
            ui.label(egui::RichText::new("Export:").strong());
            let can_export = self.tabs.iter().any(|t| t.result.is_some() && t.adjustment.enabled);
            ui.add_enabled_ui(can_export, |ui| {
                if ui.button("Export Merged Map...").clicked() {
                    let dialog = rfd::FileDialog::new()
                        .set_title("Export Merged Map")
                        .add_filter("PNG Image (*.png)", &["png"])
                        .set_file_name("homokuzel_merged.png");
                    if let Some(path) = dialog.save_file() {
                        match self.export_merged_map(&path) {
                            Ok(()) => {
                                let name = path
                                    .file_name()
                                    .map(|n| n.to_string_lossy().to_string())
                                    .unwrap_or_else(|| path.display().to_string());
                                self.status = format!("Exported merged map to {name}");
                            }
                            Err(e) => self.status = format!("Export failed: {e}"),
                        }
                    }
                }
            });
        });
    }
}

impl eframe::App for BirdseyeApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Handle drag-and-drop files
        let dropped_files = ctx.input(|i| i.raw.dropped_files.clone());
        for file in dropped_files {
            if let Some(path) = file.path {
                let ext = path
                    .extension()
                    .and_then(|s| s.to_str())
                    .unwrap_or("")
                    .to_lowercase();
                if ext == "json" {
                    if let Err(e) = self.load_project(ctx, &path) {
                        self.status = format!("Failed to load dropped project {}: {e}", path.display());
                    }
                } else if ["png", "jpg", "jpeg", "bmp", "webp", "tiff"].contains(&ext.as_str()) {
                    match self.active_tab {
                        ActiveTab::Image(curr) => {
                            if self.tabs[curr].src_image.is_none() {
                                if let Err(e) = self.tabs[curr].load_image(ctx, &path) {
                                    self.status = format!("Failed to load dropped image {}: {e}", path.display());
                                } else {
                                    let name = self.tabs[curr].name.clone();
                                    self.status = format!("Loaded {name}. Click the photo to add reference points.");
                                    self.merged_dirty = true;
                                }
                            } else {
                                self.add_image_tab(ctx, Some(&path));
                            }
                        }
                        ActiveTab::Merge => {
                            self.add_image_tab(ctx, Some(&path));
                        }
                    }
                }
            }
        }

        self.top_panel(ctx);

        // Check if active tab is valid
        if let ActiveTab::Image(idx) = self.active_tab {
            if idx >= self.tabs.len() {
                self.active_tab = ActiveTab::Image(0);
            }
        }

        match self.active_tab {
            ActiveTab::Image(idx) => {
                egui::SidePanel::left("image_side_panel")
                    .resizable(true)
                    .default_width(ctx.screen_rect().width() * 0.55)
                    .show(ctx, |ui| {
                        self.image_panel(idx, ui);
                    });

                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.vertical(|ui| {
                        self.table_panel(idx, ui);
                        ui.separator();
                        let remaining = ui.available_size();
                        ui.allocate_ui(remaining, |ui| {
                            self.birdseye_panel(idx, ui);
                        });
                    });
                });

                self.pending_point_window(idx, ctx);
            }
            ActiveTab::Merge => {
                egui::SidePanel::right("merge_controls_panel")
                    .resizable(true)
                    .default_width(340.0)
                    .min_width(280.0)
                    .show(ctx, |ui| {
                        self.merge_controls_panel(ui);
                    });

                egui::CentralPanel::default().show(ctx, |ui| {
                    self.merge_viewport(ui);
                });
            }
        }

        // Recompute dirty image tabs
        let ppm = self.pixels_per_meter;
        let margin = self.margin_m;
        for i in 0..self.tabs.len() {
            if self.tabs[i].dirty {
                if let Some(msg) = self.tabs[i].recompute(ctx, ppm, margin) {
                    if self.active_tab == ActiveTab::Image(i) {
                        self.status = msg;
                    }
                }
                self.merged_dirty = true;
            }
        }

        if self.merged_dirty {
            self.recompute_merged(ctx);
        }
    }
}
