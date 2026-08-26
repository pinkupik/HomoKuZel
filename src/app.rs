use eframe::egui;
use egui::{Color32, ColorImage, DragValue, Pos2, Rect, Sense, TextureHandle, TextureOptions, Vec2};
use image::RgbaImage;
use std::path::{Path, PathBuf};

use crate::homography::{self, Correspondence, HomographyResult};
use crate::project::{PointRecord, Project};
use crate::warp::{self, BirdseyeParams};

const HANDLE_RADIUS: f32 = 6.0;
const LOGO_BYTES: &[u8] = include_bytes!("../assets/app_logo.jpg");

pub struct BirdseyeApp {
    // logo texture
    logo_texture: Option<TextureHandle>,

    // source image
    image_path: Option<PathBuf>,
    src_image: Option<RgbaImage>,
    src_texture: Option<TextureHandle>,

    // correspondences
    points: Vec<PointRecord>,
    next_label: usize,
    pending_pixel: Option<(f64, f64)>, // clicked, awaiting world XY entry
    pending_x_str: String,
    pending_y_str: String,
    focus_pending_input: bool,
    dragging: Option<usize>,

    // view state (left image panel)
    zoom: f32,
    pan: Vec2,

    // view state (birdseye preview panel)
    birdseye_zoom: f32,
    birdseye_pan: Vec2,

    // calibration config
    pixels_per_meter: f64,
    margin_m: f64,
    export_with_grid: bool,

    // computed results
    result: Option<HomographyResult>,
    birdseye_texture: Option<TextureHandle>,
    effective_ppm: f64,

    // project & session state
    project_path: Option<PathBuf>,
    status: String,

    dirty: bool,
}

impl Default for BirdseyeApp {
    fn default() -> Self {
        Self {
            logo_texture: None,
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
            pixels_per_meter: 25.0,
            margin_m: 4.0,
            export_with_grid: true,
            result: None,
            birdseye_texture: None,
            effective_ppm: 25.0,
            project_path: None,
            status: "Load a drone photo or project to begin.".to_string(),
            dirty: false,
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
            } else if let Err(e) = app.load_image(&cc.egui_ctx, &path) {
                app.status = format!("Failed to load {}: {e}", path.display());
            }
        }
        app
    }

    fn load_image(&mut self, ctx: &egui::Context, path: &Path) -> anyhow::Result<()> {
        let img = image::open(path)?.into_rgba8();
        let (w, h) = img.dimensions();
        let color_image = ColorImage::from_rgba_unmultiplied([w as usize, h as usize], img.as_raw());
        let texture = ctx.load_texture("source_image", color_image, TextureOptions::LINEAR);

        self.src_texture = Some(texture);
        self.src_image = Some(img);
        self.image_path = Some(path.to_path_buf());
        self.points.clear();
        self.next_label = 1;
        self.result = None;
        self.birdseye_texture = None;
        self.zoom = 1.0;
        self.pan = Vec2::ZERO;
        self.birdseye_zoom = 1.0;
        self.birdseye_pan = Vec2::ZERO;
        let filename = path
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| path.display().to_string());
        self.status = format!("Loaded {filename} ({w}x{h}). Click the photo to add reference points.");
        Ok(())
    }

    fn recompute(&mut self, ctx: &egui::Context) {
        self.dirty = false;
        let Some(src) = &self.src_image else { return };

        if self.points.len() < 4 {
            self.result = None;
            self.birdseye_texture = None;
            self.status = format!("Need at least 4 points ({} so far).", self.points.len());
            return;
        }

        let correspondences: Vec<Correspondence> = self
            .points
            .iter()
            .map(|p| Correspondence { pixel: (p.pixel_u, p.pixel_v), world: (p.world_x, p.world_y) })
            .collect();

        let result = match homography::solve_homography(&correspondences) {
            Ok(r) => r,
            Err(e) => {
                self.status = format!("Homography failed: {e}");
                self.result = None;
                self.birdseye_texture = None;
                return;
            }
        };

        let Some(h_world_to_img) = result.h_world_to_img else {
            self.status = "Homography is singular (points may be collinear or duplicated).".to_string();
            self.result = Some(result);
            self.birdseye_texture = None;
            return;
        };

        let (w, h) = src.dimensions();
        let extent = warp::compute_extent(w, h, &result.h_img_to_world);
        let params = BirdseyeParams {
            pixels_per_meter: self.pixels_per_meter,
            margin_m: self.margin_m,
            max_canvas_dim: 3000,
        };
        let output = warp::warp_to_birdseye(src, &h_world_to_img, &extent, &params);

        let (cw, ch) = output.image.dimensions();
        let color_image = ColorImage::from_rgba_unmultiplied([cw as usize, ch as usize], output.image.as_raw());
        let texture = ctx.load_texture("birdseye", color_image, TextureOptions::LINEAR);

        self.status = format!(
            "RMS error: {:.1} cm  (max {:.1} cm)  |  {:.2} px/m  |  canvas {cw}x{ch}",
            result.rms_error * 100.0,
            result.max_error * 100.0,
            output.effective_ppm,
        );
        self.effective_ppm = output.effective_ppm;
        self.birdseye_texture = Some(texture);
        self.result = Some(result);
    }

    fn add_point(&mut self, pixel: (f64, f64), world: (f64, f64)) {
        let label = format!("P{}", self.next_label);
        self.next_label += 1;
        self.points.push(PointRecord { label, pixel_u: pixel.0, pixel_v: pixel.1, world_x: world.0, world_y: world.1 });
        self.dirty = true;
    }

    fn export_birdseye(&self, path: &Path) -> anyhow::Result<()> {
        let Some(result) = &self.result else { anyhow::bail!("no homography computed yet (need >= 4 points)") };
        let Some(h_world_to_img) = result.h_world_to_img else { anyhow::bail!("homography is singular") };
        let Some(src) = &self.src_image else { anyhow::bail!("no source image loaded") };
        let (w, h) = src.dimensions();
        let extent = warp::compute_extent(w, h, &result.h_img_to_world);
        let params = BirdseyeParams { pixels_per_meter: self.pixels_per_meter, margin_m: self.margin_m, max_canvas_dim: 8000 };
        let mut output = warp::warp_to_birdseye(src, &h_world_to_img, &extent, &params);
        if self.export_with_grid {
            warp::draw_grid_overlay(&mut output.image, &output.extent, output.effective_ppm);
        }
        output.image.save(path)?;
        Ok(())
    }

    fn save_project(&self, path: &Path) -> anyhow::Result<()> {
        let Some(image_path) = &self.image_path else { anyhow::bail!("no image loaded") };
        let project = Project {
            image_path: image_path.clone(),
            points: self.points.clone(),
            pixels_per_meter: self.pixels_per_meter,
            margin_m: self.margin_m,
        };
        project.save(path)
    }

    fn load_project(&mut self, ctx: &egui::Context, path: &Path) -> anyhow::Result<()> {
        let project = Project::load(path)?;
        let resolved_img_path = if project.image_path.is_absolute() || project.image_path.exists() {
            project.image_path.clone()
        } else if let Some(parent) = path.parent() {
            let candidate = parent.join(&project.image_path);
            if candidate.exists() {
                candidate
            } else {
                project.image_path.clone()
            }
        } else {
            project.image_path.clone()
        };

        self.load_image(ctx, &resolved_img_path)?;
        self.points = project.points;
        self.next_label = self.points.len() + 1;
        self.pixels_per_meter = project.pixels_per_meter;
        self.margin_m = project.margin_m;
        self.project_path = Some(path.to_path_buf());
        self.dirty = true;
        let proj_name = path
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| path.display().to_string());
        self.status = format!("Loaded project {proj_name} with {} reference points.", self.points.len());
        Ok(())
    }

    fn top_panel(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.add_space(2.0);
            ui.horizontal_wrapped(|ui| {
                ui.label(egui::RichText::new("🔷 HomoKuŽel").strong().color(Color32::from_rgb(80, 160, 255)));
                ui.separator();

                // Open Image button
                if ui.button("📂 Open Image...").on_hover_text("Open drone photo (PNG, JPG, etc.)").clicked() {
                    let mut dialog = rfd::FileDialog::new()
                        .set_title("Open Drone Photo")
                        .add_filter("Image Files", &["png", "jpg", "jpeg", "bmp", "webp", "tiff"]);
                    if let Some(ref p) = self.image_path {
                        if let Some(parent) = p.parent() {
                            dialog = dialog.set_directory(parent);
                        }
                    }
                    if let Some(path) = dialog.pick_file() {
                        if let Err(e) = self.load_image(ctx, &path) {
                            self.status = format!("Failed to load {}: {e}", path.display());
                        }
                    }
                }

                // Shortened image label with tooltip
                if let Some(ref path) = self.image_path {
                    let name = path
                        .file_name()
                        .map(|n| n.to_string_lossy().to_string())
                        .unwrap_or_else(|| path.display().to_string());
                    let label = if name.len() > 22 {
                        format!("{}...", &name[..19])
                    } else {
                        name.clone()
                    };
                    ui.label(egui::RichText::new(format!("🖼 {label}")).strong())
                        .on_hover_text(format!("Image: {}\nFull path: {}", name, path.display()));
                } else {
                    ui.label(egui::RichText::new("No image loaded").italics().color(Color32::GRAY));
                }

                ui.separator();

                // Save / Load Project buttons
                let save_enabled = self.image_path.is_some();
                ui.add_enabled_ui(save_enabled, |ui| {
                    if ui.button("💾 Save Project...").on_hover_text("Save points and calibration settings to JSON").clicked() {
                        let mut dialog = rfd::FileDialog::new()
                            .set_title("Save Calibration Project")
                            .add_filter("Birdseye Project (*.json)", &["json"])
                            .set_file_name(
                                self.project_path
                                    .as_ref()
                                    .and_then(|p| p.file_name())
                                    .map(|n| n.to_string_lossy().to_string())
                                    .unwrap_or_else(|| "project.json".to_string()),
                            );
                        if let Some(ref p) = self.project_path.as_ref().or(self.image_path.as_ref()) {
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
                });

                if ui.button("📁 Load Project...").on_hover_text("Load a saved calibration project (.json)").clicked() {
                    let mut dialog = rfd::FileDialog::new()
                        .set_title("Load Calibration Project")
                        .add_filter("Birdseye Project (*.json)", &["json"]);
                    if let Some(ref p) = self.project_path.as_ref().or(self.image_path.as_ref()) {
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

                // Settings: px/m and margin
                ui.label("px/m:");
                if ui.add(DragValue::new(&mut self.pixels_per_meter).clamp_range(1.0..=500.0).speed(1.0)).on_hover_text("Birdseye resolution (pixels per meter)").changed() {
                    self.dirty = true;
                }
                ui.label("margin (m):");
                if ui.add(DragValue::new(&mut self.margin_m).clamp_range(0.0..=50.0).speed(0.1)).on_hover_text("Border padding around track in meters").changed() {
                    self.dirty = true;
                }

                ui.separator();

                // Export button & grid checkbox
                let export_enabled = self.result.is_some() && self.src_image.is_some();
                ui.add_enabled_ui(export_enabled, |ui| {
                    if ui.button("🚀 Export Birdseye...").on_hover_text("Export full-resolution rectified birdseye PNG").clicked() {
                        let mut dialog = rfd::FileDialog::new()
                            .set_title("Export Birdseye Image")
                            .add_filter("PNG Image (*.png)", &["png"])
                            .set_file_name("birdseye_output.png");
                        if let Some(ref p) = self.image_path {
                            if let Some(parent) = p.parent() {
                                dialog = dialog.set_directory(parent);
                            }
                        }
                        if let Some(path) = dialog.save_file() {
                            match self.export_birdseye(&path) {
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
                ui.checkbox(&mut self.export_with_grid, "1m grid").on_hover_text("Bake 1m grid and origin marker into exported PNG");
            });

            ui.add_space(2.0);
            ui.horizontal(|ui| {
                let status_color = if self.status.starts_with("Failed")
                    || self.status.starts_with("Homography failed")
                    || self.status.starts_with("Export failed")
                    || self.status.starts_with("Save failed")
                {
                    Color32::from_rgb(230, 80, 80)
                } else if self.status.starts_with("RMS") {
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
    }

    fn image_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Left click: add point.  Drag a marker: reposition it.  Scroll: zoom.");
            if ui.button("Fit").clicked() {
                self.zoom = 1.0;
                self.pan = Vec2::ZERO;
            }
        });

        let Some(texture) = self.src_texture.clone() else {
            ui.centered_and_justified(|ui| {
                ui.vertical_centered(|ui| {
                    ui.add_space(20.0);
                    if let Some(ref logo) = self.logo_texture {
                        ui.image((logo.id(), egui::vec2(180.0, 180.0)));
                    }
                    ui.add_space(8.0);
                    ui.heading(egui::RichText::new("🔷 HomoKuŽel").size(24.0).strong().color(Color32::from_rgb(80, 160, 255)));
                    ui.label(egui::RichText::new("Formula Student Drone-to-Birdseye Track Map Rectifier").italics().color(Color32::from_gray(180)));
                    ui.add_space(16.0);

                    ui.horizontal(|ui| {
                        if ui.button(egui::RichText::new("📂 Open Drone Photo").size(15.0)).clicked() {
                            let dialog = rfd::FileDialog::new()
                                .set_title("Open Drone Photo")
                                .add_filter("Image Files", &["png", "jpg", "jpeg", "bmp", "webp", "tiff"]);
                            if let Some(path) = dialog.pick_file() {
                                if let Err(e) = self.load_image(ui.ctx(), &path) {
                                    self.status = format!("Failed to load {}: {e}", path.display());
                                }
                            }
                        }
                        if ui.button(egui::RichText::new("📁 Load Project").size(15.0)).clicked() {
                            let dialog = rfd::FileDialog::new()
                                .set_title("Load Calibration Project")
                                .add_filter("Birdseye Project (*.json)", &["json"]);
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
                        ui.label(egui::RichText::new("🏁 Quick Start:").strong().color(Color32::from_rgb(255, 200, 80)));
                        ui.label("1. Drag & drop a drone photo here (or click Open).");
                        ui.label("2. Left-click 4+ known points on track (cones, markings).");
                        ui.label("3. Type real-world coords in metres (Tab to move, Enter to add).");
                        ui.label("4. Check live error & Export rectified PNG for map_editor!");
                    });
                });
            });
            return;
        };

        let avail = ui.available_size();
        let img_size = texture.size_vec2();

        // Fit-to-width baseline zoom, multiplied by user zoom factor.
        let base_scale = (avail.x / img_size.x).min(2.0).max(0.05);
        let scale = base_scale * self.zoom;
        let display_size = img_size * scale;

        let (rect, response) =
            ui.allocate_exact_size(avail, Sense::click_and_drag());

        // scroll to zoom
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.0 {
                self.zoom = (self.zoom * (1.0 + scroll * 0.001)).clamp(0.1, 20.0);
            }
        }

        let origin = rect.min + self.pan;
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

        // draw existing points
        for (i, p) in self.points.iter().enumerate() {
            let center = to_screen((p.pixel_u, p.pixel_v));
            let color = match &self.result {
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

        // handle drag start: pick nearest point within handle radius
        if response.drag_started() {
            if let Some(pos) = response.interact_pointer_pos() {
                let mut best: Option<(usize, f32)> = None;
                for (i, p) in self.points.iter().enumerate() {
                    let d = to_screen((p.pixel_u, p.pixel_v)).distance(pos);
                    if d <= HANDLE_RADIUS * 2.0 && best.map_or(true, |(_, bd)| d < bd) {
                        best = Some((i, d));
                    }
                }
                self.dragging = best.map(|(i, _)| i);
            }
        }

        if response.dragged() {
            if let Some(i) = self.dragging {
                if let Some(pos) = response.interact_pointer_pos() {
                    let (u, v) = to_pixel(pos);
                    self.points[i].pixel_u = u;
                    self.points[i].pixel_v = v;
                    self.dirty = true;
                }
            } else {
                // panning
                self.pan += response.drag_delta();
            }
        }

        if response.drag_stopped() {
            self.dragging = None;
        }

        // click on empty space (drag distance ~0, and not starting on a handle) -> new point
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let near_existing = self.points.iter().any(|p| to_screen((p.pixel_u, p.pixel_v)).distance(pos) <= HANDLE_RADIUS * 2.0);
                if !near_existing {
                    self.pending_pixel = Some(to_pixel(pos));
                    self.pending_x_str.clear();
                    self.pending_y_str.clear();
                    self.focus_pending_input = true;
                }
            }
        }
    }

    fn pending_point_window(&mut self, ctx: &egui::Context) {
        let Some(pixel) = self.pending_pixel else { return };
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
                        egui::TextEdit::singleline(&mut self.pending_x_str)
                            .hint_text("0.0")
                            .desired_width(100.0),
                    );
                    if self.focus_pending_input {
                        x_edit.request_focus();
                        self.focus_pending_input = false;
                    }
                });

                ui.horizontal(|ui| {
                    ui.label("World Y (m):");
                    ui.add(
                        egui::TextEdit::singleline(&mut self.pending_y_str)
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
            let x = self.pending_x_str.trim().replace(',', ".").parse::<f64>().unwrap_or(0.0);
            let y = self.pending_y_str.trim().replace(',', ".").parse::<f64>().unwrap_or(0.0);
            self.add_point(pixel, (x, y));
            self.pending_pixel = None;
        } else if cancelled || !open {
            self.pending_pixel = None;
        }
    }

    fn table_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label(format!("{} reference points", self.points.len()));
            if !self.points.is_empty() && ui.button("Clear all").on_hover_text("Delete all reference points").clicked() {
                self.points.clear();
                self.next_label = 1;
                self.dirty = true;
            }
        });
        egui::ScrollArea::vertical().max_height(220.0).show(ui, |ui| {
            egui::Grid::new("points_table").striped(true).num_columns(6).show(ui, |ui| {
                ui.strong("Label");
                ui.strong("Pixel u,v");
                ui.strong("World X");
                ui.strong("World Y");
                ui.strong("Error");
                ui.strong("");
                ui.end_row();

                let mut to_delete: Option<usize> = None;
                for (i, p) in self.points.iter_mut().enumerate() {
                    ui.add(egui::TextEdit::singleline(&mut p.label).desired_width(40.0));
                    ui.label(format!("{:.1}, {:.1}", p.pixel_u, p.pixel_v));
                    if ui.add(DragValue::new(&mut p.world_x).speed(0.02)).changed() {
                        self.dirty = true;
                    }
                    if ui.add(DragValue::new(&mut p.world_y).speed(0.02)).changed() {
                        self.dirty = true;
                    }
                    match &self.result {
                        Some(r) => match r.reproj_errors.get(i) {
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
                    if ui.button("🗑").clicked() {
                        to_delete = Some(i);
                    }
                    ui.end_row();
                }
                if let Some(i) = to_delete {
                    self.points.remove(i);
                    self.dirty = true;
                }
            });
        });
    }

    fn birdseye_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Live birdseye preview (scroll to zoom, drag to pan)");
            if ui.button("Fit").clicked() {
                self.birdseye_zoom = 1.0;
                self.birdseye_pan = Vec2::ZERO;
            }
        });

        let Some(texture) = self.birdseye_texture.clone() else {
            ui.centered_and_justified(|ui| ui.label("Add >= 4 points to compute the homography."));
            return;
        };

        let avail = ui.available_size();
        let img_size = texture.size_vec2();

        // Fit-to-panel baseline, multiplied by user zoom factor — same
        // pattern as the source image panel.
        let base_scale = (avail.x / img_size.x).min(avail.y / img_size.y).min(2.0).max(0.01);
        let scale = base_scale * self.birdseye_zoom;
        let display_size = img_size * scale;

        let (rect, response) = ui.allocate_exact_size(avail, Sense::click_and_drag());

        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll.abs() > 0.0 {
                self.birdseye_zoom = (self.birdseye_zoom * (1.0 + scroll * 0.001)).clamp(0.1, 20.0);
            }
        }
        if response.dragged() {
            self.birdseye_pan += response.drag_delta();
        }

        let image_rect = Rect::from_min_size(rect.center() - display_size / 2.0 + self.birdseye_pan, display_size);
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_gray(20));
        painter.image(texture.id(), image_rect, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
    }
}

impl eframe::App for BirdseyeApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Handle drag-and-drop of images or project files directly onto the window
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
                    if let Err(e) = self.load_image(ctx, &path) {
                        self.status = format!("Failed to load dropped image {}: {e}", path.display());
                    }
                }
            }
        }

        self.top_panel(ctx);

        egui::SidePanel::left("image_side_panel")
            .resizable(true)
            .default_width(ctx.screen_rect().width() * 0.55)
            .show(ctx, |ui| {
                self.image_panel(ui);
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.vertical(|ui| {
                self.table_panel(ui);
                ui.separator();
                let remaining = ui.available_size();
                ui.allocate_ui(remaining, |ui| {
                    self.birdseye_panel(ui);
                });
            });
        });

        self.pending_point_window(ctx);

        if self.dirty {
            self.recompute(ctx);
        }
    }
}
