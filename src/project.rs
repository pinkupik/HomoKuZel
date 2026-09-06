//! Project file: the source image path plus every correspondence point, so a
//! calibration session can be saved and reopened (or fed to the headless CLI
//! export in `main.rs`).

use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointRecord {
    pub label: String,
    pub pixel_u: f64,
    pub pixel_v: f64,
    pub world_x: f64,
    pub world_y: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerAdjustment {
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub offset_x: f64,
    #[serde(default)]
    pub offset_y: f64,
    #[serde(default)]
    pub rotation_deg: f64,
    #[serde(default = "default_scale")]
    pub scale: f64,
    #[serde(default = "default_opacity")]
    pub opacity: f32,
}

fn default_true() -> bool {
    true
}
fn default_scale() -> f64 {
    1.0
}
fn default_opacity() -> f32 {
    1.0
}

impl Default for LayerAdjustment {
    fn default() -> Self {
        Self {
            enabled: true,
            offset_x: 0.0,
            offset_y: 0.0,
            rotation_deg: 0.0,
            scale: 1.0,
            opacity: 1.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImageTabRecord {
    pub name: String,
    #[serde(default)]
    pub image_path: Option<PathBuf>,
    #[serde(default)]
    pub points: Vec<PointRecord>,
    #[serde(default)]
    pub adjustment: LayerAdjustment,
}

fn default_ppm() -> f64 {
    25.0
}
fn default_margin() -> f64 {
    4.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Project {
    #[serde(default)]
    pub tabs: Vec<ImageTabRecord>,
    #[serde(default = "default_ppm")]
    pub pixels_per_meter: f64,
    #[serde(default = "default_margin")]
    pub margin_m: f64,

    // Legacy single-image fields (for backward compatibility)
    #[serde(default)]
    pub image_path: Option<PathBuf>,
    #[serde(default)]
    pub points: Option<Vec<PointRecord>>,
}

impl Project {
    pub fn save(&self, path: &Path) -> anyhow::Result<()> {
        let f = std::fs::File::create(path)?;
        serde_json::to_writer_pretty(f, self)?;
        Ok(())
    }

    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let f = std::fs::File::open(path)?;
        let mut project: Project = serde_json::from_reader(f)?;
        if project.tabs.is_empty() {
            if let Some(img_path) = project.image_path.take() {
                let points = project.points.take().unwrap_or_default();
                let name = img_path
                    .file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| "Image 1".to_string());
                project.tabs.push(ImageTabRecord {
                    name,
                    image_path: Some(img_path),
                    points,
                    adjustment: LayerAdjustment::default(),
                });
            }
        }
        Ok(project)
    }
}

