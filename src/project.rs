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
pub struct Project {
    pub image_path: PathBuf,
    pub points: Vec<PointRecord>,
    pub pixels_per_meter: f64,
    pub margin_m: f64,
}

impl Project {
    pub fn save(&self, path: &Path) -> anyhow::Result<()> {
        let f = std::fs::File::create(path)?;
        serde_json::to_writer_pretty(f, self)?;
        Ok(())
    }

    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let f = std::fs::File::open(path)?;
        let project: Project = serde_json::from_reader(f)?;
        Ok(project)
    }
}
