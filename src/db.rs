//! 战雷空地武器与第三人称相机拆包词库加载模块

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

const EMBEDDED_CAS_DB: &str = include_str!("../data/cas_db.json");

#[derive(Clone, Debug, Deserialize)]
pub struct TpsCameraConfig {
    pub fov: f64,
    pub zoom_fov: f64,
    pub target_offset: [f64; 3],
    pub camera_offset_neutral: [f64; 3],
    pub camera_up_neutral: [f64; 3],
    pub camera_offset_elev_up: [f64; 3],
    pub camera_offset_elev_down: [f64; 3],
    pub camera_offset_rudder_left: [f64; 3],
    pub camera_offset_rudder_right: [f64; 3],
    pub roll_angle_factor: f64,
    pub overturn_angle: f64,
    pub fov_smoothing: f64,
}

impl Default for TpsCameraConfig {
    fn default() -> Self {
        Self {
            fov: 90.0,
            zoom_fov: 30.0,
            target_offset: [500.0, 0.0, 0.0],
            camera_offset_neutral: [-17.0, 4.0, 0.0],
            camera_up_neutral: [0.0, 2.0, 0.0],
            camera_offset_elev_up: [-14.0, 4.0, 0.0],
            camera_offset_elev_down: [-14.0, 4.0, 0.0],
            camera_offset_rudder_left: [-14.0, 4.0, -1.0],
            camera_offset_rudder_right: [-14.0, 4.0, 1.0],
            roll_angle_factor: 0.01,
            overturn_angle: 80.0,
            fov_smoothing: 0.005,
        }
    }
}

#[derive(Clone, Debug, Deserialize)]
pub struct CameraConfigs {
    pub plane_tps: TpsCameraConfig,
    pub heli_tps_alt: TpsCameraConfig,
}

#[derive(Clone, Debug, Deserialize)]
pub struct BombSpec {
    pub id: String,
    pub name_zh: String,
    pub name_en: String,
    pub mass: f64,
    pub caliber: f64,
    pub length: f64,
    pub dist_cm_to_stab: f64,
    pub drag_cx: f64,
    pub cx_k: f64,
    pub brake_time: Option<[f64; 2]>,
    pub brake_cx_k: f64,
    pub explosive_mass: f64,
    pub is_napalm: bool,
}

#[derive(Clone, Debug, Deserialize)]
pub struct RocketSpec {
    pub id: String,
    pub name_zh: String,
    pub name_en: String,
    pub mass: f64,
    pub mass_end: f64,
    pub mass_end1: f64,
    pub caliber: f64,
    pub length: f64,
    pub cx_k: f64,
    pub pod_drag_cx: f64,
    pub force0: f64,
    pub time_fire0: f64,
    pub force1: f64,
    pub time_fire1: f64,
    pub fire_delay: f64,
    pub start_speed: f64,
    pub use_start_speed: bool,
    pub end_speed: f64,
    pub max_speed_stat: f64,
    pub max_distance: f64,
    pub time_life: f64,
    pub dist_cm_to_stab: f64,
    pub wd_k: f64,
    pub max_delta_angle: f64,
    pub explosive_mass: f64,
}

#[derive(Clone, Debug, Deserialize)]
pub struct MountedBomb {
    pub id: String,
    pub external: bool,
    #[serde(default)]
    pub has_internal_bay: bool,
}

#[derive(Clone, Debug, Deserialize)]
pub struct MountedRocket {
    pub id: String,
    #[serde(default)]
    pub launcher_pitch_deg: f64,
}

#[derive(Clone, Debug, Deserialize)]
pub struct AircraftSpec {
    pub id: String,
    pub name_zh: String,
    pub name_en: String,
    pub is_heli: bool,
    pub tps_offset: [f64; 3],
    #[serde(default)]
    pub static_aim_pitch_deg: f64,
    pub ccip_bombs: bool,
    pub ccip_rockets: bool,
    pub ccrp_bombs: bool,
    pub bombs: Vec<MountedBomb>,
    pub rockets: Vec<MountedRocket>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct CasDatabase {
    pub datamine_version: String,
    pub camera: CameraConfigs,
    pub bombs: HashMap<String, BombSpec>,
    pub rockets: HashMap<String, RocketSpec>,
    pub aircraft: Vec<AircraftSpec>,
    #[serde(skip)]
    aircraft_index: HashMap<String, usize>,
}

impl CasDatabase {
    pub fn load_embedded_or_disk(custom_path: Option<&Path>) -> Result<Self> {
        let mut db: Self = if let Some(p) = custom_path.filter(|p| p.exists()) {
            let txt = std::fs::read_to_string(p)
                .with_context(|| format!("读取外部 CAS 词库失败: {}", p.display()))?;
            serde_json::from_str(&txt)?
        } else {
            serde_json::from_str(EMBEDDED_CAS_DB).context("解析内置 cas_db.json 失败")?
        };
        db.aircraft_index = db
            .aircraft
            .iter()
            .enumerate()
            .map(|(i, a)| (a.id.to_lowercase(), i))
            .collect();
        Ok(db)
    }

    pub fn find_aircraft(&self, unit_id: &str) -> Option<&AircraftSpec> {
        let key = unit_id.trim().to_lowercase();
        self.aircraft_index.get(&key).map(|&idx| &self.aircraft[idx])
    }

    pub fn search_aircraft(&self, query: &str) -> Vec<usize> {
        let q = query.trim().to_lowercase();
        if q.is_empty() {
            return (0..self.aircraft.len()).collect();
        }
        self.aircraft
            .iter()
            .enumerate()
            .filter(|(_, a)| {
                a.id.contains(&q)
                    || a.name_zh.to_lowercase().contains(&q)
                    || a.name_en.to_lowercase().contains(&q)
            })
            .map(|(i, _)| i)
            .collect()
    }
}
