//! 战雷空地武器与第三人称相机拆包词库加载模块

use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};
use serde::Deserialize;

const EMBEDDED_CAS_DB: &str = include_str!("../data/cas_db.json");

fn default_up_ail_left() -> [f64; 3] {
    [0.0, 2.0, 0.7]
}

fn default_up_ail_right() -> [f64; 3] {
    [0.0, 2.0, -0.7]
}

fn default_sens_ail() -> f64 {
    0.3
}

fn default_true() -> bool {
    true
}

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
    #[serde(default = "default_up_ail_left")]
    pub camera_up_ailerons_left: [f64; 3],
    #[serde(default = "default_up_ail_right")]
    pub camera_up_ailerons_right: [f64; 3],
    #[serde(default = "default_sens_ail")]
    pub sensitivity_ailerons: f64,
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
            camera_up_ailerons_left: [0.0, 2.0, 0.7],
            camera_up_ailerons_right: [0.0, 2.0, -0.7],
            sensitivity_ailerons: 0.3,
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
    #[serde(default = "default_true")]
    pub cx_mach: bool,
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
    pub tps_target_offset_at_zoom: [f64; 3],
    #[serde(default)]
    pub static_aim_pitch_deg: f64,
    #[serde(default)]
    pub has_auto_bay_door: bool,
    pub ccip_bombs: bool,
    pub ccip_rockets: bool,
    pub ccrp_bombs: bool,
    pub bombs: Vec<MountedBomb>,
    pub rockets: Vec<MountedRocket>,
}

#[inline]
fn normalize_search_token(s: &str) -> String {
    s.chars()
        .filter(|c| !matches!(c, '-' | '_' | ' ' | '·' | '/' | '[' | ']'))
        .flat_map(char::to_lowercase)
        .collect()
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

    /// 支持去连字符/下划线模糊检索（例如输入 `a10`、`su25`、`f4e`、`ka50`、`雷电` 均可命中）
    pub fn search_aircraft(&self, query: &str) -> Vec<usize> {
        let q_raw = query.trim().to_lowercase();
        if q_raw.is_empty() {
            return (0..self.aircraft.len()).collect();
        }
        let q_norm = normalize_search_token(&q_raw);
        self.aircraft
            .iter()
            .enumerate()
            .filter(|(_, a)| {
                a.id.contains(&q_raw)
                    || a.name_zh.to_lowercase().contains(&q_raw)
                    || a.name_en.to_lowercase().contains(&q_raw)
                    || (!q_norm.is_empty()
                        && (normalize_search_token(&a.id).contains(&q_norm)
                            || normalize_search_token(&a.name_zh).contains(&q_norm)
                            || normalize_search_token(&a.name_en).contains(&q_norm)))
            })
            .map(|(i, _)| i)
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cas_db_heli_classification_and_fuzzy_search() {
        let db = CasDatabase::load_embedded_or_disk(None).unwrap();
        let ah64 = db.find_aircraft("ah_64d").expect("应包含 AH-64D");
        assert!(ah64.is_heli, "AH-64D 必须正确识别为武装直升机");
        assert_eq!(ah64.tps_target_offset_at_zoom, [50.0, 20.0, 0.0]);

        let a10 = db.find_aircraft("a_10a_early").expect("应包含 A-10A");
        assert!(!a10.is_heli);

        // 无连字符模糊搜索 `a10`、`f4e`、`ka50` 必须命中
        let hits_a10 = db.search_aircraft("a10");
        assert!(hits_a10.iter().any(|&i| db.aircraft[i].id == "a_10a_early"));
        let hits_ka50 = db.search_aircraft("ka50");
        assert!(hits_ka50.iter().any(|&i| db.aircraft[i].id == "ka_50"));
    }
}
