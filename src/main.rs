#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use image::RgbaImage;
use serde::{Deserialize, Serialize};

use wt_cas_ccip::atmosphere::Atmosphere;
use wt_cas_ccip::ballistics::{
    AircraftKinematics, BallisticTuning, BodyBasis, RocketSimOutcome, Vec3, simulate_bomb_ccip,
    simulate_rocket_ccip_detailed,
};
use wt_cas_ccip::camera_tps::{
    OneEuroFilter2D, TpsCameraState, solve_tps_camera_from_nose_pixel_ext, wrap_yaw_deg,
};
use wt_cas_ccip::capture_win::{
    BindPollOutcome, BindSlot, GameViewportInfo, GlobalHotkeys, HotkeyBindings, HotkeyEvents,
    KeyBindRecorder, ScreenCenterCapturer, ZoomTriggerMode, enable_per_monitor_dpi_v2,
    is_window_foreground, precise_frame_sleep, primary_screen_size, query_war_thunder_viewport_ext,
    vk_display_name_zh,
};
use wt_cas_ccip::db::CasDatabase;
use wt_cas_ccip::hud_render::{
    DirtyEnvelope, HudScene, ReticleTarget, render_hud_incremental, render_hud_into_bgra,
};
use wt_cas_ccip::overlay_win::{ControlPanelWindow, LayeredHudWindow, UiEvent, drain_ui_events};
use wt_cas_ccip::telemetry::{
    TelemetryPoller, build_demo_telemetry_and_nose_px, shortest_angle_diff_deg,
};
use wt_cas_ccip::terrain_pack::{TerrainSource, WtvPackReader};
use wt_cas_ccip::vision::{
    CrosshairColorPreset, VisionScratch, detect_crosshair_bgra, detect_crosshair_with_scratch,
};

const CONV_DIST_OPTIONS: [(f64, &str); 8] = [
    (250.0, "250 米"),
    (300.0, "300 米"),
    (400.0, "400 米"),
    (500.0, "500 米"),
    (600.0, "600 米"),
    (700.0, "700 米"),
    (800.0, "800 米"),
    (2000.0, "无交汇(远)"),
];

const DRAG_SCALE_OPTIONS: [f64; 7] = [0.80, 0.90, 0.95, 1.00, 1.05, 1.15, 1.25];
const RKT_TRIM_OPTIONS: [f64; 9] = [-3.0, -2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0, 3.0];

const COLOR_PRESETS: [CrosshairColorPreset; 7] = [
    CrosshairColorPreset::Auto,
    CrosshairColorPreset::Red,
    CrosshairColorPreset::White,
    CrosshairColorPreset::Magenta,
    CrosshairColorPreset::Cyan,
    CrosshairColorPreset::NeonGreen,
    CrosshairColorPreset::Yellow,
];

const FLAT_PRESETS: [(&str, Option<f64>); 5] = [
    ("[自动雷达/跑道海拔 (支持热键低空一键标定)]", None),
    ("[平地基准：海平面 0m (航母/海岛试飞)]", Some(0.0)),
    ("[平地基准：平原试飞场 65m]", Some(65.0)),
    ("[平地基准：内陆丘陵试飞场 150m]", Some(150.0)),
    ("[平地基准：高原机场 300m]", Some(300.0)),
];

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
struct UserPrefs {
    pub color_idx: usize,
    pub conv_idx: usize,
    pub drag_idx: usize,
    pub rkt_trim_idx: usize,
    pub auto_ac: bool,
    pub bay_delay_on: bool,
    pub hotkeys: HotkeyBindings,
}

impl Default for UserPrefs {
    fn default() -> Self {
        Self {
            color_idx: 0,    // Auto (自动识别红/默认白/绿/黄/青/品红)
            conv_idx: 3,     // 500 米
            drag_idx: 3,     // 1.00x
            rkt_trim_idx: 4, // 0.0°
            auto_ac: true,
            bay_delay_on: true,
            hotkeys: HotkeyBindings::default(),
        }
    }
}

impl UserPrefs {
    fn config_path() -> Option<PathBuf> {
        std::env::current_exe()
            .ok()
            .and_then(|p| p.parent().map(|d| d.join("cas_ccip_config.json")))
    }

    fn load() -> Self {
        Self::config_path()
            .and_then(|p| std::fs::read_to_string(p).ok())
            .and_then(|s| serde_json::from_str::<Self>(&s).ok())
            .unwrap_or_default()
    }

    fn save(&self) {
        if let Some(p) = Self::config_path() {
            if let Ok(txt) = serde_json::to_string_pretty(self) {
                let _ = std::fs::write(Path::new(&p), txt);
            }
        }
    }
}

/// 世界空间摄像机惯性保持器：
/// 当机炮火光、黑烟或云层短暂遮挡准星 (`< 480 ms`) 时，在**世界坐标系**下按指数阻尼角速度维持第三人称相机姿态，
/// 既不受机炮后坐力导致的机身震颤污染，也不会在重捕获瞬间产生跳变！
#[derive(Clone, Copy, Debug)]
struct WorldCameraCoaster {
    cam_yaw_deg: f64,
    cam_pitch_deg: f64,
    last_presented_yaw_deg: f64,
    last_presented_pitch_deg: f64,
    yaw_rate_dps: f64,
    pitch_rate_dps: f64,
    last_optical_hit: Instant,
    was_coasting: bool,
}

impl WorldCameraCoaster {
    fn new(yaw_deg: f64, pitch_deg: f64, now: Instant) -> Self {
        Self {
            cam_yaw_deg: yaw_deg,
            cam_pitch_deg: pitch_deg,
            last_presented_yaw_deg: yaw_deg,
            last_presented_pitch_deg: pitch_deg,
            yaw_rate_dps: 0.0,
            pitch_rate_dps: 0.0,
            last_optical_hit: now,
            was_coasting: false,
        }
    }

    fn on_optical_lock(&mut self, solved_yaw: f64, solved_pitch: f64, now: Instant) -> (f64, f64) {
        let dt = now
            .saturating_duration_since(self.last_optical_hit)
            .as_secs_f64();

        // 若刚从短暂遮挡惯性态恢复，从上一帧已呈现姿态 `last_presented` 单调平滑重入，杜绝往回倒退跳变
        let (final_yaw, final_pitch) = if self.was_coasting && dt < 0.55 {
            let dy = shortest_angle_diff_deg(solved_yaw, self.last_presented_yaw_deg);
            let dp = solved_pitch - self.last_presented_pitch_deg;
            (
                wrap_yaw_deg(self.last_presented_yaw_deg + dy * 0.65),
                (self.last_presented_pitch_deg + dp * 0.65).clamp(-88.5, 88.5),
            )
        } else {
            (solved_yaw, solved_pitch)
        };

        if (0.004..=0.14).contains(&dt) && !self.was_coasting {
            let vy = (shortest_angle_diff_deg(final_yaw, self.cam_yaw_deg) / dt).clamp(-160.0, 160.0);
            let vp = ((final_pitch - self.cam_pitch_deg) / dt).clamp(-160.0, 160.0);
            self.yaw_rate_dps = self.yaw_rate_dps * 0.55 + vy * 0.45;
            self.pitch_rate_dps = self.pitch_rate_dps * 0.55 + vp * 0.45;
        } else if self.was_coasting {
            self.yaw_rate_dps *= 0.35;
            self.pitch_rate_dps *= 0.35;
        }

        self.cam_yaw_deg = final_yaw;
        self.cam_pitch_deg = final_pitch;
        self.last_presented_yaw_deg = final_yaw;
        self.last_presented_pitch_deg = final_pitch;
        self.last_optical_hit = now;
        self.was_coasting = false;
        (final_yaw, final_pitch)
    }

    fn coast_at(&mut self, now: Instant) -> Option<(f64, f64)> {
        let dt = now
            .saturating_duration_since(self.last_optical_hit)
            .as_secs_f64();
        if dt > 0.48 {
            return None;
        }
        // 阻尼角速度解析积分：∫_0^dt w0 * exp(-5*t) dt = w0 * (1 - exp(-5*dt)) / 5
        let eff_dt = (1.0 - (-5.0 * dt).exp()) / 5.0;
        let pred_yaw = wrap_yaw_deg(self.cam_yaw_deg + self.yaw_rate_dps * eff_dt);
        let pred_pitch = (self.cam_pitch_deg + self.pitch_rate_dps * eff_dt).clamp(-88.5, 88.5);
        self.last_presented_yaw_deg = pred_yaw;
        self.last_presented_pitch_deg = pred_pitch;
        self.was_coasting = true;
        Some((pred_yaw, pred_pitch))
    }
}

fn run_headless_self_test(preview_out: Option<PathBuf>) -> Result<()> {
    let db = CasDatabase::load_embedded_or_disk(None)?;
    let terrain = if let Some(pack) = WtvPackReader::auto_discover() {
        let grid = Arc::new(pack.load_map("avg_ardennes")?);
        let cx = 0.5 * (grid.ox + grid.x1);
        let cz = 0.5 * (grid.oz + grid.z1);
        let ground_elev = grid.height_at(cx, cz);
        TerrainSource::Grid {
            grid,
            fallback_alt_m: ground_elev,
        }
    } else {
        TerrainSource::Flat { ground_alt_m: 120.0 }
    };

    let (cx, cz, ground_elev) = match &terrain {
        TerrainSource::Grid { grid, .. } => {
            let x = 0.5 * (grid.ox + grid.x1);
            let z = 0.5 * (grid.oz + grid.z1);
            (x, z, grid.height_at(x, z))
        }
        TerrainSource::Flat { ground_alt_m } => (0.0, 0.0, *ground_alt_m),
    };

    let (w, h) = (1920usize, 1080usize);
    let ac = db.find_aircraft("a_10a_early").unwrap();
    let bomb = db.bombs.get("us_500lb_mk_82_ldgp").unwrap();
    let rocket = db.rockets.get("us_hydra_70_m247").unwrap();
    let tps_cfg = &db.camera.plane_tps;

    let (telem, gt_nose_px) = build_demo_telemetry_and_nose_px(
        3.2,
        &ac.id,
        tps_cfg,
        ac.tps_offset,
        ac.tps_target_offset_at_zoom,
        ac.static_aim_pitch_deg,
        500.0,
        tps_cfg.fov,
        w as f64,
        h as f64,
        [cx, cz - 450.0],
        ground_elev,
    );

    let mut game_bgra = vec![0u8; w * h * 4];
    for y in 0..h {
        let sky = y < h * 42 / 100;
        for x in 0..w {
            let p = (y * w + x) * 4;
            if sky {
                let t = y as f32 / (h as f32 * 0.42);
                game_bgra[p] = (185.0 + 45.0 * t) as u8;
                game_bgra[p + 1] = (145.0 + 55.0 * t) as u8;
                game_bgra[p + 2] = (105.0 + 75.0 * t) as u8;
                game_bgra[p + 3] = 255;
            } else {
                let t = (y - h * 42 / 100) as f32 / (h as f32 * 0.58);
                let checker = (((x / 48) + (y / 48)) & 1) as u8 * 8;
                game_bgra[p] = (52.0 + 20.0 * t) as u8 + checker;
                game_bgra[p + 1] = (78.0 + 28.0 * t) as u8 + checker;
                game_bgra[p + 2] = (62.0 + 24.0 * t) as u8 + checker;
                game_bgra[p + 3] = 255;
            }
        }
    }

    // 同时绘制同色红色鼠标牵引大圆圈 (同心干扰) + 2px 宽半透明红色机头小十字 (列 [nx_i, nx_i+1] 几何中心恰为 nx_i + 1.0)
    let nx_i = (gt_nose_px[0] - 1.0).round() as isize;
    let ny_i = (gt_nose_px[1] - 1.0).round() as isize;
    for deg in 0..360 {
        let rad = (deg as f64).to_radians();
        let rx = (nx_i as f64 + 22.0 * rad.cos()).round() as isize;
        let ry = (ny_i as f64 + 22.0 * rad.sin()).round() as isize;
        if rx >= 0 && ry >= 0 && (rx as usize) < w && (ry as usize) < h {
            let p = (ry as usize * w + rx as usize) * 4;
            game_bgra[p] = 35;
            game_bgra[p + 1] = 35;
            game_bgra[p + 2] = 245;
        }
    }
    for d in 3..=12isize {
        for th in 0..=1isize {
            for (px, py) in [
                (nx_i - d, ny_i + th),
                (nx_i + d, ny_i + th),
                (nx_i + th, ny_i - d),
                (nx_i + th, ny_i + d),
            ] {
                if px >= 0 && py >= 0 && (px as usize) < w && (py as usize) < h {
                    let p = (py as usize * w + px as usize) * 4;
                    game_bgra[p] = (game_bgra[p] as f32 * 0.18 + 30.0 * 0.82) as u8;
                    game_bgra[p + 1] = (game_bgra[p + 1] as f32 * 0.18 + 30.0 * 0.82) as u8;
                    game_bgra[p + 2] = (game_bgra[p + 2] as f32 * 0.18 + 250.0 * 0.82) as u8;
                }
            }
        }
    }

    let roi_w = 768usize;
    let roi_h = 768usize;
    let roi_x0 = w / 2 - roi_w / 2;
    let roi_y0 = h / 2 - roi_h / 2;
    let mut roi_buf = vec![0u8; roi_w * roi_h * 4];
    for ry in 0..roi_h {
        let src_off = ((roi_y0 + ry) * w + roi_x0) * 4;
        let dst_off = ry * roi_w * 4;
        roi_buf[dst_off..dst_off + roi_w * 4]
            .copy_from_slice(&game_bgra[src_off..src_off + roi_w * 4]);
    }

    let t_bench = Instant::now();
    let mut scratch = VisionScratch::new();
    let mut det = None;
    for _ in 0..20 {
        det = detect_crosshair_with_scratch(
            &mut scratch,
            &roi_buf,
            roi_w,
            roi_h,
            CrosshairColorPreset::Auto,
            6,
        );
    }
    let avg_vision_ms = t_bench.elapsed().as_secs_f64() * 1000.0 / 20.0;
    let det = det.or_else(|| detect_crosshair_bgra(&roi_buf, roi_w, roi_h, CrosshairColorPreset::Auto, 6))
        .ok_or_else(|| anyhow::anyhow!("自检：未能识别同心圆圈内的半透明红色机头准星"))?;

    let det_u = roi_x0 as f64 + det.x_roi;
    let det_v = roi_y0 as f64 + det.y_roi;
    let nose_px_err = (det_u - gt_nose_px[0]).hypot(det_v - gt_nose_px[1]);
    anyhow::ensure!(
        nose_px_err < 1.0,
        "自检：准星亚像素误差过大 ({nose_px_err:.3} px)"
    );

    let ac_pos = Vec3::new(cx, telem.alt_asl_m, cz - 450.0);
    let cam = solve_tps_camera_from_nose_pixel_ext(
        ac_pos,
        telem.heading_deg,
        telem.pitch_deg,
        telem.roll_deg,
        ac.static_aim_pitch_deg,
        500.0,
        telem.stick_elevator,
        telem.stick_rudder,
        telem.stick_ailerons,
        det_u,
        det_v,
        tps_cfg,
        ac.tps_offset,
        ac.tps_target_offset_at_zoom,
        tps_cfg.fov,
        w as f64,
        h as f64,
    )
    .ok_or_else(|| anyhow::anyhow!("自检：2x2 高斯-牛顿相机反解未收敛"))?;

    let atm = Atmosphere::standard_15c();
    let tuning = BallisticTuning::default();
    let kin = AircraftKinematics::from_telemetry(
        ac_pos,
        telem.heading_deg,
        telem.pitch_deg,
        telem.roll_deg,
        telem.tas_mps,
        telem.vy_mps,
        telem.aoa_deg,
        telem.aos_deg,
    );
    let bomb_sol = simulate_bomb_ccip(&kin, bomb, &atm, &terrain, &tuning)
        .ok_or_else(|| anyhow::anyhow!("自检：航弹落点解算为空"))?;
    let rkt_sol = match simulate_rocket_ccip_detailed(&kin, rocket, 0.0, &atm, &terrain, &tuning) {
        RocketSimOutcome::GroundHit(s) => s,
        other => anyhow::bail!("自检：火箭弹未命中地表 ({other:?})"),
    };
    let bomb_xy = cam
        .project_world_to_screen(bomb_sol.impact_pos)
        .ok_or_else(|| anyhow::anyhow!("自检：航弹落点不在视场内"))?;
    let rkt_xy = cam
        .project_world_to_screen(rkt_sol.impact_pos)
        .ok_or_else(|| anyhow::anyhow!("自检：火箭弹落点不在视场内"))?;

    let mut hud_bgra = vec![0u8; w * h * 4];
    let scene = HudScene {
        canvas_w: w as u32,
        canvas_h: h as u32,
        vp_x: 0.0,
        vp_y: 0.0,
        vp_w: w as f64,
        vp_h: h as f64,
        demo_mode: false,
        zoomed: false,
        fov_deg: tps_cfg.fov,
        nose_locked: true,
        nose_xy: Some([det_u, det_v]),
        bomb_ccip: Some(ReticleTarget {
            screen_xy: bomb_xy,
            time_of_flight_s: bomb_sol.time_of_flight_s,
            slant_range_m: bomb_sol.slant_range_m,
            tag: "BOMB",
        }),
        rocket_ccip: Some(ReticleTarget {
            screen_xy: rkt_xy,
            time_of_flight_s: rkt_sol.time_of_flight_s,
            slant_range_m: rkt_sol.slant_range_m,
            tag: "RKT",
        }),
        hud_banner: format!(
            "CCIP LOCK | {} | ALT {:.0}M | PITCH {:.1} | TAS {:.0}KM/H",
            ac.id.to_uppercase(),
            telem.alt_asl_m,
            telem.pitch_deg,
            telem.tas_mps * 3.6
        ),
    };
    render_hud_into_bgra(&scene, &mut hud_bgra);

    if let Some(out_path) = preview_out {
        if let Some(parent) = out_path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut rgba_out = vec![0u8; w * h * 4];
        for i in 0..(w * h) {
            let p = i * 4;
            let a = hud_bgra[p + 3] as f32 / 255.0;
            // hud_bgra 为预乘 BGRA，合成到底图
            let b = (hud_bgra[p] as f32 + game_bgra[p] as f32 * (1.0 - a))
                .round()
                .clamp(0.0, 255.0) as u8;
            let g = (hud_bgra[p + 1] as f32 + game_bgra[p + 1] as f32 * (1.0 - a))
                .round()
                .clamp(0.0, 255.0) as u8;
            let r = (hud_bgra[p + 2] as f32 + game_bgra[p + 2] as f32 * (1.0 - a))
                .round()
                .clamp(0.0, 255.0) as u8;
            rgba_out[p] = r;
            rgba_out[p + 1] = g;
            rgba_out[p + 2] = b;
            rgba_out[p + 3] = 255;
        }
        let img = RgbaImage::from_raw(w as u32, h as u32, rgba_out)
            .ok_or_else(|| anyhow::anyhow!("创建预览图缓冲失败"))?;
        img.save(&out_path)?;
    }

    let heli_cnt = db.aircraft.iter().filter(|a| a.is_heli).count();
    let report = serde_json::json!({
        "status": "ok",
        "datamine_version": db.datamine_version,
        "aircraft_total": db.aircraft.len(),
        "helicopters_identified": heli_cnt,
        "matched_crosshair_color": format!("{:?}", det.matched_color),
        "nose_anchor_gt_px": gt_nose_px,
        "nose_anchor_detected_px": [det_u, det_v],
        "nose_subpixel_err_px": (nose_px_err * 1000.0).round() / 1000.0,
        "avg_768x768_vision_ms": (avg_vision_ms * 100.0).round() / 100.0,
        "solved_cam_yaw_pitch_deg": [cam.yaw_deg, cam.pitch_deg],
        "bomb_ccip_px": bomb_xy,
        "rocket_ccip_px": rkt_xy,
    });
    let report_txt = serde_json::to_string_pretty(&report)?;
    let _ = std::fs::create_dir_all("dist");
    let _ = std::fs::write("dist/self_test_report.json", &report_txt);
    println!("{report_txt}");
    Ok(())
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|a| a == "--self-test") {
        let preview = args
            .windows(2)
            .find(|w| w[0] == "--export-preview")
            .map(|w| PathBuf::from(&w[1]));
        return run_headless_self_test(preview);
    }

    enable_per_monitor_dpi_v2();
    let (init_mon_w, init_mon_h) = primary_screen_size();

    let exe_beside_db = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.join("cas_db.json")));
    let db = CasDatabase::load_embedded_or_disk(exe_beside_db.as_deref())?;
    let map_pack = WtvPackReader::auto_discover();
    let mut prefs = UserPrefs::load();

    let panel = ControlPanelWindow::create()
        .ok_or_else(|| anyhow::anyhow!("创建中文控制台窗口失败"))?;
    let mut hud_win = LayeredHudWindow::create(init_mon_w, init_mon_h)
        .ok_or_else(|| anyhow::anyhow!("创建全屏透明穿透图层失败"))?;
    let mut dirty_env = DirtyEnvelope::default();
    let mut capturer = ScreenCenterCapturer::new(768, 768);
    let mut capture_buf: Vec<u8> = Vec::with_capacity(768 * 768 * 4);
    let mut vision_scratch = VisionScratch::new();
    let mut hotkeys = GlobalHotkeys::default();
    let poller = TelemetryPoller::start();
    let atm = Atmosphere::standard_15c();

    let mut map_labels: Vec<String> = FLAT_PRESETS.iter().map(|(l, _)| (*l).to_string()).collect();
    if let Some(mp) = &map_pack {
        for m in &mp.maps {
            let title = m.title.as_deref().or(m.en.as_deref()).unwrap_or(&m.name);
            map_labels.push(format!("{title} ({})", m.name));
        }
    }
    let mut cur_map_sel = 0usize;
    panel.set_combo_items(panel.cb_map, &map_labels, cur_map_sel);

    let format_ac_label = |idx: usize| -> String {
        let a = &db.aircraft[idx];
        let kind = if a.is_heli { "[武直] " } else { "" };
        let tag = if a.ccip_bombs || a.ccip_rockets {
            " ★原生CCIP"
        } else {
            ""
        };
        format!("{kind}{} ({}){tag}", a.name_zh, a.id)
    };

    let mut cur_ac_idx = db
        .aircraft
        .iter()
        .position(|a| a.id == "a_10a_early")
        .or_else(|| db.aircraft.iter().position(|a| a.id == "p-47d-28"))
        .unwrap_or(0);
    let mut filtered_ac_indices: Vec<usize> = (0..db.aircraft.len()).collect();
    let ac_labels: Vec<String> = filtered_ac_indices.iter().map(|&i| format_ac_label(i)).collect();
    panel.set_combo_items(panel.cb_aircraft, &ac_labels, cur_ac_idx);

    let color_labels: Vec<String> = COLOR_PRESETS.iter().map(|c| c.label_zh().to_string()).collect();
    let mut color_idx = prefs.color_idx.min(COLOR_PRESETS.len() - 1);
    panel.set_combo_items(panel.cb_color, &color_labels, color_idx);

    let conv_labels: Vec<String> = CONV_DIST_OPTIONS.iter().map(|(_, l)| (*l).to_string()).collect();
    let mut conv_idx = prefs.conv_idx.min(CONV_DIST_OPTIONS.len() - 1);
    panel.set_combo_items(panel.cb_conv, &conv_labels, conv_idx);

    let drag_labels: Vec<String> = DRAG_SCALE_OPTIONS.iter().map(|s| format!("{s:.2}x")).collect();
    let mut drag_idx = prefs.drag_idx.min(DRAG_SCALE_OPTIONS.len() - 1);
    panel.set_combo_items(panel.cb_drag, &drag_labels, drag_idx);

    let trim_labels: Vec<String> = RKT_TRIM_OPTIONS
        .iter()
        .map(|deg| format!("{deg:+.1}°"))
        .collect();
    let mut rkt_trim_idx = prefs.rkt_trim_idx.min(RKT_TRIM_OPTIONS.len() - 1);
    panel.set_combo_items(panel.cb_rkt_trim, &trim_labels, rkt_trim_idx);

    let zoom_mode_labels: Vec<String> = ZoomTriggerMode::ALL
        .iter()
        .map(|m| m.label_zh().to_string())
        .collect();
    panel.set_combo_items(
        panel.cb_zoom1_mode,
        &zoom_mode_labels,
        prefs.hotkeys.zoom1_mode.to_index(),
    );
    panel.set_combo_items(
        panel.cb_zoom2_mode,
        &zoom_mode_labels,
        prefs.hotkeys.zoom2_mode.to_index(),
    );
    panel.refresh_hotkey_controls(&prefs.hotkeys, None);

    let mut auto_ac = prefs.auto_ac;
    let mut overlay_enabled = true;
    let mut demo_mode = false;
    let mut bay_delay_on = prefs.bay_delay_on;
    let mut active_bind_recorder: Option<KeyBindRecorder> = None;
    panel.set_checked(panel.chk_auto_ac, auto_ac);
    panel.set_checked(panel.chk_overlay, overlay_enabled);
    panel.set_checked(panel.chk_demo, demo_mode);
    panel.set_checked(panel.chk_bay_delay, bay_delay_on);

    // `(bomb_id, is_internal_bay_only)`
    let mut cur_bomb_entries: Vec<(Option<String>, bool)> = Vec::new();
    let mut cur_rocket_entries: Vec<(Option<String>, f64)> = Vec::new();
    let mut sel_bomb_idx = 0usize;
    let mut sel_rocket_idx = 0usize;

    let refresh_weapons_for_ac = |ac_idx: usize,
                                  cur_bomb_entries: &mut Vec<(Option<String>, bool)>,
                                  cur_rocket_entries: &mut Vec<(Option<String>, f64)>,
                                  sel_bomb_idx: &mut usize,
                                  sel_rocket_idx: &mut usize| {
        let ac = &db.aircraft[ac_idx];
        cur_bomb_entries.clear();
        let mut b_labels = vec!["[关闭航弹 CCIP]".to_string()];
        cur_bomb_entries.push((None, false));
        for mb in &ac.bombs {
            if let Some(bs) = db.bombs.get(&mb.id) {
                let is_bay_only = mb.has_internal_bay && !mb.external;
                let mount_str = if is_bay_only { "·弹舱" } else { "·外挂" };
                let kind = if bs.brake_cx_k > 1.5 {
                    "·减速伞"
                } else if bs.is_napalm {
                    "·燃烧弹"
                } else {
                    ""
                };
                b_labels.push(format!("{} ({:.0}kg{mount_str}{kind})", bs.name_zh, bs.mass));
                cur_bomb_entries.push((Some(mb.id.clone()), is_bay_only));
            }
        }
        *sel_bomb_idx = usize::from(cur_bomb_entries.len() > 1);
        panel.set_combo_items(panel.cb_bomb, &b_labels, *sel_bomb_idx);

        cur_rocket_entries.clear();
        let mut r_labels = vec!["[关闭火箭弹 CCIP]".to_string()];
        cur_rocket_entries.push((None, 0.0));
        for mr in &ac.rockets {
            if let Some(rs) = db.rockets.get(&mr.id) {
                r_labels.push(format!(
                    "{} (射程 {:.1}km · CxK={:.2})",
                    rs.name_zh,
                    rs.max_distance * 0.001,
                    rs.cx_k
                ));
                cur_rocket_entries.push((Some(mr.id.clone()), mr.launcher_pitch_deg));
            }
        }
        *sel_rocket_idx = usize::from(cur_rocket_entries.len() > 1);
        panel.set_combo_items(panel.cb_rocket, &r_labels, *sel_rocket_idx);
    };

    refresh_weapons_for_ac(
        cur_ac_idx,
        &mut cur_bomb_entries,
        &mut cur_rocket_entries,
        &mut sel_bomb_idx,
        &mut sel_rocket_idx,
    );

    let mut manual_calib_alt_m: Option<f64> = None;
    let mut loaded_grid_terrain: Option<TerrainSource> = None;
    let mut z_zoom_toggled = false;
    let mut current_fov_deg = db.camera.plane_tps.fov;

    let mut nose_filter = OneEuroFilter2D::new(2.2, 0.14, 1.0, 220.0);
    let mut bomb_filter = OneEuroFilter2D::new(2.8, 0.12, 1.0, 280.0);
    let mut rocket_filter = OneEuroFilter2D::new(2.8, 0.12, 1.0, 280.0);
    let mut coaster: Option<WorldCameraCoaster> = None;
    let mut last_matched_color: Option<CrosshairColorPreset> = None;

    let mut cached_vp = GameViewportInfo {
        vp_x: 0,
        vp_y: 0,
        vp_w: init_mon_w,
        vp_h: init_mon_h,
        mon_x: 0,
        mon_y: 0,
        mon_w: init_mon_w,
        mon_h: init_mon_h,
        is_foreground: false,
    };
    let mut last_vp_check = Instant::now() - Duration::from_secs(2);
    let mut prev_loop_inst = Instant::now();
    let mut frame_no: u64 = 0;
    let start_instant = Instant::now();
    let mut last_status_refresh = Instant::now() - Duration::from_secs(1);

    // 稳态高精度 75 FPS (`13.33 ms`) 渲染同步循环，配合 UpdateLayeredWindowIndirect 局部脏矩形提交
    let target_frame_dur = Duration::from_micros(13_333);

    loop {
        let frame_start = Instant::now();
        let dt_frame = frame_start
            .saturating_duration_since(prev_loop_inst)
            .as_secs_f64()
            .clamp(0.004, 0.10);
        prev_loop_inst = frame_start;

        if !ControlPanelWindow::pump_messages() {
            break;
        }

        let mut should_quit = false;
        let mut prefs_dirty = false;
        for ev in drain_ui_events() {
            match ev {
                UiEvent::MapChanged(idx) => {
                    cur_map_sel = idx;
                    manual_calib_alt_m = None;
                    if idx < FLAT_PRESETS.len() {
                        loaded_grid_terrain = None;
                    } else if let Some(mp) = &map_pack {
                        if let Some(m_meta) = mp.maps.get(idx - FLAT_PRESETS.len()) {
                            if let Ok(grid) = mp.load_map(&m_meta.name) {
                                let fb = 0.5 * (grid.min_elev + grid.max_elev);
                                loaded_grid_terrain = Some(TerrainSource::Grid {
                                    grid: Arc::new(grid),
                                    fallback_alt_m: fb,
                                });
                            }
                        }
                    }
                }
                UiEvent::SearchChanged(q) => {
                    let matches = db.search_aircraft(&q);
                    if !matches.is_empty() {
                        filtered_ac_indices = matches;
                        let pos = filtered_ac_indices
                            .iter()
                            .position(|&i| i == cur_ac_idx)
                            .unwrap_or(0);
                        cur_ac_idx = filtered_ac_indices[pos];
                        let labels: Vec<String> = filtered_ac_indices
                            .iter()
                            .map(|&i| format_ac_label(i))
                            .collect();
                        panel.set_combo_items(panel.cb_aircraft, &labels, pos);
                        refresh_weapons_for_ac(
                            cur_ac_idx,
                            &mut cur_bomb_entries,
                            &mut cur_rocket_entries,
                            &mut sel_bomb_idx,
                            &mut sel_rocket_idx,
                        );
                    }
                }
                UiEvent::AircraftChanged(combo_pos) => {
                    if let Some(&real_idx) = filtered_ac_indices.get(combo_pos) {
                        cur_ac_idx = real_idx;
                        refresh_weapons_for_ac(
                            cur_ac_idx,
                            &mut cur_bomb_entries,
                            &mut cur_rocket_entries,
                            &mut sel_bomb_idx,
                            &mut sel_rocket_idx,
                        );
                    }
                }
                UiEvent::BombChanged(idx) if idx < cur_bomb_entries.len() => {
                    sel_bomb_idx = idx;
                    bomb_filter.reset();
                }
                UiEvent::RocketChanged(idx) if idx < cur_rocket_entries.len() => {
                    sel_rocket_idx = idx;
                    rocket_filter.reset();
                }
                UiEvent::ColorChanged(idx) if idx < COLOR_PRESETS.len() => {
                    color_idx = idx;
                    prefs.color_idx = idx;
                    prefs_dirty = true;
                    vision_scratch.reset_color_lock();
                    nose_filter.reset();
                    coaster = None;
                }
                UiEvent::ConvChanged(idx) if idx < CONV_DIST_OPTIONS.len() => {
                    conv_idx = idx;
                    prefs.conv_idx = idx;
                    prefs_dirty = true;
                }
                UiEvent::DragChanged(idx) if idx < DRAG_SCALE_OPTIONS.len() => {
                    drag_idx = idx;
                    prefs.drag_idx = idx;
                    prefs_dirty = true;
                }
                UiEvent::RktTrimChanged(idx) if idx < RKT_TRIM_OPTIONS.len() => {
                    rkt_trim_idx = idx;
                    prefs.rkt_trim_idx = idx;
                    prefs_dirty = true;
                }
                UiEvent::ToggleAutoAc(on) => {
                    auto_ac = on;
                    prefs.auto_ac = on;
                    prefs_dirty = true;
                }
                UiEvent::ToggleOverlay(on) => {
                    overlay_enabled = on;
                    if !overlay_enabled {
                        hud_win.frame_buffer_mut().fill(0);
                        dirty_env = DirtyEnvelope::default();
                    } else {
                        dirty_env.force_full_dirty(hud_win.width as u32, hud_win.height as u32);
                    }
                    hud_win.set_visible(overlay_enabled);
                }
                UiEvent::ToggleDemo(on) => {
                    demo_mode = on;
                    nose_filter.reset();
                    bomb_filter.reset();
                    rocket_filter.reset();
                    coaster = None;
                }
                UiEvent::Zoom1ModeChanged(idx) => {
                    prefs.hotkeys.zoom1_mode = ZoomTriggerMode::from_index(idx);
                    if prefs.hotkeys.zoom1_mode != ZoomTriggerMode::Toggle
                        && prefs.hotkeys.zoom2_mode != ZoomTriggerMode::Toggle
                    {
                        z_zoom_toggled = false;
                    }
                    prefs_dirty = true;
                }
                UiEvent::Zoom2ModeChanged(idx) => {
                    prefs.hotkeys.zoom2_mode = ZoomTriggerMode::from_index(idx);
                    if prefs.hotkeys.zoom1_mode != ZoomTriggerMode::Toggle
                        && prefs.hotkeys.zoom2_mode != ZoomTriggerMode::Toggle
                    {
                        z_zoom_toggled = false;
                    }
                    prefs_dirty = true;
                }
                UiEvent::StartBind(slot) => {
                    if active_bind_recorder.as_ref().is_some_and(|r| r.slot == slot) {
                        active_bind_recorder = None;
                        panel.refresh_hotkey_controls(&prefs.hotkeys, None);
                    } else {
                        active_bind_recorder = Some(KeyBindRecorder::start(slot));
                        panel.refresh_hotkey_controls(&prefs.hotkeys, Some(slot));
                    }
                }
                UiEvent::ResetHotkeys => {
                    active_bind_recorder = None;
                    prefs.hotkeys = HotkeyBindings::default();
                    z_zoom_toggled = false;
                    panel.refresh_hotkey_controls(&prefs.hotkeys, None);
                    prefs_dirty = true;
                }
                UiEvent::ToggleBayDelay(on) => {
                    bay_delay_on = on;
                    prefs.bay_delay_on = on;
                    prefs_dirty = true;
                }
                UiEvent::FlipZoom => z_zoom_toggled = !z_zoom_toggled,
                UiEvent::Quit => should_quit = true,
                _ => {}
            }
        }
        if prefs_dirty {
            prefs.save();
        }
        if should_quit {
            break;
        }

        // 若正处于“交互式一键录键”等待态，优先捕获新按键并挂起常规热键响应
        if let Some(rec) = &mut active_bind_recorder {
            let slot = rec.slot;
            match rec.poll() {
                BindPollOutcome::Waiting => {}
                BindPollOutcome::Bound(vk) => {
                    match slot {
                        BindSlot::Zoom1 => {
                            prefs.hotkeys.zoom1_vk = Some(vk);
                            if prefs.hotkeys.zoom1_mode == ZoomTriggerMode::Disabled {
                                prefs.hotkeys.zoom1_mode = ZoomTriggerMode::Toggle;
                            }
                        }
                        BindSlot::Zoom2 => {
                            prefs.hotkeys.zoom2_vk = Some(vk);
                            if prefs.hotkeys.zoom2_mode == ZoomTriggerMode::Disabled {
                                prefs.hotkeys.zoom2_mode = ZoomTriggerMode::Hold;
                            }
                        }
                        BindSlot::CalibrateGround => prefs.hotkeys.calibrate_ground_vk = Some(vk),
                        BindSlot::ResyncZoom => prefs.hotkeys.resync_zoom_vk = Some(vk),
                        BindSlot::ToggleOverlay => prefs.hotkeys.toggle_overlay_vk = Some(vk),
                        BindSlot::ToggleDemo => prefs.hotkeys.toggle_demo_vk = Some(vk),
                    }
                    prefs.save();
                    hotkeys.sync_all_pressed();
                    active_bind_recorder = None;
                    panel.refresh_hotkey_controls(&prefs.hotkeys, None);
                }
                BindPollOutcome::Cleared => {
                    match slot {
                        BindSlot::Zoom1 => {
                            prefs.hotkeys.zoom1_vk = None;
                            prefs.hotkeys.zoom1_mode = ZoomTriggerMode::Disabled;
                        }
                        BindSlot::Zoom2 => {
                            prefs.hotkeys.zoom2_vk = None;
                            prefs.hotkeys.zoom2_mode = ZoomTriggerMode::Disabled;
                        }
                        BindSlot::CalibrateGround => prefs.hotkeys.calibrate_ground_vk = None,
                        BindSlot::ResyncZoom => prefs.hotkeys.resync_zoom_vk = None,
                        BindSlot::ToggleOverlay => prefs.hotkeys.toggle_overlay_vk = None,
                        BindSlot::ToggleDemo => prefs.hotkeys.toggle_demo_vk = None,
                    }
                    if prefs.hotkeys.zoom1_mode != ZoomTriggerMode::Toggle
                        && prefs.hotkeys.zoom2_mode != ZoomTriggerMode::Toggle
                    {
                        z_zoom_toggled = false;
                    }
                    prefs.save();
                    hotkeys.sync_all_pressed();
                    active_bind_recorder = None;
                    panel.refresh_hotkey_controls(&prefs.hotkeys, None);
                }
                BindPollOutcome::Cancelled => {
                    hotkeys.sync_all_pressed();
                    active_bind_recorder = None;
                    panel.refresh_hotkey_controls(&prefs.hotkeys, None);
                }
            }
        }

        if last_vp_check.elapsed() >= Duration::from_millis(250) {
            last_vp_check = Instant::now();
            cached_vp = if demo_mode {
                let (mw, mh) = primary_screen_size();
                GameViewportInfo {
                    vp_x: 0,
                    vp_y: 0,
                    vp_w: mw,
                    vp_h: mh,
                    mon_x: 0,
                    mon_y: 0,
                    mon_w: mw,
                    mon_h: mh,
                    is_foreground: is_window_foreground(panel.hwnd) && !panel.is_search_focused(),
                }
            } else {
                query_war_thunder_viewport_ext().unwrap_or_else(|| {
                    let (mw, mh) = primary_screen_size();
                    GameViewportInfo {
                        vp_x: 0,
                        vp_y: 0,
                        vp_w: mw,
                        vp_h: mh,
                        mon_x: 0,
                        mon_y: 0,
                        mon_w: mw,
                        mon_h: mh,
                        is_foreground: false,
                    }
                })
            };

            if hud_win.ensure_monitor_geometry(
                cached_vp.mon_x,
                cached_vp.mon_y,
                cached_vp.mon_w,
                cached_vp.mon_h,
            ) {
                dirty_env.force_full_dirty(hud_win.width as u32, hud_win.height as u32);
            }
        }

        // 游戏客户区相对于当前透明覆盖层所属显示器的局部偏移
        let local_vp_x = (cached_vp.vp_x - hud_win.mon_x).clamp(0, hud_win.width - 64);
        let local_vp_y = (cached_vp.vp_y - hud_win.mon_y).clamp(0, hud_win.height - 64);
        let view_w = cached_vp.vp_w.clamp(320, hud_win.width);
        let view_h = cached_vp.vp_h.clamp(240, hud_win.height);

        let panel_ready_for_keys = is_window_foreground(panel.hwnd) && !panel.is_search_focused();
        let hk = if active_bind_recorder.is_some() {
            HotkeyEvents::default()
        } else {
            hotkeys.poll(
                &prefs.hotkeys,
                cached_vp.is_foreground,
                cached_vp.is_foreground || panel_ready_for_keys,
            )
        };

        if hk.toggle_zoom || hk.resync_zoom {
            z_zoom_toggled = !z_zoom_toggled;
        }
        if hk.toggle_overlay {
            overlay_enabled = !overlay_enabled;
            panel.set_checked(panel.chk_overlay, overlay_enabled);
            if !overlay_enabled {
                hud_win.frame_buffer_mut().fill(0);
                dirty_env = DirtyEnvelope::default();
            } else {
                dirty_env.force_full_dirty(hud_win.width as u32, hud_win.height as u32);
            }
            hud_win.set_visible(overlay_enabled);
        }
        if hk.toggle_demo {
            demo_mode = !demo_mode;
            panel.set_checked(panel.chk_demo, demo_mode);
            nose_filter.reset();
            bomb_filter.reset();
            rocket_filter.reset();
            coaster = None;
        }

        // 根据拆包 `config/camera.blkx` 条件：固定翼与默认直升机均使用标准 `plane_tps` (90°->30°, 零横向视差偏移)
        let tps_cfg = &db.camera.plane_tps;
        let zoomed = z_zoom_toggled || hk.hold_zoom_active;
        let target_fov = if zoomed { tps_cfg.zoom_fov } else { tps_cfg.fov };
        current_fov_deg += (target_fov - current_fov_deg) * (1.0 - (-18.0 * dt_frame).exp());

        let conv_dist_m = CONV_DIST_OPTIONS[conv_idx].0;
        let rkt_trim_deg = RKT_TRIM_OPTIONS[rkt_trim_idx];
        let is_selected_bomb_bay_only = cur_bomb_entries
            .get(sel_bomb_idx)
            .is_some_and(|(_, bay_only)| *bay_only);
        let tuning = BallisticTuning {
            bomb_drag_scale: DRAG_SCALE_OPTIONS[drag_idx],
            // 智能弹舱延迟：即便勾选该项，翼下外挂航弹 (external=true) 仍自动使用 0.0s 无延迟！
            bay_door_delay_s: if bay_delay_on && is_selected_bomb_bay_only {
                0.55
            } else {
                0.0
            },
            ..Default::default()
        };

        let ac_now = &db.aircraft[cur_ac_idx];
        let (telem, active_terrain, raw_optical_px) = if demo_mode {
            let terr = loaded_grid_terrain
                .clone()
                .unwrap_or(TerrainSource::Flat { ground_alt_m: 65.0 });
            let (center_xz, g_alt) = match &terr {
                TerrainSource::Grid { grid, .. } => {
                    let cx = 0.5 * (grid.ox + grid.x1);
                    let cz = 0.5 * (grid.oz + grid.z1);
                    ([cx, cz], grid.height_at(cx, cz))
                }
                TerrainSource::Flat { ground_alt_m } => ([0.0, 0.0], *ground_alt_m),
            };
            let (tf, sim_px) = build_demo_telemetry_and_nose_px(
                start_instant.elapsed().as_secs_f64(),
                &ac_now.id,
                tps_cfg,
                ac_now.tps_offset,
                ac_now.tps_target_offset_at_zoom,
                ac_now.static_aim_pitch_deg,
                conv_dist_m,
                current_fov_deg,
                view_w as f64,
                view_h as f64,
                center_xz,
                g_alt,
            );
            last_matched_color = Some(CrosshairColorPreset::Magenta);
            (tf, terr, Some(sim_px))
        } else {
            let tf = poller.snapshot();
            if hk.calibrate_ground && tf.online {
                manual_calib_alt_m = Some((tf.alt_asl_m - 15.0).max(0.0));
            }

            if tf.online && auto_ac && !tf.unit_id.is_empty() && tf.unit_id != ac_now.id {
                if let Some(found_idx) = db.aircraft.iter().position(|a| a.id == tf.unit_id) {
                    cur_ac_idx = found_idx;
                    if let Some(combo_pos) =
                        filtered_ac_indices.iter().position(|&i| i == cur_ac_idx)
                    {
                        panel.set_combo_sel(panel.cb_aircraft, combo_pos);
                    }
                    refresh_weapons_for_ac(
                        cur_ac_idx,
                        &mut cur_bomb_entries,
                        &mut cur_rocket_entries,
                        &mut sel_bomb_idx,
                        &mut sel_rocket_idx,
                    );
                }
            }

            let terr = if let Some(grid_t) = &loaded_grid_terrain {
                grid_t.clone()
            } else if let Some(m_alt) = manual_calib_alt_m {
                TerrainSource::Flat { ground_alt_m: m_alt }
            } else if let Some((_, Some(preset_alt))) = FLAT_PRESETS.get(cur_map_sel) {
                TerrainSource::Flat {
                    ground_alt_m: *preset_alt,
                }
            } else {
                let auto_g = tf
                    .radar_ground_alt_m
                    .or(tf.auto_runway_alt_m)
                    .unwrap_or(65.0);
                TerrainSource::Flat {
                    ground_alt_m: auto_g,
                }
            };

            let detected = if tf.online && overlay_enabled {
                if let Some(cap) = &mut capturer {
                    let preset_col = COLOR_PRESETS[color_idx];
                    // 截屏矩形严格钳制在战雷客户区内部，杜绝窗口化时把桌面壁纸/浏览器红字误截入识别区
                    let clamp_center = |cx: i32, cy: i32, sub_w: i32, sub_h: i32| -> (i32, i32) {
                        let hw = (sub_w / 2).min(view_w / 2);
                        let hh = (sub_h / 2).min(view_h / 2);
                        let min_x = cached_vp.vp_x + hw;
                        let max_x = (cached_vp.vp_x + view_w - hw).max(min_x);
                        let min_y = cached_vp.vp_y + hh;
                        let max_y = (cached_vp.vp_y + view_h - hh).max(min_y);
                        (cx.clamp(min_x, max_x), cy.clamp(min_y, max_y))
                    };

                    // 已锁定时先查 264×264 零分配跟踪小窗 (< 0.15 ms)；未命中当帧立即查 768×768 大窗
                    let track_hit = nose_filter.current().and_then(|[pu, pv]| {
                        let tw = 264.min(view_w as usize);
                        let th = 264.min(view_h as usize);
                        let (cx_scr, cy_scr) = clamp_center(
                            cached_vp.vp_x + pu.round() as i32,
                            cached_vp.vp_y + pv.round() as i32,
                            tw as i32,
                            th as i32,
                        );
                        let (lx, ty, rw, rh) =
                            cap.capture_around_into(cx_scr, cy_scr, tw, th, &mut capture_buf)?;
                        let d = detect_crosshair_with_scratch(
                            &mut vision_scratch,
                            &capture_buf,
                            rw,
                            rh,
                            preset_col,
                            6,
                        )?;
                        last_matched_color = Some(d.matched_color);
                        Some([
                            (lx - cached_vp.vp_x) as f64 + d.x_roi,
                            (ty - cached_vp.vp_y) as f64 + d.y_roi,
                        ])
                    });

                    track_hit.or_else(|| {
                        let fw = 768.min(view_w as usize);
                        let fh = 768.min(view_h as usize);
                        let (cx_scr, cy_scr) = clamp_center(
                            cached_vp.vp_x + view_w / 2,
                            cached_vp.vp_y + view_h / 2,
                            fw as i32,
                            fh as i32,
                        );
                        let (lx, ty, rw, rh) =
                            cap.capture_around_into(cx_scr, cy_scr, fw, fh, &mut capture_buf)?;
                        let d = detect_crosshair_with_scratch(
                            &mut vision_scratch,
                            &capture_buf,
                            rw,
                            rh,
                            preset_col,
                            6,
                        )?;
                        last_matched_color = Some(d.matched_color);
                        Some([
                            (lx - cached_vp.vp_x) as f64 + d.x_roi,
                            (ty - cached_vp.vp_y) as f64 + d.y_roi,
                        ])
                    })
                } else {
                    None
                }
            } else {
                None
            };
            (tf, terr, detected)
        };

        let ac_active = &db.aircraft[cur_ac_idx];
        let now_inst = Instant::now();
        let ext = if demo_mode {
            telem.extrapolated_state(telem.sampled_at.unwrap_or(now_inst))
        } else {
            telem.extrapolated_state(now_inst)
        };
        let [wx, wz] = telem.map_xz_m.unwrap_or([0.0, 0.0]);
        let ac_pos = Vec3::new(wx, ext.alt_asl_m, wz);

        let mut active_cam: Option<TpsCameraState> = None;
        let mut display_nose_px: Option<[f64; 2]> = None;
        let mut is_coasting = false;

        if telem.online {
            if let Some(cur_px) = raw_optical_px {
                let filt_px = nose_filter.filter(cur_px, dt_frame);
                display_nose_px = Some(filt_px);

                if let Some(solved) = solve_tps_camera_from_nose_pixel_ext(
                    ac_pos,
                    ext.heading_deg,
                    ext.pitch_deg,
                    ext.roll_deg,
                    ac_active.static_aim_pitch_deg,
                    conv_dist_m,
                    telem.stick_elevator,
                    telem.stick_rudder,
                    telem.stick_ailerons,
                    filt_px[0],
                    filt_px[1],
                    tps_cfg,
                    ac_active.tps_offset,
                    ac_active.tps_target_offset_at_zoom,
                    current_fov_deg,
                    view_w as f64,
                    view_h as f64,
                ) {
                    let (cy, cp) = match &mut coaster {
                        Some(c) => c.on_optical_lock(solved.yaw_deg, solved.pitch_deg, now_inst),
                        None => {
                            coaster =
                                Some(WorldCameraCoaster::new(solved.yaw_deg, solved.pitch_deg, now_inst));
                            (solved.yaw_deg, solved.pitch_deg)
                        }
                    };
                    active_cam = Some(TpsCameraState::build_ext(
                        ac_pos,
                        cy,
                        cp,
                        ext.roll_deg,
                        telem.stick_elevator,
                        telem.stick_rudder,
                        telem.stick_ailerons,
                        tps_cfg,
                        ac_active.tps_offset,
                        ac_active.tps_target_offset_at_zoom,
                        current_fov_deg,
                        view_w as f64,
                        view_h as f64,
                    ));
                }
            } else if let Some(c) = &mut coaster {
                if let Some((cy, cp)) = c.coast_at(now_inst) {
                    let cam = TpsCameraState::build_ext(
                        ac_pos,
                        cy,
                        cp,
                        ext.roll_deg,
                        telem.stick_elevator,
                        telem.stick_rudder,
                        telem.stick_ailerons,
                        tps_cfg,
                        ac_active.tps_offset,
                        ac_active.tps_target_offset_at_zoom,
                        current_fov_deg,
                        view_w as f64,
                        view_h as f64,
                    );
                    let ac_basis =
                        BodyBasis::from_euler_deg(ext.heading_deg, ext.pitch_deg, ext.roll_deg);
                    let bore_pt = ac_pos
                        + ac_basis.pitched_axis(ac_active.static_aim_pitch_deg) * conv_dist_m;
                    if let Some(proj_nose) = cam.project_world_to_screen(bore_pt) {
                        display_nose_px = Some(nose_filter.filter(proj_nose, dt_frame));
                    }
                    active_cam = Some(cam);
                    is_coasting = true;
                } else {
                    nose_filter.reset();
                    bomb_filter.reset();
                    rocket_filter.reset();
                    coaster = None;
                }
            } else {
                nose_filter.reset();
                bomb_filter.reset();
                rocket_filter.reset();
            }
        } else {
            nose_filter.reset();
            bomb_filter.reset();
            rocket_filter.reset();
            coaster = None;
        }

        let mut bomb_target: Option<ReticleTarget> = None;
        let mut rocket_target: Option<ReticleTarget> = None;
        let mut rocket_self_destruct_m: Option<f64> = None;

        if let Some(cam) = &active_cam {
            let kin = AircraftKinematics::from_telemetry(
                ac_pos,
                ext.heading_deg,
                ext.pitch_deg,
                ext.roll_deg,
                ext.tas_mps,
                telem.vy_mps,
                telem.aoa_deg,
                telem.aos_deg,
            );

            if let Some((Some(bid), _)) = cur_bomb_entries.get(sel_bomb_idx) {
                if let Some(bspec) = db.bombs.get(bid) {
                    if let Some(sol) =
                        simulate_bomb_ccip(&kin, bspec, &atm, &active_terrain, &tuning)
                    {
                        if let Some(raw_xy) = cam.project_world_to_screen(sol.impact_pos) {
                            let smooth_xy = bomb_filter.filter(raw_xy, dt_frame);
                            bomb_target = Some(ReticleTarget {
                                screen_xy: smooth_xy,
                                time_of_flight_s: sol.time_of_flight_s,
                                slant_range_m: sol.slant_range_m,
                                tag: "BOMB",
                            });
                        } else {
                            bomb_filter.reset();
                        }
                    } else {
                        bomb_filter.reset();
                    }
                }
            }

            if let Some((Some(rid), l_pitch)) = cur_rocket_entries.get(sel_rocket_idx) {
                if let Some(rspec) = db.rockets.get(rid) {
                    match simulate_rocket_ccip_detailed(
                        &kin,
                        rspec,
                        *l_pitch + rkt_trim_deg,
                        &atm,
                        &active_terrain,
                        &tuning,
                    ) {
                        RocketSimOutcome::GroundHit(sol) => {
                            if let Some(raw_xy) = cam.project_world_to_screen(sol.impact_pos) {
                                let smooth_xy = rocket_filter.filter(raw_xy, dt_frame);
                                rocket_target = Some(ReticleTarget {
                                    screen_xy: smooth_xy,
                                    time_of_flight_s: sol.time_of_flight_s,
                                    slant_range_m: sol.slant_range_m,
                                    tag: "RKT",
                                });
                            } else {
                                rocket_filter.reset();
                            }
                        }
                        RocketSimOutcome::SelfDestructed { slant_range_m, .. } => {
                            rocket_self_destruct_m = Some(slant_range_m);
                            rocket_filter.reset();
                        }
                        RocketSimOutcome::Unreachable => {
                            rocket_filter.reset();
                        }
                    }
                }
            }
        }

        if overlay_enabled {
            let nose_locked = active_cam.is_some();
            let hud_banner = if demo_mode {
                format!(
                    "DEMO TPS CCIP | {} | ALT {:.0}M | PITCH {:.1} | FOV {:.0}",
                    ac_active.id.to_uppercase(),
                    ext.alt_asl_m,
                    ext.pitch_deg,
                    current_fov_deg
                )
            } else if !telem.online {
                String::new()
            } else if nose_locked {
                format!(
                    "CCIP {} | {} | ALT {:.0}M | PITCH {:.1} | TAS {:.0}KM/H",
                    if is_coasting { "HOLD" } else { "LOCK" },
                    ac_active.id.to_uppercase(),
                    ext.alt_asl_m,
                    ext.pitch_deg,
                    ext.tas_mps * 3.6
                )
            } else {
                "CCIP STANDBY - AWAITING NOSE CROSSHAIR".to_string()
            };

            let scene = HudScene {
                canvas_w: hud_win.width as u32,
                canvas_h: hud_win.height as u32,
                vp_x: local_vp_x as f64,
                vp_y: local_vp_y as f64,
                vp_w: view_w as f64,
                vp_h: view_h as f64,
                demo_mode,
                zoomed,
                fov_deg: current_fov_deg,
                nose_locked,
                nose_xy: display_nose_px,
                bomb_ccip: bomb_target.clone(),
                rocket_ccip: rocket_target.clone(),
                hud_banner,
            };
            render_hud_incremental(&scene, hud_win.frame_buffer_mut(), &mut dirty_env);
            hud_win.present_dirty(dirty_env.last_present_dirty_rect(), frame_no % 180 == 0);
        }

        if last_status_refresh.elapsed() >= Duration::from_millis(160) {
            last_status_refresh = Instant::now();
            let g_elev = active_terrain.height_at(wx, wz);
            let conn_str = if active_bind_recorder.is_some() {
                "【正在录入新按键：请直接按下任意键盘按键或鼠标右/中/侧键 (按 Esc 取消，按 Del 清空)】"
            } else if demo_mode {
                "【内置动态三维俯冲演示运行中 (可按演示热键切回实机 8111)】"
            } else if telem.online {
                "【已连接战雷 127.0.0.1:8111 实时遥测 (1-Euro 防抖 + 世界空间惯性滤波)】"
            } else {
                "【等待战雷对局遥测... (待机态 DWM 0 开销；可勾选演示模式预览)】"
            };
            let col_tag = last_matched_color
                .map(|c| format!("{:?}", c))
                .unwrap_or_else(|| "-".into());
            let lock_str = match (display_nose_px, active_cam.as_ref()) {
                (Some([u, v]), Some(c)) => format!(
                    "{}[{col_tag}] ({u:.0},{v:.0}) → 相机 航向={:.1}° 俯仰={:+.1}°",
                    if is_coasting { "惯性维持" } else { "光学锁定" },
                    c.yaw_deg,
                    c.pitch_deg
                ),
                _ if telem.online => {
                    "未锁定 (默认 Auto 自动识别红/白/绿/黄/青/品红十字准星)".to_string()
                }
                _ => "待机中 (未进入飞行状态时屏幕保持净空)".to_string(),
            };
            let b_str = bomb_target.map_or("未启用/超界".to_string(), |b| {
                format!(
                    "屏幕({:.0},{:.0}) · 飞行 {:.1}s · 斜距 {:.0}m{}",
                    b.screen_xy[0],
                    b.screen_xy[1],
                    b.time_of_flight_s,
                    b.slant_range_m,
                    if tuning.bay_door_delay_s > 0.0 {
                        " [含弹舱延迟]"
                    } else {
                        ""
                    }
                )
            });
            let r_str = if let Some(r) = rocket_target {
                format!(
                    "屏幕({:.0},{:.0}) · 飞行 {:.1}s · 斜距 {:.0}m",
                    r.screen_xy[0], r.screen_xy[1], r.time_of_flight_s, r.slant_range_m
                )
            } else if let Some(sd_m) = rocket_self_destruct_m {
                format!("超出最大射程空中自毁 ({sd_m:.0}m) · 请拉近距离")
            } else {
                "未启用/超界".to_string()
            };

            let fmt_z = |vk: Option<u16>, m: ZoomTriggerMode| -> String {
                match (vk, m) {
                    (_, ZoomTriggerMode::Disabled) | (None, _) => "禁用".to_string(),
                    (Some(k), ZoomTriggerMode::Toggle) => {
                        format!("{}(单点)", vk_display_name_zh(Some(k)))
                    }
                    (Some(k), ZoomTriggerMode::Hold) => {
                        format!("{}(长按)", vk_display_name_zh(Some(k)))
                    }
                }
            };
            let hk_summary = format!(
                "当前热键：缩放=[{} / {}] | 标定={} | 翻转={} | 显隐={} | 演示={}",
                fmt_z(prefs.hotkeys.zoom1_vk, prefs.hotkeys.zoom1_mode),
                fmt_z(prefs.hotkeys.zoom2_vk, prefs.hotkeys.zoom2_mode),
                vk_display_name_zh(prefs.hotkeys.calibrate_ground_vk),
                vk_display_name_zh(prefs.hotkeys.resync_zoom_vk),
                vk_display_name_zh(prefs.hotkeys.toggle_overlay_vk),
                vk_display_name_zh(prefs.hotkeys.toggle_demo_vk),
            );

            let status = format!(
                "{conn_str}\r\n\
                 ● 飞行状态：海拔 {:.0}m (地面 {:.0}m / 离地 {:.0}m) | 真空速 {:.0}km/h\r\n\
                 ● 姿态校验：机头俯仰 {:+.1}° (俯冲为负) | 横滚 {:+.1}° | 攻角 {:+.1}°\r\n\
                 ● 视觉锚点：{lock_str}\r\n\
                 ● 视口对齐：{}×{} @(X={},Y={}) | FOV {:.0}° ({})\r\n\
                 ● 琥珀金圆环 [航弹]：{b_str}\r\n\
                 ● 天青蓝菱形 [火箭]：{r_str}\r\n\
                 {hk_summary}",
                ext.alt_asl_m,
                g_elev,
                (ext.alt_asl_m - g_elev).max(0.0),
                ext.tas_mps * 3.6,
                ext.pitch_deg,
                ext.roll_deg,
                telem.aoa_deg,
                view_w,
                view_h,
                cached_vp.vp_x,
                cached_vp.vp_y,
                current_fov_deg,
                if zoomed { "30°放大态" } else { "90°广角" },
            );
            panel.set_status_text(&status);
        }

        frame_no = frame_no.wrapping_add(1);
        precise_frame_sleep(target_frame_dur, frame_start);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_world_camera_coaster_monotonic_reentry() {
        let t0 = Instant::now();
        let mut c = WorldCameraCoaster::new(100.0, -20.0, t0);
        // 连续跟踪两帧建立 ~40 deg/s 向右甩镜角速度
        let _ = c.on_optical_lock(100.8, -20.0, t0 + Duration::from_millis(20));
        let _ = c.on_optical_lock(101.6, -20.0, t0 + Duration::from_millis(40));

        // 模拟机炮火光遮挡 200ms：世界空间阻尼外推应继续平滑向右推进 (> 104°)
        let (coast_yaw, _) = c
            .coast_at(t0 + Duration::from_millis(240))
            .expect("200ms 应在惯性保持窗内");
        assert!(coast_yaw > 103.5 && coast_yaw < 109.0, "coast_yaw={coast_yaw}");

        // 光学重锁在 108.5°：重入角必须位于 `[coast_yaw, 108.5°]` 之间单调前进，绝不倒退回遮挡前的 101.6°！
        let (re_yaw, _) = c.on_optical_lock(108.5, -20.0, t0 + Duration::from_millis(255));
        assert!(
            re_yaw > coast_yaw && re_yaw <= 108.5,
            "重入角发生了倒退跳变: coast_yaw={coast_yaw}, re_yaw={re_yaw}"
        );
    }
}
