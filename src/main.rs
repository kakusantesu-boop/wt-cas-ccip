#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use image::RgbaImage;

use wt_cas_ccip::atmosphere::Atmosphere;
use wt_cas_ccip::ballistics::{
    AircraftKinematics, BallisticTuning, BodyBasis, Vec3, simulate_bomb_ccip, simulate_rocket_ccip,
};
use wt_cas_ccip::camera_tps::{TpsCameraState, solve_tps_camera_from_nose_pixel};
use wt_cas_ccip::capture_win::{
    GlobalHotkeys, ScreenCenterCapturer, enable_per_monitor_dpi_v2, primary_screen_size,
    query_war_thunder_client_viewport,
};
use wt_cas_ccip::db::CasDatabase;
use wt_cas_ccip::hud_render::{
    DirtyEnvelope, HudScene, ReticleTarget, render_hud_incremental, render_hud_into_bgra,
};
use wt_cas_ccip::overlay_win::{ControlPanelWindow, LayeredHudWindow, UiEvent, drain_ui_events};
use wt_cas_ccip::telemetry::{TelemetryPoller, build_demo_telemetry_and_nose_px};
use wt_cas_ccip::terrain_pack::{TerrainSource, WtvPackReader};
use wt_cas_ccip::vision::{CrosshairColorPreset, detect_crosshair_bgra};

const CONV_DIST_OPTIONS: [f64; 6] = [300.0, 400.0, 500.0, 600.0, 700.0, 800.0];
const DRAG_SCALE_OPTIONS: [f64; 7] = [0.75, 0.85, 0.95, 1.00, 1.05, 1.15, 1.30];
const COLOR_PRESETS: [CrosshairColorPreset; 5] = [
    CrosshairColorPreset::Red,
    CrosshairColorPreset::Magenta,
    CrosshairColorPreset::Cyan,
    CrosshairColorPreset::NeonGreen,
    CrosshairColorPreset::Yellow,
];

const FLAT_PRESETS: [(&str, Option<f64>); 5] = [
    ("[自动雷达/跑道海拔 (按F7可低空一键标定)]", None),
    ("[平地基准：海平面 0m (航母/海岛试飞)]", Some(0.0)),
    ("[平地基准：平原试飞场 65m]", Some(65.0)),
    ("[平地基准：内陆丘陵试飞场 150m]", Some(150.0)),
    ("[平地基准：高原机场 300m]", Some(300.0)),
];

#[inline]
fn shortest_yaw_diff(a_deg: f64, b_deg: f64) -> f64 {
    let mut d = (a_deg - b_deg) % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d < -180.0 {
        d += 360.0;
    }
    d
}

fn run_headless_self_test(preview_out: Option<PathBuf>) -> Result<()> {
    let db = CasDatabase::load_embedded_or_disk(None)?;
    let pack = WtvPackReader::auto_discover()
        .ok_or_else(|| anyhow::anyhow!("未找到婴儿弹道计算机内置地形包"))?;
    let grid = Arc::new(pack.load_map("avg_ardennes")?);
    let cx = 0.5 * (grid.ox + grid.x1);
    let cz = 0.5 * (grid.oz + grid.z1);
    let ground_elev = grid.height_at(cx, cz);
    let terrain = TerrainSource::Grid {
        grid: Arc::clone(&grid),
        fallback_alt_m: ground_elev,
    };

    let (w, h) = (1920usize, 1080usize);
    let ac = db.find_aircraft("p-47d-28").unwrap();
    let bomb = db.bombs.get("us_500lb_anm64a1").unwrap();
    let rocket = db.rockets.get("us_5_in_hvar").unwrap();
    let tps_cfg = &db.camera.plane_tps;

    let (telem, gt_nose_px) = build_demo_telemetry_and_nose_px(
        3.2,
        &ac.id,
        tps_cfg,
        ac.tps_offset,
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

    // 同时绘制同色红色鼠标牵引大圆圈 (屏幕正中央) + 半透明红色机头小十字，验证抗干扰红色识别
    for deg in 0..360 {
        let rad = (deg as f64).to_radians();
        let rx = (w as f64 * 0.5 + 38.0 * rad.cos()).round() as usize;
        let ry = (h as f64 * 0.5 + 38.0 * rad.sin()).round() as usize;
        let p = (ry * w + rx) * 4;
        game_bgra[p] = 35;
        game_bgra[p + 1] = 35;
        game_bgra[p + 2] = 245;
    }

    let (nx_i, ny_i) = (gt_nose_px[0].round() as usize, gt_nose_px[1].round() as usize);
    for d in 4..=12 {
        for th in 0..=1 {
            for &(px, py) in &[
                (nx_i - d, ny_i + th),
                (nx_i + d, ny_i + th),
                (nx_i + th, ny_i - d),
                (nx_i + th, ny_i + d),
            ] {
                let p = (py * w + px) * 4;
                game_bgra[p] = 32;
                game_bgra[p + 1] = 32;
                game_bgra[p + 2] = 248;
            }
        }
    }

    let (roi_w, roi_h) = (720usize, 720usize);
    let roi_left = (w - roi_w) / 2;
    let roi_top = (h - roi_h) / 2;
    let mut roi_buf = vec![0u8; roi_w * roi_h * 4];
    for ry in 0..roi_h {
        let src_off = ((roi_top + ry) * w + roi_left) * 4;
        let dst_off = ry * roi_w * 4;
        roi_buf[dst_off..dst_off + roi_w * 4]
            .copy_from_slice(&game_bgra[src_off..src_off + roi_w * 4]);
    }
    let det = detect_crosshair_bgra(&roi_buf, roi_w, roi_h, CrosshairColorPreset::Red, 0)
        .ok_or_else(|| anyhow::anyhow!("自检：未能从模拟画面识别红色机头准星"))?;
    let det_u = roi_left as f64 + det.x_roi;
    let det_v = roi_top as f64 + det.y_roi;

    let ac_pos = Vec3::new(cx, telem.alt_asl_m, cz - 450.0);
    let cam = solve_tps_camera_from_nose_pixel(
        ac_pos,
        telem.heading_deg,
        telem.pitch_deg,
        telem.roll_deg,
        ac.static_aim_pitch_deg,
        500.0,
        telem.stick_elevator,
        telem.stick_rudder,
        det_u,
        det_v,
        tps_cfg,
        ac.tps_offset,
        tps_cfg.fov,
        w as f64,
        h as f64,
    )
    .ok_or_else(|| anyhow::anyhow!("自检：TPS 相机反解未收敛"))?;

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
    let atm = Atmosphere::standard_15c();
    let tuning = BallisticTuning::default();

    let sol_bomb = simulate_bomb_ccip(&kin, bomb, &atm, &terrain, &tuning)
        .ok_or_else(|| anyhow::anyhow!("自检：航弹弹道未命中地面"))?;
    let sol_rkt = simulate_rocket_ccip(&kin, rocket, 0.0, &atm, &terrain, &tuning)
        .ok_or_else(|| anyhow::anyhow!("自检：火箭弹弹道未命中地面"))?;

    let bomb_xy = cam
        .project_world_to_screen(sol_bomb.impact_pos)
        .ok_or_else(|| anyhow::anyhow!("自检：航弹落点不在相机前方"))?;
    let rkt_xy = cam
        .project_world_to_screen(sol_rkt.impact_pos)
        .ok_or_else(|| anyhow::anyhow!("自检：火箭弹落点不在相机前方"))?;

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
            time_of_flight_s: sol_bomb.time_of_flight_s,
            slant_range_m: sol_bomb.slant_range_m,
            tag: "AN-M64",
        }),
        rocket_ccip: Some(ReticleTarget {
            screen_xy: rkt_xy,
            time_of_flight_s: sol_rkt.time_of_flight_s,
            slant_range_m: sol_rkt.slant_range_m,
            tag: "HVAR",
        }),
        hud_banner: format!(
            "TPS CCIP LOCKED | P-47D-28 | ALT {:.0}M | PITCH {:.1} | TAS {:.0}KM/H",
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
            let ha = hud_bgra[p + 3] as f32 / 255.0;
            let r = (hud_bgra[p + 2] as f32 + game_bgra[p + 2] as f32 * (1.0 - ha))
                .clamp(0.0, 255.0) as u8;
            let g = (hud_bgra[p + 1] as f32 + game_bgra[p + 1] as f32 * (1.0 - ha))
                .clamp(0.0, 255.0) as u8;
            let b = (hud_bgra[p] as f32 + game_bgra[p] as f32 * (1.0 - ha))
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

    let report = serde_json::json!({
        "status": "ok",
        "datamine_version": db.datamine_version,
        "nose_anchor_gt_px": gt_nose_px,
        "nose_anchor_detected_px": [det_u, det_v],
        "solved_cam_yaw_pitch_deg": [cam.yaw_deg, cam.pitch_deg],
        "bomb_ccip_px": bomb_xy,
        "rocket_ccip_px": rkt_xy,
    });
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

/// 惯性保持状态：当火光/云层短暂遮挡准星 1~25 帧 (<450ms) 时保持相机相对偏角连续不断火，彻底消灭闪烁！
#[derive(Clone, Copy, Debug)]
struct CoastingCameraLock {
    delta_yaw_deg: f64,
    delta_pitch_deg: f64,
    last_optical_hit: Instant,
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
    let (mon_w, mon_h) = primary_screen_size();

    let exe_beside_db = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.join("cas_db.json")));
    let db = CasDatabase::load_embedded_or_disk(exe_beside_db.as_deref())?;
    let map_pack = WtvPackReader::auto_discover();

    let panel = ControlPanelWindow::create()
        .ok_or_else(|| anyhow::anyhow!("创建中文控制台窗口失败"))?;
    let mut hud_win = LayeredHudWindow::create(mon_w, mon_h)
        .ok_or_else(|| anyhow::anyhow!("创建全屏透明穿透图层失败"))?;
    let mut dirty_env = DirtyEnvelope::default();
    let mut capturer = ScreenCenterCapturer::new(768, 768);
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

    let ac_labels: Vec<String> = db
        .aircraft
        .iter()
        .map(|a| {
            let tag = if a.ccip_bombs || a.ccip_rockets {
                " ★原生CCIP对照"
            } else {
                ""
            };
            format!("{} ({}){tag}", a.name_zh, a.id)
        })
        .collect();
    let mut cur_ac_idx = db
        .aircraft
        .iter()
        .position(|a| a.id == "p-47d-28")
        .or_else(|| db.aircraft.iter().position(|a| a.id == "a_10a_early"))
        .unwrap_or(0);
    panel.set_combo_items(panel.cb_aircraft, &ac_labels, cur_ac_idx);

    let color_labels: Vec<String> = COLOR_PRESETS.iter().map(|c| c.label_zh().to_string()).collect();
    // 默认选中第 0 项“红色”（也可一键切品红/青/绿/黄）
    let mut color_idx = 0usize;
    panel.set_combo_items(panel.cb_color, &color_labels, color_idx);

    let conv_labels: Vec<String> = CONV_DIST_OPTIONS.iter().map(|d| format!("{d:.0} 米")).collect();
    let mut conv_idx = 2usize;
    panel.set_combo_items(panel.cb_conv, &conv_labels, conv_idx);

    let drag_labels: Vec<String> = DRAG_SCALE_OPTIONS.iter().map(|s| format!("{s:.2}x")).collect();
    let mut drag_idx = 3usize;
    panel.set_combo_items(panel.cb_drag, &drag_labels, drag_idx);

    let mut auto_ac = true;
    let mut overlay_enabled = true;
    let mut demo_mode = false;
    let mut rmb_zoom = false;
    let mut bay_delay_on = false;
    panel.set_checked(panel.chk_auto_ac, auto_ac);
    panel.set_checked(panel.chk_overlay, overlay_enabled);
    panel.set_checked(panel.chk_demo, demo_mode);
    panel.set_checked(panel.chk_rmb_zoom, rmb_zoom);
    panel.set_checked(panel.chk_bay_delay, bay_delay_on);

    let mut cur_bomb_ids: Vec<Option<String>> = Vec::new();
    let mut cur_rocket_ids: Vec<(Option<String>, f64)> = Vec::new();
    let mut sel_bomb_idx = 0usize;
    let mut sel_rocket_idx = 0usize;

    let refresh_weapons_for_ac = |ac_idx: usize,
                                  cur_bomb_ids: &mut Vec<Option<String>>,
                                  cur_rocket_ids: &mut Vec<(Option<String>, f64)>,
                                  sel_bomb_idx: &mut usize,
                                  sel_rocket_idx: &mut usize| {
        let ac = &db.aircraft[ac_idx];
        cur_bomb_ids.clear();
        let mut b_labels = vec!["[关闭航弹 CCIP]".to_string()];
        cur_bomb_ids.push(None);
        for mb in &ac.bombs {
            if let Some(bs) = db.bombs.get(&mb.id) {
                let kind = if bs.brake_cx_k > 1.5 {
                    "·减速弹"
                } else if bs.is_napalm {
                    "·燃烧弹"
                } else {
                    ""
                };
                b_labels.push(format!("{} ({:.0}kg{kind})", bs.name_zh, bs.mass));
                cur_bomb_ids.push(Some(mb.id.clone()));
            }
        }
        *sel_bomb_idx = usize::from(cur_bomb_ids.len() > 1);
        panel.set_combo_items(panel.cb_bomb, &b_labels, *sel_bomb_idx);

        cur_rocket_ids.clear();
        let mut r_labels = vec!["[关闭火箭弹 CCIP]".to_string()];
        cur_rocket_ids.push((None, 0.0));
        for mr in &ac.rockets {
            if let Some(rs) = db.rockets.get(&mr.id) {
                r_labels.push(format!("{} (CxK={:.2})", rs.name_zh, rs.cx_k));
                cur_rocket_ids.push((Some(mr.id.clone()), mr.launcher_pitch_deg));
            }
        }
        *sel_rocket_idx = usize::from(cur_rocket_ids.len() > 1);
        panel.set_combo_items(panel.cb_rocket, &r_labels, *sel_rocket_idx);
    };

    refresh_weapons_for_ac(
        cur_ac_idx,
        &mut cur_bomb_ids,
        &mut cur_rocket_ids,
        &mut sel_bomb_idx,
        &mut sel_rocket_idx,
    );

    let mut manual_calib_alt_m: Option<f64> = None;
    let mut loaded_grid_terrain: Option<TerrainSource> = None;
    let mut z_zoom_toggled = false;
    let mut current_fov_deg = db.camera.plane_tps.fov;
    let mut smoothed_nose_px: Option<[f64; 2]> = None;
    let mut coast_lock: Option<CoastingCameraLock> = None;
    let mut cached_viewport = (0i32, 0i32, mon_w, mon_h);
    let mut last_vp_check = Instant::now() - Duration::from_secs(2);
    let mut frame_no: u64 = 0;
    let start_instant = Instant::now();
    let mut last_status_refresh = Instant::now() - Duration::from_secs(1);

    // 稳态 ~65 FPS 渲染同步循环 (与 DWM 合成节奏对齐，消除高频抢占闪烁)
    loop {
        let frame_start = Instant::now();
        if !ControlPanelWindow::pump_messages() {
            break;
        }

        let mut should_quit = false;
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
                UiEvent::AircraftChanged(idx) if idx < db.aircraft.len() => {
                    cur_ac_idx = idx;
                    refresh_weapons_for_ac(
                        cur_ac_idx,
                        &mut cur_bomb_ids,
                        &mut cur_rocket_ids,
                        &mut sel_bomb_idx,
                        &mut sel_rocket_idx,
                    );
                }
                UiEvent::BombChanged(idx) if idx < cur_bomb_ids.len() => sel_bomb_idx = idx,
                UiEvent::RocketChanged(idx) if idx < cur_rocket_ids.len() => sel_rocket_idx = idx,
                UiEvent::ColorChanged(idx) if idx < COLOR_PRESETS.len() => {
                    color_idx = idx;
                    smoothed_nose_px = None;
                    coast_lock = None;
                }
                UiEvent::ConvChanged(idx) if idx < CONV_DIST_OPTIONS.len() => conv_idx = idx,
                UiEvent::DragChanged(idx) if idx < DRAG_SCALE_OPTIONS.len() => drag_idx = idx,
                UiEvent::ToggleAutoAc(on) => auto_ac = on,
                UiEvent::ToggleOverlay(on) => {
                    overlay_enabled = on;
                    if !overlay_enabled {
                        hud_win.frame_buffer_mut().fill(0);
                        dirty_env = DirtyEnvelope::default();
                    }
                    hud_win.set_visible(overlay_enabled);
                }
                UiEvent::ToggleDemo(on) => {
                    demo_mode = on;
                    smoothed_nose_px = None;
                    coast_lock = None;
                }
                UiEvent::ToggleRmbZoom(on) => rmb_zoom = on,
                UiEvent::ToggleBayDelay(on) => bay_delay_on = on,
                UiEvent::FlipZoom => z_zoom_toggled = !z_zoom_toggled,
                UiEvent::Quit => should_quit = true,
                _ => {}
            }
        }
        if should_quit {
            break;
        }

        if last_vp_check.elapsed() >= Duration::from_millis(500) {
            last_vp_check = Instant::now();
            cached_viewport = if demo_mode {
                (0, 0, mon_w, mon_h)
            } else {
                query_war_thunder_client_viewport().unwrap_or((0, 0, mon_w, mon_h))
            };
        }
        let (vp_x, vp_y, vp_w, vp_h) = cached_viewport;
        let view_w = vp_w.clamp(320, mon_w);
        let view_h = vp_h.clamp(240, mon_h);

        let hk = hotkeys.poll();
        if hk.toggle_zoom_z || hk.resync_zoom_f8 {
            z_zoom_toggled = !z_zoom_toggled;
        }
        if hk.toggle_overlay_f9 {
            overlay_enabled = !overlay_enabled;
            panel.set_checked(panel.chk_overlay, overlay_enabled);
            if !overlay_enabled {
                hud_win.frame_buffer_mut().fill(0);
                dirty_env = DirtyEnvelope::default();
            }
            hud_win.set_visible(overlay_enabled);
        }
        if hk.toggle_demo_f10 {
            demo_mode = !demo_mode;
            panel.set_checked(panel.chk_demo, demo_mode);
            smoothed_nose_px = None;
            coast_lock = None;
        }

        let ac_now = &db.aircraft[cur_ac_idx];
        let tps_cfg = if ac_now.is_heli {
            &db.camera.heli_tps_alt
        } else {
            &db.camera.plane_tps
        };
        let zoomed = z_zoom_toggled || (rmb_zoom && hk.hold_rmb);
        let target_fov = if zoomed { tps_cfg.zoom_fov } else { tps_cfg.fov };
        current_fov_deg += (target_fov - current_fov_deg) * 0.32;

        let conv_dist_m = CONV_DIST_OPTIONS[conv_idx];
        let tuning = BallisticTuning {
            bomb_drag_scale: DRAG_SCALE_OPTIONS[drag_idx],
            bay_door_delay_s: if bay_delay_on { 0.60 } else { 0.0 },
            ..Default::default()
        };

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
                ac_now.static_aim_pitch_deg,
                conv_dist_m,
                current_fov_deg,
                view_w as f64,
                view_h as f64,
                center_xz,
                g_alt,
            );
            (tf, terr, Some(sim_px))
        } else {
            let tf = poller.snapshot();
            if hk.calibrate_ground_f7 && tf.online {
                manual_calib_alt_m = Some((tf.alt_asl_m - 15.0).max(0.0));
            }

            if tf.online && auto_ac && !tf.unit_id.is_empty() && tf.unit_id != ac_now.id {
                if let Some(found_idx) = db.aircraft.iter().position(|a| a.id == tf.unit_id) {
                    cur_ac_idx = found_idx;
                    panel.set_combo_sel(panel.cb_aircraft, cur_ac_idx);
                    refresh_weapons_for_ac(
                        cur_ac_idx,
                        &mut cur_bomb_ids,
                        &mut cur_rocket_ids,
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
                    // 已锁定时先查 260×260 跟踪小窗；未命中当帧立即查 768×768 大窗
                    let track_hit = smoothed_nose_px.and_then(|[pu, pv]| {
                        let cx_scr = vp_x + pu.round() as i32;
                        let cy_scr = vp_y + pv.round() as i32;
                        let ((lx, ty, rw, rh), sub_buf) =
                            cap.capture_around(cx_scr, cy_scr, 260, 260)?;
                        let d = detect_crosshair_bgra(&sub_buf, rw, rh, preset_col, 6)?;
                        Some([(lx - vp_x) as f64 + d.x_roi, (ty - vp_y) as f64 + d.y_roi])
                    });

                    track_hit.or_else(|| {
                        let cx_scr = vp_x + view_w / 2;
                        let cy_scr = vp_y + view_h / 2;
                        let ((lx, ty, rw, rh), full_buf) =
                            cap.capture_around(cx_scr, cy_scr, 768, 768)?;
                        let d = detect_crosshair_bgra(&full_buf, rw, rh, preset_col, 6)?;
                        Some([(lx - vp_x) as f64 + d.x_roi, (ty - vp_y) as f64 + d.y_roi])
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
        let [wx, wz] = telem.map_xz_m.unwrap_or([0.0, 0.0]);
        let ac_pos = Vec3::new(wx, telem.alt_asl_m, wz);
        let now_inst = Instant::now();
        let (ext_hdg, ext_pitch, ext_roll) = if demo_mode {
            (telem.heading_deg, telem.pitch_deg, telem.roll_deg)
        } else {
            telem.extrapolated_attitude(now_inst)
        };

        // 结合光学命中与 450ms 惯性保持，彻底消灭丢帧引起的准星频闪！
        let mut active_cam: Option<TpsCameraState> = None;
        let mut is_coasting = false;

        if telem.online {
            if let Some(cur_px) = raw_optical_px {
                let filt_px = match smoothed_nose_px {
                    Some(prev) => {
                        let dist = (cur_px[0] - prev[0]).hypot(cur_px[1] - prev[1]);
                        let alpha = if dist > 8.0 { 0.88 } else { 0.52 };
                        [
                            prev[0] * (1.0 - alpha) + cur_px[0] * alpha,
                            prev[1] * (1.0 - alpha) + cur_px[1] * alpha,
                        ]
                    }
                    None => cur_px,
                };
                smoothed_nose_px = Some(filt_px);

                if let Some(cam) = solve_tps_camera_from_nose_pixel(
                    ac_pos,
                    ext_hdg,
                    ext_pitch,
                    ext_roll,
                    ac_active.static_aim_pitch_deg,
                    conv_dist_m,
                    telem.stick_elevator,
                    telem.stick_rudder,
                    filt_px[0],
                    filt_px[1],
                    tps_cfg,
                    ac_active.tps_offset,
                    current_fov_deg,
                    view_w as f64,
                    view_h as f64,
                ) {
                    coast_lock = Some(CoastingCameraLock {
                        delta_yaw_deg: shortest_yaw_diff(cam.yaw_deg, ext_hdg),
                        delta_pitch_deg: cam.pitch_deg - ext_pitch,
                        last_optical_hit: now_inst,
                    });
                    active_cam = Some(cam);
                }
            } else if let Some(cl) = coast_lock {
                // 光学短暂遮挡 (< 450ms)：沿用上一帧相机相对机身角偏移 + 实时 8111 姿态惯性维持，不闪烁！
                if now_inst.saturating_duration_since(cl.last_optical_hit)
                    <= Duration::from_millis(450)
                {
                    let c_yaw = (ext_hdg + cl.delta_yaw_deg).rem_euclid(360.0);
                    let c_pitch = (ext_pitch + cl.delta_pitch_deg).clamp(-88.5, 88.5);
                    let cam = TpsCameraState::build(
                        ac_pos,
                        c_yaw,
                        c_pitch,
                        ext_roll,
                        telem.stick_elevator,
                        telem.stick_rudder,
                        tps_cfg,
                        ac_active.tps_offset,
                        current_fov_deg,
                        view_w as f64,
                        view_h as f64,
                    );
                    let ac_basis = BodyBasis::from_euler_deg(ext_hdg, ext_pitch, ext_roll);
                    let bore_pt =
                        ac_pos + ac_basis.pitched_axis(ac_active.static_aim_pitch_deg) * conv_dist_m;
                    smoothed_nose_px = cam.project_world_to_screen(bore_pt);
                    active_cam = Some(cam);
                    is_coasting = true;
                } else {
                    smoothed_nose_px = None;
                    coast_lock = None;
                }
            } else {
                smoothed_nose_px = None;
            }
        } else {
            smoothed_nose_px = None;
            coast_lock = None;
        }

        let mut bomb_target: Option<ReticleTarget> = None;
        let mut rocket_target: Option<ReticleTarget> = None;

        if let Some(cam) = &active_cam {
            let kin = AircraftKinematics::from_telemetry(
                ac_pos,
                ext_hdg,
                ext_pitch,
                ext_roll,
                telem.tas_mps,
                telem.vy_mps,
                telem.aoa_deg,
                telem.aos_deg,
            );

            if let Some(Some(bid)) = cur_bomb_ids.get(sel_bomb_idx) {
                if let Some(bspec) = db.bombs.get(bid) {
                    if let Some(sol) =
                        simulate_bomb_ccip(&kin, bspec, &atm, &active_terrain, &tuning)
                    {
                        if let Some(xy) = cam.project_world_to_screen(sol.impact_pos) {
                            bomb_target = Some(ReticleTarget {
                                screen_xy: xy,
                                time_of_flight_s: sol.time_of_flight_s,
                                slant_range_m: sol.slant_range_m,
                                tag: "BOMB",
                            });
                        }
                    }
                }
            }

            if let Some((Some(rid), l_pitch)) = cur_rocket_ids.get(sel_rocket_idx) {
                if let Some(rspec) = db.rockets.get(rid) {
                    if let Some(sol) = simulate_rocket_ccip(
                        &kin,
                        rspec,
                        *l_pitch,
                        &atm,
                        &active_terrain,
                        &tuning,
                    ) {
                        if let Some(xy) = cam.project_world_to_screen(sol.impact_pos) {
                            rocket_target = Some(ReticleTarget {
                                screen_xy: xy,
                                time_of_flight_s: sol.time_of_flight_s,
                                slant_range_m: sol.slant_range_m,
                                tag: "RKT",
                            });
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
                    telem.alt_asl_m,
                    ext_pitch,
                    current_fov_deg
                )
            } else if !telem.online {
                String::new()
            } else if nose_locked {
                format!(
                    "CCIP {} | ALT {:.0}M | PITCH {:.1} | TAS {:.0}KM/H",
                    if is_coasting { "HOLD" } else { "LOCK" },
                    telem.alt_asl_m,
                    ext_pitch,
                    telem.tas_mps * 3.6
                )
            } else {
                "CCIP STANDBY - AWAITING NOSE CROSSHAIR".to_string()
            };

            let scene = HudScene {
                canvas_w: mon_w as u32,
                canvas_h: mon_h as u32,
                vp_x: vp_x as f64,
                vp_y: vp_y as f64,
                vp_w: view_w as f64,
                vp_h: view_h as f64,
                demo_mode,
                zoomed,
                fov_deg: current_fov_deg,
                nose_locked,
                nose_xy: smoothed_nose_px,
                bomb_ccip: bomb_target.clone(),
                rocket_ccip: rocket_target.clone(),
                hud_banner,
            };
            render_hud_incremental(&scene, hud_win.frame_buffer_mut(), &mut dirty_env);
            hud_win.present_fullscreen(frame_no % 180 == 0);
        }

        if last_status_refresh.elapsed() >= Duration::from_millis(180) {
            last_status_refresh = Instant::now();
            let g_elev = active_terrain.height_at(wx, wz);
            let conn_str = if demo_mode {
                "【内置动态三维俯冲演示运行中 (按 F10 切回实机 8111)】"
            } else if telem.online {
                "【已连接战雷 127.0.0.1:8111 实时遥测 (零闪烁惯性滤波已开启)】"
            } else {
                "【等待战雷对局遥测... (可勾选上方“内置演示模式”直接预览准星效果)】"
            };
            let lock_str = match (smoothed_nose_px, active_cam.as_ref()) {
                (Some([u, v]), Some(c)) => format!(
                    "{} ({u:.0}, {v:.0}) → 相机 航向={:.1}° 俯仰={:+.1}°",
                    if is_coasting { "惯性维持" } else { "光学锁定" },
                    c.yaw_deg,
                    c.pitch_deg
                ),
                _ if telem.online => {
                    "未锁定 (已支持红/品红/青/绿/黄准星，请核对面板所选准星颜色)".to_string()
                }
                _ => "待机中 (未进入飞行状态时屏幕保持净空)".to_string(),
            };
            let b_str = bomb_target.map_or("未启用/超界".to_string(), |b| {
                format!(
                    "屏幕({:.0},{:.0}) · 飞行 {:.1}s · 斜距 {:.0}m",
                    b.screen_xy[0], b.screen_xy[1], b.time_of_flight_s, b.slant_range_m
                )
            });
            let r_str = rocket_target.map_or("未启用/超界".to_string(), |r| {
                format!(
                    "屏幕({:.0},{:.0}) · 飞行 {:.1}s · 斜距 {:.0}m",
                    r.screen_xy[0], r.screen_xy[1], r.time_of_flight_s, r.slant_range_m
                )
            });

            let status = format!(
                "{conn_str}\r\n\
                 ● 飞行姿态：海拔 {:.0}m (地面海拔 {:.0}m / 真实离地 {:.0}m) | 空速 {:.0}km/h\r\n\
                 ● 姿态校验：机头俯仰 {:+.1}° (俯冲为负) | 横滚 {:+.1}° | 攻角 {:+.1}°\r\n\
                 ● 视觉锚点：{lock_str}\r\n\
                 ● 视口对齐：{}×{} @(X={},Y={}) | FOV {:.0}° ({})\r\n\
                 ● 琥珀金圆环 [航弹]：{b_str}\r\n\
                 ● 天青蓝菱形 [火箭]：{r_str}\r\n\
                 快捷键：Z=切换缩放 | F7=低空一键标定地面高 | F8=翻转缩放 | F9=显隐 | F10=演示",
                telem.alt_asl_m,
                g_elev,
                (telem.alt_asl_m - g_elev).max(0.0),
                telem.tas_mps * 3.6,
                ext_pitch,
                ext_roll,
                telem.aoa_deg,
                view_w,
                view_h,
                vp_x,
                vp_y,
                current_fov_deg,
                if zoomed { "30°放大态" } else { "90°广角" },
            );
            panel.set_status_text(&status);
        }

        frame_no = frame_no.wrapping_add(1);
        let elapsed = frame_start.elapsed();
        if elapsed < Duration::from_millis(15) {
            std::thread::sleep(Duration::from_millis(15) - elapsed);
        }
    }

    Ok(())
}
