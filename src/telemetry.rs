//! 战雷官方 `http://127.0.0.1:8111` 高频无阻塞遥测采集器 + 姿态角速度平滑外推器
//!
//! 关键防坑规则（实测与开源库 `PowerBroker2/WarThunder` 双重验证）：
//! 1. `/indicators` 中的 `aviahorizon_pitch` 与 `aviahorizon_roll` 均为俄式地平仪刻度盘相对位移，符号与飞机真实俯仰/横滚**完全相反**：
//!    - 飞机俯冲 `-20°` 时，`aviahorizon_pitch` 输出 `+20.0`，必须取反 `-aviahorizon_pitch`！
//!    - 飞机右倾 `+15°` 时，`aviahorizon_roll` 输出 `-15.0`，必须取反 `-aviahorizon_roll`！
//! 2. `/indicators` 中的 `bank` 在美系等飞机上是侧滑仪小球（限幅 `[-8, +8]`），绝不能优先于 `aviahorizon_roll` 当作横滚角！
//! 3. 将低频小地图请求 (`/map_info.json`, `/map_obj.json`) 拆至独立慢速线程，姿态主线程保持 `~80-100 Hz` 持久连接采集，并记录角速度供渲染线程 `120 FPS` 帧间平滑外推，彻底消除 `20 Hz` 阶梯顿挫感。

use std::io::{BufRead, BufReader, Read, Write};
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;

use crate::ballistics::{BodyBasis, Vec3};
use crate::camera_tps::TpsCameraState;
use crate::db::TpsCameraConfig;

#[derive(Clone, Debug)]
pub struct TelemetryFrame {
    pub online: bool,
    pub unit_id: String,
    pub alt_asl_m: f64,
    pub tas_mps: f64,
    pub ias_mps: f64,
    pub mach: f64,
    pub vy_mps: Option<f64>,
    pub aoa_deg: f64,
    pub aos_deg: f64,
    pub heading_deg: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    /// 角速度 (deg/s)，用于 120 FPS 渲染帧间无顿挫外推
    pub yaw_rate_dps: f64,
    pub pitch_rate_dps: f64,
    pub roll_rate_dps: f64,
    pub stick_elevator: f64,
    pub stick_rudder: f64,
    pub airbrake_pct: f64,
    /// 无线电高度表推算的实时地面海拔 (若该机配备无线电高度表且在量程内)
    pub radar_ground_alt_m: Option<f64>,
    /// 跑道滑跑自动记录的机场海拔
    pub auto_runway_alt_m: Option<f64>,
    pub map_xz_m: Option<[f64; 2]>,
    pub map_bounds: Option<[f64; 4]>,
    pub sampled_at: Option<Instant>,
}

impl Default for TelemetryFrame {
    fn default() -> Self {
        Self {
            online: false,
            unit_id: String::new(),
            alt_asl_m: 500.0,
            tas_mps: 0.0,
            ias_mps: 0.0,
            mach: 0.0,
            vy_mps: None,
            aoa_deg: 0.0,
            aos_deg: 0.0,
            heading_deg: 0.0,
            pitch_deg: 0.0,
            roll_deg: 0.0,
            yaw_rate_dps: 0.0,
            pitch_rate_dps: 0.0,
            roll_rate_dps: 0.0,
            stick_elevator: 0.0,
            stick_rudder: 0.0,
            airbrake_pct: 0.0,
            radar_ground_alt_m: None,
            auto_runway_alt_m: None,
            map_xz_m: None,
            map_bounds: None,
            sampled_at: None,
        }
    }
}

impl TelemetryFrame {
    /// 按当前渲染时刻外推平滑后的 (heading, pitch, roll)，上限外推 45ms 防止过冲
    pub fn extrapolated_attitude(&self, now: Instant) -> (f64, f64, f64) {
        let Some(t0) = self.sampled_at else {
            return (self.heading_deg, self.pitch_deg, self.roll_deg);
        };
        let dt = now.saturating_duration_since(t0).as_secs_f64().clamp(0.0, 0.045);
        let hdg = (self.heading_deg + self.yaw_rate_dps * dt).rem_euclid(360.0);
        let pitch = (self.pitch_deg + self.pitch_rate_dps * dt).clamp(-89.0, 89.0);
        let roll = self.roll_deg + self.roll_rate_dps * dt;
        (hdg, pitch, roll)
    }
}

/// 支持 HTTP/1.1 Keep-Alive 复用的轻量客户端，将单次轮询延迟压至 < 0.5 ms
struct KeepAlive8111 {
    stream: Option<BufReader<TcpStream>>,
    timeout: Duration,
}

impl KeepAlive8111 {
    fn new(timeout_ms: u64) -> Self {
        Self {
            stream: None,
            timeout: Duration::from_millis(timeout_ms),
        }
    }

    fn get_json(&mut self, path: &str) -> Option<Value> {
        if let Some(res) = self.try_request(path) {
            return Some(res);
        }
        self.stream = None;
        self.try_request(path)
    }

    fn try_request(&mut self, path: &str) -> Option<Value> {
        if self.stream.is_none() {
            let addr = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 8111));
            let s = TcpStream::connect_timeout(&addr, self.timeout).ok()?;
            s.set_nodelay(true).ok()?;
            s.set_read_timeout(Some(self.timeout)).ok()?;
            s.set_write_timeout(Some(self.timeout)).ok()?;
            self.stream = Some(BufReader::with_capacity(8192, s));
        }

        let reader = self.stream.as_mut()?;
        let req = format!(
            "GET {path} HTTP/1.1\r\nHost: 127.0.0.1:8111\r\nConnection: keep-alive\r\n\r\n"
        );
        if reader.get_mut().write_all(req.as_bytes()).is_err() {
            self.stream = None;
            return None;
        }

        let mut content_len: Option<usize> = None;
        let mut close_conn = false;
        let mut line = String::new();

        // 状态行
        if reader.read_line(&mut line).ok()? == 0 || !line.contains("200") {
            self.stream = None;
            return None;
        }
        loop {
            line.clear();
            if reader.read_line(&mut line).ok()? == 0 {
                self.stream = None;
                return None;
            }
            let t = line.trim();
            if t.is_empty() {
                break;
            }
            if let Some((k, v)) = t.split_once(':') {
                if k.eq_ignore_ascii_case("content-length") {
                    content_len = v.trim().parse::<usize>().ok();
                } else if k.eq_ignore_ascii_case("connection")
                    && v.to_ascii_lowercase().contains("close")
                {
                    close_conn = true;
                }
            }
        }

        let body_bytes = if let Some(n) = content_len {
            let mut b = vec![0u8; n];
            if reader.read_exact(&mut b).is_err() {
                self.stream = None;
                return None;
            }
            b
        } else {
            let mut b = Vec::new();
            let _ = reader.read_to_end(&mut b);
            close_conn = true;
            b
        };

        if close_conn {
            self.stream = None;
        }
        serde_json::from_slice(&body_bytes).ok()
    }
}

pub fn val_f64(v: &Value, key: &str) -> Option<f64> {
    match v.get(key)? {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.trim().parse::<f64>().ok(),
        _ => None,
    }
}

#[inline]
fn shortest_angle_diff_deg(to_deg: f64, from_deg: f64) -> f64 {
    let mut d = (to_deg - from_deg) % 360.0;
    if d > 180.0 {
        d -= 360.0;
    } else if d < -180.0 {
        d += 360.0;
    }
    d
}

/// 从 `/state` 与 `/indicators` 原始 JSON 解析出物理符号全部对齐的瞬时飞行参数（抽出独立纯函数便于严格单测）
pub fn parse_state_and_indicators(sj: &Value, ij: &Value) -> TelemetryFrame {
    let unit_id = ij
        .get("type")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_lowercase();
    let alt_asl_m = val_f64(sj, "H, m").unwrap_or(0.0);
    let tas_mps = val_f64(sj, "TAS, km/h").unwrap_or(0.0) / 3.6;
    let ias_mps = val_f64(sj, "IAS, km/h").unwrap_or(0.0) / 3.6;
    let mach = val_f64(sj, "M").or_else(|| val_f64(ij, "mach")).unwrap_or(0.0);
    let vy_mps = val_f64(sj, "Vy, m/s").or_else(|| val_f64(ij, "vario"));
    let aoa_deg = val_f64(sj, "AoA, deg").unwrap_or(0.0);
    let aos_deg = val_f64(sj, "AoS, deg").unwrap_or(0.0);

    let heading_deg = val_f64(ij, "compass1")
        .or_else(|| val_f64(ij, "compass"))
        .unwrap_or(0.0)
        .rem_euclid(360.0);

    // 横滚角：必须优先读 `-aviahorizon_roll`（量程 ±180°），绝不能先读 `bank`（美系飞机上为 ±8° 侧滑仪小球！）
    let roll_deg = val_f64(ij, "aviahorizon_roll")
        .or_else(|| val_f64(ij, "aviahorizon_roll1"))
        .map(|r| -r)
        .or_else(|| val_f64(ij, "bank"))
        .unwrap_or(0.0);

    // 俯仰角：战雷 `aviahorizon_pitch` 抬头为负、俯冲为正，必须取反 `-aviahorizon_pitch`！
    // 若无座舱机型缺失该字段，则由航迹倾角 arcsin(Vy/TAS) + AoA*cos(roll) 精确重构。
    let kin_pitch_deg = if let Some(vy) = vy_mps {
        if tas_mps > 5.0 {
            let gamma_deg = (vy / tas_mps).clamp(-1.0, 1.0).asin().to_degrees();
            Some(gamma_deg + aoa_deg * roll_deg.to_radians().cos())
        } else {
            None
        }
    } else {
        None
    };

    let pitch_deg = val_f64(ij, "aviahorizon_pitch")
        .or_else(|| val_f64(ij, "aviahorizon_pitch1"))
        .map(|p| -p)
        .or(kin_pitch_deg)
        .unwrap_or(0.0);

    let roll_rate_dps = val_f64(sj, "Wx, deg/s").unwrap_or(0.0);
    let stick_elevator = val_f64(ij, "stick_elevator").unwrap_or(0.0);
    let stick_rudder = val_f64(ij, "pedals")
        .or_else(|| val_f64(ij, "pedals1"))
        .unwrap_or(0.0);
    let airbrake_pct = val_f64(sj, "airbrake, %")
        .or_else(|| val_f64(ij, "airbrake_lever").map(|v| v * 100.0))
        .unwrap_or(0.0);

    // 若该机型带无线电高度表 (`radio_altitude`)，自动识别英尺/米制并反推脚下真实地面海拔
    let radar_ground_alt_m = val_f64(ij, "radio_altitude").and_then(|ra_raw| {
        if ra_raw <= 0.5 {
            return None;
        }
        // 对照气压表 altitude_10k 与 H, m 自动判定座舱仪表是否为英尺制 (koef ≈ 3.28084)
        let is_feet = val_f64(ij, "altitude_10k")
            .filter(|_| alt_asl_m > 40.0)
            .is_some_and(|a10k| ((a10k / alt_asl_m) - 3.28084).abs() < 0.45);
        let ra_m = if is_feet { ra_raw * 0.3048 } else { ra_raw };
        // 排除满量程钉死值 (通常 1500m 或 5000ft ≈ 1524m)
        if ra_m > 1.5 && ra_m < 1420.0 {
            Some(alt_asl_m - ra_m)
        } else {
            None
        }
    });

    TelemetryFrame {
        online: true,
        unit_id,
        alt_asl_m,
        tas_mps,
        ias_mps,
        mach,
        vy_mps,
        aoa_deg,
        aos_deg,
        heading_deg,
        pitch_deg,
        roll_deg,
        yaw_rate_dps: 0.0,
        pitch_rate_dps: 0.0,
        roll_rate_dps,
        stick_elevator,
        stick_rudder,
        airbrake_pct,
        radar_ground_alt_m,
        auto_runway_alt_m: None,
        map_xz_m: None,
        map_bounds: None,
        sampled_at: Some(Instant::now()),
    }
}

#[derive(Default, Clone, Copy)]
struct MapPositionState {
    bounds: Option<[f64; 4]>,
    raw_xz: Option<([f64; 2], Instant)>,
}

pub struct TelemetryPoller {
    pub shared: Arc<Mutex<TelemetryFrame>>,
    stop: Arc<AtomicBool>,
}

impl TelemetryPoller {
    pub fn start() -> Self {
        let shared = Arc::new(Mutex::new(TelemetryFrame::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let map_state = Arc::new(Mutex::new(MapPositionState::default()));

        // 独立慢速线程：专职轮询 `/map_info.json` 与 `/map_obj.json`，绝不阻塞姿态主线程
        {
            let map_clone = Arc::clone(&map_state);
            let stop_map = Arc::clone(&stop);
            let _ = std::thread::Builder::new()
                .name("wt-8111-map".into())
                .spawn(move || {
                    let mut client = KeepAlive8111::new(150);
                    let mut tick = 0u64;
                    let mut cur_bounds: Option<[f64; 4]> = None;
                    while !stop_map.load(Ordering::Relaxed) {
                        if tick % 10 == 0 || cur_bounds.is_none() {
                            if let Some(mi) = client.get_json("/map_info.json") {
                                let p_arr = |k: &str| -> Option<[f64; 2]> {
                                    let a = mi.get(k)?.as_array()?;
                                    if a.len() < 2 {
                                        return None;
                                    }
                                    let parse_el = |el: &Value| {
                                        el.as_f64().or_else(|| el.as_str()?.parse::<f64>().ok())
                                    };
                                    Some([parse_el(&a[0])?, parse_el(&a[1])?])
                                };
                                if let (Some(mn), Some(mx)) = (p_arr("map_min"), p_arr("map_max")) {
                                    cur_bounds = Some([mn[0], mn[1], mx[0], mx[1]]);
                                }
                            }
                        }
                        if let (Some(bounds), Some(objs)) =
                            (cur_bounds, client.get_json("/map_obj.json"))
                        {
                            if let Some(arr) = objs.as_array() {
                                if let Some(player) = arr.iter().find(|o| {
                                    o.get("icon").and_then(Value::as_str) == Some("Player")
                                }) {
                                    if let (Some(xn), Some(yn)) =
                                        (val_f64(player, "x"), val_f64(player, "y"))
                                    {
                                        let [xmin, zmin, xmax, zmax] = bounds;
                                        let wx = xmin + xn * (xmax - xmin);
                                        let wz = zmax - yn * (zmax - zmin);
                                        if let Ok(mut mg) = map_clone.lock() {
                                            mg.bounds = Some(bounds);
                                            mg.raw_xz = Some(([wx, wz], Instant::now()));
                                        }
                                    }
                                }
                            }
                        }
                        tick = tick.wrapping_add(1);
                        std::thread::sleep(Duration::from_millis(160));
                    }
                });
        }

        // 高频姿态主线程：专职轮询 `/state` 与 `/indicators` (~80 Hz Keep-Alive)
        {
            let sh_clone = Arc::clone(&shared);
            let map_clone = Arc::clone(&map_state);
            let stop_att = Arc::clone(&stop);
            let _ = std::thread::Builder::new()
                .name("wt-8111-attitude".into())
                .spawn(move || {
                    let mut client = KeepAlive8111::new(90);
                    let mut runway_alt: Option<f64> = None;
                    let mut prev_att: Option<(f64, f64, f64, Instant)> = None;
                    let mut filt_yaw_dps = 0.0;
                    let mut filt_pitch_dps = 0.0;

                    while !stop_att.load(Ordering::Relaxed) {
                        let sj_opt = client.get_json("/state");
                        let ij_opt = client.get_json("/indicators");

                        let ok = sj_opt
                            .as_ref()
                            .and_then(|j| j.get("valid"))
                            .and_then(Value::as_bool)
                            .unwrap_or(false)
                            && ij_opt
                                .as_ref()
                                .and_then(|j| j.get("valid"))
                                .and_then(Value::as_bool)
                                .unwrap_or(false);

                        if ok {
                            let now = Instant::now();
                            let mut frame =
                                parse_state_and_indicators(&sj_opt.unwrap(), &ij_opt.unwrap());
                            frame.sampled_at = Some(now);

                            // 仅当 8111 内部数值发生实质更新时计算平滑角速度（滤除重复采样帧）
                            if let Some((ph, pp, _pr, pt)) = prev_att {
                                let dt = now.saturating_duration_since(pt).as_secs_f64();
                                let dh = shortest_angle_diff_deg(frame.heading_deg, ph);
                                let dp = frame.pitch_deg - pp;
                                if dt >= 0.012 && (dh.abs() > 1e-4 || dp.abs() > 1e-4 || dt > 0.06) {
                                    let raw_yr = (dh / dt).clamp(-180.0, 180.0);
                                    let raw_pr = (dp / dt).clamp(-180.0, 180.0);
                                    filt_yaw_dps = filt_yaw_dps * 0.45 + raw_yr * 0.55;
                                    filt_pitch_dps = filt_pitch_dps * 0.45 + raw_pr * 0.55;
                                    prev_att = Some((frame.heading_deg, frame.pitch_deg, frame.roll_deg, now));
                                }
                            } else {
                                prev_att = Some((frame.heading_deg, frame.pitch_deg, frame.roll_deg, now));
                            }
                            frame.yaw_rate_dps = filt_yaw_dps;
                            frame.pitch_rate_dps = filt_pitch_dps;

                            if frame.tas_mps < 12.0
                                && frame.vy_mps.unwrap_or(0.0).abs() < 1.2
                                && frame.alt_asl_m > -100.0
                            {
                                runway_alt = Some(frame.alt_asl_m);
                            }
                            frame.auto_runway_alt_m = runway_alt;

                            if let Ok(mg) = map_clone.lock() {
                                frame.map_bounds = mg.bounds;
                                frame.map_xz_m = mg.raw_xz.map(|([bx, bz], t0)| {
                                    let dt = now.saturating_duration_since(t0).as_secs_f64().clamp(0.0, 0.35);
                                    let vy = frame.vy_mps.unwrap_or(0.0);
                                    let vh = (frame.tas_mps * frame.tas_mps - vy * vy).max(0.0).sqrt();
                                    let hdg_rad = frame.heading_deg.to_radians();
                                    [bx + hdg_rad.sin() * vh * dt, bz + hdg_rad.cos() * vh * dt]
                                });
                            }

                            if let Ok(mut g) = sh_clone.lock() {
                                *g = frame;
                            }
                            std::thread::sleep(Duration::from_millis(10));
                        } else {
                            prev_att = None;
                            if let Ok(mut g) = sh_clone.lock() {
                                g.online = false;
                            }
                            std::thread::sleep(Duration::from_millis(400));
                        }
                    }
                });
        }

        Self { shared, stop }
    }

    pub fn snapshot(&self) -> TelemetryFrame {
        self.shared.lock().map(|g| g.clone()).unwrap_or_default()
    }
}

impl Drop for TelemetryPoller {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// 内置无游戏动态三维俯冲打靶演示状态生成器
#[allow(clippy::too_many_arguments)]
pub fn build_demo_telemetry_and_nose_px(
    elapsed_s: f64,
    unit_id: &str,
    tps_cfg: &TpsCameraConfig,
    unit_tps_offset: [f64; 3],
    static_aim_pitch_deg: f64,
    gun_conv_dist_m: f64,
    fov_deg: f64,
    screen_w: f64,
    screen_h: f64,
    center_xz: [f64; 2],
    ground_alt_m: f64,
) -> (TelemetryFrame, [f64; 2]) {
    let phase = (elapsed_s % 14.0) / 14.0;
    let alt_agl = 620.0 - 340.0 * phase;
    let alt_asl = ground_alt_m + alt_agl;
    let ac_hdg = 35.0 + 12.0 * (elapsed_s * 0.7).sin();
    let ac_pitch = -20.0 + 4.0 * (elapsed_s * 1.1).cos();
    let ac_roll = 18.0 * (elapsed_s * 0.7).cos();
    let tas_mps = 165.0 + 35.0 * phase;
    let aoa_deg = 2.4;
    let vy_mps = tas_mps * (ac_pitch - aoa_deg).to_radians().sin();

    let sim_cam_yaw = ac_hdg + 5.5 * (elapsed_s * 1.35).sin();
    let sim_cam_pitch = ac_pitch - 3.0 + 2.8 * (elapsed_s * 0.95).cos();

    let ac_pos = Vec3::new(center_xz[0], alt_asl, center_xz[1]);
    let sim_cam = TpsCameraState::build(
        ac_pos,
        sim_cam_yaw,
        sim_cam_pitch,
        ac_roll,
        0.15,
        0.0,
        tps_cfg,
        unit_tps_offset,
        fov_deg,
        screen_w,
        screen_h,
    );
    let ac_basis = BodyBasis::from_euler_deg(ac_hdg, ac_pitch, ac_roll);
    let bore_pt = ac_pos + ac_basis.pitched_axis(static_aim_pitch_deg) * gun_conv_dist_m;
    let nose_px = sim_cam
        .project_world_to_screen(bore_pt)
        .unwrap_or([0.5 * screen_w, 0.5 * screen_h]);

    let frame = TelemetryFrame {
        online: true,
        unit_id: unit_id.to_string(),
        alt_asl_m: alt_asl,
        tas_mps,
        ias_mps: tas_mps * 0.96,
        mach: tas_mps / 340.0,
        vy_mps: Some(vy_mps),
        aoa_deg,
        aos_deg: 0.0,
        heading_deg: ac_hdg,
        pitch_deg: ac_pitch,
        roll_deg: ac_roll,
        yaw_rate_dps: 0.0,
        pitch_rate_dps: 0.0,
        roll_rate_dps: 0.0,
        stick_elevator: 0.15,
        stick_rudder: 0.0,
        airbrake_pct: 0.0,
        radar_ground_alt_m: Some(ground_alt_m),
        auto_runway_alt_m: Some(ground_alt_m),
        map_xz_m: Some(center_xz),
        map_bounds: None,
        sampled_at: Some(Instant::now()),
    };
    (frame, nose_px)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_war_thunder_inverted_aviahorizon_signs() {
        // 模拟真实战雷 8111 俯冲状态 (-22° 俯冲，右倾 +28°，A-10A 英尺高度表 + 无线电高度表)
        // 战雷 /indicators 原始输出：aviahorizon_pitch 为 +22.0 (反向！)，aviahorizon_roll 为 -28.0 (反向！)，bank 为 0.3 (侧滑球！)
        let sj = serde_json::json!({
            "valid": true,
            "H, m": 500.0,
            "TAS, km/h": 648.0,
            "IAS, km/h": 630.0,
            "Vy, m/s": -72.0,
            "AoA, deg": 2.0,
            "AoS, deg": 0.0,
            "Wx, deg/s": 5.0
        });
        let ij = serde_json::json!({
            "valid": true,
            "type": "a_10a_early",
            "compass": 135.0,
            "aviahorizon_pitch": 22.0,
            "aviahorizon_roll": -28.0,
            "bank": 0.3,
            "altitude_10k": 1640.4,   // 500m * 3.28084 ft
            "radio_altitude": 984.25  // 300m AGL in ft -> 地面真实海拔 = 500 - 300 = 200m!
        });

        let tf = parse_state_and_indicators(&sj, &ij);
        assert!(
            (tf.pitch_deg - (-22.0)).abs() < 1e-6,
            "俯冲时 pitch_deg 必须为负 (-22.0°)，实际={}",
            tf.pitch_deg
        );
        assert!(
            (tf.roll_deg - 28.0).abs() < 1e-6,
            "必须取 -aviahorizon_roll (+28.0°) 而非侧滑球 bank (0.3°)，实际={}",
            tf.roll_deg
        );
        let rg = tf.radar_ground_alt_m.expect("应由英尺无线电高度表解出真实地面海拔");
        assert!((rg - 200.0).abs() < 1.0, "预期地面海拔约 200m，实际={rg}");
    }
}
