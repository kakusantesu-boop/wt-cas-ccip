//! 第三人称追尾摄像机 (`TpsFlightCamera`) 透视投影、视觉锚点 2×2 高斯-牛顿反解器与 1-Euro 自适应防抖滤波器
//! 严格对齐 DagorEngine `cameraView.cpp.inl` (`EFM_HOR_PLUS` 16:9 归一化) 与拆包 `config/camera.blkx`

use crate::ballistics::{BodyBasis, DEG_TO_RAD, Vec3};
use crate::db::TpsCameraConfig;

pub const CAMERA_FOV_REF_ASPECT: f64 = 16.0 / 9.0;

#[derive(Clone, Copy, Debug)]
pub struct PerspectiveParams {
    pub wk: f64,
    pub hk: f64,
    pub screen_w: f64,
    pub screen_h: f64,
}

impl PerspectiveParams {
    /// 对齐 DagorEngine `calc_hor_ver_fov(fov_degree, EFM_HOR_PLUS, view_w, view_h)`
    pub fn from_hor_plus_fov(fov_deg: f64, screen_w: f64, screen_h: f64) -> Self {
        let fov_in_tan = 1.0 / (0.5 * fov_deg.clamp(5.0, 150.0) * DEG_TO_RAD).tan();
        let hk = fov_in_tan * CAMERA_FOV_REF_ASPECT;
        let wk = hk * (screen_h / screen_w.max(1.0));
        Self {
            wk,
            hk,
            screen_w: screen_w.max(1.0),
            screen_h: screen_h.max(1.0),
        }
    }

    #[inline]
    pub fn pixel_to_norm_ray(&self, u_px: f64, v_px: f64) -> [f64; 2] {
        let nx = (2.0 * u_px / self.screen_w - 1.0) / self.wk;
        let ny = (1.0 - 2.0 * v_px / self.screen_h) / self.hk;
        [nx, ny]
    }

    #[inline]
    pub fn norm_ray_to_pixel(&self, nx: f64, ny: f64) -> [f64; 2] {
        let u = 0.5 * self.screen_w * (1.0 + nx * self.wk);
        let v = 0.5 * self.screen_h * (1.0 - ny * self.hk);
        [u, v]
    }
}

#[derive(Clone, Copy, Debug)]
pub struct TpsCameraState {
    pub eye_world: Vec3,
    pub cam_right: Vec3,
    pub cam_up: Vec3,
    pub cam_fwd: Vec3,
    pub yaw_deg: f64,
    pub pitch_deg: f64,
    pub persp: PerspectiveParams,
}

impl TpsCameraState {
    /// 完整第三人称追尾相机几何构造（含单机 `tpsCameraOffset`、开镜靶点偏移 `tpsCameraTargetOffsetAtZoom` 与三轴舵面微倾）
    #[allow(clippy::too_many_arguments)]
    pub fn build_ext(
        aircraft_pos: Vec3,
        cam_yaw_deg: f64,
        cam_pitch_deg: f64,
        aircraft_roll_deg: f64,
        stick_elevator: f64,
        stick_rudder: f64,
        stick_ailerons: f64,
        cfg: &TpsCameraConfig,
        unit_tps_offset: [f64; 3],
        unit_zoom_target_offset: [f64; 3],
        fov_deg: f64,
        screen_w: f64,
        screen_h: f64,
    ) -> Self {
        let cam_roll = if cam_pitch_deg.abs() < cfg.overturn_angle {
            aircraft_roll_deg * cfg.roll_angle_factor
        } else {
            aircraft_roll_deg
        };
        let orbit = BodyBasis::from_euler_deg(cam_yaw_deg, cam_pitch_deg, cam_roll);

        // 1. 升降舵与方向舵输入导致的轻微相机前后/左右位移 (camera.blkx)
        let mut local_off =
            Vec3::from_array(cfg.camera_offset_neutral) + Vec3::from_array(unit_tps_offset);
        let elev = stick_elevator.clamp(-1.0, 1.0);
        if elev > 0.0 {
            let target =
                Vec3::from_array(cfg.camera_offset_elev_up) + Vec3::from_array(unit_tps_offset);
            local_off = local_off.lerp(target, elev * 0.35);
        } else if elev < 0.0 {
            let target =
                Vec3::from_array(cfg.camera_offset_elev_down) + Vec3::from_array(unit_tps_offset);
            local_off = local_off.lerp(target, (-elev) * 0.35);
        }
        let rud = stick_rudder.clamp(-1.0, 1.0);
        local_off.z += rud * 0.35;

        // 2. 开镜缩放比例插值 `tpsCameraTargetOffsetAtZoom`（直升机/大型轰炸机在 30° 放大态会自动抬升注视点防遮挡）
        let fov_span = (cfg.fov - cfg.zoom_fov).abs().max(1.0);
        let zoom_frac = ((cfg.fov - fov_deg) / fov_span).clamp(0.0, 1.0);
        let eff_target = Vec3::from_array(cfg.target_offset)
            + Vec3::from_array(unit_zoom_target_offset) * zoom_frac;

        let eye_world =
            aircraft_pos + orbit.local_to_world([local_off.x, local_off.y, local_off.z]);
        let look_target =
            aircraft_pos + orbit.local_to_world([eff_target.x, eff_target.y, eff_target.z]);

        // 3. 副翼横滚微倾 `cameraUpAileronsLeft / Right` (sensitivityAilerons)
        let ail = stick_ailerons.clamp(-1.0, 1.0);
        let mut local_up = Vec3::from_array(cfg.camera_up_neutral);
        if ail > 0.0 {
            let t_up = Vec3::from_array(cfg.camera_up_ailerons_right);
            local_up = local_up.lerp(t_up, (ail * cfg.sensitivity_ailerons).clamp(0.0, 1.0));
        } else if ail < 0.0 {
            let t_up = Vec3::from_array(cfg.camera_up_ailerons_left);
            local_up = local_up.lerp(t_up, ((-ail) * cfg.sensitivity_ailerons).clamp(0.0, 1.0));
        }
        let ref_up = orbit
            .local_to_world([local_up.x, local_up.y, local_up.z])
            .normalized_or(orbit.up);

        // 4. 左手系施密特正交化：主光轴 `cam_fwd` 精确对准 `look_target`，`cam_up` 对齐 `ref_up` 在垂直光轴平面上的投影
        let cam_fwd = (look_target - eye_world).normalized_or(orbit.fwd);
        let up_proj = ref_up - cam_fwd * ref_up.dot(cam_fwd);
        let cam_up = up_proj.normalized_or(orbit.up);
        // 左手系：`right = up × fwd`（验证：`up=(0,1,0), fwd=(0,0,1) -> (1*1-0, 0-0, 0-0) = (1,0,0)`）
        let cross_right = Vec3::new(
            cam_up.y * cam_fwd.z - cam_up.z * cam_fwd.y,
            cam_up.z * cam_fwd.x - cam_up.x * cam_fwd.z,
            cam_up.x * cam_fwd.y - cam_up.y * cam_fwd.x,
        );
        let cam_right = if cross_right.dot(orbit.right) >= 0.0 {
            cross_right.normalized_or(orbit.right)
        } else {
            (cross_right * -1.0).normalized_or(orbit.right)
        };

        Self {
            eye_world,
            cam_right,
            cam_up,
            cam_fwd,
            yaw_deg: cam_yaw_deg,
            pitch_deg: cam_pitch_deg,
            persp: PerspectiveParams::from_hor_plus_fov(fov_deg, screen_w, screen_h),
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn build(
        aircraft_pos: Vec3,
        cam_yaw_deg: f64,
        cam_pitch_deg: f64,
        aircraft_roll_deg: f64,
        stick_elevator: f64,
        stick_rudder: f64,
        cfg: &TpsCameraConfig,
        unit_tps_offset: [f64; 3],
        fov_deg: f64,
        screen_w: f64,
        screen_h: f64,
    ) -> Self {
        Self::build_ext(
            aircraft_pos,
            cam_yaw_deg,
            cam_pitch_deg,
            aircraft_roll_deg,
            stick_elevator,
            stick_rudder,
            0.0,
            cfg,
            unit_tps_offset,
            [0.0, 0.0, 0.0],
            fov_deg,
            screen_w,
            screen_h,
        )
    }

    /// 世界坐标点转相机归一化射线 `[x_c / z_c, y_c / z_c]`；若点在相机后方返回 `None`
    #[inline]
    pub fn world_to_norm_ray(&self, world_pt: Vec3) -> Option<[f64; 2]> {
        let d = world_pt - self.eye_world;
        let zc = d.dot(self.cam_fwd);
        if zc <= 0.2 {
            return None;
        }
        let xc = d.dot(self.cam_right);
        let yc = d.dot(self.cam_up);
        Some([xc / zc, yc / zc])
    }

    /// 世界坐标转屏幕像素坐标 `[u_px, v_px]` (对齐 `bhvProjectionES.cpp.inl`)
    #[inline]
    pub fn project_world_to_screen(&self, world_pt: Vec3) -> Option<[f64; 2]> {
        let [nx, ny] = self.world_to_norm_ray(world_pt)?;
        Some(self.persp.norm_ray_to_pixel(nx, ny))
    }
}

#[inline]
pub fn wrap_yaw_deg(mut deg: f64) -> f64 {
    deg = deg.rem_euclid(360.0);
    if deg < 0.0 {
        deg += 360.0;
    }
    deg
}

/// 核心视觉反推器（完整版）：已知机头三维姿态与屏幕上检测到的机头小十字像素 `(u_nose, v_nose)`，
/// 利用 2×2 阻尼高斯-牛顿法在数微秒内唯一反解第三人称相机姿态 `(cam_yaw_deg, cam_pitch_deg)` 及视差位置。
#[allow(clippy::too_many_arguments)]
pub fn solve_tps_camera_from_nose_pixel_ext(
    aircraft_pos: Vec3,
    ac_heading_deg: f64,
    ac_pitch_deg: f64,
    ac_roll_deg: f64,
    static_aim_pitch_deg: f64,
    gun_conv_dist_m: f64,
    stick_elevator: f64,
    stick_rudder: f64,
    stick_ailerons: f64,
    u_nose_px: f64,
    v_nose_px: f64,
    cfg: &TpsCameraConfig,
    unit_tps_offset: [f64; 3],
    unit_zoom_target_offset: [f64; 3],
    fov_deg: f64,
    screen_w: f64,
    screen_h: f64,
) -> Option<TpsCameraState> {
    let ac_basis = BodyBasis::from_euler_deg(ac_heading_deg, ac_pitch_deg, ac_roll_deg);
    let gun_dir = ac_basis.pitched_axis(static_aim_pitch_deg);
    let bore_world = aircraft_pos + gun_dir * gun_conv_dist_m.clamp(100.0, 5000.0);

    let persp = PerspectiveParams::from_hor_plus_fov(fov_deg, screen_w, screen_h);
    let obs = persp.pixel_to_norm_ray(u_nose_px, v_nose_px);

    // 初始猜测：考虑开镜靶点抬升角补偿后从机头方向偏移，确保一步落入全局单调收敛盆地
    let fov_span = (cfg.fov - cfg.zoom_fov).abs().max(1.0);
    let zoom_frac = ((cfg.fov - fov_deg) / fov_span).clamp(0.0, 1.0);
    let tx = (cfg.target_offset[0] + unit_zoom_target_offset[0] * zoom_frac).max(50.0);
    let ty = cfg.target_offset[1] + unit_zoom_target_offset[1] * zoom_frac;
    let zoom_pitch_bias_deg = (ty / tx).atan().to_degrees();

    let mut yaw = wrap_yaw_deg(ac_heading_deg - obs[0].atan().to_degrees());
    let mut pitch =
        (ac_pitch_deg - zoom_pitch_bias_deg - obs[1].atan().to_degrees()).clamp(-88.0, 88.0);

    let eval = |y_deg: f64, p_deg: f64| -> Option<(TpsCameraState, [f64; 2])> {
        let st = TpsCameraState::build_ext(
            aircraft_pos,
            y_deg,
            p_deg,
            ac_roll_deg,
            stick_elevator,
            stick_rudder,
            stick_ailerons,
            cfg,
            unit_tps_offset,
            unit_zoom_target_offset,
            fov_deg,
            screen_w,
            screen_h,
        );
        let pred = st.world_to_norm_ray(bore_world)?;
        Some((st, [pred[0] - obs[0], pred[1] - obs[1]]))
    };

    let eps = 1e-3; // deg
    for _ in 0..8 {
        let (st, r) = eval(yaw, pitch)?;
        if r[0].hypot(r[1]) < 1e-7 {
            return Some(st);
        }
        let (_, ry) = eval(wrap_yaw_deg(yaw + eps), pitch)?;
        let (_, rp) = eval(yaw, (pitch + eps).clamp(-89.0, 89.0))?;

        let j00 = (ry[0] - r[0]) / eps;
        let j10 = (ry[1] - r[1]) / eps;
        let j01 = (rp[0] - r[0]) / eps;
        let j11 = (rp[1] - r[1]) / eps;

        let det = j00 * j11 - j01 * j10;
        if det.abs() < 1e-10 {
            break;
        }
        let dyaw = -(j11 * r[0] - j01 * r[1]) / det;
        let dpitch = -(-j10 * r[0] + j00 * r[1]) / det;

        yaw = wrap_yaw_deg(yaw + dyaw.clamp(-25.0, 25.0));
        pitch = (pitch + dpitch.clamp(-25.0, 25.0)).clamp(-88.5, 88.5);
    }

    let (final_cam, _) = eval(yaw, pitch)?;
    let [pu, pv] = final_cam.project_world_to_screen(bore_world)?;
    if (pu - u_nose_px).hypot(pv - v_nose_px) <= 2.0 {
        Some(final_cam)
    } else {
        None
    }
}

#[allow(clippy::too_many_arguments)]
pub fn solve_tps_camera_from_nose_pixel(
    aircraft_pos: Vec3,
    ac_heading_deg: f64,
    ac_pitch_deg: f64,
    ac_roll_deg: f64,
    static_aim_pitch_deg: f64,
    gun_conv_dist_m: f64,
    stick_elevator: f64,
    stick_rudder: f64,
    u_nose_px: f64,
    v_nose_px: f64,
    cfg: &TpsCameraConfig,
    unit_tps_offset: [f64; 3],
    fov_deg: f64,
    screen_w: f64,
    screen_h: f64,
) -> Option<TpsCameraState> {
    solve_tps_camera_from_nose_pixel_ext(
        aircraft_pos,
        ac_heading_deg,
        ac_pitch_deg,
        ac_roll_deg,
        static_aim_pitch_deg,
        gun_conv_dist_m,
        stick_elevator,
        stick_rudder,
        0.0,
        u_nose_px,
        v_nose_px,
        cfg,
        unit_tps_offset,
        [0.0, 0.0, 0.0],
        fov_deg,
        screen_w,
        screen_h,
    )
}

/// 一维 1-Euro 自适应低通滤波器 (Casiez et al., CHI 2012)
/// 静止瞄准时以极低截止频率滤除像素抖动与遥测阶梯噪声；大幅甩鼠标时连续抬升截止频率实现近零相位滞后
#[derive(Clone, Copy, Debug)]
pub struct OneEuroFilter1D {
    min_cutoff_hz: f64,
    beta: f64,
    d_cutoff_hz: f64,
    x_prev: Option<f64>,
    dx_prev: f64,
}

impl OneEuroFilter1D {
    pub const fn new(min_cutoff_hz: f64, beta: f64, d_cutoff_hz: f64) -> Self {
        Self {
            min_cutoff_hz,
            beta,
            d_cutoff_hz,
            x_prev: None,
            dx_prev: 0.0,
        }
    }

    #[inline]
    fn smoothing_alpha(cutoff_hz: f64, dt_s: f64) -> f64 {
        let tau = 1.0 / (2.0 * std::f64::consts::PI * cutoff_hz.max(1e-3));
        1.0 / (1.0 + tau / dt_s.max(1e-4))
    }

    pub fn reset(&mut self) {
        self.x_prev = None;
        self.dx_prev = 0.0;
    }

    pub fn filter(&mut self, x: f64, dt_s: f64) -> f64 {
        let dt = dt_s.clamp(1e-3, 0.25);
        let Some(prev) = self.x_prev else {
            self.x_prev = Some(x);
            self.dx_prev = 0.0;
            return x;
        };
        let dx_raw = (x - prev) / dt;
        let a_d = Self::smoothing_alpha(self.d_cutoff_hz, dt);
        let dx_hat = self.dx_prev + a_d * (dx_raw - self.dx_prev);
        let cutoff = self.min_cutoff_hz + self.beta * dx_hat.abs();
        let a_x = Self::smoothing_alpha(cutoff, dt);
        let x_hat = prev + a_x * (x - prev);
        self.x_prev = Some(x_hat);
        self.dx_prev = dx_hat;
        x_hat
    }
}

/// 二维屏幕坐标 1-Euro 自适应防抖滤波器（含突变跳变自动重置门控）
#[derive(Clone, Copy, Debug)]
pub struct OneEuroFilter2D {
    fx: OneEuroFilter1D,
    fy: OneEuroFilter1D,
    jump_reset_px: f64,
}

impl OneEuroFilter2D {
    pub const fn new(min_cutoff_hz: f64, beta: f64, d_cutoff_hz: f64, jump_reset_px: f64) -> Self {
        Self {
            fx: OneEuroFilter1D::new(min_cutoff_hz, beta, d_cutoff_hz),
            fy: OneEuroFilter1D::new(min_cutoff_hz, beta, d_cutoff_hz),
            jump_reset_px,
        }
    }

    pub fn reset(&mut self) {
        self.fx.reset();
        self.fy.reset();
    }

    pub fn current(&self) -> Option<[f64; 2]> {
        Some([self.fx.x_prev?, self.fy.x_prev?])
    }

    pub fn filter(&mut self, xy: [f64; 2], dt_s: f64) -> [f64; 2] {
        if let (Some(px), Some(py)) = (self.fx.x_prev, self.fy.x_prev) {
            if (xy[0] - px).hypot(xy[1] - py) > self.jump_reset_px {
                self.reset();
            }
        }
        [self.fx.filter(xy[0], dt_s), self.fy.filter(xy[1], dt_s)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_inverse_tps_camera_solver_exact_roundtrip() {
        let cfg = TpsCameraConfig::default();
        let ac_pos = Vec3::new(1250.0, 680.0, -430.0);
        let (ac_hdg, ac_pitch, ac_roll) = (132.5, -22.0, 34.0);
        // 模拟鼠标大幅向右下方牵引相机（机身滞后 18° 偏航、11° 俯仰）
        let (true_cam_yaw, true_cam_pitch) = (150.5, -33.0);

        for &(w, h, fov, zoom_off, ail) in &[
            (1920.0, 1080.0, 90.0, [0.0, 0.0, 0.0], 0.0),
            (2560.0, 1440.0, 30.0, [50.0, 20.0, 0.0], 0.65), // 阿帕奇/卡-50 开镜靶点抬升 + 副翼滚转
            (3440.0, 1440.0, 30.0, [80.0, 60.0, 0.0], -0.5), // AH-1 眼镜蛇 21:9 带鱼屏放大态
        ] {
            let gt_cam = TpsCameraState::build_ext(
                ac_pos,
                true_cam_yaw,
                true_cam_pitch,
                ac_roll,
                0.4,
                -0.2,
                ail,
                &cfg,
                [-1.0, 0.0, 0.0],
                zoom_off,
                fov,
                w,
                h,
            );
            let ac_basis = BodyBasis::from_euler_deg(ac_hdg, ac_pitch, ac_roll);
            let bore_pt = ac_pos + ac_basis.fwd * 500.0;
            let [u_nose, v_nose] = gt_cam
                .project_world_to_screen(bore_pt)
                .expect("机头准星应在视场内");

            let solved_cam = solve_tps_camera_from_nose_pixel_ext(
                ac_pos,
                ac_hdg,
                ac_pitch,
                ac_roll,
                0.0,
                500.0,
                0.4,
                -0.2,
                ail,
                u_nose,
                v_nose,
                &cfg,
                [-1.0, 0.0, 0.0],
                zoom_off,
                fov,
                w,
                h,
            )
            .expect("高斯-牛顿反解必须收敛");

            let ground_hit = Vec3::new(1780.0, 45.0, -910.0);
            let [u_gt, v_gt] = gt_cam.project_world_to_screen(ground_hit).unwrap();
            let [u_sol, v_sol] = solved_cam.project_world_to_screen(ground_hit).unwrap();
            let px_err = (u_gt - u_sol).hypot(v_gt - v_sol);
            assert!(
                px_err < 0.02,
                "反解相机投影地面落点像素误差过大: {px_err} px (w={w}, fov={fov})"
            );
        }
    }

    #[test]
    fn test_one_euro_filter_suppresses_jitter_with_minimal_pan_lag() {
        let mut f = OneEuroFilter2D::new(2.0, 0.14, 1.0, 240.0);
        let dt = 1.0 / 75.0;

        // 1. 静止瞄准叠加 ±0.55 px 高频零均值抖动，导数低通 (1Hz) 抵消正负交替，稳态摆幅 < 0.16 px
        let mut max_dev = 0.0f64;
        for i in 0..90 {
            let n = if i % 2 == 0 { 0.55 } else { -0.55 };
            let out = f.filter([960.0 + n, 540.0 - n], dt);
            if i > 25 {
                max_dev = max_dev.max((out[0] - 960.0).hypot(out[1] - 540.0));
            }
        }
        assert!(max_dev < 0.18, "静止防抖残余摆幅过大: {max_dev} px");

        // 2. 高速甩动鼠标 (600 px/s -> 每帧 8 px)，渐近理论滞后 1/(2*pi*0.14) ≈ 1.14 px < 1.5 px
        let mut pos = 960.0;
        let mut lag = 0.0;
        for _ in 0..30 {
            pos += 8.0;
            let out = f.filter([pos, 540.0], dt);
            lag = (pos - out[0]).abs();
        }
        assert!(lag < 1.5, "高速甩镜相位滞后过大: {lag} px");
    }
}
