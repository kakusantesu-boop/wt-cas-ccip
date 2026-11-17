//! 第三人称追尾摄像机 (TpsFlightCamera) 透视投影与视觉锚点 2×2 高斯-牛顿反解器
//! 严格对齐 DagorEngine `cameraView.cpp.inl` (EFM_HOR_PLUS 16:9 归一化) 与拆包 `config/camera.blkx`

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
    /// 按拆包 `camera.blkx` 构造第三人称相机：
    /// 1. 轨道基向量由 (cam_yaw_deg, cam_pitch_deg, ac_roll_deg * roll_angle_factor) 确定；
    /// 2. 相机眼睛位于 aircraft_pos + orbit.local_to_world(offset_neutral + unit_tps_offset + 控制面微偏)；
    /// 3. 相机光轴从 eye_world 指向注视收敛点 aircraft_pos + orbit.local_to_world(target_offset = [500, 0, 0])。
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
        let cam_roll = if cam_pitch_deg.abs() < cfg.overturn_angle {
            aircraft_roll_deg * cfg.roll_angle_factor
        } else {
            aircraft_roll_deg
        };
        let orbit = BodyBasis::from_euler_deg(cam_yaw_deg, cam_pitch_deg, cam_roll);

        // 升降舵与方向舵输入导致的轻微相机前后/左右位移 (camera.blkx)
        let mut local_off = Vec3::from_array(cfg.camera_offset_neutral)
            + Vec3::from_array(unit_tps_offset);
        let elev = stick_elevator.clamp(-1.0, 1.0);
        if elev > 0.0 {
            let target = Vec3::from_array(cfg.camera_offset_elev_up) + Vec3::from_array(unit_tps_offset);
            local_off = local_off.lerp(target, elev * 0.35);
        } else if elev < 0.0 {
            let target = Vec3::from_array(cfg.camera_offset_elev_down) + Vec3::from_array(unit_tps_offset);
            local_off = local_off.lerp(target, (-elev) * 0.35);
        }
        let rud = stick_rudder.clamp(-1.0, 1.0);
        local_off.z += rud * 0.35;

        let eye_world = aircraft_pos + orbit.local_to_world([local_off.x, local_off.y, local_off.z]);
        let look_target = aircraft_pos + orbit.local_to_world(cfg.target_offset);

        let cam_fwd = (look_target - eye_world).normalized_or(orbit.fwd);
        // 左手系正交化：保持水平右向与 orbit.right 对齐，消除畸变
        let right_proj = orbit.right - cam_fwd * orbit.right.dot(cam_fwd);
        let cam_right = right_proj.normalized_or(orbit.right);
        // 左手系: up = fwd × right (这里直接用正交补并保持与 orbit.up 同向)
        let cross_lh = Vec3::new(
            cam_fwd.y * cam_right.z - cam_fwd.z * cam_right.y,
            cam_fwd.z * cam_right.x - cam_fwd.x * cam_right.z,
            cam_fwd.x * cam_right.y - cam_fwd.y * cam_right.x,
        );
        let cam_up = if cross_lh.dot(orbit.up) >= 0.0 {
            cross_lh.normalized_or(orbit.up)
        } else {
            (cross_lh * -1.0).normalized_or(orbit.up)
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

    /// 世界坐标点转相机归一化射线 [x_c / z_c, y_c / z_c]；若点在相机后方返回 None
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

    /// 世界坐标转屏幕像素坐标 [u_px, v_px] (对齐 bhvProjectionES.cpp.inl)
    #[inline]
    pub fn project_world_to_screen(&self, world_pt: Vec3) -> Option<[f64; 2]> {
        let [nx, ny] = self.world_to_norm_ray(world_pt)?;
        Some(self.persp.norm_ray_to_pixel(nx, ny))
    }
}

#[inline]
fn wrap_yaw_deg(mut deg: f64) -> f64 {
    deg = deg.rem_euclid(360.0);
    if deg < 0.0 {
        deg += 360.0;
    }
    deg
}

/// 核心视觉反推器：已知机头三维姿态与屏幕上检测到的机头小十字像素 (u_nose, v_nose)，
/// 利用 2×2 阻尼高斯-牛顿法在数微秒内唯一反解第三人称相机姿态 (cam_yaw_deg, cam_pitch_deg) 及视差位置！
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
    let ac_basis = BodyBasis::from_euler_deg(ac_heading_deg, ac_pitch_deg, ac_roll_deg);
    let gun_dir = ac_basis.pitched_axis(static_aim_pitch_deg);
    let bore_world = aircraft_pos + gun_dir * gun_conv_dist_m.clamp(100.0, 2000.0);

    let persp = PerspectiveParams::from_hor_plus_fov(fov_deg, screen_w, screen_h);
    let obs = persp.pixel_to_norm_ray(u_nose_px, v_nose_px);

    // 初始猜测：用小角度线性近似从机头方向直接偏移，瞬间落入收敛盆地
    let mut yaw = wrap_yaw_deg(ac_heading_deg - obs[0].atan().to_degrees());
    let mut pitch = (ac_pitch_deg - obs[1].atan().to_degrees()).clamp(-88.0, 88.0);

    let eval = |y_deg: f64, p_deg: f64| -> Option<(TpsCameraState, [f64; 2])> {
        let st = TpsCameraState::build(
            aircraft_pos,
            y_deg,
            p_deg,
            ac_roll_deg,
            stick_elevator,
            stick_rudder,
            cfg,
            unit_tps_offset,
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

        for &(w, h, fov) in &[
            (1920.0, 1080.0, 90.0),
            (2560.0, 1440.0, 30.0), // 右键放大态
            (3440.0, 1440.0, 90.0), // 21:9 带鱼屏 EFM_HOR_PLUS
        ] {
            let gt_cam = TpsCameraState::build(
                ac_pos,
                true_cam_yaw,
                true_cam_pitch,
                ac_roll,
                0.4,
                -0.2,
                &cfg,
                [0.0, 0.0, 0.0],
                fov,
                w,
                h,
            );
            let ac_basis = BodyBasis::from_euler_deg(ac_hdg, ac_pitch, ac_roll);
            let bore_pt = ac_pos + ac_basis.fwd * 500.0;
            let [u_nose, v_nose] = gt_cam
                .project_world_to_screen(bore_pt)
                .expect("机头准星应在视场内");

            let solved_cam = solve_tps_camera_from_nose_pixel(
                ac_pos,
                ac_hdg,
                ac_pitch,
                ac_roll,
                0.0,
                500.0,
                0.4,
                -0.2,
                u_nose,
                v_nose,
                &cfg,
                [0.0, 0.0, 0.0],
                fov,
                w,
                h,
            )
            .expect("高斯-牛顿反解必须收敛");

            // 验证地面任意落点在反解相机与真实相机中的屏幕像素误差 < 0.02 px！
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
}
