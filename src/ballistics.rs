//! 三维空地航弹与空射火箭弹物理解算引擎 (RK2 / Heun + 地形高程精确求交)
//! 世界坐标系约定 (与 Dagor / 婴儿弹道计算机一致)：+X 向东，+Y 向上 (海拔 m)，+Z 向北。

use crate::atmosphere::Atmosphere;
use crate::db::{BombSpec, RocketSpec};
use crate::terrain_pack::TerrainSource;

pub const G: f64 = 9.81;
pub const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;
pub const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn from_array(a: [f64; 3]) -> Self {
        Self::new(a[0], a[1], a[2])
    }

    #[inline]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    #[inline]
    pub fn norm_sq(self) -> f64 {
        self.dot(self)
    }

    #[inline]
    pub fn norm(self) -> f64 {
        self.norm_sq().sqrt()
    }

    #[inline]
    pub fn normalized_or(self, fallback: Self) -> Self {
        let n = self.norm();
        if n > 1e-9 {
            Self::new(self.x / n, self.y / n, self.z / n)
        } else {
            fallback
        }
    }

    #[inline]
    pub fn lerp(self, other: Self, t: f64) -> Self {
        self + (other - self) * t
    }
}

impl std::ops::Add for Vec3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl std::ops::Mul<f64> for Vec3 {
    type Output = Self;
    #[inline]
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}

/// 左手正交机体基向量 (fwd: 机头向前, up: 座舱向上, right: 右翼向右)
#[derive(Clone, Copy, Debug)]
pub struct BodyBasis {
    pub fwd: Vec3,
    pub up: Vec3,
    pub right: Vec3,
}

impl BodyBasis {
    /// heading_deg: 罗盘航向 (0°=正北+Z, 90°=正东+X)
    /// pitch_deg: 机头俯仰 (>0 抬头向上)
    /// roll_deg: 机体横滚 (>0 右翼下沉)
    pub fn from_euler_deg(heading_deg: f64, pitch_deg: f64, roll_deg: f64) -> Self {
        let psi = heading_deg * DEG_TO_RAD;
        let theta = pitch_deg * DEG_TO_RAD;
        let phi = roll_deg * DEG_TO_RAD;

        let (s_psi, c_psi) = psi.sin_cos();
        let (s_th, c_th) = theta.sin_cos();
        let (s_phi, c_phi) = phi.sin_cos();

        let horiz_right = Vec3::new(c_psi, 0.0, -s_psi);
        let fwd = Vec3::new(c_th * s_psi, s_th, c_th * c_psi);
        let up0 = Vec3::new(-s_th * s_psi, c_th, -s_th * c_psi);

        let right = horiz_right * c_phi - up0 * s_phi;
        let up = horiz_right * s_phi + up0 * c_phi;
        Self { fwd, up, right }
    }

    /// 将机体局部坐标 [x_fwd, y_up, z_right] 变换为世界偏移向量
    #[inline]
    pub fn local_to_world(&self, local_xyz: [f64; 3]) -> Vec3 {
        self.fwd * local_xyz[0] + self.up * local_xyz[1] + self.right * local_xyz[2]
    }

    /// 绕右翼轴额外俯仰 delta_pitch_deg (>0 向上抬)
    pub fn pitched_axis(&self, delta_pitch_deg: f64) -> Vec3 {
        let a = delta_pitch_deg * DEG_TO_RAD;
        let (sa, ca) = a.sin_cos();
        (self.fwd * ca + self.up * sa).normalized_or(self.fwd)
    }
}

/// 飞机瞬时三维运动学状态
#[derive(Clone, Copy, Debug)]
pub struct AircraftKinematics {
    pub pos_world: Vec3,
    pub vel_world: Vec3,
    pub basis: BodyBasis,
    pub heading_deg: f64,
    pub pitch_deg: f64,
    pub roll_deg: f64,
    pub tas_mps: f64,
}

impl AircraftKinematics {
    pub fn from_telemetry(
        pos_world: Vec3,
        heading_deg: f64,
        pitch_deg: f64,
        roll_deg: f64,
        tas_mps: f64,
        vy_mps_opt: Option<f64>,
        aoa_deg: f64,
        aos_deg: f64,
    ) -> Self {
        let basis = BodyBasis::from_euler_deg(heading_deg, pitch_deg, roll_deg);
        let alpha = aoa_deg * DEG_TO_RAD;
        let beta = aos_deg * DEG_TO_RAD;
        let (sa, ca) = alpha.sin_cos();
        let (sb, cb) = beta.sin_cos();

        // 正攻角 alpha > 0 时机头高于气流速度矢量，速度方向沿机体 -up 偏转
        let aero_dir = (basis.fwd * (ca * cb) - basis.up * (sa * cb) + basis.right * sb)
            .normalized_or(basis.fwd);
        let mut vel_world = aero_dir * tas_mps.max(0.0);

        // 若 8111 提供了精确垂直升降率 Vy，修正垂直分量并保持总真空速模长守恒
        if let Some(vy) = vy_mps_opt.filter(|v| v.is_finite()) {
            let spd = tas_mps.max(0.0);
            if spd > 1.0 && vy.abs() <= spd * 0.995 {
                let horiz_sq = (spd * spd - vy * vy).max(0.0);
                let cur_h = vel_world.x.hypot(vel_world.z);
                if cur_h > 1e-4 {
                    let scale_h = horiz_sq.sqrt() / cur_h;
                    vel_world.x *= scale_h;
                    vel_world.z *= scale_h;
                    vel_world.y = vy;
                }
            }
        }

        Self {
            pos_world,
            vel_world,
            basis,
            heading_deg,
            pitch_deg,
            roll_deg,
            tas_mps,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BallisticTuning {
    /// 航弹有效阻力面积比例系数 (默认 1.0，试飞场可微调对齐)
    pub bomb_drag_scale: f64,
    /// 内埋弹舱开门延迟 (秒，翼下外挂为 0.0)
    pub bay_door_delay_s: f64,
    /// 炸弹脱离挂架向下弹射初速 (m/s，默认 1.5 m/s)
    pub bomb_eject_down_mps: f64,
    /// 积分步长 (秒，默认 0.005)
    pub dt: f64,
    /// 最大解算时长 (秒，默认 35.0)
    pub max_time_s: f64,
}

impl Default for BallisticTuning {
    fn default() -> Self {
        Self {
            bomb_drag_scale: 1.0,
            bay_door_delay_s: 0.0,
            bomb_eject_down_mps: 1.5,
            dt: 0.005,
            max_time_s: 35.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ImpactSolution {
    pub impact_pos: Vec3,
    pub time_of_flight_s: f64,
    pub impact_speed_mps: f64,
    pub slant_range_m: f64,
    pub ground_elev_m: f64,
}

/// 三维航弹 CCIP 解算 (支持普通流线弹、钝头弹、Snakeye/AIR 延时开伞减速弹)
pub fn simulate_bomb_ccip(
    kin: &AircraftKinematics,
    bomb: &BombSpec,
    atm: &Atmosphere,
    terrain: &TerrainSource,
    tuning: &BallisticTuning,
) -> Option<ImpactSolution> {
    let dt = tuning.dt.clamp(0.001, 0.02);
    let delay = tuning.bay_door_delay_s.max(0.0);

    // 挂架位于机身下方约 0.8m；若有弹舱延迟则飞机先沿当前速度匀速前飞 delay 秒
    let release_offset = kin.basis.local_to_world([0.0, -0.8, 0.0]);
    let mut pos = kin.pos_world + release_offset + kin.vel_world * delay;
    let mut vel = kin.vel_world - kin.basis.up * tuning.bomb_eject_down_mps;

    if pos.y <= terrain.height_at(pos.x, pos.z) {
        return None;
    }

    let mass = bomb.mass.max(0.5);
    let base_cds = (bomb.drag_cx * bomb.cx_k * tuning.bomb_drag_scale).max(1e-6);

    let brake_mult_at = |t_drop: f64| -> f64 {
        if bomb.brake_cx_k <= 1.0 {
            return 1.0;
        }
        match bomb.brake_time {
            Some([t0, t1]) if t_drop > t0 => {
                let span = (t1 - t0).max(1e-3);
                let frac = ((t_drop - t0) / span).clamp(0.0, 1.0);
                1.0 + (bomb.brake_cx_k - 1.0) * frac
            }
            _ => 1.0,
        }
    };

    let accel = |p: Vec3, v: Vec3, t_drop: f64| -> Vec3 {
        let spd = v.norm();
        if spd < 1e-4 {
            return Vec3::new(0.0, -G, 0.0);
        }
        let cds = base_cds * brake_mult_at(t_drop);
        let k_drag = 0.5 * atm.density(p.y) * cds * spd / mass;
        Vec3::new(-k_drag * v.x, -G - k_drag * v.y, -k_drag * v.z)
    };

    let mut t = 0.0;
    while t < tuning.max_time_s {
        let prev_pos = pos;
        let prev_ground = terrain.height_at(prev_pos.x, prev_pos.z);

        let a1 = accel(pos, vel, t);
        let vel_pred = vel + a1 * dt;
        let pos_pred = pos + vel * dt;
        let a2 = accel(pos_pred, vel_pred, t + dt);

        pos = pos + (vel + vel_pred) * (0.5 * dt);
        vel = vel + (a1 + a2) * (0.5 * dt);
        t += dt;

        let cur_ground = terrain.height_at(pos.x, pos.z);
        let h_after = pos.y - cur_ground;
        if h_after <= 0.0 {
            let h_before = (prev_pos.y - prev_ground).max(0.0);
            let denom = h_before - h_after;
            let frac = if denom > 1e-9 { (h_before / denom).clamp(0.0, 1.0) } else { 0.0 };
            let hit = prev_pos.lerp(pos, frac);
            let ground_y = terrain.height_at(hit.x, hit.z);
            let exact_hit = Vec3::new(hit.x, ground_y, hit.z);
            return Some(ImpactSolution {
                impact_pos: exact_hit,
                time_of_flight_s: delay + (t - dt + frac * dt),
                impact_speed_mps: vel.norm(),
                slant_range_m: (exact_hit - kin.pos_world).norm(),
                ground_elev_m: ground_y,
            });
        }
    }
    None
}

/// 三维空射无制导火箭弹 CCIP 解算 (支持两级推进、烧蚀减重、导轨离轨与尾翼风标效应)
pub fn simulate_rocket_ccip(
    kin: &AircraftKinematics,
    rocket: &RocketSpec,
    launcher_pitch_deg: f64,
    atm: &Atmosphere,
    terrain: &TerrainSource,
    tuning: &BallisticTuning,
) -> Option<ImpactSolution> {
    let dt = tuning.dt.clamp(0.001, 0.01);
    let max_t = rocket.time_life.clamp(3.0, tuning.max_time_s);

    let rail_dir = kin.basis.pitched_axis(launcher_pitch_deg);
    let mount_offset = kin.basis.local_to_world([0.5, -0.65, 0.0]);
    let mut pos = kin.pos_world + mount_offset;
    let base_vel = if rocket.use_start_speed { kin.vel_world } else { Vec3::default() };
    let mut vel = base_vel + rail_dir * rocket.start_speed.max(0.0);
    let mut body_axis = rail_dir;

    if pos.y <= terrain.height_at(pos.x, pos.z) {
        return None;
    }

    let area = std::f64::consts::PI * 0.25 * rocket.caliber * rocket.caliber;
    let c_drag = 0.5 * rocket.cx_k * area;

    let t0_start = rocket.fire_delay.max(0.0);
    let t0_end = t0_start + rocket.time_fire0.max(0.0);
    let t1_end = t0_end + rocket.time_fire1.max(0.0);
    let m0 = rocket.mass.max(0.1);
    let m1 = rocket.mass_end.clamp(0.05, m0);
    let m2 = rocket.mass_end1.clamp(0.05, m1);

    let stage_at = |tt: f64| -> (f64, f64) {
        if tt < t0_start {
            (m0, 0.0)
        } else if rocket.time_fire0 > 0.0 && tt < t0_end {
            let f = (tt - t0_start) / rocket.time_fire0;
            (m0 + (m1 - m0) * f, rocket.force0)
        } else if rocket.time_fire1 > 0.0 && tt < t1_end {
            let f = (tt - t0_end) / rocket.time_fire1;
            (m1 + (m2 - m1) * f, rocket.force1)
        } else {
            (m2, 0.0)
        }
    };

    // 尾翼风标效应：离轨后火箭纵轴按动压与稳定臂快速转向相对速度方向
    let weathercock_rate = (4.0 + 18.0 * rocket.dist_cm_to_stab.clamp(0.01, 0.5)).clamp(4.0, 16.0);

    let accel = |p: Vec3, v: Vec3, axis: Vec3, tt: f64| -> Vec3 {
        let (mass, thrust) = stage_at(tt);
        let spd = v.norm();
        let v_dir = v.normalized_or(axis);
        let f_drag = c_drag * atm.density(p.y) * spd * spd;
        (axis * thrust - v_dir * f_drag) * (1.0 / mass) + Vec3::new(0.0, -G, 0.0)
    };

    let mut t = 0.0;
    while t < max_t {
        let prev_pos = pos;
        let prev_ground = terrain.height_at(prev_pos.x, prev_pos.z);

        // 导轨离轨前 ~0.03s 锁死发射轴，离轨后受气动尾翼对准相对气流
        if t > 0.03 && vel.norm() > 5.0 {
            let v_hat = vel.normalized_or(body_axis);
            let blend = (weathercock_rate * dt).clamp(0.0, 1.0);
            body_axis = body_axis.lerp(v_hat, blend).normalized_or(v_hat);
        }

        let a1 = accel(pos, vel, body_axis, t);
        let mut vel_pred = vel + a1 * dt;
        if vel_pred.norm() > rocket.end_speed && rocket.end_speed > 50.0 {
            vel_pred = vel_pred.normalized_or(body_axis) * rocket.end_speed;
        }
        let pos_pred = pos + vel * dt;
        let a2 = accel(pos_pred, vel_pred, body_axis, t + dt);

        pos = pos + (vel + vel_pred) * (0.5 * dt);
        vel = vel + (a1 + a2) * (0.5 * dt);
        if vel.norm() > rocket.end_speed && rocket.end_speed > 50.0 {
            vel = vel.normalized_or(body_axis) * rocket.end_speed;
        }
        t += dt;

        let cur_ground = terrain.height_at(pos.x, pos.z);
        let h_after = pos.y - cur_ground;
        if h_after <= 0.0 {
            let h_before = (prev_pos.y - prev_ground).max(0.0);
            let denom = h_before - h_after;
            let frac = if denom > 1e-9 { (h_before / denom).clamp(0.0, 1.0) } else { 0.0 };
            let hit = prev_pos.lerp(pos, frac);
            let ground_y = terrain.height_at(hit.x, hit.z);
            let exact_hit = Vec3::new(hit.x, ground_y, hit.z);
            return Some(ImpactSolution {
                impact_pos: exact_hit,
                time_of_flight_s: t - dt + frac * dt,
                impact_speed_mps: vel.norm(),
                slant_range_m: (exact_hit - kin.pos_world).norm(),
                ground_elev_m: ground_y,
            });
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::db::CasDatabase;

    #[test]
    fn test_slick_vs_snakeye_retarded_bomb() {
        let db = CasDatabase::load_embedded_or_disk(None).unwrap();
        let slick = db.bombs.get("us_500lb_mk_82_ldgp").unwrap();
        let snakeye = db.bombs.get("us_500lb_mk_82_ldgp_snakeye").unwrap();
        let atm = Atmosphere::standard_15c();
        let terrain = TerrainSource::Flat { ground_alt_m: 50.0 };
        let tuning = BallisticTuning::default();

        // 250 m AGL, 240 m/s, -15 deg shallow CAS pass heading North (+Z)
        let kin = AircraftKinematics::from_telemetry(
            Vec3::new(0.0, 300.0, 0.0),
            0.0,
            -15.0,
            0.0,
            240.0,
            None,
            2.0,
            0.0,
        );
        let sol_slick = simulate_bomb_ccip(&kin, slick, &atm, &terrain, &tuning).unwrap();
        let sol_snake = simulate_bomb_ccip(&kin, snakeye, &atm, &terrain, &tuning).unwrap();

        assert!(
            sol_slick.impact_pos.z > sol_snake.impact_pos.z + 200.0,
            "低阻弹应显著远于蛇眼高阻减速弹: slick_z={}, snake_z={}",
            sol_slick.impact_pos.z,
            sol_snake.impact_pos.z
        );
        assert!(sol_snake.time_of_flight_s > sol_slick.time_of_flight_s + 0.8);
        assert!(sol_snake.impact_speed_mps < sol_slick.impact_speed_mps * 0.5);
    }

    #[test]
    fn test_hydra_and_hvar_rocket_ccip() {
        let db = CasDatabase::load_embedded_or_disk(None).unwrap();
        let hydra = db.rockets.get("us_hydra_70_m247").unwrap();
        let hvar = db.rockets.get("us_5_in_hvar").unwrap();
        let atm = Atmosphere::standard_15c();
        let terrain = TerrainSource::Flat { ground_alt_m: 0.0 };
        let tuning = BallisticTuning::default();

        let kin = AircraftKinematics::from_telemetry(
            Vec3::new(0.0, 450.0, 0.0),
            90.0, // East (+X)
            -25.0,
            0.0,
            180.0,
            None,
            1.5,
            0.0,
        );
        let s_hydra = simulate_rocket_ccip(&kin, hydra, 0.0, &atm, &terrain, &tuning).unwrap();
        let s_hvar = simulate_rocket_ccip(&kin, hvar, 0.0, &atm, &terrain, &tuning).unwrap();

        // Hydra-70 推重比远高于二战 HVAR，同等俯冲角飞行时间更短、下坠更小（落点更远）
        assert!(s_hydra.time_of_flight_s < s_hvar.time_of_flight_s);
        assert!(s_hydra.impact_pos.x > s_hvar.impact_pos.x + 15.0);
    }
}
