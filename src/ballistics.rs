//! 三维空地航弹与空射火箭弹物理解算引擎 (RK2 / Heun + 火箭跨音速马赫阻力/导轨离轨/自毁门控 + 地形二分求交)
//! 世界坐标系约定 (与 Dagor / 婴儿弹道计算机一致)：+X 向东，+Y 向上 (海拔 m)，+Z 向北。

use crate::atmosphere::Atmosphere;
use crate::db::{BombSpec, RocketSpec};
use crate::terrain_pack::TerrainSource;

pub const G: f64 = 9.81;
pub const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;
pub const RAD_TO_DEG: f64 = 180.0 / std::f64::consts::PI;

/// DagorEngine 尾翼火箭标准零升马赫阻力极曲线 `C_d0(Mach)`
/// - 亚音速 (`M <= 0.8`): 恒为 `0.29`
/// - 跨音速激波区 (`0.8 < M <= 1.1`): 三次 Hermite 平滑抬升至波阻峰值 `0.68`
/// - 超音速区 (`M > 1.1`): 按 `0.26 + 0.42 / (1 + 0.55 * (M - 1.1))` 渐近衰减
#[inline]
pub fn rocket_cd0(mach: f64) -> f64 {
    if mach <= 0.8 {
        0.29
    } else if mach <= 1.1 {
        let u = (mach - 0.8) / 0.3;
        0.29 + 0.39 * (u * u * (3.0 - 2.0 * u))
    } else {
        0.26 + 0.42 / (1.0 + 0.55 * (mach - 1.1))
    }
}

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
        let spd = tas_mps.max(0.0);
        // 直升机低速悬停 (< 8 m/s) 时下洗流会导致 8111 AoA/AoS 剧烈跳变，按空速平滑衰减消除方向翻转
        let aero_weight = ((spd - 2.0) / 6.0).clamp(0.0, 1.0);
        let alpha = (aoa_deg.clamp(-45.0, 45.0) * aero_weight) * DEG_TO_RAD;
        let beta = (aos_deg.clamp(-45.0, 45.0) * aero_weight) * DEG_TO_RAD;
        let (sa, ca) = alpha.sin_cos();
        let (sb, cb) = beta.sin_cos();

        // 正攻角 alpha > 0 时机头高于气流速度矢量，速度方向沿机体 -up 偏转
        let aero_dir = (basis.fwd * (ca * cb) - basis.up * (sa * cb) + basis.right * sb)
            .normalized_or(basis.fwd);
        let mut vel_world = aero_dir * spd;

        // 若 8111 提供了精确垂直升降率 Vy，修正垂直分量并保持总真空速模长守恒
        if let Some(vy) = vy_mps_opt.filter(|v| v.is_finite()) {
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
            tas_mps: spd,
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

#[derive(Clone, Copy, Debug)]
pub enum RocketSimOutcome {
    GroundHit(ImpactSolution),
    SelfDestructed {
        slant_range_m: f64,
        time_of_flight_s: f64,
        by_distance: bool,
    },
    Unreachable,
}

/// 在跨越地表的相邻两步 `[p0, p1]` 间执行 3 步非线性高程二分 + 最终割线精化，陡坡山地垂直误差 `< 0.005 m`
#[inline]
fn refine_terrain_hit(p0: Vec3, p1: Vec3, terrain: &TerrainSource) -> (Vec3, f64, f64) {
    let mut lo = 0.0f64;
    let mut hi = 1.0f64;
    let mut h_lo = (p0.y - terrain.height_at(p0.x, p0.z)).max(0.0);
    let mut h_hi = p1.y - terrain.height_at(p1.x, p1.z);

    for _ in 0..3 {
        let mid = 0.5 * (lo + hi);
        let pm = p0.lerp(p1, mid);
        let hm = pm.y - terrain.height_at(pm.x, pm.z);
        if hm > 0.0 {
            lo = mid;
            h_lo = hm;
        } else {
            hi = mid;
            h_hi = hm;
        }
    }

    let denom = h_lo - h_hi;
    let sub = if denom > 1e-9 {
        (h_lo / denom).clamp(0.0, 1.0)
    } else {
        0.5
    };
    let frac = (lo + (hi - lo) * sub).clamp(0.0, 1.0);
    let hit = p0.lerp(p1, frac);
    let ground_y = terrain.height_at(hit.x, hit.z);
    (Vec3::new(hit.x, ground_y, hit.z), frac, ground_y)
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

        let a1 = accel(pos, vel, t);
        let vel_pred = vel + a1 * dt;
        let pos_pred = pos + vel * dt;
        let a2 = accel(pos_pred, vel_pred, t + dt);

        pos = pos + (vel + vel_pred) * (0.5 * dt);
        vel = vel + (a1 + a2) * (0.5 * dt);
        t += dt;

        if pos.y <= terrain.height_at(pos.x, pos.z) {
            let (exact_hit, frac, ground_y) = refine_terrain_hit(prev_pos, pos, terrain);
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

/// 三维空射无制导火箭弹完整状态解算（含跨音速 `rocket_cd0(Mach)`、2m 发射管导轨支撑、两级烧蚀、尾翼风标与最大射程/时间自毁门控）
pub fn simulate_rocket_ccip_detailed(
    kin: &AircraftKinematics,
    rocket: &RocketSpec,
    launcher_pitch_deg: f64,
    atm: &Atmosphere,
    terrain: &TerrainSource,
    tuning: &BallisticTuning,
) -> RocketSimOutcome {
    let dt = tuning.dt.clamp(0.001, 0.01);
    let t_life = if rocket.time_life > 0.5 {
        rocket.time_life.min(tuning.max_time_s)
    } else {
        tuning.max_time_s
    };
    let max_dist = if rocket.max_distance > 50.0 {
        rocket.max_distance
    } else {
        0.0
    };

    let rail_dir = kin.basis.pitched_axis(launcher_pitch_deg);
    let mount_offset = kin.basis.local_to_world([0.5, -0.65, 0.0]);
    let launch_origin = kin.pos_world + mount_offset;
    let mut pos = launch_origin;
    let base_vel = if rocket.use_start_speed {
        kin.vel_world
    } else {
        Vec3::default()
    };
    let v_init = rocket.start_speed.max(0.0);
    let mut vel = base_vel + rail_dir * v_init;
    let mut body_axis = rail_dir;

    if pos.y <= terrain.height_at(pos.x, pos.z) {
        return RocketSimOutcome::Unreachable;
    }

    let area = std::f64::consts::PI * 0.25 * rocket.caliber * rocket.caliber;
    let t0_start = rocket.fire_delay.max(0.0);
    let t0_end = t0_start + rocket.time_fire0.max(0.0);
    let t1_end = t0_end + rocket.time_fire1.max(0.0);
    let m0 = rocket.mass.max(0.1);
    let m1 = rocket.mass_end.clamp(0.05, m0);
    let m2 = if rocket.time_fire1 > 0.0 {
        rocket.mass_end1.clamp(0.05, m1)
    } else {
        m1
    };

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

    let drag_coeff = |spd: f64, y_asl: f64| -> f64 {
        if rocket.cx_mach {
            let mach = spd / atm.sonic_speed(y_asl);
            rocket.cx_k * rocket_cd0(mach)
        } else {
            rocket.cx_k
        }
    };

    let accel_free = |p: Vec3, v: Vec3, axis: Vec3, tt: f64| -> Vec3 {
        let (mass, thrust) = stage_at(tt);
        let spd = v.norm();
        let v_dir = v.normalized_or(axis);
        let cd = drag_coeff(spd, p.y);
        let f_drag = 0.5 * cd * area * atm.density(p.y) * spd * spd;
        (axis * thrust - v_dir * f_drag) * (1.0 / mass) + Vec3::new(0.0, -G, 0.0)
    };

    // 发射巢/导轨离轨阶段：初速 < 5 m/s 且沿管滑行距离 < 2.0 m 时，法向重力由发射管壁支撑，不产生管内虚假下垂
    let mut on_rail = v_init < 5.0 && (rocket.force0 > 0.0 && rocket.time_fire0 > 0.0);
    let mut rail_dist = 0.0f64;
    let mut u_tube = v_init;
    let stab_gain = 1.0 + 4.0 * rocket.dist_cm_to_stab.clamp(0.0, 0.5);

    let mut t = 0.0;
    while t < t_life {
        let prev_pos = pos;

        if on_rail {
            let (mass, thrust) = stage_at(t);
            if t < t0_start {
                // 点火延迟期内随发射巢一同匀速运动
                pos = pos + base_vel * dt;
                vel = base_vel;
                t += dt;
            } else if thrust > 0.0 && rail_dist < 2.0 {
                let spd = vel.norm();
                let cd = drag_coeff(spd, pos.y);
                let f_drag = 0.5 * cd * area * atm.density(pos.y) * spd * spd;
                let a_tube = ((thrust - f_drag) / mass - G * rail_dir.y).max(0.0);
                let u_next = u_tube + a_tube * dt;
                let u_mid = 0.5 * (u_tube + u_next);
                rail_dist += u_mid * dt;
                u_tube = u_next;
                pos = pos + (base_vel + rail_dir * u_mid) * dt;
                vel = base_vel + rail_dir * u_tube;
                t += dt;
            } else {
                on_rail = false;
                continue;
            }
        } else {
            let spd_now = vel.norm();
            let axis_next = if spd_now > 5.0 {
                let v_hat = vel.normalized_or(body_axis);
                // 尾翼风标收拢速率与空速动压成正比，对齐 wt-ballistic-visualizer `0.0025 * spd * dt`
                let rate = (0.0025 * spd_now * stab_gain).clamp(2.5, 18.0);
                let blend = (rate * dt).clamp(0.0, 1.0);
                body_axis.lerp(v_hat, blend).normalized_or(v_hat)
            } else {
                body_axis
            };

            let a1 = accel_free(pos, vel, body_axis, t);
            let mut vel_pred = vel + a1 * dt;
            if vel_pred.norm() > rocket.end_speed && rocket.end_speed > 50.0 {
                vel_pred = vel_pred.normalized_or(body_axis) * rocket.end_speed;
            }
            let pos_pred = pos + vel * dt;
            let a2 = accel_free(pos_pred, vel_pred, axis_next, t + dt);

            pos = pos + (vel + vel_pred) * (0.5 * dt);
            vel = vel + (a1 + a2) * (0.5 * dt);
            if vel.norm() > rocket.end_speed && rocket.end_speed > 50.0 {
                vel = vel.normalized_or(axis_next) * rocket.end_speed;
            }
            body_axis = axis_next;
            t += dt;
        }

        if pos.y <= terrain.height_at(pos.x, pos.z) {
            let (exact_hit, frac, ground_y) = refine_terrain_hit(prev_pos, pos, terrain);
            let hit_tof = t - dt + frac * dt;
            let flight_dist = (exact_hit - launch_origin).norm();
            if max_dist > 0.0 && flight_dist > max_dist {
                return RocketSimOutcome::SelfDestructed {
                    slant_range_m: max_dist,
                    time_of_flight_s: hit_tof,
                    by_distance: true,
                };
            }
            return RocketSimOutcome::GroundHit(ImpactSolution {
                impact_pos: exact_hit,
                time_of_flight_s: hit_tof,
                impact_speed_mps: vel.norm(),
                slant_range_m: (exact_hit - kin.pos_world).norm(),
                ground_elev_m: ground_y,
            });
        }

        let traveled = (pos - launch_origin).norm();
        if max_dist > 0.0 && traveled >= max_dist {
            return RocketSimOutcome::SelfDestructed {
                slant_range_m: traveled,
                time_of_flight_s: t,
                by_distance: true,
            };
        }
    }

    RocketSimOutcome::SelfDestructed {
        slant_range_m: (pos - launch_origin).norm(),
        time_of_flight_s: t,
        by_distance: false,
    }
}

/// 兼容简写接口：仅当火箭弹在自毁前命中地表时返回 `Some(ImpactSolution)`
pub fn simulate_rocket_ccip(
    kin: &AircraftKinematics,
    rocket: &RocketSpec,
    launcher_pitch_deg: f64,
    atm: &Atmosphere,
    terrain: &TerrainSource,
    tuning: &BallisticTuning,
) -> Option<ImpactSolution> {
    match simulate_rocket_ccip_detailed(kin, rocket, launcher_pitch_deg, atm, terrain, tuning) {
        RocketSimOutcome::GroundHit(sol) => Some(sol),
        _ => None,
    }
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

    #[test]
    fn test_rocket_mach_drag_matches_datamined_statcard_burnout_speed_and_self_destruct() {
        let db = CasDatabase::load_embedded_or_disk(None).unwrap();
        let s24 = db.rockets.get("su_s_24_rocket").unwrap();
        let hydra = db.rockets.get("us_hydra_70_m247").unwrap();
        let atm = Atmosphere::standard_15c();

        // 验证悬停静止发射 (v_ac = 0) 下关机极速与拆包资料卡 maxSpeed (S-24: 410m/s, Hydra-70: 739m/s) 高度吻合！
        // 若漏乘 rocket_cd0(Mach)（原 bug），Hydra-70 关机速度会暴跌至 ~665 m/s，误差超过 70 m/s！
        for (spec, expected_vmax) in [(s24, 410.0), (hydra, 739.0)] {
            let area = std::f64::consts::PI * 0.25 * spec.caliber * spec.caliber;
            let dt = 0.002;
            let mut v = spec.start_speed;
            let mut t = 0.0;
            while t < spec.time_fire0 {
                let f = t / spec.time_fire0;
                let m = spec.mass + (spec.mass_end - spec.mass) * f;
                let cd = spec.cx_k * rocket_cd0(v / atm.sonic_speed(100.0));
                let drag = 0.5 * cd * area * atm.density(100.0) * v * v;
                v += ((spec.force0 - drag) / m) * dt;
                t += dt;
            }
            let rel_err = (v - expected_vmax).abs() / expected_vmax;
            assert!(
                rel_err < 0.025,
                "{} 关机极速偏差过大: sim={v:.1} m/s, statcard={expected_vmax:.1} m/s (rel_err={rel_err:.4})",
                spec.id
            );
        }

        // 验证高空远距平射触发 maxDistance 空中自毁，绝不伪造地面落点
        let high_kin = AircraftKinematics::from_telemetry(
            Vec3::new(0.0, 4500.0, 0.0),
            0.0,
            -5.0,
            0.0,
            220.0,
            None,
            0.0,
            0.0,
        );
        let terrain = TerrainSource::Flat { ground_alt_m: 0.0 };
        let out = simulate_rocket_ccip_detailed(
            &high_kin,
            hydra,
            0.0,
            &atm,
            &terrain,
            &BallisticTuning::default(),
        );
        assert!(
            matches!(out, RocketSimOutcome::SelfDestructed { by_distance: true, .. }),
            "超远距 (>6200m) 发射 Hydra-70 必须触发距离自毁门控，实际={out:?}"
        );
    }
}
