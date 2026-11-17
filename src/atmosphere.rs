//! DagorEngine 标准大气密度、压强、温度与音速模型
//! 严格对齐 GaijinEntertainment/DagorEngine/prog/gameLibs/gamePhys/props/atmosphere.cpp

const DENSITY_POLY: [f64; 5] = [1.0, -9.59387e-5, 3.53118e-9, -5.83556e-14, 2.28719e-19];
const PRESSURE_POLY: [f64; 5] = [1.0, -0.000118441, 5.6763e-9, -1.3738e-13, 1.60373e-18];
const TEMP_POLY: [f64; 5] = [1.0, -2.27712e-5, 2.18069e-10, -5.71104e-14, 3.97306e-18];

pub const STD_RO0: f64 = 1.225;
pub const H_MAX: f64 = 18300.0;

#[inline]
fn poly4(c: [f64; 5], h: f64) -> f64 {
    (((c[4] * h + c[3]) * h + c[2]) * h + c[1]) * h + c[0]
}

#[derive(Clone, Copy, Debug)]
pub struct Atmosphere {
    pressure_pa: f64,
    temp_k: f64,
    sea_level_rho: f64,
}

impl Atmosphere {
    /// 按气压 (mmHg) 与海平面温度 (°C) 构造大气环境
    pub fn new(pressure_mmhg: f64, temp_celsius: f64) -> Self {
        let pressure_pa = pressure_mmhg * (101300.0 / 760.0);
        let temp_k = temp_celsius + 273.16;
        let sea_level_rho = STD_RO0 * (pressure_pa / 101300.0) * (288.16 / temp_k);
        Self {
            pressure_pa,
            temp_k,
            sea_level_rho,
        }
    }

    /// 战争雷霆默认试飞/标准昼间大气 (760 mmHg, 15 °C -> rho0 = 1.225 kg/m^3)
    pub fn standard_15c() -> Self {
        Self::new(760.0, 15.0)
    }

    /// 社区导弹解算器常用 18 °C 环境 (760 mmHg, 18 °C)
    pub fn standard_18c() -> Self {
        Self::new(760.0, 18.0)
    }

    #[inline]
    pub fn density(&self, h_m: f64) -> f64 {
        let h_clamped = h_m.clamp(-500.0, H_MAX);
        let scale_above = if h_m > H_MAX { H_MAX / h_m } else { 1.0 };
        (self.sea_level_rho * poly4(DENSITY_POLY, h_clamped) * scale_above).max(1e-5)
    }

    #[inline]
    pub fn temperature_k(&self, h_m: f64) -> f64 {
        self.temp_k * poly4(TEMP_POLY, h_m.clamp(-500.0, H_MAX))
    }

    #[inline]
    pub fn pressure_pa(&self, h_m: f64) -> f64 {
        let h_clamped = h_m.clamp(-500.0, H_MAX);
        let scale_above = if h_m > H_MAX { H_MAX / h_m } else { 1.0 };
        self.pressure_pa * poly4(PRESSURE_POLY, h_clamped) * scale_above
    }

    #[inline]
    pub fn sonic_speed(&self, h_m: f64) -> f64 {
        20.1 * self.temperature_k(h_m).max(50.0).sqrt()
    }

    #[inline]
    pub fn ias_to_tas(&self, ias_mps: f64, h_m: f64) -> f64 {
        ias_mps * (STD_RO0 / self.density(h_m)).sqrt()
    }
}

impl Default for Atmosphere {
    fn default() -> Self {
        Self::standard_15c()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dagor_atmosphere_profile() {
        let atm = Atmosphere::standard_15c();
        assert!((atm.density(0.0) - 1.225).abs() < 1e-4);
        let rho_5k = atm.density(5000.0);
        assert!(rho_5k > 0.70 && rho_5k < 0.76, "rho(5000m)={rho_5k}");
        let rho_10k = atm.density(10000.0);
        assert!(rho_10k > 0.38 && rho_10k < 0.45, "rho(10000m)={rho_10k}");
        // TAS > IAS at altitude
        assert!(atm.ias_to_tas(150.0, 4000.0) > 175.0);
        assert!((atm.sonic_speed(0.0) - 341.2).abs() < 2.0);
    }
}
