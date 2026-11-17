//! 抗干扰积分图十字准星亚像素检测器 (`O(1)` 矩形积分图 + 正交四臂/对角空腔滤波 + 分离轴质心)
//!
//! 解决三大实机痛点：
//! 1. **支持红色准星 (`Red`) 及半透明混合**：改用色度突出量 (`Chromatic Excess`) 软评分，解决战雷准星 `75%~85%` 半透明叠加在蓝天/草地/雪地上时 RGB 阈值失效的问题；
//! 2. **彻底排斥同色“鼠标牵引大圆圈”与敌方红标**：鼠标大圆圈像素虽多但中心空心、圆弧无四向正交十字臂；敌方红标/文字对角象限充满像素。通过“四向正交臂激活 + 四对角象限空腔惩罚”积分图卷积，即使小十字与鼠标大圆圈部分重叠也能精确剥离；
//! 3. **分离轴亚像素定位**：用竖臂算 `X` 质心、横臂算 `Y` 质心，误差压至 `< 0.5 px`。

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CrosshairColorPreset {
    /// 战雷玩家最常用的红色准星 (已做敌方红标与鼠标大圆圈抗干扰抑制)
    Red,
    /// 高亮品红 #FF00FF
    Magenta,
    /// 高亮青色 #00FFFF
    Cyan,
    /// 荧光绿 #00FF00
    NeonGreen,
    /// 黄色 / 橙黄色
    Yellow,
}

impl CrosshairColorPreset {
    pub fn label_zh(self) -> &'static str {
        match self {
            Self::Red => "红色 (兼容游戏默认红/橙红准星)",
            Self::Magenta => "品红 #FF00FF (高对比极低干扰)",
            Self::Cyan => "青色 #00FFFF",
            Self::NeonGreen => "荧光绿 #00FF00",
            Self::Yellow => "黄色 / 琥珀橙",
        }
    }

    /// 计算单像素与目标准星颜色的软匹配强度 `(0 ..= 255)`，兼容战雷 HUD 半透明与抗锯齿边缘
    #[inline]
    pub fn pixel_response(self, r: u8, g: u8, b: u8, tol: i32) -> u32 {
        let (r, g, b) = (r as i32, g as i32, b as i32);
        let min_c = (68 - tol * 2).max(35);
        match self {
            Self::Red => {
                // 红色：R 显著高于 G 与 B（蓝天 B>R、草地 G>R、白云 R≈G≈B 均得 0 分）
                if r < 145 - tol {
                    return 0;
                }
                let ex = r - g.max(b);
                if ex >= min_c {
                    ((ex - min_c) * 3 + 64).clamp(0, 255) as u32
                } else {
                    0
                }
            }
            Self::Magenta => {
                if r < 140 - tol || b < 140 - tol {
                    return 0;
                }
                let ex = r.min(b) - g;
                if ex >= min_c && (r - b).abs() <= 95 {
                    ((ex - min_c) * 3 + 64).clamp(0, 255) as u32
                } else {
                    0
                }
            }
            Self::Cyan => {
                if g < 140 - tol || b < 140 - tol {
                    return 0;
                }
                let ex = g.min(b) - r;
                // 限制 |G - B| 防止深蓝天空误触发青色
                if ex >= min_c && (g - b).abs() <= 65 {
                    ((ex - min_c) * 3 + 64).clamp(0, 255) as u32
                } else {
                    0
                }
            }
            Self::NeonGreen => {
                if g < 140 - tol {
                    return 0;
                }
                let ex = g - r.max(b);
                if ex >= min_c {
                    ((ex - min_c) * 3 + 64).clamp(0, 255) as u32
                } else {
                    0
                }
            }
            Self::Yellow => {
                if r < 150 - tol || g < 115 - tol {
                    return 0;
                }
                let ex = r.min(g) - b;
                if ex >= min_c && (r - g) >= -35 && (r - g) <= 110 {
                    ((ex - min_c) * 3 + 64).clamp(0, 255) as u32
                } else {
                    0
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CrosshairDetection {
    pub x_roi: f64,
    pub y_roi: f64,
    pub score: f64,
    pub best_scale: usize,
}

/// 二维积分图 (`(w + 1) × (h + 1)`)，`O(1)` 查询任意轴对齐矩形像素响应和
struct IntegralMap {
    w1: usize,
    w: usize,
    h: usize,
    sum: Vec<u32>,
}

impl IntegralMap {
    fn from_bgra(bgra: &[u8], w: usize, h: usize, preset: CrosshairColorPreset, tol: i32) -> (Self, u64) {
        let w1 = w + 1;
        let mut sum = vec![0u32; w1 * (h + 1)];
        let mut total = 0u64;
        for y in 0..h {
            let mut row_acc = 0u32;
            let src_row = y * w * 4;
            let prev_int = y * w1;
            let cur_int = (y + 1) * w1;
            for x in 0..w {
                let p = src_row + x * 4;
                let resp = preset.pixel_response(bgra[p + 2], bgra[p + 1], bgra[p], tol);
                row_acc = row_acc.wrapping_add(resp);
                sum[cur_int + x + 1] = sum[prev_int + x + 1].wrapping_add(row_acc);
            }
            total += row_acc as u64;
        }
        (Self { w1, w, h, sum }, total)
    }

    /// 闭开区间 `[x0..x1, y0..y1]` 矩形和
    #[inline]
    fn rect_sum(&self, x0: isize, y0: isize, x1: isize, y1: isize) -> f64 {
        let cx0 = x0.clamp(0, self.w as isize) as usize;
        let cy0 = y0.clamp(0, self.h as isize) as usize;
        let cx1 = x1.clamp(0, self.w as isize) as usize;
        let cy1 = y1.clamp(0, self.h as isize) as usize;
        if cx1 <= cx0 || cy1 <= cy0 {
            return 0.0;
        }
        let a = self.sum[cy0 * self.w1 + cx0] as i64;
        let b = self.sum[cy0 * self.w1 + cx1] as i64;
        let c = self.sum[cy1 * self.w1 + cx0] as i64;
        let d = self.sum[cy1 * self.w1 + cx1] as i64;
        (d - b - c + a).max(0) as f64
    }
}

/// 评估候选中心 `(x, y)` 在十字臂长规格 `(inner, outer, half_thick)` 下的正交十字纯净度得分
#[inline]
fn eval_cross_template(imap: &IntegralMap, x: isize, y: isize, inner: isize, outer: isize, ht: isize) -> f64 {
    let left = imap.rect_sum(x - outer, y - ht, x - inner + 1, y + ht + 1);
    let right = imap.rect_sum(x + inner, y - ht, x + outer + 1, y + ht + 1);
    let top = imap.rect_sum(x - ht, y - outer, x + ht + 1, y - inner + 1);
    let bottom = imap.rect_sum(x - ht, y + inner, x + ht + 1, y + outer + 1);

    // 1. 上下左右四臂必须全部存在，且外半段臂尖也必须有像素（直接淘汰小圆点/小方块敌标与圆弧）
    let min_arm = left.min(right).min(top).min(bottom);
    if min_arm < 180.0 {
        return 0.0;
    }
    let mid = (inner + outer) / 2;
    let tip_l = imap.rect_sum(x - outer, y - ht, x - mid + 1, y + ht + 1);
    let tip_r = imap.rect_sum(x + mid, y - ht, x + outer + 1, y + ht + 1);
    let tip_t = imap.rect_sum(x - ht, y - outer, x + ht + 1, y - mid + 1);
    let tip_b = imap.rect_sum(x - ht, y + mid, x + ht + 1, y + outer + 1);
    if tip_l.min(tip_r).min(tip_t).min(tip_b) < 60.0 {
        return 0.0;
    }

    // 2. 对称平衡度检验
    let lr_ratio = left.min(right) / left.max(right);
    let tb_ratio = top.min(bottom) / top.max(bottom);
    if lr_ratio < 0.30 || tb_ratio < 0.30 {
        return 0.0;
    }

    // 3. 紧贴十字臂外侧的四对角象限空腔检测 (`|dx| > ht && |dy| > ht`)
    // 正交十字在对角象限应几乎全空 (< 12%)，而实心敌方坦克菱形/方块/文字团块在对角象限充满像素！
    let q0 = ht + 1;
    let q1 = outer;
    let diag_nw = imap.rect_sum(x - q1, y - q1, x - q0 + 1, y - q0 + 1);
    let diag_ne = imap.rect_sum(x + q0, y - q1, x + q1 + 1, y - q0 + 1);
    let diag_sw = imap.rect_sum(x - q1, y + q0, x - q0 + 1, y + q1 + 1);
    let diag_se = imap.rect_sum(x + q0, y + q0, x + q1 + 1, y + q1 + 1);
    let diag_sum = diag_nw + diag_ne + diag_sw + diag_se;

    let arm_sum = left + right + top + bottom;
    if diag_sum > arm_sum * 0.14 {
        return 0.0;
    }

    arm_sum + 3.5 * min_arm - 4.0 * diag_sum
}

/// 在 BGRA8 局部截图中高精度定位战雷机头小十字准星中心
pub fn detect_crosshair_bgra(
    bgra: &[u8],
    width: usize,
    height: usize,
    preset: CrosshairColorPreset,
    tolerance: i32,
) -> Option<CrosshairDetection> {
    if width < 40 || height < 40 || bgra.len() < width * height * 4 {
        return None;
    }

    let (imap, total_resp) = IntegralMap::from_bgra(bgra, width, height, preset, tolerance);
    if total_resp < 700 {
        return None;
    }

    // 覆盖 1080p / 1440p / 4K 不同 UI 缩放下的战雷机头小十字尺寸规格: (inner_gap, outer_arm, half_thick)
    const SCALES: [(isize, isize, isize); 3] = [
        (3, 12, 2), // 1080p / 标准小准星
        (4, 16, 2), // 1440p / 中等准星
        (5, 22, 3), // 4K / 大尺寸 UI 缩放
    ];

    let margin = 24isize;
    if (width as isize) <= margin * 2 || (height as isize) <= margin * 2 {
        return None;
    }

    // 第一阶段：步长 2 粗扫全局峰值
    let mut best_score = 0.0f64;
    let (mut best_x, mut best_y, mut best_si) = (0isize, 0isize, 0usize);

    let mut y = margin;
    while y < (height as isize) - margin {
        let mut x = margin;
        while x < (width as isize) - margin {
            // 快速预筛：中心 ±14px 内若几乎无目标色像素则直接跳过
            if imap.rect_sum(x - 14, y - 14, x + 15, y + 15) >= 700.0 {
                for (si, &(inn, out, ht)) in SCALES.iter().enumerate() {
                    let sc = eval_cross_template(&imap, x, y, inn, out, ht);
                    if sc > best_score {
                        best_score = sc;
                        best_x = x;
                        best_y = y;
                        best_si = si;
                    }
                }
            }
            x += 2;
        }
        y += 2;
    }

    if best_score < 1200.0 {
        return None;
    }

    // 第二阶段：在粗峰值周围 ±3px 内做步长 1 逐像素精搜
    let (mut px_best_x, mut px_best_y) = (best_x, best_y);
    for dy in -3..=3 {
        for dx in -3..=3 {
            let cx = best_x + dx;
            let cy = best_y + dy;
            for (si, &(inn, out, ht)) in SCALES.iter().enumerate() {
                let sc = eval_cross_template(&imap, cx, cy, inn, out, ht);
                if sc > best_score {
                    best_score = sc;
                    px_best_x = cx;
                    px_best_y = cy;
                    best_si = si;
                }
            }
        }
    }

    // 第三阶段：分离轴亚像素质心精化！
    // 用上下竖臂（窄 X 宽 `±ht`，长 Y `±outer`）解精确 `sub_x`，
    // 用左右横臂（长 X `±outer`，窄 Y 高 `±ht`）解精确 `sub_y`，彻底隔离外部圆圈边缘牵引误差！
    let (inn, out, ht) = SCALES[best_si];
    let mut wx_sum = 0.0f64;
    let mut wx_w = 0.0f64;
    for yy in (px_best_y - out).max(0)..=(px_best_y + out).min(height as isize - 1) {
        if (yy - px_best_y).abs() < inn - 1 {
            continue;
        }
        for xx in (px_best_x - ht - 1).max(0)..=(px_best_x + ht + 1).min(width as isize - 1) {
            let p = (yy as usize * width + xx as usize) * 4;
            let w = preset.pixel_response(bgra[p + 2], bgra[p + 1], bgra[p], tolerance) as f64;
            wx_sum += (xx as f64 + 0.5) * w;
            wx_w += w;
        }
    }

    let mut wy_sum = 0.0f64;
    let mut wy_w = 0.0f64;
    for yy in (px_best_y - ht - 1).max(0)..=(px_best_y + ht + 1).min(height as isize - 1) {
        for xx in (px_best_x - out).max(0)..=(px_best_x + out).min(width as isize - 1) {
            if (xx - px_best_x).abs() < inn - 1 {
                continue;
            }
            let p = (yy as usize * width + xx as usize) * 4;
            let w = preset.pixel_response(bgra[p + 2], bgra[p + 1], bgra[p], tolerance) as f64;
            wy_sum += (yy as f64 + 0.5) * w;
            wy_w += w;
        }
    }

    let final_x = if wx_w > 10.0 { wx_sum / wx_w } else { px_best_x as f64 + 0.5 };
    let final_y = if wy_w > 10.0 { wy_sum / wy_w } else { px_best_y as f64 + 0.5 };

    Some(CrosshairDetection {
        x_roi: final_x,
        y_roi: final_y,
        score: best_score,
        best_scale: best_si,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_red_crosshair_ignores_mouse_aim_circle_and_enemy_markers() {
        let (w, h) = (420usize, 420usize);
        let mut buf = vec![0u8; w * h * 4];
        // 填充蓝天 (上半) 与绿林草地 (下半) 背景
        for y in 0..h {
            for x in 0..w {
                let p = (y * w + x) * 4;
                if y < 190 {
                    buf[p] = 215;     // B
                    buf[p + 1] = 168; // G
                    buf[p + 2] = 122; // R
                } else {
                    buf[p] = 65;
                    buf[p + 1] = 98;
                    buf[p + 2] = 74;
                }
                buf[p + 3] = 255;
            }
        }

        // 模拟战雷 78% 半透明红色 HUD 画笔混合函数
        let blend_red = |buf: &mut [u8], x: isize, y: isize, alpha: f32| {
            if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
                let p = (y as usize * w + x as usize) * 4;
                buf[p] = (buf[p] as f32 * (1.0 - alpha) + 28.0 * alpha) as u8;
                buf[p + 1] = (buf[p + 1] as f32 * (1.0 - alpha) + 28.0 * alpha) as u8;
                buf[p + 2] = (buf[p + 2] as f32 * (1.0 - alpha) + 250.0 * alpha) as u8;
            }
        };

        // 干扰物 1：屏幕中央 (210, 210) 画一个巨大的同色红色“鼠标牵引大圆圈” (半径 34px，像素总数远多于机头小十字！)
        for deg in 0..360 {
            let rad = (deg as f64).to_radians();
            for r in [33.5, 34.5] {
                let cx = (210.0 + r * rad.cos()).round() as isize;
                let cy = (210.0 + r * rad.sin()).round() as isize;
                blend_red(&mut buf, cx, cy, 0.82);
            }
        }

        // 干扰物 2：(115, 275) 画一个实心红色敌方坦克标记块 (14×14 实心菱形/方块)
        for dy in -7..=7 {
            for dx in -7..=7 {
                blend_red(&mut buf, 115 + dx, 275 + dy, 0.90);
            }
        }

        // 仅存在大圆圈与敌方红标、没有机头小十字时：必须返回 None（绝不误锁大圆圈或敌方红标！）
        assert!(
            detect_crosshair_bgra(&buf, w, h, CrosshairColorPreset::Red, 0).is_none(),
            "无小十字时不应误锁同色鼠标大圆圈或敌方坦克红标"
        );

        // 现在在 (236, 182) (蓝天背景且紧挨鼠标大圆圈边缘！) 绘制 78% 半透明红色机头小十字
        let (tx, ty) = (236isize, 182isize);
        for d in 4..=12 {
            for th in 0..=1 {
                blend_red(&mut buf, tx - d, ty + th, 0.78);
                blend_red(&mut buf, tx + d, ty + th, 0.78);
                blend_red(&mut buf, tx + th, ty - d, 0.78);
                blend_red(&mut buf, tx + th, ty + d, 0.78);
            }
        }

        let det = detect_crosshair_bgra(&buf, w, h, CrosshairColorPreset::Red, 0)
            .expect("必须成功识别蓝天上半透明红色机头小十字");
        assert!(
            (det.x_roi - 236.5).abs() < 0.8 && (det.y_roi - 182.5).abs() < 0.8,
            "红色机头小十字亚像素定位偏差过大: ({}, {}), 预期 (236.5, 182.5)",
            det.x_roi,
            det.y_roi
        );
    }
}
