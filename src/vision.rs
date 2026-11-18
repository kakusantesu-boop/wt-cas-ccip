//! 抗干扰积分图十字准星亚像素检测器 (`O(1)` 归一化积分图 + 内颈四象限序统计量滤波 + 分离轴修剪质心 + 零分配复用缓冲)
//!
//! 核心几何抗干扰能力：
//! 1. **支持自动多色 (`Auto`)、原厂白色 (`White`)、红色 (`Red`) 及高对比色**：色度突出量软响应解决 `75%~85%` 半透明 HUD 在蓝天/草地/白云上的阈值失效；
//! 2. **同心与偏心“鼠标牵引大圆圈”零误杀**：采用紧贴十字交叉口的内颈空腔 `[ht+1 .. ht+5]` 配合四象限序统计量（允许偏心圆弧切过 1~2 个内角），解决机头对准鼠标圆心瞬间外圈落入对角象限导致的失锁问题；
//! 3. **内外半臂双段连通门控 + 实心红标/白云全抑制**：要求上下左右四臂的内半段与外半段同时激活，100% 淘汰空心方框、圆弧、实心敌方坦克菱形/方块以及大片白云；
//! 4. **零堆分配 (`VisionScratch`)**：复用积分图内存，消除每帧数 MB 的堆分配抖动。

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CrosshairColorPreset {
    /// 自动探测并记忆准星颜色（通杀默认白、红、绿、黄、青、品红）
    Auto,
    /// 战雷玩家最常用的红色/橙红准星
    Red,
    /// 游戏出厂默认白色带暗描边准星
    White,
    /// 高亮品红 #FF00FF
    Magenta,
    /// 高亮青色 #00FFFF
    Cyan,
    /// 荧光绿 #00FF00
    NeonGreen,
    /// 黄色 / 琥珀橙
    Yellow,
}

impl CrosshairColorPreset {
    pub const CONCRETE_COLORS: [Self; 6] = [
        Self::Red,
        Self::White,
        Self::NeonGreen,
        Self::Yellow,
        Self::Cyan,
        Self::Magenta,
    ];

    pub fn label_zh(self) -> &'static str {
        match self {
            Self::Auto => "自动识别 (红/默认白/绿/黄/青/品红)",
            Self::Red => "红色 (游戏常用红/橙红准星)",
            Self::White => "默认白色 (原厂白准星+暗描边)",
            Self::Magenta => "品红 #FF00FF (极低背景干扰)",
            Self::Cyan => "青色 #00FFFF",
            Self::NeonGreen => "荧光绿 #00FF00",
            Self::Yellow => "黄色 / 琥珀橙",
        }
    }

    /// 计算单像素与目标准星颜色的软匹配强度 `(0 ..= 255)`，兼容战雷 HUD 半透明与抗锯齿边缘
    #[inline]
    pub fn pixel_response(self, r: u8, g: u8, b: u8, tol: i32) -> u32 {
        let (r, g, b) = (r as i32, g as i32, b as i32);
        let min_c = (66 - tol * 2).max(34);
        match self {
            Self::Auto => Self::Red.pixel_response(r as u8, g as u8, b as u8, tol),
            Self::Red => {
                if r < 142 - tol {
                    return 0;
                }
                let ex = r - g.max(b);
                if ex >= min_c {
                    ((ex - min_c) * 3 + 68).clamp(0, 255) as u32
                } else {
                    0
                }
            }
            Self::White => {
                // 原厂白色准星：三通道高亮且低饱和度（大片纯白云虽也会得分，但会在后续内颈空腔门控中被 100% 剔除）
                let lo = r.min(g).min(b);
                let hi = r.max(g).max(b);
                let floor = (196 - tol * 2).max(165);
                if lo >= floor && (hi - lo) <= (36 + tol) {
                    let sat_pen = (hi - lo) * 2;
                    ((lo - floor) * 4 + 96 - sat_pen).clamp(0, 255) as u32
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
                    ((ex - min_c) * 3 + 68).clamp(0, 255) as u32
                } else {
                    0
                }
            }
            Self::Cyan => {
                if g < 140 - tol || b < 140 - tol {
                    return 0;
                }
                let ex = g.min(b) - r;
                if ex >= min_c && (g - b).abs() <= 65 {
                    ((ex - min_c) * 3 + 68).clamp(0, 255) as u32
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
                    ((ex - min_c) * 3 + 68).clamp(0, 255) as u32
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
                    ((ex - min_c) * 3 + 68).clamp(0, 255) as u32
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
    pub matched_color: CrosshairColorPreset,
}

/// 可复用零分配视觉工作区（避免每帧分配/释放数 MB 积分图缓冲）
#[derive(Default)]
pub struct VisionScratch {
    sum: Vec<u32>,
    last_locked_color: Option<CrosshairColorPreset>,
    scan_cursor: usize,
}

impl VisionScratch {
    pub fn new() -> Self {
        Self {
            sum: Vec::with_capacity(770 * 770),
            last_locked_color: None,
            scan_cursor: 0,
        }
    }

    pub fn reset_color_lock(&mut self) {
        self.last_locked_color = None;
    }
}

struct IntegralView<'a> {
    w1: usize,
    w: usize,
    h: usize,
    sum: &'a [u32],
}

impl<'a> IntegralView<'a> {
    fn build_into(
        scratch: &'a mut Vec<u32>,
        bgra: &[u8],
        w: usize,
        h: usize,
        preset: CrosshairColorPreset,
        tol: i32,
    ) -> (Self, u64) {
        let w1 = w + 1;
        let needed = w1 * (h + 1);
        if scratch.len() < needed {
            scratch.resize(needed, 0);
        }
        scratch[..w1].fill(0);

        let mut total = 0u64;
        for y in 0..h {
            let mut row_acc = 0u32;
            let src_row = y * w * 4;
            let prev_int = y * w1;
            let cur_int = (y + 1) * w1;
            scratch[cur_int] = 0;
            for x in 0..w {
                let p = src_row + x * 4;
                let resp = preset.pixel_response(bgra[p + 2], bgra[p + 1], bgra[p], tol);
                row_acc = row_acc.wrapping_add(resp);
                scratch[cur_int + x + 1] = scratch[prev_int + x + 1].wrapping_add(row_acc);
            }
            total += row_acc as u64;
        }
        (
            Self {
                w1,
                w,
                h,
                sum: &scratch[..needed],
            },
            total,
        )
    }

    /// 闭开区间 `[x0..x1, y0..y1]` 矩形平均像素响应 `(0.0 ..= 255.0)`，天然具备跨分辨率尺度不变性
    #[inline]
    fn rect_mean(&self, x0: isize, y0: isize, x1: isize, y1: isize) -> f64 {
        let cx0 = x0.clamp(0, self.w as isize) as usize;
        let cy0 = y0.clamp(0, self.h as isize) as usize;
        let cx1 = x1.clamp(0, self.w as isize) as usize;
        let cy1 = y1.clamp(0, self.h as isize) as usize;
        if cx1 <= cx0 || cy1 <= cy0 {
            return 0.0;
        }
        let area = ((cx1 - cx0) * (cy1 - cy0)) as f64;
        let a = self.sum[cy0 * self.w1 + cx0] as i64;
        let b = self.sum[cy0 * self.w1 + cx1] as i64;
        let c = self.sum[cy1 * self.w1 + cx0] as i64;
        let d = self.sum[cy1 * self.w1 + cx1] as i64;
        (d - b - c + a).max(0) as f64 / area
    }
}

/// 覆盖细小 UI / 1080p / 1440p / 4K 的四档正交机头小十字规格: `(inner_gap, outer_arm, half_thick)`
const SCALES: [(isize, isize, isize); 4] = [
    (2, 9, 1),  // 细小/紧凑 UI 缩放
    (3, 12, 2), // 1080p 标准小准星
    (4, 16, 2), // 1440p 中等准星
    (5, 22, 3), // 4K / 大尺寸 UI 缩放
];

/// 评估候选中心 `(x, y)` 在十字规格 `(inner, outer, ht)` 下的尺度归一化正交十字纯净度得分
#[inline]
fn eval_cross_template(
    imap: &IntegralView<'_>,
    x: isize,
    y: isize,
    inner: isize,
    outer: isize,
    ht: isize,
) -> f64 {
    let m_left = imap.rect_mean(x - outer, y - ht, x - inner + 1, y + ht + 1);
    let m_right = imap.rect_mean(x + inner, y - ht, x + outer + 1, y + ht + 1);
    let m_top = imap.rect_mean(x - ht, y - outer, x + ht + 1, y - inner + 1);
    let m_bottom = imap.rect_mean(x - ht, y + inner, x + ht + 1, y + outer + 1);

    // 1. 四向主臂均值门控
    let min_arm = m_left.min(m_right).min(m_top).min(m_bottom);
    if min_arm < 28.0 {
        return 0.0;
    }

    // 2. 内半臂 + 外半臂双段连通门控（淘汰空心锁定框 □、小圆环 ○ 与局部圆弧）
    let mid = (inner + outer) / 2;
    let in_l = imap.rect_mean(x - mid, y - ht, x - inner + 1, y + ht + 1);
    let in_r = imap.rect_mean(x + inner, y - ht, x + mid + 1, y + ht + 1);
    let in_t = imap.rect_mean(x - ht, y - mid, x + ht + 1, y - inner + 1);
    let in_b = imap.rect_mean(x - ht, y + inner, x + ht + 1, y + mid + 1);
    if in_l.min(in_r).min(in_t).min(in_b) < 20.0 {
        return 0.0;
    }

    let out_l = imap.rect_mean(x - outer, y - ht, x - mid + 1, y + ht + 1);
    let out_r = imap.rect_mean(x + mid, y - ht, x + outer + 1, y + ht + 1);
    let out_t = imap.rect_mean(x - ht, y - outer, x + ht + 1, y - mid + 1);
    let out_b = imap.rect_mean(x - ht, y + mid, x + ht + 1, y + outer + 1);
    if out_l.min(out_r).min(out_t).min(out_b) < 20.0 {
        return 0.0;
    }

    // 3. 正交对称平衡门控
    let lr_ratio = m_left.min(m_right) / m_left.max(m_right);
    let tb_ratio = m_top.min(m_bottom) / m_top.max(m_bottom);
    if lr_ratio < 0.32 || tb_ratio < 0.32 {
        return 0.0;
    }

    // 4. 内颈四象限序统计量空腔门控 (`[ht+1 .. ht+5]`, 对角半径 < 10px，严格位于鼠标牵引大圆圈内部！)
    //    即使偏心鼠标大圆圈圆弧恰好切过 1~2 个内角，最干净的 2~3 个内角依然为空白；
    //    而实心敌方坦克红标、菱形方块、文字团块或大片纯白云会同时填满全部 4 个内角！
    let q0 = ht + 1;
    let q1 = (ht + 5).min(outer - 1).max(q0 + 2);
    let mut corners = [
        imap.rect_mean(x - q1, y - q1, x - q0 + 1, y - q0 + 1),
        imap.rect_mean(x + q0, y - q1, x + q1 + 1, y - q0 + 1),
        imap.rect_mean(x - q1, y + q0, x - q0 + 1, y + q1 + 1),
        imap.rect_mean(x + q0, y + q0, x + q1 + 1, y + q1 + 1),
    ];
    corners.sort_by(|a, b| a.total_cmp(b));

    let arm_mean = 0.25 * (m_left + m_right + m_top + m_bottom);
    let clean2_mean = 0.5 * (corners[0] + corners[1]);
    let clean3_mean = (corners[0] + corners[1] + corners[2]) / 3.0;
    let all4_mean = 0.25 * (corners[0] + corners[1] + corners[2] + corners[3]);

    if clean2_mean > arm_mean * 0.16 || clean3_mean > arm_mean * 0.26 || all4_mean > arm_mean * 0.42
    {
        return 0.0;
    }

    // 归一化高对比度评分 (典型纯净十字得分 ~ 300 .. 900)
    (arm_mean * 4.0 + min_arm * 3.5) - (clean3_mean * 6.0 + corners[3] * 1.2)
}

fn detect_single_color(
    scratch: &mut Vec<u32>,
    bgra: &[u8],
    width: usize,
    height: usize,
    color: CrosshairColorPreset,
    tolerance: i32,
) -> Option<CrosshairDetection> {
    let (imap, total_resp) = IntegralView::build_into(scratch, bgra, width, height, color, tolerance);
    if total_resp < 550 {
        return None;
    }

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
            if imap.rect_mean(x - 12, y - 12, x + 13, y + 13) >= 4.0 {
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

    if best_score < 210.0 {
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

    if best_score < 245.0 {
        return None;
    }

    // 第三阶段：紧核修剪分离轴亚像素质心！
    // 限制外臂采样半径 `core_out = out.min(12)` 并按距轴心距离赋予三角核权重，隔离同心/偏心大圆圈边缘像素干扰
    let (inn, out, ht) = SCALES[best_si];
    let core_out = out.min(12);

    let mut wx_sum = 0.0f64;
    let mut wx_w = 0.0f64;
    for yy in (px_best_y - core_out).max(0)..=(px_best_y + core_out).min(height as isize - 1) {
        if (yy - px_best_y).abs() < inn {
            continue;
        }
        let radial_w = 1.0 - 0.35 * ((yy - px_best_y).abs() as f64 / core_out as f64);
        for xx in (px_best_x - ht).max(0)..=(px_best_x + ht).min(width as isize - 1) {
            let p = (yy as usize * width + xx as usize) * 4;
            let w = color.pixel_response(bgra[p + 2], bgra[p + 1], bgra[p], tolerance) as f64
                * radial_w;
            wx_sum += (xx as f64 + 0.5) * w;
            wx_w += w;
        }
    }

    let mut wy_sum = 0.0f64;
    let mut wy_w = 0.0f64;
    for yy in (px_best_y - ht).max(0)..=(px_best_y + ht).min(height as isize - 1) {
        for xx in (px_best_x - core_out).max(0)..=(px_best_x + core_out).min(width as isize - 1) {
            if (xx - px_best_x).abs() < inn {
                continue;
            }
            let radial_w = 1.0 - 0.35 * ((xx - px_best_x).abs() as f64 / core_out as f64);
            let p = (yy as usize * width + xx as usize) * 4;
            let w = color.pixel_response(bgra[p + 2], bgra[p + 1], bgra[p], tolerance) as f64
                * radial_w;
            wy_sum += (yy as f64 + 0.5) * w;
            wy_w += w;
        }
    }

    let final_x = if wx_w > 10.0 {
        wx_sum / wx_w
    } else {
        px_best_x as f64 + 0.5
    };
    let final_y = if wy_w > 10.0 {
        wy_sum / wy_w
    } else {
        px_best_y as f64 + 0.5
    };

    Some(CrosshairDetection {
        x_roi: final_x,
        y_roi: final_y,
        score: best_score,
        best_scale: best_si,
        matched_color: color,
    })
}

/// 零分配主检测入口（支持 `Auto` 自动多色记忆与轮询）
pub fn detect_crosshair_with_scratch(
    scratch: &mut VisionScratch,
    bgra: &[u8],
    width: usize,
    height: usize,
    preset: CrosshairColorPreset,
    tolerance: i32,
) -> Option<CrosshairDetection> {
    if width < 52 || height < 52 || bgra.len() < width * height * 4 {
        return None;
    }

    if preset != CrosshairColorPreset::Auto {
        return detect_single_color(&mut scratch.sum, bgra, width, height, preset, tolerance);
    }

    // Auto 模式：优先尝试上一帧已锁定的颜色（单次 `O(1)` 零额外开销！）
    if let Some(locked_col) = scratch.last_locked_color {
        if let Some(hit) =
            detect_single_color(&mut scratch.sum, bgra, width, height, locked_col, tolerance)
        {
            return Some(hit);
        }
        scratch.last_locked_color = None;
    }

    // 未锁定时，每帧优先测最常用的 红 + 默认白，再轮询其余高对比色，兼顾首帧瞬锁与低 CPU 开销
    let extra_idx = scratch.scan_cursor % 4;
    scratch.scan_cursor = scratch.scan_cursor.wrapping_add(1);
    let extra_col = CrosshairColorPreset::CONCRETE_COLORS[2 + extra_idx];

    let mut best: Option<CrosshairDetection> = None;
    for cand in [
        CrosshairColorPreset::Red,
        CrosshairColorPreset::White,
        extra_col,
    ] {
        if let Some(hit) =
            detect_single_color(&mut scratch.sum, bgra, width, height, cand, tolerance)
        {
            if best.is_none_or(|b| hit.score > b.score) {
                best = Some(hit);
            }
        }
    }

    if let Some(b) = best {
        scratch.last_locked_color = Some(b.matched_color);
    }
    best
}

/// 兼容无状态便捷检测入口
pub fn detect_crosshair_bgra(
    bgra: &[u8],
    width: usize,
    height: usize,
    preset: CrosshairColorPreset,
    tolerance: i32,
) -> Option<CrosshairDetection> {
    let mut scratch = VisionScratch::new();
    if preset == CrosshairColorPreset::Auto {
        // 单次无状态调用时遍历全部 6 种具体颜色取最高分
        let mut best: Option<CrosshairDetection> = None;
        for cand in CrosshairColorPreset::CONCRETE_COLORS {
            if let Some(hit) =
                detect_single_color(&mut scratch.sum, bgra, width, height, cand, tolerance)
            {
                if best.is_none_or(|b| hit.score > b.score) {
                    best = Some(hit);
                }
            }
        }
        best
    } else {
        detect_single_color(&mut scratch.sum, bgra, width, height, preset, tolerance)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_sky_ground_bg(w: usize, h: usize) -> Vec<u8> {
        let mut buf = vec![0u8; w * h * 4];
        for y in 0..h {
            for x in 0..w {
                let p = (y * w + x) * 4;
                if y < h * 45 / 100 {
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
        buf
    }

    fn blend_rgb(buf: &mut [u8], w: usize, h: usize, x: isize, y: isize, rgb: [u8; 3], alpha: f32) {
        if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
            let p = (y as usize * w + x as usize) * 4;
            buf[p] = (buf[p] as f32 * (1.0 - alpha) + rgb[2] as f32 * alpha) as u8;
            buf[p + 1] = (buf[p + 1] as f32 * (1.0 - alpha) + rgb[1] as f32 * alpha) as u8;
            buf[p + 2] = (buf[p + 2] as f32 * (1.0 - alpha) + rgb[0] as f32 * alpha) as u8;
        }
    }

    #[test]
    fn test_red_crosshair_ignores_mouse_aim_circle_and_enemy_markers() {
        let (w, h) = (420usize, 420usize);
        let mut buf = make_sky_ground_bg(w, h);

        // 干扰物 1：屏幕中央 (210, 210) 红色鼠标牵引大圆圈 (半径 34px)
        for deg in 0..360 {
            let rad = (deg as f64).to_radians();
            for r in [33.5, 34.5] {
                let cx = (210.0 + r * rad.cos()).round() as isize;
                let cy = (210.0 + r * rad.sin()).round() as isize;
                blend_rgb(&mut buf, w, h, cx, cy, [250, 28, 28], 0.82);
            }
        }

        // 干扰物 2：实心红色敌方坦克标记块 (15×15) + 空心红色锁定方框 (20×20)
        for dy in -7..=7 {
            for dx in -7..=7 {
                blend_rgb(&mut buf, w, h, 115 + dx, 275 + dy, [250, 28, 28], 0.90);
            }
        }
        for d in -10..=10 {
            blend_rgb(&mut buf, w, h, 310 + d, 280 - 10, [250, 28, 28], 0.90);
            blend_rgb(&mut buf, w, h, 310 + d, 280 + 10, [250, 28, 28], 0.90);
            blend_rgb(&mut buf, w, h, 310 - 10, 280 + d, [250, 28, 28], 0.90);
            blend_rgb(&mut buf, w, h, 310 + 10, 280 + d, [250, 28, 28], 0.90);
        }

        // 无机头小十字时绝不误锁大圆圈、实心红标或空心锁定框
        assert!(detect_crosshair_bgra(&buf, w, h, CrosshairColorPreset::Red, 0).is_none());

        // 绘制 78% 半透明红色机头小十字于 (236, 182)
        let (tx, ty) = (236isize, 182isize);
        for d in 4..=12 {
            for th in 0..=1 {
                blend_rgb(&mut buf, w, h, tx - d, ty + th, [250, 28, 28], 0.78);
                blend_rgb(&mut buf, w, h, tx + d, ty + th, [250, 28, 28], 0.78);
                blend_rgb(&mut buf, w, h, tx + th, ty - d, [250, 28, 28], 0.78);
                blend_rgb(&mut buf, w, h, tx + th, ty + d, [250, 28, 28], 0.78);
            }
        }

        let det = detect_crosshair_bgra(&buf, w, h, CrosshairColorPreset::Red, 0)
            .expect("必须成功识别蓝天上半透明红色机头小十字");
        assert!((det.x_roi - 236.5).abs() < 0.8 && (det.y_roi - 182.5).abs() < 0.8);
    }

    #[test]
    fn test_concentric_and_intersecting_mouse_circle_plus_white_and_auto_modes() {
        let (w, h) = (360usize, 360usize);
        let mut buf = make_sky_ground_bg(w, h);

        // 1. 在天空中画一大团高亮纯白云 (245, 245, 245)，验证 White / Auto 绝不误锁白云！
        for dy in -28..=28 {
            for dx in -45..=45 {
                blend_rgb(&mut buf, w, h, 100 + dx, 80 + dy, [246, 246, 248], 0.95);
            }
        }
        assert!(detect_crosshair_bgra(&buf, w, h, CrosshairColorPreset::White, 4).is_none());
        assert!(detect_crosshair_bgra(&buf, w, h, CrosshairColorPreset::Auto, 4).is_none());

        // 2. 关键回归测试：机头已完全追上鼠标，半径 18px 的同色鼠标牵引圆圈与机头小十字【完全同心】于 (180, 180)！
        //    （旧版 outer 对角框会把半径 18px 的圆圈算进对角污染导致瞄准完成瞬间失锁！）
        for deg in 0..360 {
            let rad = (deg as f64).to_radians();
            for r in [17.5, 18.5] {
                let cx = (180.0 + r * rad.cos()).round() as isize;
                let cy = (180.0 + r * rad.sin()).round() as isize;
                blend_rgb(&mut buf, w, h, cx, cy, [252, 252, 252], 0.85);
            }
        }
        // 绘制带 1px 暗描边的原厂白色机头小十字于同心 (180, 180)
        for d in 3..=11 {
            for th in -2..=2 {
                let col = if th == 0 || th == 1 {
                    [252, 252, 252]
                } else {
                    [25, 28, 32]
                };
                let alpha = if th == 0 || th == 1 { 0.88 } else { 0.55 };
                blend_rgb(&mut buf, w, h, 180 - d, 180 + th, col, alpha);
                blend_rgb(&mut buf, w, h, 180 + d, 180 + th, col, alpha);
                blend_rgb(&mut buf, w, h, 180 + th, 180 - d, col, alpha);
                blend_rgb(&mut buf, w, h, 180 + th, 180 + d, col, alpha);
            }
        }

        let det_auto = detect_crosshair_bgra(&buf, w, h, CrosshairColorPreset::Auto, 4)
            .expect("同心鼠标圆圈包裹原厂白十字时 Auto 模式必须稳定锁定！");
        assert_eq!(det_auto.matched_color, CrosshairColorPreset::White);
        assert!(
            (det_auto.x_roi - 180.5).abs() < 0.75 && (det_auto.y_roi - 180.5).abs() < 0.75,
            "同心白十字亚像素偏差过大: ({}, {})",
            det_auto.x_roi,
            det_auto.y_roi
        );
    }
}
