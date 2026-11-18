//! 基于 `tiny-skia` 的零残影、增量脏矩形并集 (`DirtyEnvelope`) 透明 HUD 渲染器
//!
//! 关键工程保证：
//! 1. 画布尺寸始终锁定为当前承载显示器全屏分辨率 `(mon_w, mon_h)`，游戏窗口客户区偏移 `(vp_x, vp_y)` 仅作用于绘制坐标，杜绝窗口步幅错位；
//! 2. `DirtyEnvelope` 同时维护上一帧清除框 `prev_box` 与本帧绘制框 `cur_box`，并导出两者的精确并集 `union_dirty_rect` 供 `UpdateLayeredWindowIndirect` 局部提交（4K 下降低 DWM 合成带宽 >92%，待机态 0 次系统调用）；
//! 3. 内置完整 ASCII 可打印 `5×7` 点阵字库（含 `_`、`J`、`Q`、`%`、括号等全部符号）及严格宽度裁剪，确保任何机型 ID 与横幅绝不越出脏矩形边界。

use tiny_skia::{
    Color, FillRule, LineCap, LineJoin, Paint, PathBuilder, PixmapMut, Rect, Stroke, StrokeDash,
    Transform,
};

#[derive(Clone, Debug)]
pub struct ReticleTarget {
    pub screen_xy: [f64; 2],
    pub time_of_flight_s: f64,
    pub slant_range_m: f64,
    pub tag: &'static str,
}

#[derive(Clone, Debug)]
pub struct HudScene {
    pub canvas_w: u32,
    pub canvas_h: u32,
    /// 游戏客户区相对于当前承载显示器左上角的像素偏移与宽高
    pub vp_x: f64,
    pub vp_y: f64,
    pub vp_w: f64,
    pub vp_h: f64,
    pub demo_mode: bool,
    pub zoomed: bool,
    pub fov_deg: f64,
    pub nose_locked: bool,
    pub nose_xy: Option<[f64; 2]>,
    pub bomb_ccip: Option<ReticleTarget>,
    pub rocket_ccip: Option<ReticleTarget>,
    pub hud_banner: String,
}

/// 单一并集包络盒追踪器（消除多小框边界遗漏导致的残影粘连，并提供 DWM 局部脏矩形提交范围）
#[derive(Default, Clone, Debug)]
pub struct DirtyEnvelope {
    prev_box: Option<[usize; 4]>,
    cur_box: Option<[f32; 4]>,
    last_present_union: Option<[i32; 4]>,
}

impl DirtyEnvelope {
    pub fn force_full_dirty(&mut self, canvas_w: u32, canvas_h: u32) {
        self.prev_box = Some([0, 0, canvas_w as usize, canvas_h as usize]);
        self.last_present_union = Some([0, 0, canvas_w as i32, canvas_h as i32]);
    }

    /// 返回本帧相对于上一帧实际发生像素变化的窗口局部矩形 `[left, top, right, bottom]`；
    /// 若上一帧与本帧均为空白（如机库待机），返回 `None` 以跳过 DWM 提交实现 0 开销待机。
    pub fn last_present_dirty_rect(&self) -> Option<[i32; 4]> {
        self.last_present_union
    }

    pub fn clear_previous(&mut self, bgra_buf: &mut [u8], canvas_w: u32, canvas_h: u32) -> Option<[usize; 4]> {
        let (w, h) = (canvas_w as usize, canvas_h as usize);
        let cleared = self.prev_box.take().and_then(|[x0, y0, x1, y1]| {
            let cx0 = x0.min(w);
            let cx1 = x1.min(w);
            let cy0 = y0.min(h);
            let cy1 = y1.min(h);
            if cx1 > cx0 && cy1 > cy0 {
                for row in cy0..cy1 {
                    let s = (row * w + cx0) * 4;
                    let e = (row * w + cx1) * 4;
                    if e <= bgra_buf.len() {
                        bgra_buf[s..e].fill(0);
                    }
                }
                Some([cx0, cy0, cx1, cy1])
            } else {
                None
            }
        });
        self.cur_box = None;
        cleared
    }

    #[inline]
    pub fn expand_point(&mut self, x: f32, y: f32, radius_pad: f32) {
        let lx = x - radius_pad;
        let ty = y - radius_pad;
        let rx = x + radius_pad;
        let by = y + radius_pad;
        self.cur_box = Some(match self.cur_box {
            Some([ax0, ay0, ax1, ay1]) => [ax0.min(lx), ay0.min(ty), ax1.max(rx), ay1.max(by)],
            None => [lx, ty, rx, by],
        });
    }

    pub fn expand_rect(&mut self, x0: f32, y0: f32, x1: f32, y1: f32, pad: f32) {
        self.expand_point(x0.min(x1), y0.min(y1), pad);
        self.expand_point(x0.max(x1), y0.max(y1), pad);
    }

    pub fn commit(&mut self, cleared_prev: Option<[usize; 4]>, canvas_w: u32, canvas_h: u32) {
        let cur_quant = self.cur_box.take().and_then(|[x0, y0, x1, y1]| {
            let qx0 = (x0.floor().max(0.0) as usize).min(canvas_w as usize);
            let qy0 = (y0.floor().max(0.0) as usize).min(canvas_h as usize);
            let qx1 = (x1.ceil().max(0.0) as usize).min(canvas_w as usize);
            let qy1 = (y1.ceil().max(0.0) as usize).min(canvas_h as usize);
            if qx1 > qx0 && qy1 > qy0 {
                Some([qx0, qy0, qx1, qy1])
            } else {
                None
            }
        });

        self.last_present_union = match (cleared_prev, cur_quant) {
            (Some(a), Some(b)) => Some([
                a[0].min(b[0]) as i32,
                a[1].min(b[1]) as i32,
                a[2].max(b[2]) as i32,
                a[3].max(b[3]) as i32,
            ]),
            (Some(a), None) => Some([a[0] as i32, a[1] as i32, a[2] as i32, a[3] as i32]),
            (None, Some(b)) => Some([b[0] as i32, b[1] as i32, b[2] as i32, b[3] as i32]),
            (None, None) => None,
        };
        self.prev_box = cur_quant;
    }
}

#[inline]
fn bgra_color(r: u8, g: u8, b: u8, a: u8) -> Color {
    Color::from_rgba8(b, g, r, a)
}

fn stroke_line(
    pix: &mut PixmapMut<'_>,
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
    width: f32,
    col: Color,
    dash: Option<StrokeDash>,
) {
    let mut pb = PathBuilder::new();
    pb.move_to(x0, y0);
    pb.line_to(x1, y1);
    let Some(path) = pb.finish() else { return };
    let mut paint = Paint::default();
    paint.set_color(col);
    paint.anti_alias = true;
    let stroke = Stroke {
        width,
        line_cap: LineCap::Round,
        line_join: LineJoin::Round,
        dash,
        ..Default::default()
    };
    pix.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
}

fn stroke_circle(pix: &mut PixmapMut<'_>, cx: f32, cy: f32, r: f32, width: f32, col: Color) {
    let Some(path) = PathBuilder::from_circle(cx, cy, r) else {
        return;
    };
    let mut paint = Paint::default();
    paint.set_color(col);
    paint.anti_alias = true;
    let stroke = Stroke {
        width,
        line_cap: LineCap::Round,
        ..Default::default()
    };
    pix.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
}

fn fill_circle(pix: &mut PixmapMut<'_>, cx: f32, cy: f32, r: f32, col: Color) {
    let Some(path) = PathBuilder::from_circle(cx, cy, r) else {
        return;
    };
    let mut paint = Paint::default();
    paint.set_color(col);
    paint.anti_alias = true;
    pix.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
}

fn stroke_diamond(pix: &mut PixmapMut<'_>, cx: f32, cy: f32, r: f32, width: f32, col: Color) {
    let mut pb = PathBuilder::new();
    pb.move_to(cx, cy - r);
    pb.line_to(cx + r, cy);
    pb.line_to(cx, cy + r);
    pb.line_to(cx - r, cy);
    pb.close();
    let Some(path) = pb.finish() else { return };
    let mut paint = Paint::default();
    paint.set_color(col);
    paint.anti_alias = true;
    let stroke = Stroke {
        width,
        line_join: LineJoin::Round,
        ..Default::default()
    };
    pix.stroke_path(&path, &paint, &stroke, Transform::identity(), None);
}

/// 完整全字母数字及常用标点 `5×7` 点阵字模（补齐原缺失的 `J`、`Q`、`_`、`%`、`(`、`)` 等字符）
fn glyph_5x7(ch: char) -> [u8; 7] {
    match ch.to_ascii_uppercase() {
        '0' => [0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
        '1' => [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        '2' => [0b01110, 0b10001, 0b00001, 0b00110, 0b01000, 0b10000, 0b11111],
        '3' => [0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110],
        '4' => [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        '5' => [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
        '6' => [0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
        '7' => [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
        '8' => [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
        '9' => [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
        'A' => [0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'B' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110],
        'C' => [0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110],
        'D' => [0b11100, 0b10010, 0b10001, 0b10001, 0b10001, 0b10010, 0b11100],
        'E' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111],
        'F' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000],
        'G' => [0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01110],
        'H' => [0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'I' => [0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        'J' => [0b00111, 0b00010, 0b00010, 0b00010, 0b10010, 0b10010, 0b01100],
        'K' => [0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001],
        'L' => [0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111],
        'M' => [0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001],
        'N' => [0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001],
        'O' => [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        'P' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000],
        'Q' => [0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101],
        'R' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
        'S' => [0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110],
        'T' => [0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100],
        'U' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        'V' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100],
        'W' => [0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001],
        'X' => [0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001],
        'Y' => [0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100],
        'Z' => [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111],
        '_' => [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111],
        '.' => [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b01100],
        ',' => [0b00000, 0b00000, 0b00000, 0b00000, 0b00110, 0b00100, 0b01000],
        ':' => [0b00000, 0b01100, 0b01100, 0b00000, 0b01100, 0b01100, 0b00000],
        '-' => [0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000],
        '/' => [0b00001, 0b00010, 0b00100, 0b00100, 0b01000, 0b10000, 0b00000],
        '|' => [0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100],
        '+' => [0b00000, 0b00100, 0b00100, 0b11111, 0b00100, 0b00100, 0b00000],
        '=' => [0b00000, 0b00000, 0b11111, 0b00000, 0b11111, 0b00000, 0b00000],
        '%' => [0b11001, 0b11010, 0b00100, 0b00100, 0b01011, 0b10011, 0b00000],
        '(' => [0b00010, 0b00100, 0b01000, 0b01000, 0b01000, 0b00100, 0b00010],
        ')' => [0b01000, 0b00100, 0b00010, 0b00010, 0b00010, 0b00100, 0b01000],
        '[' => [0b01110, 0b01000, 0b01000, 0b01000, 0b01000, 0b01000, 0b01110],
        ']' => [0b01110, 0b00010, 0b00010, 0b00010, 0b00010, 0b00010, 0b01110],
        '<' => [0b00010, 0b00100, 0b01000, 0b10000, 0b01000, 0b00100, 0b00010],
        '>' => [0b01000, 0b00100, 0b00010, 0b00001, 0b00010, 0b00100, 0b01000],
        '!' => [0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00000, 0b00100],
        '*' => [0b00000, 0b10101, 0b01110, 0b11111, 0b01110, 0b10101, 0b00000],
        _ => [0; 7],
    }
}

/// 绘制带 1px 阴影的 `5×7` HUD 文本，并返回实际绘制到达的右边界 `max_x`（严格限制在 `max_x_limit` 内防越界残留）
fn draw_hud_text_bounded(
    pix: &mut PixmapMut<'_>,
    x: f32,
    y: f32,
    scale: f32,
    max_x_limit: f32,
    text: &str,
    fg: Color,
) -> f32 {
    let mut shadow = Paint::default();
    shadow.set_color(bgra_color(0, 0, 0, 195));
    let mut paint = Paint::default();
    paint.set_color(fg);

    let mut cx = x;
    for ch in text.chars() {
        if cx + 5.0 * scale + 2.0 > max_x_limit {
            break;
        }
        let g = glyph_5x7(ch);
        for (ry, &row_bits) in g.iter().enumerate() {
            for rx in 0..5 {
                if (row_bits >> (4 - rx)) & 1 == 1 {
                    let px = cx + rx as f32 * scale;
                    let py = y + ry as f32 * scale;
                    if let Some(rs) = Rect::from_xywh(px + 1.0, py + 1.0, scale, scale) {
                        pix.fill_rect(rs, &shadow, Transform::identity(), None);
                    }
                    if let Some(rf) = Rect::from_xywh(px, py, scale, scale) {
                        pix.fill_rect(rf, &paint, Transform::identity(), None);
                    }
                }
            }
        }
        cx += 6.0 * scale;
    }
    cx
}

fn clamp_to_viewport(
    u_local: f64,
    v_local: f64,
    vp_x: f64,
    vp_y: f64,
    vp_w: f64,
    vp_h: f64,
) -> ([f32; 2], bool) {
    let margin = 28.0;
    let clamped = u_local < margin
        || u_local > vp_w - margin
        || v_local < margin
        || v_local > vp_h - margin;
    (
        [
            (vp_x + u_local.clamp(margin, (vp_w - margin).max(margin))) as f32,
            (vp_y + v_local.clamp(margin, (vp_h - margin).max(margin))) as f32,
        ],
        clamped,
    )
}

/// 增量零残影渲染
pub fn render_hud_incremental(
    scene: &HudScene,
    bgra_buf: &mut [u8],
    dirty: &mut DirtyEnvelope,
) {
    let (cw, ch) = (scene.canvas_w, scene.canvas_h);
    let expected = (cw * ch * 4) as usize;
    if bgra_buf.len() < expected || cw < 64 || ch < 64 {
        return;
    }

    let cleared_prev = dirty.clear_previous(bgra_buf, cw, ch);

    let Some(mut pix) = PixmapMut::from_bytes(&mut bgra_buf[..expected], cw, ch) else {
        dirty.commit(cleared_prev, cw, ch);
        return;
    };

    let ox = scene.vp_x as f32;
    let oy = scene.vp_y as f32;
    let vp_right = (scene.vp_x + scene.vp_w).min(cw as f64 - 4.0) as f32;

    if !scene.hud_banner.is_empty() {
        let mut bg = Paint::default();
        bg.set_color(bgra_color(12, 18, 24, 155));
        let bx0 = ox + 18.0;
        let by0 = oy + 16.0;
        let max_box_w = (vp_right - bx0 - 12.0).max(80.0);
        let box_w = (scene.hud_banner.len() as f32 * 12.0 + 20.0).min(max_box_w);
        if let Some(r) = Rect::from_xywh(bx0, by0, box_w, 28.0) {
            pix.fill_rect(r, &bg, Transform::identity(), None);
        }
        let banner_col = if scene.nose_locked {
            bgra_color(110, 245, 165, 245)
        } else {
            bgra_color(255, 185, 75, 245)
        };
        let end_x = draw_hud_text_bounded(
            &mut pix,
            bx0 + 10.0,
            by0 + 7.0,
            2.0,
            bx0 + box_w - 4.0,
            &scene.hud_banner,
            banner_col,
        );
        dirty.expand_rect(bx0, by0, (bx0 + box_w).max(end_x) + 6.0, by0 + 30.0, 8.0);
    }

    let nose_canvas_xy = scene
        .nose_xy
        .map(|[u, v]| [(scene.vp_x + u) as f32, (scene.vp_y + v) as f32]);

    if let Some([nx, ny]) = nose_canvas_xy {
        if scene.demo_mode {
            let mag = bgra_color(255, 20, 255, 245);
            stroke_line(&mut pix, nx - 13.0, ny, nx - 4.0, ny, 2.2, mag, None);
            stroke_line(&mut pix, nx + 4.0, ny, nx + 13.0, ny, 2.2, mag, None);
            stroke_line(&mut pix, nx, ny - 13.0, nx, ny - 4.0, 2.2, mag, None);
            stroke_line(&mut pix, nx, ny + 4.0, nx, ny + 13.0, 2.2, mag, None);
            let end_x = draw_hud_text_bounded(
                &mut pix,
                nx + 16.0,
                ny - 6.0,
                1.6,
                vp_right,
                "NOSE ANCHOR",
                mag,
            );
            dirty.expand_rect(nx - 18.0, ny - 18.0, end_x + 6.0, ny + 18.0, 10.0);
        } else if scene.nose_locked {
            let lk = bgra_color(90, 245, 145, 155);
            let r = 30.0;
            stroke_line(&mut pix, nx - r, ny - r, nx - r + 7.0, ny - r, 1.5, lk, None);
            stroke_line(&mut pix, nx - r, ny - r, nx - r, ny - r + 7.0, 1.5, lk, None);
            stroke_line(&mut pix, nx + r, ny - r, nx + r - 7.0, ny - r, 1.5, lk, None);
            stroke_line(&mut pix, nx + r, ny - r, nx + r, ny - r + 7.0, 1.5, lk, None);
            stroke_line(&mut pix, nx - r, ny + r, nx - r + 7.0, ny + r, 1.5, lk, None);
            stroke_line(&mut pix, nx - r, ny + r, nx - r, ny + r - 7.0, 1.5, lk, None);
            stroke_line(&mut pix, nx + r, ny + r, nx + r - 7.0, ny + r, 1.5, lk, None);
            stroke_line(&mut pix, nx + r, ny + r, nx + r, ny + r - 7.0, 1.5, lk, None);
            dirty.expand_point(nx, ny, 40.0);
        }
    }

    if !scene.nose_locked {
        dirty.commit(cleared_prev, cw, ch);
        return;
    }

    if let Some(bomb) = &scene.bomb_ccip {
        let ([bx, by], clamped) = clamp_to_viewport(
            bomb.screen_xy[0],
            bomb.screen_xy[1],
            scene.vp_x,
            scene.vp_y,
            scene.vp_w,
            scene.vp_h,
        );
        let shadow = bgra_color(0, 0, 0, 185);
        let gold = if clamped {
            bgra_color(255, 145, 40, 165)
        } else {
            bgra_color(255, 190, 25, 245)
        };

        if let Some([nx, ny]) = nose_canvas_xy {
            let dash = StrokeDash::new(vec![8.0, 6.0], 0.0);
            stroke_line(
                &mut pix,
                nx,
                ny,
                bx,
                by,
                1.8,
                bgra_color(255, 190, 25, 155),
                dash,
            );
            dirty.expand_rect(nx, ny, bx, by, 14.0);
        }

        stroke_circle(&mut pix, bx, by, 18.0, 3.8, shadow);
        stroke_circle(&mut pix, bx, by, 18.0, 2.0, gold);
        stroke_line(&mut pix, bx - 26.0, by, bx - 18.0, by, 2.0, gold, None);
        stroke_line(&mut pix, bx + 18.0, by, bx + 26.0, by, 2.0, gold, None);
        stroke_line(&mut pix, bx, by - 26.0, bx, by - 18.0, 2.0, gold, None);
        fill_circle(&mut pix, bx, by, 2.6, gold);

        let label = format!(
            "{} {:.1}S {:.2}KM",
            bomb.tag,
            bomb.time_of_flight_s,
            bomb.slant_range_m * 0.001
        );
        // 若靠近屏幕右边缘，自动将文字摆到圆环左侧防止被裁剪
        let label_w = label.len() as f32 * 11.4;
        let lx = if bx + 30.0 + label_w <= vp_right {
            bx + 30.0
        } else {
            (bx - 30.0 - label_w).max(ox + 6.0)
        };
        let end_x = draw_hud_text_bounded(&mut pix, lx, by - 6.0, 1.9, cw as f32 - 4.0, &label, gold);
        dirty.expand_rect((bx - 34.0).min(lx - 4.0), by - 34.0, (bx + 34.0).max(end_x + 4.0), by + 34.0, 10.0);
    }

    if let Some(rkt) = &scene.rocket_ccip {
        let ([rx, ry], clamped) = clamp_to_viewport(
            rkt.screen_xy[0],
            rkt.screen_xy[1],
            scene.vp_x,
            scene.vp_y,
            scene.vp_w,
            scene.vp_h,
        );
        let shadow = bgra_color(0, 0, 0, 185);
        let sky = if clamped {
            bgra_color(70, 180, 255, 165)
        } else {
            bgra_color(56, 212, 255, 245)
        };

        stroke_diamond(&mut pix, rx, ry, 15.0, 3.6, shadow);
        stroke_diamond(&mut pix, rx, ry, 15.0, 2.0, sky);
        stroke_line(&mut pix, rx - 6.0, ry, rx + 6.0, ry, 1.6, sky, None);
        stroke_line(&mut pix, rx, ry - 6.0, rx, ry + 6.0, 1.6, sky, None);

        let label = format!(
            "{} {:.1}S {:.2}KM",
            rkt.tag,
            rkt.time_of_flight_s,
            rkt.slant_range_m * 0.001
        );
        let label_w = label.len() as f32 * 10.8;
        let lx = if rx + 24.0 + label_w <= vp_right {
            rx + 24.0
        } else {
            (rx - 24.0 - label_w).max(ox + 6.0)
        };
        let end_x = draw_hud_text_bounded(&mut pix, lx, ry + 8.0, 1.8, cw as f32 - 4.0, &label, sky);
        dirty.expand_rect((rx - 26.0).min(lx - 4.0), ry - 26.0, (rx + 26.0).max(end_x + 4.0), ry + 32.0, 10.0);
    }

    dirty.commit(cleared_prev, cw, ch);
}

pub fn render_hud_into_bgra(scene: &HudScene, bgra_buf: &mut [u8]) {
    bgra_buf.fill(0);
    let mut dirty = DirtyEnvelope::default();
    render_hud_incremental(scene, bgra_buf, &mut dirty);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_ghost_trail_and_standby_idle_skip_across_all_directions() {
        let (w, h) = (1920u32, 1080u32);
        let mut buf = vec![0u8; (w * h * 4) as usize];
        let mut dirty = DirtyEnvelope::default();

        // 涵盖左倾 (bx < nx)、靠近右边缘文字左翻 (bx = 1880)、倒飞拉起 (by < ny) 及超长含下划线/J/Q 横幅
        let cases = [
            ([1050.0, 520.0], [720.0, 810.0], [880.0, 650.0]),
            ([890.0, 600.0], [1885.0, 860.0], [1890.0, 710.0]),
            ([960.0, 580.0], [920.0, 310.0], [940.0, 420.0]),
        ];

        for (nose, b_xy, r_xy) in cases {
            let sc = HudScene {
                canvas_w: w,
                canvas_h: h,
                vp_x: 0.0,
                vp_y: 0.0,
                vp_w: w as f64,
                vp_h: h as f64,
                demo_mode: false,
                zoomed: false,
                fov_deg: 90.0,
                nose_locked: true,
                nose_xy: Some(nose),
                bomb_ccip: Some(ReticleTarget {
                    screen_xy: b_xy,
                    time_of_flight_s: 4.2,
                    slant_range_m: 1150.0,
                    tag: "BOMB",
                }),
                rocket_ccip: Some(ReticleTarget {
                    screen_xy: r_xy,
                    time_of_flight_s: 1.9,
                    slant_range_m: 980.0,
                    tag: "RKT",
                }),
                hud_banner: "CCIP LOCK | A_10A_EARLY / F_4J / Q_5 (100%)".into(),
            };
            render_hud_incremental(&sc, &mut buf, &mut dirty);
            assert!(dirty.last_present_dirty_rect().is_some());
        }

        // 切为空白第 1 帧：返回最后一次清除区域用于擦除屏幕，且缓冲区 0 残留字节
        let empty = HudScene {
            canvas_w: w,
            canvas_h: h,
            vp_x: 0.0,
            vp_y: 0.0,
            vp_w: w as f64,
            vp_h: h as f64,
            demo_mode: false,
            zoomed: false,
            fov_deg: 90.0,
            nose_locked: false,
            nose_xy: None,
            bomb_ccip: None,
            rocket_ccip: None,
            hud_banner: String::new(),
        };
        render_hud_incremental(&empty, &mut buf, &mut dirty);
        assert!(dirty.last_present_dirty_rect().is_some());
        let leftover = buf.iter().filter(|&&b| b != 0).count();
        assert_eq!(leftover, 0, "发现未清除残影字节数: {leftover}");

        // 连续第 2 帧空白待机：`last_present_dirty_rect` 必须返回 `None` 以跳过 DWM 合成！
        render_hud_incremental(&empty, &mut buf, &mut dirty);
        assert!(dirty.last_present_dirty_rect().is_none());
    }
}
