//! Windows 零侵入多显示器视口定位、零堆分配局部快速截屏与全自定义键鼠快捷键引擎

use std::ptr::{null, null_mut};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};
use windows_sys::Win32::Foundation::{HWND, POINT, RECT};
use windows_sys::Win32::Graphics::Gdi::{
    BI_RGB, BITMAPINFO, BITMAPINFOHEADER, BitBlt, ClientToScreen, CreateCompatibleDC,
    CreateDIBSection, DIB_RGB_COLORS, DeleteDC, DeleteObject, GetDC, GetMonitorInfoW, HBITMAP, HDC,
    MONITOR_DEFAULTTONEAREST, MONITORINFO, MonitorFromWindow, ReleaseDC, SRCCOPY, SelectObject,
};
use windows_sys::Win32::UI::HiDpi::{
    DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2, SetProcessDpiAwarenessContext,
};
use windows_sys::Win32::UI::Input::KeyboardAndMouse::{
    GetAsyncKeyState, VK_BACK, VK_DELETE, VK_ESCAPE, VK_F7, VK_F8, VK_F9, VK_F10, VK_LBUTTON,
    VK_RBUTTON,
};
use windows_sys::Win32::UI::WindowsAndMessaging::{
    FindWindowW, GetClientRect, GetForegroundWindow, GetSystemMetrics, IsIconic, IsWindowVisible,
    SM_CXSCREEN, SM_CYSCREEN,
};

pub fn enable_per_monitor_dpi_v2() {
    unsafe {
        let _ = SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    }
}

pub fn primary_screen_size() -> (i32, i32) {
    unsafe {
        let w = GetSystemMetrics(SM_CXSCREEN).max(640);
        let h = GetSystemMetrics(SM_CYSCREEN).max(480);
        (w, h)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GameViewportInfo {
    /// 游戏客户区在虚拟桌面坐标系中的左上角与宽高
    pub vp_x: i32,
    pub vp_y: i32,
    pub vp_w: i32,
    pub vp_h: i32,
    /// 当前承载游戏窗口的物理显示器矩形 (支持负坐标左副屏、双屏异构分辨率与带鱼屏)
    pub mon_x: i32,
    pub mon_y: i32,
    pub mon_w: i32,
    pub mon_h: i32,
    /// 游戏窗口当前是否处于前台输入焦点
    pub is_foreground: bool,
}

/// 查询《战争雷霆》(`DagorWClass`) 实际 3D 客户区、所在物理显示器边界及前台焦点状态
pub fn query_war_thunder_viewport_ext() -> Option<GameViewportInfo> {
    unsafe {
        let cls: Vec<u16> = "DagorWClass\0".encode_utf16().collect();
        let hwnd = FindWindowW(cls.as_ptr(), null());
        if hwnd.is_null() || IsWindowVisible(hwnd) == 0 || IsIconic(hwnd) != 0 {
            return None;
        }
        let mut rc = RECT {
            left: 0,
            top: 0,
            right: 0,
            bottom: 0,
        };
        if GetClientRect(hwnd, &mut rc) == 0 {
            return None;
        }
        let vp_w = rc.right - rc.left;
        let vp_h = rc.bottom - rc.top;
        if vp_w < 320 || vp_h < 240 {
            return None;
        }
        let mut pt = POINT { x: 0, y: 0 };
        if ClientToScreen(hwnd, &mut pt) == 0 {
            return None;
        }

        let (def_w, def_h) = primary_screen_size();
        let (mut mon_x, mut mon_y, mut mon_w, mut mon_h) = (0, 0, def_w, def_h);
        let hmon = MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST);
        if !hmon.is_null() {
            let mut mi: MONITORINFO = std::mem::zeroed();
            mi.cbSize = std::mem::size_of::<MONITORINFO>() as u32;
            if GetMonitorInfoW(hmon, &mut mi) != 0 {
                let mw = mi.rcMonitor.right - mi.rcMonitor.left;
                let mh = mi.rcMonitor.bottom - mi.rcMonitor.top;
                if mw >= 640 && mh >= 480 {
                    mon_x = mi.rcMonitor.left;
                    mon_y = mi.rcMonitor.top;
                    mon_w = mw;
                    mon_h = mh;
                }
            }
        }

        let fg = GetForegroundWindow();
        Some(GameViewportInfo {
            vp_x: pt.x,
            vp_y: pt.y,
            vp_w,
            vp_h,
            mon_x,
            mon_y,
            mon_w,
            mon_h,
            is_foreground: fg == hwnd,
        })
    }
}

pub fn query_war_thunder_client_viewport() -> Option<(i32, i32, i32, i32)> {
    let info = query_war_thunder_viewport_ext()?;
    Some((info.vp_x, info.vp_y, info.vp_w, info.vp_h))
}

pub fn is_window_foreground(hwnd: HWND) -> bool {
    if hwnd.is_null() {
        return false;
    }
    unsafe { GetForegroundWindow() == hwnd }
}

/// 持久化屏幕局部区域捕获器（支持零堆分配缓冲复用 `capture_around_into`）
pub struct ScreenCenterCapturer {
    pub max_w: usize,
    pub max_h: usize,
    mem_dc: HDC,
    dib: HBITMAP,
    old_obj: *mut core::ffi::c_void,
    bits_ptr: *const u8,
}

impl ScreenCenterCapturer {
    pub fn new(max_w: usize, max_h: usize) -> Option<Self> {
        let max_w = max_w.clamp(128, 1600);
        let max_h = max_h.clamp(128, 1600);
        unsafe {
            let screen_dc = GetDC(null_mut());
            if screen_dc.is_null() {
                return None;
            }
            let mem_dc = CreateCompatibleDC(screen_dc);
            ReleaseDC(null_mut(), screen_dc);
            if mem_dc.is_null() {
                return None;
            }

            let bmi = BITMAPINFO {
                bmiHeader: BITMAPINFOHEADER {
                    biSize: std::mem::size_of::<BITMAPINFOHEADER>() as u32,
                    biWidth: max_w as i32,
                    biHeight: -(max_h as i32),
                    biPlanes: 1,
                    biBitCount: 32,
                    biCompression: BI_RGB,
                    biSizeImage: (max_w * max_h * 4) as u32,
                    biXPelsPerMeter: 0,
                    biYPelsPerMeter: 0,
                    biClrUsed: 0,
                    biClrImportant: 0,
                },
                bmiColors: [Default::default(); 1],
            };

            let mut bits: *mut core::ffi::c_void = null_mut();
            let dib = CreateDIBSection(mem_dc, &bmi, DIB_RGB_COLORS, &mut bits, null_mut(), 0);
            if dib.is_null() || bits.is_null() {
                DeleteDC(mem_dc);
                return None;
            }
            let old_obj = SelectObject(mem_dc, dib);
            Some(Self {
                max_w,
                max_h,
                mem_dc,
                dib,
                old_obj,
                bits_ptr: bits as *const u8,
            })
        }
    }

    /// 零堆分配截屏：将 `(center_x, center_y)` 周围 `sub_w × sub_h` 像素直接写入调用方复用的 `out_buf`
    pub fn capture_around_into(
        &mut self,
        center_x: i32,
        center_y: i32,
        sub_w: usize,
        sub_h: usize,
        out_buf: &mut Vec<u8>,
    ) -> Option<(i32, i32, usize, usize)> {
        let w = sub_w.clamp(64, self.max_w);
        let h = sub_h.clamp(64, self.max_h);
        let left = center_x - (w as i32) / 2;
        let top = center_y - (h as i32) / 2;
        let needed = w * h * 4;
        if out_buf.len() < needed {
            out_buf.resize(needed, 0);
        }

        unsafe {
            let screen_dc = GetDC(null_mut());
            if screen_dc.is_null() {
                return None;
            }
            let ok = BitBlt(
                self.mem_dc,
                0,
                0,
                w as i32,
                h as i32,
                screen_dc,
                left,
                top,
                SRCCOPY,
            );
            ReleaseDC(null_mut(), screen_dc);
            if ok == 0 {
                return None;
            }
            if w == self.max_w {
                let src = std::slice::from_raw_parts(self.bits_ptr, needed);
                out_buf[..needed].copy_from_slice(src);
            } else {
                let full_stride = self.max_w * 4;
                let sub_stride = w * 4;
                for row in 0..h {
                    let src = std::slice::from_raw_parts(
                        self.bits_ptr.add(row * full_stride),
                        sub_stride,
                    );
                    out_buf[row * sub_stride..(row + 1) * sub_stride].copy_from_slice(src);
                }
            }
            Some((left, top, w, h))
        }
    }
}

impl Drop for ScreenCenterCapturer {
    fn drop(&mut self) {
        unsafe {
            if !self.mem_dc.is_null() {
                if !self.old_obj.is_null() {
                    SelectObject(self.mem_dc, self.old_obj);
                }
                if !self.dib.is_null() {
                    DeleteObject(self.dib);
                }
                DeleteDC(self.mem_dc);
            }
        }
    }
}

/// 游戏视角缩放触发模式：支持单点切换 (`Toggle`)、长按保持 (`Hold`) 或禁用 (`Disabled`)
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ZoomTriggerMode {
    Toggle,
    Hold,
    Disabled,
}

impl ZoomTriggerMode {
    pub const ALL: [Self; 3] = [Self::Toggle, Self::Hold, Self::Disabled];

    pub fn label_zh(self) -> &'static str {
        match self {
            Self::Toggle => "单点切换",
            Self::Hold => "长按保持",
            Self::Disabled => "禁用此键",
        }
    }

    pub fn to_index(self) -> usize {
        match self {
            Self::Toggle => 0,
            Self::Hold => 1,
            Self::Disabled => 2,
        }
    }

    pub fn from_index(idx: usize) -> Self {
        match idx {
            0 => Self::Toggle,
            1 => Self::Hold,
            _ => Self::Disabled,
        }
    }
}

/// 可持久化的全套自定义键鼠绑定配置
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HotkeyBindings {
    pub zoom1_vk: Option<u16>,
    pub zoom1_mode: ZoomTriggerMode,
    pub zoom2_vk: Option<u16>,
    pub zoom2_mode: ZoomTriggerMode,
    pub calibrate_ground_vk: Option<u16>,
    pub resync_zoom_vk: Option<u16>,
    pub toggle_overlay_vk: Option<u16>,
    pub toggle_demo_vk: Option<u16>,
}

impl Default for HotkeyBindings {
    fn default() -> Self {
        Self {
            zoom1_vk: Some(b'Z' as u16),
            zoom1_mode: ZoomTriggerMode::Toggle,
            zoom2_vk: Some(VK_RBUTTON),
            zoom2_mode: ZoomTriggerMode::Disabled,
            calibrate_ground_vk: Some(VK_F7),
            resync_zoom_vk: Some(VK_F8),
            toggle_overlay_vk: Some(VK_F9),
            toggle_demo_vk: Some(VK_F10),
        }
    }
}

/// 将 Win32 虚拟键码 (`VK_*`) 格式化为简明中文可读名称（支持鼠标右/中/侧键4/侧键5、左右修饰键、全字母数字、小键盘与功能键）
pub fn vk_display_name_zh(vk_opt: Option<u16>) -> String {
    let Some(vk) = vk_opt else {
        return "无".to_string();
    };
    match vk {
        0x02 => "鼠标右键".to_string(),
        0x04 => "鼠标中键".to_string(),
        0x05 => "鼠标侧键4".to_string(),
        0x06 => "鼠标侧键5".to_string(),
        0x09 => "Tab".to_string(),
        0x0D => "Enter".to_string(),
        0x10 => "Shift".to_string(),
        0x11 => "Ctrl".to_string(),
        0x12 => "Alt".to_string(),
        0x14 => "CapsLock".to_string(),
        0x20 => "空格".to_string(),
        0x21 => "PgUp".to_string(),
        0x22 => "PgDn".to_string(),
        0x23 => "End".to_string(),
        0x24 => "Home".to_string(),
        0x25 => "方向←".to_string(),
        0x26 => "方向↑".to_string(),
        0x27 => "方向→".to_string(),
        0x28 => "方向↓".to_string(),
        0x2D => "Insert".to_string(),
        0x30..=0x39 => char::from(vk as u8).to_string(),
        0x41..=0x5A => char::from(vk as u8).to_string(),
        0x60..=0x69 => format!("小键{}", vk - 0x60),
        0x6A => "小键*".to_string(),
        0x6B => "小键+".to_string(),
        0x6D => "小键-".to_string(),
        0x6E => "小键.".to_string(),
        0x6F => "小键/".to_string(),
        0x70..=0x87 => format!("F{}", vk - 0x70 + 1),
        0xA0 => "左Shift".to_string(),
        0xA1 => "右Shift".to_string(),
        0xA2 => "左Ctrl".to_string(),
        0xA3 => "右Ctrl".to_string(),
        0xA4 => "左Alt".to_string(),
        0xA5 => "右Alt".to_string(),
        0xBA => ";".to_string(),
        0xBB => "=".to_string(),
        0xBC => ",".to_string(),
        0xBD => "-".to_string(),
        0xBE => ".".to_string(),
        0xBF => "/".to_string(),
        0xC0 => "`(~)".to_string(),
        0xDB => "[".to_string(),
        0xDC => "\\".to_string(),
        0xDD => "]".to_string(),
        0xDE => "'".to_string(),
        other => format!("VK_{other:02X}"),
    }
}

/// 允许用户录入的全部虚拟键码列表（排除鼠标左键 `0x01` 以防点击界面冲突，具体左右修饰键优先于泛化修饰键）
const BINDABLE_VKS: &[u16] = &[
    // 鼠标按键：右键、中键、侧键4(X1)、侧键5(X2)
    0x02, 0x04, 0x05, 0x06,
    // 左右独立修饰键
    0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5,
    // 常用控制与导航键
    0x09, 0x14, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x2D,
    // 数字行 0..9
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    // 字母 A..Z
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D,
    0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A,
    // 小键盘 0..9 与运算符
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6D, 0x6E, 0x6F,
    // 功能键 F1..F24
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B,
    0x7C, 0x7D, 0x7E, 0x7F, 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    // 符号键 (` - = [ ] \ ; ' , . /)
    0xBA, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xC0, 0xDB, 0xDC, 0xDD, 0xDE,
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BindSlot {
    Zoom1,
    Zoom2,
    CalibrateGround,
    ResyncZoom,
    ToggleOverlay,
    ToggleDemo,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BindPollOutcome {
    Waiting,
    Bound(u16),
    Cleared,
    Cancelled,
}

/// 交互式一键录键状态机（先等待点击按钮时的鼠标左键松开，再捕获任意新按下的键鼠按键；`Esc` 取消，`Del/Backspace` 清空）
pub struct KeyBindRecorder {
    pub slot: BindSlot,
    started_at: Instant,
    lbutton_released: bool,
    prev_down: [bool; 256],
}

impl KeyBindRecorder {
    pub fn start(slot: BindSlot) -> Self {
        let mut prev_down = [false; 256];
        for vk in 1u16..255 {
            prev_down[vk as usize] = os_key_down(vk);
        }
        let lmb = prev_down[VK_LBUTTON as usize];
        Self {
            slot,
            started_at: Instant::now(),
            lbutton_released: !lmb,
            prev_down,
        }
    }

    pub fn poll(&mut self) -> BindPollOutcome {
        self.poll_with(os_key_down)
    }

    pub fn poll_with(&mut self, down_fn: impl Fn(u16) -> bool) -> BindPollOutcome {
        if self.started_at.elapsed() > Duration::from_secs(12) {
            return BindPollOutcome::Cancelled;
        }
        let lmb = down_fn(VK_LBUTTON);
        if !self.lbutton_released {
            if !lmb {
                self.lbutton_released = true;
            }
            for vk in 1u16..255 {
                self.prev_down[vk as usize] = down_fn(vk);
            }
            return BindPollOutcome::Waiting;
        }

        let rising = |vk: u16, prev: &[bool; 256]| -> bool {
            let cur = down_fn(vk);
            cur && !prev[vk as usize]
        };

        if rising(VK_ESCAPE, &self.prev_down) {
            return BindPollOutcome::Cancelled;
        }
        if rising(VK_DELETE, &self.prev_down) || rising(VK_BACK, &self.prev_down) {
            return BindPollOutcome::Cleared;
        }

        let mut captured: Option<u16> = None;
        for &vk in BINDABLE_VKS {
            if rising(vk, &self.prev_down) {
                captured = Some(vk);
                break;
            }
        }

        for vk in 1u16..255 {
            self.prev_down[vk as usize] = down_fn(vk);
        }

        match captured {
            Some(vk) => BindPollOutcome::Bound(vk),
            None => BindPollOutcome::Waiting,
        }
    }
}

#[inline]
fn os_key_down(vk: u16) -> bool {
    unsafe { (GetAsyncKeyState(vk as i32) as u16 & 0x8000) != 0 }
}

#[inline]
fn is_function_key(vk: u16) -> bool {
    (0x70..=0x87).contains(&vk)
}

pub struct GlobalHotkeys {
    prev_down: [bool; 256],
}

impl Default for GlobalHotkeys {
    fn default() -> Self {
        Self {
            prev_down: [false; 256],
        }
    }
}

#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
pub struct HotkeyEvents {
    pub toggle_zoom: bool,
    pub hold_zoom_active: bool,
    pub calibrate_ground: bool,
    pub resync_zoom: bool,
    pub toggle_overlay: bool,
    pub toggle_demo: bool,
}

impl GlobalHotkeys {
    /// 录键刚完成时同步当前按键状态，避免刚录完的那一下按键立刻触发动作
    pub fn sync_all_pressed(&mut self) {
        for vk in 1u16..255 {
            self.prev_down[vk as usize] = os_key_down(vk);
        }
    }

    pub fn poll(
        &mut self,
        bindings: &HotkeyBindings,
        allow_zoom_keys: bool,
        allow_non_fn_utility_keys: bool,
    ) -> HotkeyEvents {
        self.evaluate_with(
            os_key_down,
            bindings,
            allow_zoom_keys,
            allow_non_fn_utility_keys,
        )
    }

    pub fn evaluate_with(
        &mut self,
        down_fn: impl Fn(u16) -> bool,
        bindings: &HotkeyBindings,
        allow_zoom_keys: bool,
        allow_non_fn_utility_keys: bool,
    ) -> HotkeyEvents {
        let mut ev = HotkeyEvents::default();

        // 评估主/副两个缩放槽位 (各自独立支持 Toggle / Hold / Disabled)
        for (vk_opt, mode) in [
            (bindings.zoom1_vk, bindings.zoom1_mode),
            (bindings.zoom2_vk, bindings.zoom2_mode),
        ] {
            if let Some(vk) = vk_opt.filter(|&k| (k as usize) < 256) {
                let cur = down_fn(vk);
                let prev = self.prev_down[vk as usize];
                if allow_zoom_keys {
                    match mode {
                        ZoomTriggerMode::Toggle => {
                            if cur && !prev {
                                ev.toggle_zoom = true;
                            }
                        }
                        ZoomTriggerMode::Hold => {
                            if cur {
                                ev.hold_zoom_active = true;
                            }
                        }
                        ZoomTriggerMode::Disabled => {}
                    }
                }
            }
        }

        // 功能热键：F1..F24 允许全局触发；若用户改绑为普通字母/数字/鼠标键，则仅在游戏或本面板前台(且未在搜索框打字)时触发
        let check_util = |vk_opt: Option<u16>, prev_arr: &[bool; 256]| -> bool {
            let Some(vk) = vk_opt.filter(|&k| (k as usize) < 256) else {
                return false;
            };
            if !is_function_key(vk) && !allow_non_fn_utility_keys {
                return false;
            }
            let cur = down_fn(vk);
            cur && !prev_arr[vk as usize]
        };

        ev.calibrate_ground = check_util(bindings.calibrate_ground_vk, &self.prev_down);
        ev.resync_zoom = check_util(bindings.resync_zoom_vk, &self.prev_down);
        ev.toggle_overlay = check_util(bindings.toggle_overlay_vk, &self.prev_down);
        ev.toggle_demo = check_util(bindings.toggle_demo_vk, &self.prev_down);

        // 更新所有已绑定键的边沿历史（即使两个槽位绑了同一个键也能在统一循环后再写回，互不吞边沿！）
        for vk in [
            bindings.zoom1_vk,
            bindings.zoom2_vk,
            bindings.calibrate_ground_vk,
            bindings.resync_zoom_vk,
            bindings.toggle_overlay_vk,
            bindings.toggle_demo_vk,
        ]
        .into_iter()
        .flatten()
        {
            if (vk as usize) < 256 {
                self.prev_down[vk as usize] = down_fn(vk);
            }
        }

        ev
    }
}

/// 混合高精度帧间隔休眠：绕过 Windows 默认 `15.625 ms` 粗粒度时钟引起的帧率忽快忽慢
pub fn precise_frame_sleep(target_frame_dur: Duration, frame_start: Instant) {
    let elapsed = frame_start.elapsed();
    if elapsed >= target_frame_dur {
        return;
    }
    let rem = target_frame_dur - elapsed;
    let spin_margin = Duration::from_micros(1800);
    if rem > spin_margin {
        std::thread::sleep(rem - spin_margin);
    }
    while frame_start.elapsed() < target_frame_dur {
        std::hint::spin_loop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_custom_zoom_modes_mouse_side_button_and_foreground_gate() {
        let mut hk = GlobalHotkeys::default();
        // 自定义：主缩放键 = V (长按保持 Hold)，副缩放键 = 鼠标侧键4 (0x05, 单点切换 Toggle)，显隐图层 = F9
        let bindings = HotkeyBindings {
            zoom1_vk: Some(b'V' as u16),
            zoom1_mode: ZoomTriggerMode::Hold,
            zoom2_vk: Some(0x05), // VK_XBUTTON1 鼠标侧键4
            zoom2_mode: ZoomTriggerMode::Toggle,
            calibrate_ground_vk: Some(b'G' as u16), // 非 F 键，需前台门控
            resync_zoom_vk: None,                   // 清空解绑
            toggle_overlay_vk: Some(VK_F9),         // F 键，支持全局
            toggle_demo_vk: Some(VK_F10),
        };

        // 1. 游戏不在前台 (`allow_zoom = false, allow_util = false`) 时按 V、侧键4、G：均不误触发；但按 F9 仍正常全局响应！
        let ev_bg = hk.evaluate_with(
            |vk| vk == b'V' as u16 || vk == 0x05 || vk == b'G' as u16 || vk == VK_F9,
            &bindings,
            false,
            false,
        );
        assert!(!ev_bg.hold_zoom_active);
        assert!(!ev_bg.toggle_zoom);
        assert!(!ev_bg.calibrate_ground);
        assert!(ev_bg.toggle_overlay, "F9 功能键应支持后台切回显隐");

        // 松开全部按键
        let _ = hk.evaluate_with(|_| false, &bindings, true, true);

        // 2. 游戏在前台：按住 V 触发长按缩放 (`hold_zoom_active = true`)，松开 V 立即恢复 (`false`)
        let ev_hold_on = hk.evaluate_with(|vk| vk == b'V' as u16, &bindings, true, true);
        assert!(ev_hold_on.hold_zoom_active && !ev_hold_on.toggle_zoom);
        let ev_hold_keep = hk.evaluate_with(|vk| vk == b'V' as u16, &bindings, true, true);
        assert!(ev_hold_keep.hold_zoom_active);
        let ev_hold_off = hk.evaluate_with(|_| false, &bindings, true, true);
        assert!(!ev_hold_off.hold_zoom_active);

        // 3. 按下鼠标侧键4 (`0x05`)：仅在按下首帧触发一次 `toggle_zoom`，按住不重复翻转
        let ev_side1 = hk.evaluate_with(|vk| vk == 0x05, &bindings, true, true);
        assert!(ev_side1.toggle_zoom);
        let ev_side2 = hk.evaluate_with(|vk| vk == 0x05, &bindings, true, true);
        assert!(!ev_side2.toggle_zoom);
    }

    #[test]
    fn test_key_bind_recorder_waits_for_lmb_release_and_supports_side_button_or_delete() {
        let mut rec = KeyBindRecorder {
            slot: BindSlot::Zoom1,
            started_at: Instant::now(),
            lbutton_released: false,
            prev_down: [false; 256],
        };
        rec.prev_down[VK_LBUTTON as usize] = true;

        // 鼠标左键尚未松开时，绝不误录入左键
        assert_eq!(rec.poll_with(|vk| vk == VK_LBUTTON), BindPollOutcome::Waiting);
        // 左键松开，进入监听态
        assert_eq!(rec.poll_with(|_| false), BindPollOutcome::Waiting);
        // 按下鼠标侧键5 (`0x06`)，立即成功捕获！
        assert_eq!(rec.poll_with(|vk| vk == 0x06), BindPollOutcome::Bound(0x06));
        assert_eq!(vk_display_name_zh(Some(0x06)), "鼠标侧键5");

        // 测试 Delete 清空解绑
        let mut rec_del = KeyBindRecorder {
            slot: BindSlot::Zoom2,
            started_at: Instant::now(),
            lbutton_released: true,
            prev_down: [false; 256],
        };
        assert_eq!(rec_del.poll_with(|vk| vk == VK_DELETE), BindPollOutcome::Cleared);
    }
}
