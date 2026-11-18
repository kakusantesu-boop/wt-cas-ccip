//! Win32 多显示器点击穿透分层透明覆盖窗口 (`UpdateLayeredWindowIndirect` 局部脏矩形提交 + `WDA_EXCLUDEFROMCAPTURE`)
//! 与支持全套自定义键鼠快捷键录入、单点/长按缩放模式切换的原生中文控制面板模块

use std::ptr::{null, null_mut};
use std::sync::Mutex;

use windows_sys::Win32::Foundation::{HWND, LPARAM, LRESULT, POINT, RECT, SIZE, WPARAM};
use windows_sys::Win32::Graphics::Gdi::{
    AC_SRC_ALPHA, AC_SRC_OVER, BI_RGB, BITMAPINFO, BITMAPINFOHEADER, BLENDFUNCTION,
    CreateCompatibleDC, CreateDIBSection, CreateFontW, DIB_RGB_COLORS, DeleteDC, DeleteObject,
    GetDC, HBITMAP, HDC, HFONT, ReleaseDC, SelectObject,
};
use windows_sys::Win32::System::LibraryLoader::GetModuleHandleW;
use windows_sys::Win32::UI::Input::KeyboardAndMouse::GetFocus;
use windows_sys::Win32::UI::WindowsAndMessaging::{
    BM_GETCHECK, BM_SETCHECK, BS_AUTOCHECKBOX, BS_PUSHBUTTON, CB_ADDSTRING, CB_GETCURSEL,
    CB_RESETCONTENT, CB_SETCURSEL, CBN_SELCHANGE, CBS_DROPDOWNLIST, CBS_HASSTRINGS, CS_HREDRAW,
    CS_VREDRAW, CW_USEDEFAULT, CreateWindowExW, DefWindowProcW, DestroyWindow, DispatchMessageW,
    EN_CHANGE, ES_AUTOHSCROLL, GetWindowTextLengthW, GetWindowTextW, HWND_TOPMOST, IDC_ARROW,
    LoadCursorW, MSG, PM_REMOVE, PeekMessageW, PostQuitMessage, RegisterClassW, SW_HIDE, SW_SHOW,
    SW_SHOWNOACTIVATE, SWP_NOACTIVATE, SWP_NOMOVE, SWP_NOSIZE, SendMessageW,
    SetWindowDisplayAffinity, SetWindowPos, SetWindowTextW, ShowWindow, TranslateMessage,
    ULW_ALPHA, UPDATELAYEREDWINDOWINFO, UpdateLayeredWindowIndirect, WDA_EXCLUDEFROMCAPTURE,
    WM_COMMAND, WM_DESTROY, WNDCLASSW, WS_BORDER, WS_CAPTION, WS_CHILD, WS_EX_CLIENTEDGE,
    WS_EX_LAYERED, WS_EX_NOACTIVATE, WS_EX_TOOLWINDOW, WS_EX_TOPMOST, WS_EX_TRANSPARENT,
    WS_MINIMIZEBOX, WS_OVERLAPPED, WS_SYSMENU, WS_TABSTOP, WS_VISIBLE, WS_VSCROLL,
};

use crate::capture_win::{BindSlot, HotkeyBindings, vk_display_name_zh};

pub const ID_CB_MAP: usize = 101;
pub const ID_CB_AIRCRAFT: usize = 102;
pub const ID_CB_BOMB: usize = 103;
pub const ID_CB_ROCKET: usize = 104;
pub const ID_CB_COLOR: usize = 105;
pub const ID_CB_CONV: usize = 106;
pub const ID_CB_DRAG: usize = 107;
pub const ID_CHK_AUTO_AC: usize = 108;
pub const ID_CHK_OVERLAY: usize = 109;
pub const ID_CHK_DEMO: usize = 110;
pub const ID_CHK_BAY_DELAY: usize = 112;
pub const ID_BTN_FLIP_ZOOM: usize = 113;
pub const ID_EDT_SEARCH: usize = 114;
pub const ID_CB_RKT_TRIM: usize = 115;

pub const ID_BTN_BIND_ZOOM1: usize = 120;
pub const ID_CB_ZOOM1_MODE: usize = 121;
pub const ID_BTN_BIND_ZOOM2: usize = 122;
pub const ID_CB_ZOOM2_MODE: usize = 123;
pub const ID_BTN_BIND_CALIB: usize = 124;
pub const ID_BTN_BIND_RESYNC: usize = 125;
pub const ID_BTN_BIND_OVERLAY: usize = 126;
pub const ID_BTN_BIND_DEMO: usize = 127;
pub const ID_BTN_RESET_KEYS: usize = 128;

#[derive(Clone, Debug)]
pub enum UiEvent {
    MapChanged(usize),
    SearchChanged(String),
    AircraftChanged(usize),
    BombChanged(usize),
    RocketChanged(usize),
    ColorChanged(usize),
    ConvChanged(usize),
    DragChanged(usize),
    RktTrimChanged(usize),
    Zoom1ModeChanged(usize),
    Zoom2ModeChanged(usize),
    StartBind(BindSlot),
    ResetHotkeys,
    ToggleAutoAc(bool),
    ToggleOverlay(bool),
    ToggleDemo(bool),
    ToggleBayDelay(bool),
    FlipZoom,
    Quit,
}

static UI_EVENTS: Mutex<Vec<UiEvent>> = Mutex::new(Vec::new());

fn push_ev(ev: UiEvent) {
    if let Ok(mut q) = UI_EVENTS.lock() {
        q.push(ev);
    }
}

pub fn drain_ui_events() -> Vec<UiEvent> {
    if let Ok(mut q) = UI_EVENTS.lock() {
        std::mem::take(&mut *q)
    } else {
        Vec::new()
    }
}

pub fn wide_null(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

unsafe fn read_hwnd_text(hwnd: HWND) -> String {
    unsafe {
        let len = GetWindowTextLengthW(hwnd);
        if len <= 0 {
            return String::new();
        }
        let mut buf = vec![0u16; (len as usize) + 1];
        let got = GetWindowTextW(hwnd, buf.as_mut_ptr(), buf.len() as i32);
        if got <= 0 {
            return String::new();
        }
        String::from_utf16_lossy(&buf[..got as usize])
    }
}

/// 全屏点击穿透透明 HUD 窗口 (内置 `WDA_EXCLUDEFROMCAPTURE` + `UpdateLayeredWindowIndirect` 脏矩形合成)
pub struct LayeredHudWindow {
    pub hwnd: HWND,
    pub mon_x: i32,
    pub mon_y: i32,
    pub width: i32,
    pub height: i32,
    mem_dc: HDC,
    dib: HBITMAP,
    old_obj: *mut core::ffi::c_void,
    dib_slice_ptr: *mut u8,
    visible: bool,
}

unsafe extern "system" fn overlay_wnd_proc(
    hwnd: HWND,
    msg: u32,
    wparam: WPARAM,
    lparam: LPARAM,
) -> LRESULT {
    unsafe { DefWindowProcW(hwnd, msg, wparam, lparam) }
}

impl LayeredHudWindow {
    unsafe fn alloc_dib(
        width: i32,
        height: i32,
    ) -> Option<(HDC, HBITMAP, *mut core::ffi::c_void, *mut u8)> {
        unsafe {
            let screen_dc = GetDC(null_mut());
            if screen_dc.is_null() {
                return None;
            }
            let mem_dc = CreateCompatibleDC(screen_dc);
            let bmi = BITMAPINFO {
                bmiHeader: BITMAPINFOHEADER {
                    biSize: std::mem::size_of::<BITMAPINFOHEADER>() as u32,
                    biWidth: width,
                    biHeight: -height,
                    biPlanes: 1,
                    biBitCount: 32,
                    biCompression: BI_RGB,
                    biSizeImage: (width * height * 4) as u32,
                    biXPelsPerMeter: 0,
                    biYPelsPerMeter: 0,
                    biClrUsed: 0,
                    biClrImportant: 0,
                },
                bmiColors: [Default::default(); 1],
            };
            let mut bits: *mut core::ffi::c_void = null_mut();
            let dib = CreateDIBSection(mem_dc, &bmi, DIB_RGB_COLORS, &mut bits, null_mut(), 0);
            ReleaseDC(null_mut(), screen_dc);
            if mem_dc.is_null() || dib.is_null() || bits.is_null() {
                if !dib.is_null() {
                    DeleteObject(dib);
                }
                if !mem_dc.is_null() {
                    DeleteDC(mem_dc);
                }
                return None;
            }
            let old_obj = SelectObject(mem_dc, dib);
            Some((mem_dc, dib, old_obj, bits as *mut u8))
        }
    }

    pub fn create(width: i32, height: i32) -> Option<Self> {
        unsafe {
            let hinst = GetModuleHandleW(null());
            let cls_name = wide_null("WtCasCcipLayeredHudClass");
            let wc = WNDCLASSW {
                style: 0,
                lpfnWndProc: Some(overlay_wnd_proc),
                cbClsExtra: 0,
                cbWndExtra: 0,
                hInstance: hinst,
                hIcon: null_mut(),
                hCursor: LoadCursorW(null_mut(), IDC_ARROW),
                hbrBackground: null_mut(),
                lpszMenuName: null(),
                lpszClassName: cls_name.as_ptr(),
            };
            let _ = RegisterClassW(&wc);

            let title = wide_null("WT CAS CCIP Overlay");
            let ex_style = WS_EX_LAYERED
                | WS_EX_TRANSPARENT
                | WS_EX_TOPMOST
                | WS_EX_NOACTIVATE
                | WS_EX_TOOLWINDOW;
            let hwnd = CreateWindowExW(
                ex_style,
                cls_name.as_ptr(),
                title.as_ptr(),
                0x8000_0000, // WS_POPUP
                0,
                0,
                width,
                height,
                null_mut(),
                null_mut(),
                hinst,
                null(),
            );
            if hwnd.is_null() {
                return None;
            }

            let _ = SetWindowDisplayAffinity(hwnd, WDA_EXCLUDEFROMCAPTURE);
            ShowWindow(hwnd, SW_SHOWNOACTIVATE);

            let Some((mem_dc, dib, old_obj, dib_slice_ptr)) = Self::alloc_dib(width, height) else {
                DestroyWindow(hwnd);
                return None;
            };

            Some(Self {
                hwnd,
                mon_x: 0,
                mon_y: 0,
                width,
                height,
                mem_dc,
                dib,
                old_obj,
                dib_slice_ptr,
                visible: true,
            })
        }
    }

    /// 自动跟随承载战雷窗口的物理显示器位置与分辨率
    pub fn ensure_monitor_geometry(
        &mut self,
        mon_x: i32,
        mon_y: i32,
        mon_w: i32,
        mon_h: i32,
    ) -> bool {
        let mon_w = mon_w.max(640);
        let mon_h = mon_h.max(480);
        if self.mon_x == mon_x && self.mon_y == mon_y && self.width == mon_w && self.height == mon_h
        {
            return false;
        }
        if self.width != mon_w || self.height != mon_h {
            unsafe {
                if let Some((new_dc, new_dib, new_old, new_ptr)) = Self::alloc_dib(mon_w, mon_h) {
                    if !self.mem_dc.is_null() {
                        if !self.old_obj.is_null() {
                            SelectObject(self.mem_dc, self.old_obj);
                        }
                        if !self.dib.is_null() {
                            DeleteObject(self.dib);
                        }
                        DeleteDC(self.mem_dc);
                    }
                    self.mem_dc = new_dc;
                    self.dib = new_dib;
                    self.old_obj = new_old;
                    self.dib_slice_ptr = new_ptr;
                    self.width = mon_w;
                    self.height = mon_h;
                }
            }
        }
        self.mon_x = mon_x;
        self.mon_y = mon_y;
        self.present_dirty(Some([0, 0, self.width, self.height]), true);
        true
    }

    pub fn set_visible(&mut self, show: bool) {
        if self.visible == show {
            return;
        }
        self.visible = show;
        unsafe {
            ShowWindow(self.hwnd, if show { SW_SHOWNOACTIVATE } else { SW_HIDE });
        }
    }

    pub fn frame_buffer_mut(&mut self) -> &mut [u8] {
        unsafe {
            std::slice::from_raw_parts_mut(
                self.dib_slice_ptr,
                (self.width as usize) * (self.height as usize) * 4,
            )
        }
    }

    /// 局部脏矩形合成：仅向 Windows DWM 提交 `dirty_rect` 发生变化的区域；若 `dirty_rect` 为 `None` 则待机 0 开销跳过
    pub fn present_dirty(&self, dirty_rect: Option<[i32; 4]>, reassert_topmost: bool) {
        if !self.visible {
            return;
        }
        unsafe {
            if let Some([x0, y0, x1, y1]) = dirty_rect {
                let rc = RECT {
                    left: x0.clamp(0, self.width),
                    top: y0.clamp(0, self.height),
                    right: x1.clamp(0, self.width),
                    bottom: y1.clamp(0, self.height),
                };
                if rc.right > rc.left && rc.bottom > rc.top {
                    let screen_dc = GetDC(null_mut());
                    let pt_dst = POINT {
                        x: self.mon_x,
                        y: self.mon_y,
                    };
                    let pt_src = POINT { x: 0, y: 0 };
                    let sz = SIZE {
                        cx: self.width,
                        cy: self.height,
                    };
                    let blend = BLENDFUNCTION {
                        BlendOp: AC_SRC_OVER as u8,
                        BlendFlags: 0,
                        SourceConstantAlpha: 255,
                        AlphaFormat: AC_SRC_ALPHA as u8,
                    };
                    let info = UPDATELAYEREDWINDOWINFO {
                        cbSize: std::mem::size_of::<UPDATELAYEREDWINDOWINFO>() as u32,
                        hdcDst: screen_dc,
                        pptDst: &pt_dst,
                        psize: &sz,
                        hdcSrc: self.mem_dc,
                        pptSrc: &pt_src,
                        crKey: 0,
                        pblend: &blend,
                        dwFlags: ULW_ALPHA,
                        prcDirty: &rc,
                    };
                    let _ = UpdateLayeredWindowIndirect(self.hwnd, &info);
                    ReleaseDC(null_mut(), screen_dc);
                }
            }
            if reassert_topmost {
                let _ = SetWindowPos(
                    self.hwnd,
                    HWND_TOPMOST,
                    0,
                    0,
                    0,
                    0,
                    SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE,
                );
            }
        }
    }
}

impl Drop for LayeredHudWindow {
    fn drop(&mut self) {
        unsafe {
            if !self.hwnd.is_null() {
                DestroyWindow(self.hwnd);
            }
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

/// 原生中文控制台窗口（支持搜机过滤、双槽自定义缩放键+单点/长按切换及四项功能热键一键改绑）
pub struct ControlPanelWindow {
    pub hwnd: HWND,
    pub font: HFONT,
    pub cb_map: HWND,
    pub edt_search: HWND,
    pub cb_aircraft: HWND,
    pub cb_bomb: HWND,
    pub cb_rocket: HWND,
    pub cb_color: HWND,
    pub cb_conv: HWND,
    pub cb_drag: HWND,
    pub cb_rkt_trim: HWND,
    pub btn_bind_zoom1: HWND,
    pub cb_zoom1_mode: HWND,
    pub btn_bind_zoom2: HWND,
    pub cb_zoom2_mode: HWND,
    pub btn_bind_calib: HWND,
    pub btn_bind_resync: HWND,
    pub btn_bind_overlay: HWND,
    pub btn_bind_demo: HWND,
    pub chk_auto_ac: HWND,
    pub chk_overlay: HWND,
    pub chk_demo: HWND,
    pub chk_bay_delay: HWND,
    pub lbl_status: HWND,
}

unsafe extern "system" fn panel_wnd_proc(
    hwnd: HWND,
    msg: u32,
    wparam: WPARAM,
    lparam: LPARAM,
) -> LRESULT {
    match msg {
        WM_COMMAND => {
            let id = wparam & 0xFFFF;
            let code = ((wparam >> 16) & 0xFFFF) as u32;
            let ctrl_hwnd = lparam as HWND;
            unsafe {
                if id == ID_EDT_SEARCH && code == EN_CHANGE {
                    push_ev(UiEvent::SearchChanged(read_hwnd_text(ctrl_hwnd)));
                } else if code == CBN_SELCHANGE {
                    let sel = SendMessageW(ctrl_hwnd, CB_GETCURSEL, 0, 0);
                    if sel >= 0 {
                        let idx = sel as usize;
                        match id {
                            ID_CB_MAP => push_ev(UiEvent::MapChanged(idx)),
                            ID_CB_AIRCRAFT => push_ev(UiEvent::AircraftChanged(idx)),
                            ID_CB_BOMB => push_ev(UiEvent::BombChanged(idx)),
                            ID_CB_ROCKET => push_ev(UiEvent::RocketChanged(idx)),
                            ID_CB_COLOR => push_ev(UiEvent::ColorChanged(idx)),
                            ID_CB_CONV => push_ev(UiEvent::ConvChanged(idx)),
                            ID_CB_DRAG => push_ev(UiEvent::DragChanged(idx)),
                            ID_CB_RKT_TRIM => push_ev(UiEvent::RktTrimChanged(idx)),
                            ID_CB_ZOOM1_MODE => push_ev(UiEvent::Zoom1ModeChanged(idx)),
                            ID_CB_ZOOM2_MODE => push_ev(UiEvent::Zoom2ModeChanged(idx)),
                            _ => {}
                        }
                    }
                } else if code == 0 {
                    let checked = || SendMessageW(ctrl_hwnd, BM_GETCHECK, 0, 0) == 1;
                    match id {
                        ID_CHK_AUTO_AC => push_ev(UiEvent::ToggleAutoAc(checked())),
                        ID_CHK_OVERLAY => push_ev(UiEvent::ToggleOverlay(checked())),
                        ID_CHK_DEMO => push_ev(UiEvent::ToggleDemo(checked())),
                        ID_CHK_BAY_DELAY => push_ev(UiEvent::ToggleBayDelay(checked())),
                        ID_BTN_FLIP_ZOOM => push_ev(UiEvent::FlipZoom),
                        ID_BTN_BIND_ZOOM1 => push_ev(UiEvent::StartBind(BindSlot::Zoom1)),
                        ID_BTN_BIND_ZOOM2 => push_ev(UiEvent::StartBind(BindSlot::Zoom2)),
                        ID_BTN_BIND_CALIB => push_ev(UiEvent::StartBind(BindSlot::CalibrateGround)),
                        ID_BTN_BIND_RESYNC => push_ev(UiEvent::StartBind(BindSlot::ResyncZoom)),
                        ID_BTN_BIND_OVERLAY => push_ev(UiEvent::StartBind(BindSlot::ToggleOverlay)),
                        ID_BTN_BIND_DEMO => push_ev(UiEvent::StartBind(BindSlot::ToggleDemo)),
                        ID_BTN_RESET_KEYS => push_ev(UiEvent::ResetHotkeys),
                        _ => {}
                    }
                }
            }
            0
        }
        WM_DESTROY => {
            push_ev(UiEvent::Quit);
            unsafe { PostQuitMessage(0) };
            0
        }
        _ => unsafe { DefWindowProcW(hwnd, msg, wparam, lparam) },
    }
}

impl ControlPanelWindow {
    pub fn create() -> Option<Self> {
        unsafe {
            let hinst = GetModuleHandleW(null());
            let cls_name = wide_null("WtCasCcipControlPanelClass");
            let wc = WNDCLASSW {
                style: CS_HREDRAW | CS_VREDRAW,
                lpfnWndProc: Some(panel_wnd_proc),
                cbClsExtra: 0,
                cbWndExtra: 0,
                hInstance: hinst,
                hIcon: null_mut(),
                hCursor: LoadCursorW(null_mut(), IDC_ARROW),
                hbrBackground: 16 as _, // COLOR_BTNFACE + 1
                lpszMenuName: null(),
                lpszClassName: cls_name.as_ptr(),
            };
            let _ = RegisterClassW(&wc);

            let title =
                wide_null("战雷第三人称 CAS 弹道计算机 · 路线C验证控制台 (v0.2 零内存读取)");
            let style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_VISIBLE;
            let hwnd = CreateWindowExW(
                0,
                cls_name.as_ptr(),
                title.as_ptr(),
                style,
                CW_USEDEFAULT,
                CW_USEDEFAULT,
                648,
                688,
                null_mut(),
                null_mut(),
                hinst,
                null(),
            );
            if hwnd.is_null() {
                return None;
            }

            let face = wide_null("Microsoft YaHei UI");
            let font = CreateFontW(
                -15,
                0,
                0,
                0,
                400,
                0,
                0,
                0,
                1,
                0,
                0,
                5, // CLEARTYPE_QUALITY
                0,
                face.as_ptr(),
            );

            let mk_ctrl =
                |ex: u32, cls: &str, txt: &str, st: u32, x: i32, y: i32, w: i32, h: i32, id: usize| -> HWND {
                    let c_w = wide_null(cls);
                    let t_w = wide_null(txt);
                    let ch = CreateWindowExW(
                        ex,
                        c_w.as_ptr(),
                        t_w.as_ptr(),
                        WS_CHILD | WS_VISIBLE | st,
                        x,
                        y,
                        w,
                        h,
                        hwnd,
                        id as _,
                        hinst,
                        null(),
                    );
                    if !ch.is_null() && !font.is_null() {
                        SendMessageW(ch, 0x0030 /* WM_SETFONT */, font as WPARAM, 1);
                    }
                    ch
                };

            let combo_style =
                (CBS_DROPDOWNLIST as u32) | (CBS_HASSTRINGS as u32) | WS_VSCROLL | WS_TABSTOP;
            let chk_style = (BS_AUTOCHECKBOX as u32) | WS_TABSTOP;
            let btn_style = (BS_PUSHBUTTON as u32) | WS_TABSTOP;
            let edit_style = (ES_AUTOHSCROLL as u32) | WS_BORDER | WS_TABSTOP;

            let _ = mk_ctrl(0, "STATIC", "三维地图高程：", 0, 18, 16, 115, 24, 0);
            let cb_map = mk_ctrl(0, "COMBOBOX", "", combo_style, 136, 12, 478, 320, ID_CB_MAP);

            let _ = mk_ctrl(0, "STATIC", "快速搜机过滤：", 0, 18, 50, 115, 24, 0);
            let edt_search = mk_ctrl(
                WS_EX_CLIENTEDGE,
                "EDIT",
                "",
                edit_style,
                136,
                47,
                220,
                25,
                ID_EDT_SEARCH,
            );
            let _ = mk_ctrl(
                0,
                "STATIC",
                "← 输 a10 / su25 / ka50 / 雷电 实时筛选",
                0,
                366,
                50,
                250,
                24,
                0,
            );

            let _ = mk_ctrl(0, "STATIC", "当前驾驶机型：", 0, 18, 84, 115, 24, 0);
            let cb_aircraft = mk_ctrl(
                0,
                "COMBOBOX",
                "",
                combo_style,
                136,
                80,
                478,
                380,
                ID_CB_AIRCRAFT,
            );

            let _ = mk_ctrl(0, "STATIC", "挂载航弹型号：", 0, 18, 118, 115, 24, 0);
            let cb_bomb = mk_ctrl(0, "COMBOBOX", "", combo_style, 136, 114, 478, 280, ID_CB_BOMB);

            let _ = mk_ctrl(0, "STATIC", "挂载火箭型号：", 0, 18, 152, 115, 24, 0);
            let cb_rocket =
                mk_ctrl(0, "COMBOBOX", "", combo_style, 136, 148, 478, 280, ID_CB_ROCKET);

            let _ = mk_ctrl(0, "STATIC", "机头准星颜色：", 0, 18, 186, 115, 24, 0);
            let cb_color =
                mk_ctrl(0, "COMBOBOX", "", combo_style, 136, 182, 478, 200, ID_CB_COLOR);

            let _ = mk_ctrl(0, "STATIC", "交汇距离：", 0, 18, 222, 80, 24, 0);
            let cb_conv = mk_ctrl(0, "COMBOBOX", "", combo_style, 96, 218, 125, 180, ID_CB_CONV);

            let _ = mk_ctrl(0, "STATIC", "弹阻倍率：", 0, 238, 222, 80, 24, 0);
            let cb_drag = mk_ctrl(0, "COMBOBOX", "", combo_style, 314, 218, 96, 200, ID_CB_DRAG);

            let _ = mk_ctrl(0, "STATIC", "箭巢仰角：", 0, 428, 222, 80, 24, 0);
            let cb_rkt_trim = mk_ctrl(
                0,
                "COMBOBOX",
                "",
                combo_style,
                504,
                218,
                110,
                220,
                ID_CB_RKT_TRIM,
            );

            // === 自定义缩放键与功能快捷键区 ===
            let _ = mk_ctrl(0, "STATIC", "主缩放键：", 0, 18, 259, 76, 24, 0);
            let btn_bind_zoom1 = mk_ctrl(
                0,
                "BUTTON",
                "按键: Z",
                btn_style,
                94,
                255,
                120,
                27,
                ID_BTN_BIND_ZOOM1,
            );
            let cb_zoom1_mode = mk_ctrl(
                0,
                "COMBOBOX",
                "",
                combo_style,
                218,
                256,
                96,
                160,
                ID_CB_ZOOM1_MODE,
            );

            let _ = mk_ctrl(0, "STATIC", "副缩放键：", 0, 326, 259, 76, 24, 0);
            let btn_bind_zoom2 = mk_ctrl(
                0,
                "BUTTON",
                "按键: 鼠标右键",
                btn_style,
                400,
                255,
                114,
                27,
                ID_BTN_BIND_ZOOM2,
            );
            let cb_zoom2_mode = mk_ctrl(
                0,
                "COMBOBOX",
                "",
                combo_style,
                518,
                256,
                96,
                160,
                ID_CB_ZOOM2_MODE,
            );

            let _ = mk_ctrl(0, "STATIC", "功能热键：", 0, 18, 293, 76, 24, 0);
            let btn_bind_calib = mk_ctrl(
                0,
                "BUTTON",
                "标定高: F7",
                btn_style,
                94,
                289,
                100,
                27,
                ID_BTN_BIND_CALIB,
            );
            let btn_bind_resync = mk_ctrl(
                0,
                "BUTTON",
                "翻转镜: F8",
                btn_style,
                198,
                289,
                100,
                27,
                ID_BTN_BIND_RESYNC,
            );
            let btn_bind_overlay = mk_ctrl(
                0,
                "BUTTON",
                "显隐层: F9",
                btn_style,
                302,
                289,
                100,
                27,
                ID_BTN_BIND_OVERLAY,
            );
            let btn_bind_demo = mk_ctrl(
                0,
                "BUTTON",
                "演示态: F10",
                btn_style,
                406,
                289,
                108,
                27,
                ID_BTN_BIND_DEMO,
            );
            let _btn_reset_keys = mk_ctrl(
                0,
                "BUTTON",
                "恢复默认",
                btn_style,
                518,
                289,
                96,
                27,
                ID_BTN_RESET_KEYS,
            );

            let chk_auto_ac = mk_ctrl(
                0,
                "BUTTON",
                "随 8111 自动识别当前所驾载具",
                chk_style,
                18,
                326,
                280,
                24,
                ID_CHK_AUTO_AC,
            );
            let chk_overlay = mk_ctrl(
                0,
                "BUTTON",
                "启用全屏透明穿透准星层",
                chk_style,
                326,
                326,
                280,
                24,
                ID_CHK_OVERLAY,
            );

            let chk_demo = mk_ctrl(
                0,
                "BUTTON",
                "内置无游戏三维俯冲演示模式",
                chk_style,
                18,
                354,
                280,
                24,
                ID_CHK_DEMO,
            );
            let chk_bay_delay = mk_ctrl(
                0,
                "BUTTON",
                "内埋弹舱未预开门延迟 (自动跳过外挂)",
                chk_style,
                326,
                354,
                288,
                24,
                ID_CHK_BAY_DELAY,
            );

            let _btn_flip = mk_ctrl(
                0,
                "BUTTON",
                "立即手动翻转当前开镜/关镜状态",
                btn_style,
                18,
                384,
                268,
                28,
                ID_BTN_FLIP_ZOOM,
            );
            let _ = mk_ctrl(
                0,
                "STATIC",
                "改键说明：点按钮后按任意键/鼠标侧键录入 (Esc取消, Del清空)",
                0,
                296,
                389,
                322,
                24,
                0,
            );

            let lbl_status = mk_ctrl(
                0,
                "STATIC",
                "正在初始化 8111 遥测与三维地图高程库...",
                0,
                18,
                424,
                596,
                214,
                0,
            );

            ShowWindow(hwnd, SW_SHOW);
            Some(Self {
                hwnd,
                font,
                cb_map,
                edt_search,
                cb_aircraft,
                cb_bomb,
                cb_rocket,
                cb_color,
                cb_conv,
                cb_drag,
                cb_rkt_trim,
                btn_bind_zoom1,
                cb_zoom1_mode,
                btn_bind_zoom2,
                cb_zoom2_mode,
                btn_bind_calib,
                btn_bind_resync,
                btn_bind_overlay,
                btn_bind_demo,
                chk_auto_ac,
                chk_overlay,
                chk_demo,
                chk_bay_delay,
                lbl_status,
            })
        }
    }

    pub fn is_search_focused(&self) -> bool {
        !self.edt_search.is_null() && unsafe { GetFocus() == self.edt_search }
    }

    pub fn set_ctrl_text(&self, ctrl: HWND, txt: &str) {
        if ctrl.is_null() {
            return;
        }
        unsafe {
            let w = wide_null(txt);
            SetWindowTextW(ctrl, w.as_ptr());
        }
    }

    pub fn refresh_hotkey_controls(
        &self,
        bindings: &HotkeyBindings,
        capturing: Option<BindSlot>,
    ) {
        let fmt_slot = |slot: BindSlot, prefix: &str, vk: Option<u16>| -> String {
            if capturing == Some(slot) {
                "请按键/鼠...".to_string()
            } else {
                format!("{prefix}: {}", vk_display_name_zh(vk))
            }
        };
        self.set_ctrl_text(
            self.btn_bind_zoom1,
            &fmt_slot(BindSlot::Zoom1, "按键", bindings.zoom1_vk),
        );
        self.set_ctrl_text(
            self.btn_bind_zoom2,
            &fmt_slot(BindSlot::Zoom2, "按键", bindings.zoom2_vk),
        );
        self.set_ctrl_text(
            self.btn_bind_calib,
            &fmt_slot(BindSlot::CalibrateGround, "标定", bindings.calibrate_ground_vk),
        );
        self.set_ctrl_text(
            self.btn_bind_resync,
            &fmt_slot(BindSlot::ResyncZoom, "翻转", bindings.resync_zoom_vk),
        );
        self.set_ctrl_text(
            self.btn_bind_overlay,
            &fmt_slot(BindSlot::ToggleOverlay, "显隐", bindings.toggle_overlay_vk),
        );
        self.set_ctrl_text(
            self.btn_bind_demo,
            &fmt_slot(BindSlot::ToggleDemo, "演示", bindings.toggle_demo_vk),
        );
        self.set_combo_sel(self.cb_zoom1_mode, bindings.zoom1_mode.to_index());
        self.set_combo_sel(self.cb_zoom2_mode, bindings.zoom2_mode.to_index());
    }

    pub fn set_combo_items(&self, cb: HWND, items: &[String], selected: usize) {
        unsafe {
            SendMessageW(cb, CB_RESETCONTENT, 0, 0);
            for it in items {
                let w = wide_null(it);
                SendMessageW(cb, CB_ADDSTRING, 0, w.as_ptr() as LPARAM);
            }
            if !items.is_empty() {
                SendMessageW(cb, CB_SETCURSEL, selected.min(items.len() - 1), 0);
            }
        }
    }

    pub fn set_combo_sel(&self, cb: HWND, selected: usize) {
        unsafe {
            SendMessageW(cb, CB_SETCURSEL, selected, 0);
        }
    }

    pub fn set_checked(&self, chk: HWND, on: bool) {
        unsafe {
            SendMessageW(chk, BM_SETCHECK, usize::from(on), 0);
        }
    }

    pub fn set_status_text(&self, txt: &str) {
        self.set_ctrl_text(self.lbl_status, txt);
    }

    /// 非阻塞泵取所有待处理 Win32 消息；若窗口已关闭返回 `false`
    pub fn pump_messages() -> bool {
        unsafe {
            let mut msg: MSG = std::mem::zeroed();
            while PeekMessageW(&mut msg, null_mut(), 0, 0, PM_REMOVE) != 0 {
                if msg.message == 0x0012
                /* WM_QUIT */
                {
                    return false;
                }
                TranslateMessage(&msg);
                DispatchMessageW(&msg);
            }
            true
        }
    }
}

impl Drop for ControlPanelWindow {
    fn drop(&mut self) {
        unsafe {
            if !self.font.is_null() {
                DeleteObject(self.font);
            }
        }
    }
}
