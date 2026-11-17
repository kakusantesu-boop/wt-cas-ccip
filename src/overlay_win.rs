//! Win32 全屏点击穿透分层透明覆盖窗口 (`UpdateLayeredWindow` + `WDA_EXCLUDEFROMCAPTURE`)
//! 与原生中文控制面板窗口组装模块

use std::ptr::{null, null_mut};
use std::sync::Mutex;

use windows_sys::Win32::Foundation::{HWND, LPARAM, LRESULT, POINT, SIZE, WPARAM};
use windows_sys::Win32::Graphics::Gdi::{
    AC_SRC_ALPHA, AC_SRC_OVER, BI_RGB, BITMAPINFO, BITMAPINFOHEADER, BLENDFUNCTION,
    CreateCompatibleDC, CreateDIBSection, CreateFontW, DIB_RGB_COLORS, DeleteDC, DeleteObject,
    GetDC, HBITMAP, HDC, HFONT, ReleaseDC, SelectObject,
};
use windows_sys::Win32::System::LibraryLoader::GetModuleHandleW;
use windows_sys::Win32::UI::WindowsAndMessaging::{
    BM_GETCHECK, BM_SETCHECK, BS_AUTOCHECKBOX, BS_PUSHBUTTON, CB_ADDSTRING, CB_GETCURSEL,
    CB_RESETCONTENT, CB_SETCURSEL, CBN_SELCHANGE, CBS_DROPDOWNLIST, CBS_HASSTRINGS, CS_HREDRAW,
    CS_VREDRAW, CW_USEDEFAULT, CreateWindowExW, DefWindowProcW, DestroyWindow, DispatchMessageW,
    HWND_TOPMOST, IDC_ARROW, LoadCursorW, MSG, PM_REMOVE, PeekMessageW, PostQuitMessage,
    RegisterClassW, SW_HIDE, SW_SHOW, SW_SHOWNOACTIVATE, SWP_NOACTIVATE, SWP_NOMOVE, SWP_NOSIZE,
    SendMessageW, SetWindowDisplayAffinity, SetWindowPos, SetWindowTextW, ShowWindow,
    TranslateMessage, ULW_ALPHA, UpdateLayeredWindow, WDA_EXCLUDEFROMCAPTURE, WM_COMMAND,
    WM_DESTROY, WNDCLASSW, WS_CAPTION, WS_CHILD, WS_EX_LAYERED, WS_EX_NOACTIVATE,
    WS_EX_TOOLWINDOW, WS_EX_TOPMOST, WS_EX_TRANSPARENT, WS_MINIMIZEBOX, WS_OVERLAPPED, WS_SYSMENU,
    WS_TABSTOP, WS_VISIBLE, WS_VSCROLL,
};

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
pub const ID_CHK_RMB_ZOOM: usize = 111;
pub const ID_CHK_BAY_DELAY: usize = 112;
pub const ID_BTN_FLIP_ZOOM: usize = 113;

#[derive(Clone, Debug)]
pub enum UiEvent {
    MapChanged(usize),
    AircraftChanged(usize),
    BombChanged(usize),
    RocketChanged(usize),
    ColorChanged(usize),
    ConvChanged(usize),
    DragChanged(usize),
    ToggleAutoAc(bool),
    ToggleOverlay(bool),
    ToggleDemo(bool),
    ToggleRmbZoom(bool),
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

/// 全屏点击穿透透明 HUD 窗口 (内置 WDA_EXCLUDEFROMCAPTURE 防止自身图像被屏幕捕获形成回授)
pub struct LayeredHudWindow {
    pub hwnd: HWND,
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

            // 关键防自激护栏：让 Windows DWM 在一切屏幕截图中自动隐去本透明层，但玩家肉眼正常可见！
            let _ = SetWindowDisplayAffinity(hwnd, WDA_EXCLUDEFROMCAPTURE);
            ShowWindow(hwnd, SW_SHOWNOACTIVATE);

            let screen_dc = GetDC(null_mut());
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
            if dib.is_null() || bits.is_null() {
                DestroyWindow(hwnd);
                DeleteDC(mem_dc);
                return None;
            }
            let old_obj = SelectObject(mem_dc, dib);

            Some(Self {
                hwnd,
                width,
                height,
                mem_dc,
                dib,
                old_obj,
                dib_slice_ptr: bits as *mut u8,
                visible: true,
            })
        }
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

    /// 无闪烁全屏合成：绝不在每帧调用改变尺寸的 `SetWindowPos`，消除 Windows DWM 合成器闪烁
    pub fn present_fullscreen(&self, reassert_topmost: bool) {
        if !self.visible {
            return;
        }
        unsafe {
            let screen_dc = GetDC(null_mut());
            let pt_dst = POINT { x: 0, y: 0 };
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
            let _ = UpdateLayeredWindow(
                self.hwnd,
                screen_dc,
                &pt_dst,
                &sz,
                self.mem_dc,
                &pt_src,
                0,
                &blend,
                ULW_ALPHA,
            );
            ReleaseDC(null_mut(), screen_dc);
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

/// 原生中文控制台窗口
pub struct ControlPanelWindow {
    pub hwnd: HWND,
    pub font: HFONT,
    pub cb_map: HWND,
    pub cb_aircraft: HWND,
    pub cb_bomb: HWND,
    pub cb_rocket: HWND,
    pub cb_color: HWND,
    pub cb_conv: HWND,
    pub cb_drag: HWND,
    pub chk_auto_ac: HWND,
    pub chk_overlay: HWND,
    pub chk_demo: HWND,
    pub chk_rmb_zoom: HWND,
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
                if code == CBN_SELCHANGE {
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
                            _ => {}
                        }
                    }
                } else if code == 0 {
                    // BN_CLICKED
                    let checked = || SendMessageW(ctrl_hwnd, BM_GETCHECK, 0, 0) == 1;
                    match id {
                        ID_CHK_AUTO_AC => push_ev(UiEvent::ToggleAutoAc(checked())),
                        ID_CHK_OVERLAY => push_ev(UiEvent::ToggleOverlay(checked())),
                        ID_CHK_DEMO => push_ev(UiEvent::ToggleDemo(checked())),
                        ID_CHK_RMB_ZOOM => push_ev(UiEvent::ToggleRmbZoom(checked())),
                        ID_CHK_BAY_DELAY => push_ev(UiEvent::ToggleBayDelay(checked())),
                        ID_BTN_FLIP_ZOOM => push_ev(UiEvent::FlipZoom),
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

            let title = wide_null("战雷第三人称 CAS 弹道计算机 · 路线C验证控制台 (零内存读取)");
            let style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX | WS_VISIBLE;
            let hwnd = CreateWindowExW(
                0,
                cls_name.as_ptr(),
                title.as_ptr(),
                style,
                CW_USEDEFAULT,
                CW_USEDEFAULT,
                560,
                565,
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

            let mk_ctrl = |cls: &str, txt: &str, st: u32, x: i32, y: i32, w: i32, h: i32, id: usize| -> HWND {
                let c_w = wide_null(cls);
                let t_w = wide_null(txt);
                let ch = CreateWindowExW(
                    0,
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

            let combo_style = (CBS_DROPDOWNLIST as u32) | (CBS_HASSTRINGS as u32) | WS_VSCROLL | WS_TABSTOP;
            let chk_style = (BS_AUTOCHECKBOX as u32) | WS_TABSTOP;
            let btn_style = (BS_PUSHBUTTON as u32) | WS_TABSTOP;

            let _ = mk_ctrl("STATIC", "三维地图高程：", 0, 18, 18, 115, 24, 0);
            let cb_map = mk_ctrl("COMBOBOX", "", combo_style, 138, 14, 390, 320, ID_CB_MAP);

            let _ = mk_ctrl("STATIC", "当前驾驶机型：", 0, 18, 54, 115, 24, 0);
            let cb_aircraft = mk_ctrl("COMBOBOX", "", combo_style, 138, 50, 390, 380, ID_CB_AIRCRAFT);

            let _ = mk_ctrl("STATIC", "挂载航弹型号：", 0, 18, 90, 115, 24, 0);
            let cb_bomb = mk_ctrl("COMBOBOX", "", combo_style, 138, 86, 390, 280, ID_CB_BOMB);

            let _ = mk_ctrl("STATIC", "挂载火箭型号：", 0, 18, 126, 115, 24, 0);
            let cb_rocket = mk_ctrl("COMBOBOX", "", combo_style, 138, 122, 390, 280, ID_CB_ROCKET);

            let _ = mk_ctrl("STATIC", "机头准星颜色：", 0, 18, 162, 115, 24, 0);
            let cb_color = mk_ctrl("COMBOBOX", "", combo_style, 138, 158, 390, 180, ID_CB_COLOR);

            let _ = mk_ctrl("STATIC", "机枪交汇距离：", 0, 18, 198, 115, 24, 0);
            let cb_conv = mk_ctrl("COMBOBOX", "", combo_style, 138, 194, 155, 180, ID_CB_CONV);

            let _ = mk_ctrl("STATIC", "航弹阻力倍率：", 0, 308, 198, 105, 24, 0);
            let cb_drag = mk_ctrl("COMBOBOX", "", combo_style, 412, 194, 116, 200, ID_CB_DRAG);

            let chk_auto_ac = mk_ctrl(
                "BUTTON",
                "随 8111 自动识别当前所驾飞机",
                chk_style,
                18,
                236,
                245,
                24,
                ID_CHK_AUTO_AC,
            );
            let chk_overlay = mk_ctrl(
                "BUTTON",
                "启用全屏透明穿透准星层 (快捷键 F9)",
                chk_style,
                275,
                236,
                260,
                24,
                ID_CHK_OVERLAY,
            );

            let chk_demo = mk_ctrl(
                "BUTTON",
                "内置无游戏三维俯冲演示模式 (F10)",
                chk_style,
                18,
                266,
                250,
                24,
                ID_CHK_DEMO,
            );
            let chk_rmb_zoom = mk_ctrl(
                "BUTTON",
                "按住鼠标右键也触发 30° 缩放视角",
                chk_style,
                275,
                266,
                255,
                24,
                ID_CHK_RMB_ZOOM,
            );

            let chk_bay_delay = mk_ctrl(
                "BUTTON",
                "内埋弹舱未预开门 (+0.6s 投弹延迟)",
                chk_style,
                18,
                296,
                255,
                24,
                ID_CHK_BAY_DELAY,
            );
            let _btn_flip = mk_ctrl(
                "BUTTON",
                "翻转 Z 键缩放同步 (F8)",
                btn_style,
                275,
                294,
                253,
                28,
                ID_BTN_FLIP_ZOOM,
            );

            let lbl_status = mk_ctrl(
                "STATIC",
                "正在初始化 8111 遥测与三维地图高程库...",
                0,
                18,
                334,
                510,
                185,
                0,
            );

            ShowWindow(hwnd, SW_SHOW);
            Some(Self {
                hwnd,
                font,
                cb_map,
                cb_aircraft,
                cb_bomb,
                cb_rocket,
                cb_color,
                cb_conv,
                cb_drag,
                chk_auto_ac,
                chk_overlay,
                chk_demo,
                chk_rmb_zoom,
                chk_bay_delay,
                lbl_status,
            })
        }
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
        unsafe {
            let w = wide_null(txt);
            SetWindowTextW(self.lbl_status, w.as_ptr());
        }
    }

    /// 非阻塞泵取所有待处理 Win32 消息；若窗口已关闭返回 false
    pub fn pump_messages() -> bool {
        unsafe {
            let mut msg: MSG = std::mem::zeroed();
            while PeekMessageW(&mut msg, null_mut(), 0, 0, PM_REMOVE) != 0 {
                if msg.message == 0x0012 /* WM_QUIT */ {
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
