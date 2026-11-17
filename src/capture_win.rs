//! Windows 零侵入视口定位、自适应跟踪窗口快速截屏与全局快捷键边缘检测器

use std::ptr::{null, null_mut};

use windows_sys::Win32::Foundation::{POINT, RECT};
use windows_sys::Win32::Graphics::Gdi::{
    BI_RGB, BITMAPINFO, BITMAPINFOHEADER, BitBlt, ClientToScreen, CreateCompatibleDC,
    CreateDIBSection, DIB_RGB_COLORS, DeleteDC, DeleteObject, GetDC, HBITMAP, HDC, ReleaseDC,
    SRCCOPY, SelectObject,
};
use windows_sys::Win32::UI::HiDpi::{
    DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2, SetProcessDpiAwarenessContext,
};
use windows_sys::Win32::UI::Input::KeyboardAndMouse::{
    GetAsyncKeyState, VK_F7, VK_F8, VK_F9, VK_F10, VK_RBUTTON,
};
use windows_sys::Win32::UI::WindowsAndMessaging::{
    FindWindowW, GetClientRect, GetSystemMetrics, IsIconic, IsWindowVisible, SM_CXSCREEN,
    SM_CYSCREEN,
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

/// 通过公开桌面窗口管理 API 查询《战争雷霆》(`DagorWClass`) 实际 3D 客户区在屏幕上的像素矩形 `(left, top, width, height)`。
/// 完美兼容无边框全屏、带标题栏窗口化、2K/4K/带鱼屏及副屏摆放，零进程附加、零内存接触。
pub fn query_war_thunder_client_viewport() -> Option<(i32, i32, i32, i32)> {
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
        let w = rc.right - rc.left;
        let h = rc.bottom - rc.top;
        if w < 320 || h < 240 {
            return None;
        }
        let mut pt = POINT { x: 0, y: 0 };
        if ClientToScreen(hwnd, &mut pt) == 0 {
            return None;
        }
        Some((pt.x, pt.y, w, h))
    }
}

/// 持久化屏幕局部区域捕获器（支持全中心 `720×720` 搜索与锁定后 `224×224` 极速跟踪双模式）
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

    /// 截取指定中心 `(center_x, center_y)` 周围 `sub_w × sub_h` 的紧凑矩形（自动按 `max_w` 步幅紧凑重排或整宽截取）
    pub fn capture_around(
        &mut self,
        center_x: i32,
        center_y: i32,
        sub_w: usize,
        sub_h: usize,
    ) -> Option<((i32, i32, usize, usize), Vec<u8>)> {
        let w = sub_w.clamp(64, self.max_w);
        let h = sub_h.clamp(64, self.max_h);
        let left = center_x - (w as i32) / 2;
        let top = center_y - (h as i32) / 2;
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
                let slice = std::slice::from_raw_parts(self.bits_ptr, w * h * 4);
                return Some(((left, top, w, h), slice.to_vec()));
            }
            let mut compact = vec![0u8; w * h * 4];
            let full_stride = self.max_w * 4;
            let sub_stride = w * 4;
            for row in 0..h {
                let src = std::slice::from_raw_parts(self.bits_ptr.add(row * full_stride), sub_stride);
                compact[row * sub_stride..(row + 1) * sub_stride].copy_from_slice(src);
            }
            Some(((left, top, w, h), compact))
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

#[derive(Default)]
pub struct GlobalHotkeys {
    prev_z: bool,
    prev_f7: bool,
    prev_f8: bool,
    prev_f9: bool,
    prev_f10: bool,
}

#[derive(Default, Clone, Copy)]
pub struct HotkeyEvents {
    pub toggle_zoom_z: bool,
    pub hold_rmb: bool,
    pub calibrate_ground_f7: bool,
    pub resync_zoom_f8: bool,
    pub toggle_overlay_f9: bool,
    pub toggle_demo_f10: bool,
}

impl GlobalHotkeys {
    pub fn poll(&mut self) -> HotkeyEvents {
        unsafe {
            let down = |vk: i32| (GetAsyncKeyState(vk) as u16 & 0x8000) != 0;
            let z = down(b'Z' as i32);
            let rmb = down(VK_RBUTTON as i32);
            let f7 = down(VK_F7 as i32);
            let f8 = down(VK_F8 as i32);
            let f9 = down(VK_F9 as i32);
            let f10 = down(VK_F10 as i32);

            let ev = HotkeyEvents {
                toggle_zoom_z: z && !self.prev_z,
                hold_rmb: rmb,
                calibrate_ground_f7: f7 && !self.prev_f7,
                resync_zoom_f8: f8 && !self.prev_f8,
                toggle_overlay_f9: f9 && !self.prev_f9,
                toggle_demo_f10: f10 && !self.prev_f10,
            };
            self.prev_z = z;
            self.prev_f7 = f7;
            self.prev_f8 = f8;
            self.prev_f9 = f9;
            self.prev_f10 = f10;
            ev
        }
    }
}
