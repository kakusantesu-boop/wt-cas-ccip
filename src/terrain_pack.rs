//! 只读挂载《婴儿弹道计算机》内置 WTVPACK2 56 张陆战地图三维地形高程包
//! 采用 DagorEngine 原版 4 三角形网格地表插值，缺失时平滑降级为平面基准海拔。

use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, Result, ensure};
use base64::Engine as _;
use serde::Deserialize;
use serde_json::Value;

const MAGIC: &[u8; 8] = b"WTVPACK2";

#[derive(Clone, Debug, Deserialize)]
pub struct MapSummary {
    pub name: String,
    #[serde(default)]
    pub title: Option<String>,
    #[serde(default)]
    pub en: Option<String>,
    pub width: usize,
    pub cell: f64,
}

#[derive(Clone, Debug, Deserialize)]
struct RawMapMeta {
    pub width: usize,
    pub height: usize,
    pub cell: f64,
    pub ox: f64,
    pub oz: f64,
    pub hmin: f64,
    pub hmax: f64,
}

/// Dagor 4-triangle cell interpolated heightmap (+X East, +Z North, height in meters ASL)
#[derive(Clone, Debug)]
pub struct TerrainGrid {
    pub id: String,
    pub title_zh: String,
    pub h: Vec<f32>,
    pub w: usize,
    pub hh: usize,
    pub cell: f64,
    pub ox: f64,
    pub oz: f64,
    pub x1: f64,
    pub z1: f64,
    pub min_elev: f64,
    pub max_elev: f64,
}

impl TerrainGrid {
    pub fn inside(&self, x: f64, z: f64) -> bool {
        x >= self.ox && z >= self.oz && x <= self.x1 && z <= self.z1
    }

    pub fn height_at(&self, x: f64, z: f64) -> f64 {
        let (w, h) = (self.w as f64, self.hh as f64);
        let px = ((x - self.ox) / self.cell).clamp(0.0, w - 1.000001);
        let pz = ((z - self.oz) / self.cell).clamp(0.0, h - 1.000001);
        let (x0, z0) = (px as usize, pz as usize);
        let i0 = z0 * self.w + x0;
        let (h00, h10, h01, h11) = (
            self.h[i0] as f64,
            self.h[i0 + 1] as f64,
            self.h[i0 + self.w] as f64,
            self.h[i0 + self.w + 1] as f64,
        );
        let hm = 0.25 * (h00 + h10 + h01 + h11);
        let (fx, fz) = (px - x0 as f64 - 0.5, pz - z0 as f64 - 0.5);
        if fx.abs() >= fz.abs() {
            if fx >= 0.0 {
                hm + (h10 + h11 - 2.0 * hm) * fx + (h11 - h10) * fz
            } else {
                hm + (2.0 * hm - h00 - h01) * fx + (h01 - h00) * fz
            }
        } else if fz >= 0.0 {
            hm + (h11 - h01) * fx + (h01 + h11 - 2.0 * hm) * fz
        } else {
            hm + (h10 - h00) * fx + (2.0 * hm - h00 - h10) * fz
        }
    }
}

#[derive(Clone, Debug)]
pub enum TerrainSource {
    Flat { ground_alt_m: f64 },
    Grid { grid: Arc<TerrainGrid>, fallback_alt_m: f64 },
}

impl TerrainSource {
    pub fn height_at(&self, x: f64, z: f64) -> f64 {
        match self {
            TerrainSource::Flat { ground_alt_m } => *ground_alt_m,
            TerrainSource::Grid { grid, fallback_alt_m } => {
                if grid.inside(x, z) {
                    grid.height_at(x, z)
                } else {
                    // Clamp to nearest map edge if within 4 km buffer, else fallback
                    let dx = (grid.ox - x).max(0.0).max(x - grid.x1);
                    let dz = (grid.oz - z).max(0.0).max(z - grid.z1);
                    if dx.hypot(dz) < 4000.0 {
                        grid.height_at(x, z)
                    } else {
                        *fallback_alt_m
                    }
                }
            }
        }
    }
}

pub struct WtvPackReader {
    pack_file: PathBuf,
    entries: HashMap<String, (u64, u64, u8)>,
    pub maps: Vec<MapSummary>,
}

impl WtvPackReader {
    /// 自动探测工作区或同级目录下的《婴儿弹道计算机.exe》只读挂载地图包
    pub fn auto_discover() -> Option<Self> {
        let mut candidates = Vec::new();
        if let Ok(env_p) = std::env::var("WTV_PACK_EXE") {
            candidates.push(PathBuf::from(env_p));
        }
        if let Ok(exe) = std::env::current_exe() {
            if let Some(dir) = exe.parent() {
                candidates.push(dir.join("婴儿弹道计算机.exe"));
                candidates.push(dir.join("../wt-ballistic-visualizer/dist/婴儿弹道计算机/婴儿弹道计算机.exe"));
                candidates.push(dir.join("../../wt-ballistic-visualizer/dist/婴儿弹道计算机/婴儿弹道计算机.exe"));
            }
        }
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        candidates.push(
            manifest_dir.join("../wt-ballistic-visualizer/dist/婴儿弹道计算机/婴儿弹道计算机.exe"),
        );

        for c in candidates {
            if c.exists() {
                if let Ok(reader) = Self::open(&c) {
                    return Some(reader);
                }
            }
        }
        None
    }

    pub fn open(path: &Path) -> Result<Self> {
        let mut f = File::open(path).with_context(|| format!("打开地图包失败: {}", path.display()))?;
        let size = f.metadata()?.len();
        ensure!(size >= 24, "文件过小，不含 WTVPACK2 尾部索引");
        f.seek(SeekFrom::Start(size - 24))?;
        let mut foot = [0u8; 24];
        f.read_exact(&mut foot)?;
        ensure!(&foot[16..] == MAGIC, "未检测到 WTVPACK2 签名");
        let off = u64::from_le_bytes(foot[..8].try_into()?);
        let len = u64::from_le_bytes(foot[8..16].try_into()?);
        ensure!(off + len + 24 == size, "WTVPACK2 索引长度校验不匹配");
        f.seek(SeekFrom::Start(off))?;
        let mut idx_raw = vec![0u8; len as usize];
        f.read_exact(&mut idx_raw)?;
        let list: Vec<(String, u64, u64, u8)> = serde_json::from_slice(&idx_raw)?;
        let entries: HashMap<String, (u64, u64, u8)> =
            list.into_iter().map(|(p, o, n, c)| (p, (o, n, c))).collect();

        let mut reader = Self {
            pack_file: path.to_path_buf(),
            entries,
            maps: Vec::new(),
        };
        let mut args = reader.call_args("index.js", "WTV_INDEX")?;
        ensure!(!args.is_empty(), "index.js 参数为空");
        #[derive(Deserialize)]
        struct PartialIndex {
            maps: Vec<MapSummary>,
        }
        let parsed: PartialIndex = serde_json::from_value(args.remove(0))?;
        reader.maps = parsed.maps;
        Ok(reader)
    }

    fn read_bytes(&self, rel: &str) -> Result<Vec<u8>> {
        let &(off, len, codec) = self
            .entries
            .get(rel)
            .with_context(|| format!("地图包缺少条目: {rel}"))?;
        ensure!(codec == 1, "{rel}: 未知压缩编码 {codec}");
        let mut f = File::open(&self.pack_file)?;
        f.seek(SeekFrom::Start(off))?;
        let mut z = vec![0u8; len as usize];
        f.read_exact(&mut z)?;
        let mut out = Vec::new();
        liblzma::read::XzDecoder::new(&z[..]).read_to_end(&mut out)?;
        Ok(out)
    }

    fn read_text(&self, rel: &str) -> Result<String> {
        String::from_utf8(self.read_bytes(rel)?).with_context(|| format!("{rel} 非 UTF-8 文本"))
    }

    fn call_args(&self, rel: &str, fn_name: &str) -> Result<Vec<Value>> {
        let s = self.read_text(rel)?;
        let s = s.trim();
        let body = s
            .strip_prefix(fn_name)
            .and_then(|r| r.strip_prefix('('))
            .with_context(|| format!("{rel} 不是 {fn_name}(...) 格式"))?;
        let body = body
            .trim_end_matches(';')
            .trim_end()
            .strip_suffix(')')
            .context("JS 调用括号未闭合")?;
        Ok(serde_json::from_str(&format!("[{body}]"))?)
    }

    fn decode_payload(&self, s: &str) -> Result<Vec<u8>> {
        if let Some(rel) = s.strip_prefix('@') {
            return self.read_bytes(rel);
        }
        let raw_z = base64::engine::general_purpose::STANDARD.decode(s)?;
        let mut out = Vec::new();
        flate2::read::ZlibDecoder::new(&raw_z[..]).read_to_end(&mut out)?;
        Ok(out)
    }

    pub fn load_map(&self, map_name: &str) -> Result<TerrainGrid> {
        let summary = self
            .maps
            .iter()
            .find(|m| m.name == map_name)
            .with_context(|| format!("未找到地图: {map_name}"))?;
        let mut args = self.call_args(&format!("maps/{map_name}.js"), "WTV_MAP")?;
        let mut obj = args.pop().context("maps 文件为空")?;
        let h_str = obj
            .as_object_mut()
            .and_then(|o| o.remove("h"))
            .and_then(|v| v.as_str().map(str::to_owned))
            .context("地图缺少 h 高程字段")?;
        let meta: RawMapMeta = serde_json::from_value(obj)?;
        let mut h_bytes = self.decode_payload(&h_str)?;
        if h_str.ends_with(".hpred") {
            unpredict_hpred(&mut h_bytes, meta.width);
        }
        ensure!(
            h_bytes.len() == meta.width * meta.height * 2,
            "高度图字节数不匹配: 实际 {}, 预期 {}",
            h_bytes.len(),
            meta.width * meta.height * 2
        );
        let sc = (meta.hmax - meta.hmin) / 65535.0;
        let mut hv = Vec::with_capacity(meta.width * meta.height);
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for chunk in h_bytes.chunks_exact(2) {
            let q = u16::from_le_bytes([chunk[0], chunk[1]]);
            let elev = meta.hmin + q as f64 * sc;
            lo = lo.min(elev);
            hi = hi.max(elev);
            hv.push(elev as f32);
        }
        let title_zh = summary
            .title
            .clone()
            .or_else(|| summary.en.clone())
            .unwrap_or_else(|| map_name.to_string());
        Ok(TerrainGrid {
            id: map_name.to_string(),
            title_zh,
            h: hv,
            w: meta.width,
            hh: meta.height,
            cell: meta.cell,
            ox: meta.ox,
            oz: meta.oz,
            x1: meta.ox + (meta.width - 1) as f64 * meta.cell,
            z1: meta.oz + (meta.height - 1) as f64 * meta.cell,
            min_elev: lo,
            max_elev: hi,
        })
    }
}

fn unpredict_hpred(b: &mut [u8], w: usize) {
    let get = |b: &[u8], i: usize| u16::from_le_bytes([b[i * 2], b[i * 2 + 1]]);
    for i in 0..b.len() / 2 {
        let x = i % w;
        let l = if x > 0 { get(b, i - 1) } else { 0 };
        let u = if i >= w { get(b, i - w) } else { 0 };
        let ul = if x > 0 && i > w { get(b, i - w - 1) } else { 0 };
        let v = get(b, i).wrapping_add(l.wrapping_add(u).wrapping_sub(ul));
        b[i * 2..i * 2 + 2].copy_from_slice(&v.to_le_bytes());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_discover_and_load_wtv_terrain_pack() {
        let Some(reader) = WtvPackReader::auto_discover() else {
            // 独立克隆仓库且同级未放置《婴儿弹道计算机.exe》时跳过外部地形包断言，运行期自动退化为平面/雷达海拔模式
            return;
        };
        assert_eq!(reader.maps.len(), 56, "预期挂载全部 56 张陆战地图");
        let first_map = &reader.maps[0].name;
        let grid = reader.load_map(first_map).expect("加载首张陆战高程图失败");
        let cx = 0.5 * (grid.ox + grid.x1);
        let cz = 0.5 * (grid.oz + grid.z1);
        let h = grid.height_at(cx, cz);
        assert!(
            h.is_finite() && h >= grid.min_elev - 1.0 && h <= grid.max_elev + 1.0,
            "中心点高程异常: {h}"
        );
    }
}
