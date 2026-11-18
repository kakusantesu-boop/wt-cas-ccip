#!/usr/bin/env python3
"""
只读从本地 War-Thunder-Datamine (2.61.0.21) 与中文词表 CSV 提取：
1. 全局第三人称摄像机参数 (config/camera.blkx: tps / tps_alt)
2. 全部无制导航弹/燃烧弹/高阻减速弹物理参数 (gamedata/weapons/bombguns/*.blkx)
3. 全部无制导空射火箭弹两级发动机与气动参数 (gamedata/weapons/rocketGuns/*.blkx)
4. 全部携挂航弹或火箭弹的固定翼飞机与武装直升机词条、tpsCameraOffset、原生 CCIP 标记及挂载清单

严守边界：只读访问 ../wt-ballistic-visualizer/i18n_src，输出至 ../data/cas_db.json。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROJECT_ROOT = HERE.parent
WTV_ROOT = PROJECT_ROOT.parent / "wt-ballistic-visualizer"
I18N_DIR = WTV_ROOT / "i18n_src"
DM_ROOT = I18N_DIR / "wt-datamine"
ACES_DIR = DM_ROOT / "aces.vromfs.bin_u"
FM_DIR = ACES_DIR / "gamedata" / "flightmodels"
PRESET_DIR = FM_DIR / "weaponpresets"
ROCKET_DIR = ACES_DIR / "gamedata" / "weapons" / "rocketGuns"
CONTAINER_DIR = ACES_DIR / "gamedata" / "weapons" / "containers"
OUT_PATH = PROJECT_ROOT / "data" / "cas_db.json"

NATION_TAGS = {
    "us": "美", "usa": "美",
    "germ": "德", "germany": "德", "de": "德",
    "ussr": "苏", "su": "苏", "ru": "俄",
    "uk": "英", "britain": "英",
    "jp": "日", "japan": "日",
    "cn": "中", "china": "中",
    "it": "意", "italy": "意",
    "fr": "法", "france": "法",
    "sw": "瑞", "sweden": "瑞", "sws": "瑞",
    "il": "以", "israel": "以",
}

SUBTREE_GLYPHS = {
    "\u2417": "中系", "\u2419": "杯赛", "\u2420": "特别版",
    "\u2580": "缴获", "\u2582": "租借", "\u2583": "外购",
    "\u2584": "援助", "\u2585": "自卫队", "\u25c4": "西德",
    "\u25c6": "纪念", "\u25ca": "东德", "\u25cb": "礼包",
    "\u25cc": "瑞士", "\u25cd": "高级", "\u25d0": "希腊",
    "\u25d4": "匈牙利", "\u25d7": "荷兰", "\u25d8": "荷属",
    "\u25e1": "哥伦比亚", "\u25e2": "挪威", "\u25e5": "印尼",
    "\uf059": "以军",
}

EXCLUDED_UNIT_SUBSTRINGS = ("_football", "aaa_dummy", "tutorial_target", "killstreak", "uav_quadcopter")


def clean_wt(s: str) -> str:
    s = s.replace("\u200b", "").replace("\u200c", "").replace("\u200d", "").replace("\ufeff", "").replace("\xa0", " ")
    cyr_to_lat = str.maketrans({
        "А": "A", "В": "B", "Е": "E", "К": "K", "М": "M",
        "Н": "H", "О": "O", "Р": "P", "С": "C", "Т": "T", "Х": "X",
    })
    s = s.translate(cyr_to_lat)
    return re.sub(r"\s+", " ", s).strip()


def strip_wt_glyphs(s: str) -> str:
    badges = []
    out_chars = []
    for ch in s:
        if ch in SUBTREE_GLYPHS:
            badges.append(SUBTREE_GLYPHS[ch])
        elif 0xE000 <= ord(ch) <= 0xF8FF or 0x2400 <= ord(ch) <= 0x243F or (0x2500 <= ord(ch) <= 0x27BF):
            continue
        else:
            out_chars.append(ch)
    base = re.sub(r"\s+", " ", "".join(out_chars)).strip()
    if badges and base:
        return f"【{'/'.join(dict.fromkeys(badges))}】{base}"
    return base


def load_csv(fn: str) -> dict[str, tuple[str, str]]:
    out: dict[str, tuple[str, str]] = {}
    path = I18N_DIR / fn
    if not path.exists():
        return out
    with open(path, encoding="utf-8-sig", newline="") as f:
        r = csv.reader(f, delimiter=";", quotechar='"')
        hdr = next(r)
        ei, ci = hdr.index("<English>"), hdr.index("<Chinese>")
        for row in r:
            if len(row) > max(ei, ci) and row[0] and not row[0].startswith("<"):
                out[row[0].lower()] = (clean_wt(row[ei]), clean_wt(row[ci]))
    return out


def first(v):
    return v[0] if isinstance(v, list) and v else v


def as_list(v):
    return v if isinstance(v, list) else ([] if v is None else [v])


def iter_dicts(o):
    if isinstance(o, dict):
        yield o
        for v in o.values():
            yield from iter_dicts(v)
    elif isinstance(o, list):
        for v in o:
            yield from iter_dicts(v)


def num(v, default: float = 0.0) -> float:
    v = first(v)
    return float(v) if isinstance(v, (int, float)) and math.isfinite(v) else default


def opt_pos(v) -> float | None:
    v = first(v)
    return float(v) if isinstance(v, (int, float)) and math.isfinite(v) and v > 0 else None


def vec3(v, default: list[float]) -> list[float]:
    # Unwrap duplicate BLK key overrides serialized as list-of-lists [[x,y,z], [x,y,z]]
    while isinstance(v, list) and v and isinstance(v[-1], list):
        v = v[-1]
    if isinstance(v, list) and len(v) >= 3 and all(isinstance(x, (int, float)) for x in v[:3]):
        return [round(float(v[0]), 4), round(float(v[1]), 4), round(float(v[2]), 4)]
    return list(default)


def git_read_blobs(git_dir: Path, tree_paths: list[str]) -> dict[str, bytes]:
    """Read blobs from git HEAD via a single `git cat-file --batch` stream without modifying working tree."""
    req = "\n".join(f"HEAD:{p}" for p in tree_paths) + "\n"
    proc = subprocess.run(
        ["git", "-C", str(git_dir), "cat-file", "--batch"],
        input=req.encode("utf-8"),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    buf = proc.stdout
    pos = 0
    out: dict[str, bytes] = {}
    for p in tree_paths:
        nl = buf.index(b"\n", pos)
        header = buf[pos:nl].decode("utf-8", errors="replace")
        pos = nl + 1
        parts = header.split()
        if len(parts) >= 3 and parts[1] == "blob":
            size = int(parts[2])
            out[p] = buf[pos:pos + size]
            pos += size + 1  # trailing LF
    return out


def git_list_bombguns(git_dir: Path) -> list[str]:
    proc = subprocess.run(
        ["git", "-C", str(git_dir), "ls-tree", "-r", "--name-only", "HEAD", "aces.vromfs.bin_u/gamedata/weapons/bombguns"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
    )
    lines = proc.stdout.decode("utf-8").splitlines()
    return [ln.strip() for ln in lines if ln.strip().endswith(".blkx")]


def translate_weapon(stem: str, bullet_name: str | None, weap_lang: dict[str, tuple[str, str]]) -> tuple[str, str]:
    cands = []
    for raw in filter(None, [stem, bullet_name, re.sub(r"_default$", "", stem)]):
        low = raw.lower()
        cands.extend([
            f"weapons/{low}/short",
            f"weapons/{low}",
            f"{low}/name/short",
            f"{low}/name",
            low,
        ])
    for c in cands:
        hit = weap_lang.get(c)
        if hit and (hit[1] or hit[0]):
            zh = strip_wt_glyphs(hit[1] or hit[0])
            en = strip_wt_glyphs(hit[0] or hit[1])
            if zh:
                return zh, en
    fallback = stem.removesuffix("_default").replace("_", " ")
    return fallback, fallback


def extract_camera_config(raw_cam_bytes: bytes) -> dict:
    cam = json.loads(raw_cam_bytes.decode("utf-8"))

    def parse_cam_block(blk: dict) -> dict:
        return {
            "fov": num(blk.get("fov"), 90.0),
            "zoom_fov": num(blk.get("zoomFov"), 30.0),
            "target_offset": vec3(blk.get("targetOffset"), [500.0, 0.0, 0.0]),
            "camera_offset_neutral": vec3(blk.get("cameraOffsetNeutral"), [-17.0, 4.0, 0.0]),
            "camera_up_neutral": vec3(blk.get("cameraUpNeutral"), [0.0, 2.0, 0.0]),
            "camera_offset_elev_up": vec3(blk.get("cameraOffsetElevatorUp"), [-14.0, 4.0, 0.0]),
            "camera_offset_elev_down": vec3(blk.get("cameraOffsetElevatorDown"), [-14.0, 4.0, 0.0]),
            "camera_offset_rudder_left": vec3(blk.get("cameraOffsetRudderLeft"), [-14.0, 4.0, -1.0]),
            "camera_offset_rudder_right": vec3(blk.get("cameraOffsetRudderRight"), [-14.0, 4.0, 1.0]),
            "camera_up_ailerons_left": vec3(blk.get("cameraUpAileronsLeft"), [0.0, 2.0, 0.7]),
            "camera_up_ailerons_right": vec3(blk.get("cameraUpAileronsRight"), [0.0, 2.0, -0.7]),
            "sensitivity_ailerons": num(blk.get("sensitivityAilerons"), 0.3),
            "roll_angle_factor": num(blk.get("rollAngleFactor"), 0.01),
            "overturn_angle": num(blk.get("overturnAngle"), 80.0),
            "fov_smoothing": num(blk.get("cameraFovSmoothing"), 0.005),
        }

    return {
        "plane_tps": parse_cam_block(cam.get("tps") or {}),
        "heli_tps_alt": parse_cam_block(cam.get("tps_alt") or {}),
    }


def extract_bombs(bomb_blobs: dict[str, bytes], weap_lang: dict[str, tuple[str, str]]) -> dict[str, dict]:
    bombs: dict[str, dict] = {}
    for rel_path, raw in sorted(bomb_blobs.items()):
        stem = Path(rel_path).stem.lower()
        try:
            d = json.loads(raw.decode("utf-8"))
        except Exception:
            continue
        b = d.get("bomb")
        if not isinstance(b, dict):
            continue
        # Skip guided glide/laser/TV bombs and naval antisub depth charges/mortars
        if b.get("guidance") or b.get("guidanceType") or b.get("operated") or b.get("autoAiming"):
            continue
        if stem.startswith("bomb_uk_hedgehog") or stem.startswith("bomb_uk_limbo") or stem.startswith("bomb_ussr_mbu") or stem.startswith("bomb_ussr_rbu") or stem.startswith("bomb_ussr_rgb") or "srboc" in stem:
            continue

        mass = opt_pos(b.get("mass"))
        cal = opt_pos(b.get("caliber"))
        if not mass or not cal:
            continue

        drag_cx = opt_pos(b.get("dragCx")) or 0.035
        cx_k = opt_pos(b.get("CxK")) or 1.0
        bt_raw = b.get("brakeTime")
        brake_time = None
        if isinstance(bt_raw, list) and len(bt_raw) >= 2 and all(isinstance(x, (int, float)) for x in bt_raw[:2]):
            brake_time = [round(float(bt_raw[0]), 4), round(float(bt_raw[1]), 4)]
        brake_cx_k = opt_pos(b.get("brakeCxK")) or 1.0

        exp_type = str(first(b.get("explosiveType")) or "")
        icon_type = str(first(b.get("iconType")) or "")
        is_napalm = ("napalm" in exp_type.lower()) or ("napalm" in icon_type.lower()) or ("fire" in exp_type.lower())
        bname = str(first(b.get("bulletName")) or "") or None
        zh, en = translate_weapon(stem, bname, weap_lang)

        bombs[stem] = {
            "id": stem,
            "name_zh": zh,
            "name_en": en,
            "mass": round(mass, 3),
            "caliber": round(cal, 4),
            "length": round(num(b.get("length"), 1.6), 4),
            "dist_cm_to_stab": round(num(b.get("distFromCmToStab"), 0.75), 4),
            "drag_cx": round(drag_cx, 6),
            "cx_k": round(cx_k, 6),
            "brake_time": brake_time,
            "brake_cx_k": round(brake_cx_k, 4),
            "explosive_mass": round(num(b.get("explosiveMass"), 0.0), 3),
            "is_napalm": is_napalm,
        }
    return bombs


def extract_rockets(weap_lang: dict[str, tuple[str, str]]) -> dict[str, dict]:
    rockets: dict[str, dict] = {}
    for p in sorted(ROCKET_DIR.glob("*.blkx")):
        stem = p.stem.lower()
        try:
            d = json.loads(p.read_text("utf-8"))
        except Exception:
            continue
        r = d.get("rocket")
        if not isinstance(r, dict):
            continue
        # Filter out guided missiles / ATGMs / SAMs / AAMs / countermeasures
        if r.get("guidance") or r.get("guidanceType") or r.get("operated") or r.get("autoAiming"):
            continue
        btype = str(first(r.get("bulletType")) or "").lower()
        if "aam" in btype or "atgm" in btype or "sam" in btype or "flare" in btype or "chaff" in btype:
            continue
        if "countermeasure" in stem or "flare" in stem or "chaff" in stem or "dipole" in stem:
            continue

        mass = opt_pos(r.get("mass"))
        cal = opt_pos(r.get("caliber"))
        if not mass or not cal:
            continue

        force0 = num(r.get("force") if r.get("force") is not None else r.get("force0"), 0.0)
        t_fire0 = num(r.get("timeFire") if r.get("timeFire") is not None else r.get("timeFire0"), 0.0)
        v_start = num(r.get("startSpeed"), 0.0)
        if force0 <= 0.0 and v_start <= 0.0:
            continue

        m_end0 = opt_pos(r.get("massEnd")) or mass
        force1 = num(r.get("force1"), 0.0)
        t_fire1 = num(r.get("timeFire1"), 0.0)
        m_end1 = opt_pos(r.get("massEnd1")) or m_end0
        fire_delay = num(r.get("timeFireDelay") if r.get("timeFireDelay") is not None else r.get("fireDelay"), 0.0)

        # Flight drag uses CxK * rocket_cd0(Mach)! Never confuse pod parasite dragCx (e.g. 0.00001) with flight CxK.
        raw_cx_k = opt_pos(r.get("CxK"))
        cx_mach = raw_cx_k is not None
        cx_k = raw_cx_k or opt_pos(r.get("dragCx")) or 1.25
        pod_drag_cx = num(r.get("dragCx"), 0.0)

        wdk_raw = r.get("WdK")
        wd_k = num(wdk_raw[0] if isinstance(wdk_raw, list) and wdk_raw else wdk_raw, 0.1)

        bname = str(first(r.get("bulletName")) or "") or None
        zh, en = translate_weapon(stem, bname, weap_lang)

        rockets[stem] = {
            "id": stem,
            "name_zh": zh,
            "name_en": en,
            "mass": round(mass, 4),
            "mass_end": round(m_end0, 4),
            "mass_end1": round(m_end1, 4),
            "caliber": round(cal, 4),
            "length": round(num(r.get("length"), 1.5), 4),
            "cx_k": round(cx_k, 5),
            "cx_mach": cx_mach,
            "pod_drag_cx": round(pod_drag_cx, 6),
            "force0": round(force0, 2),
            "time_fire0": round(t_fire0, 4),
            "force1": round(force1, 2),
            "time_fire1": round(t_fire1, 4),
            "fire_delay": round(fire_delay, 4),
            "start_speed": round(v_start, 3),
            "use_start_speed": bool(r.get("useStartSpeed", True)),
            "end_speed": round(num(r.get("endSpeed"), 2000.0), 2),
            "max_speed_stat": round(num(r.get("maxSpeed"), 0.0), 2),
            "max_distance": round(num(r.get("maxDistance") or r.get("rangeMax"), 6000.0), 1),
            "time_life": round(num(r.get("timeLife"), 20.0), 2),
            "dist_cm_to_stab": round(num(r.get("distFromCmToStab"), 0.05), 4),
            "wd_k": round(wd_k, 4),
            "max_delta_angle": round(num(r.get("maxDeltaAngle"), 1.0), 3),
            "explosive_mass": round(num(r.get("explosiveMass"), 0.0), 3),
        }
    return rockets


def build_container_map() -> dict[str, list[str]]:
    raw_refs: dict[str, list[str]] = {}
    for p in sorted(CONTAINER_DIR.glob("*.blkx")):
        stem = p.stem.lower()
        try:
            d = json.loads(p.read_text("utf-8"))
        except Exception:
            continue
        blks = []
        for sub in iter_dicts(d):
            b = sub.get("blk")
            if isinstance(b, str):
                blks.append(b.replace("\\", "/").lower())
        if blks:
            raw_refs[stem] = list(dict.fromkeys(blks))

    resolved: dict[str, list[str]] = {}

    def resolve_one(cstem: str, depth: int = 0) -> list[str]:
        if depth > 4:
            return []
        if cstem in resolved:
            return resolved[cstem]
        leaves = []
        for b in raw_refs.get(cstem, []):
            bstem = Path(b).stem.lower()
            if "/containers/" in b:
                leaves.extend(resolve_one(bstem, depth + 1))
            else:
                leaves.append(b)
        resolved[cstem] = list(dict.fromkeys(leaves))
        return resolved[cstem]

    for k in raw_refs:
        resolve_one(k)
    return resolved


def extract_aircraft(
    bombs: dict[str, dict],
    rockets: dict[str, dict],
    containers: dict[str, list[str]],
    units_lang: dict[str, tuple[str, str]],
    heli_ids: set[str],
) -> list[dict]:
    preset_files = {p.stem.lower(): p for p in PRESET_DIR.glob("*.blkx")} if PRESET_DIR.exists() else {}
    preset_cache: dict[str, dict | None] = {}

    def load_preset(pstem: str) -> dict | None:
        if pstem not in preset_cache:
            pf = preset_files.get(pstem)
            if not pf:
                preset_cache[pstem] = None
            else:
                try:
                    preset_cache[pstem] = json.loads(pf.read_text("utf-8"))
                except Exception:
                    preset_cache[pstem] = None
        return preset_cache[pstem]

    aircraft_list: list[dict] = []
    for f in sorted(FM_DIR.glob("*.blkx")):
        uid = f.stem.lower()
        if any(ex in uid for ex in EXCLUDED_UNIT_SUBSTRINGS):
            continue
        try:
            d = json.loads(f.read_text("utf-8"))
        except Exception:
            continue
        if not isinstance(d, dict):
            continue

        shop_hit = units_lang.get(f"{uid}_shop") or units_lang.get(f"{uid}_0") or units_lang.get(uid)
        en_name = strip_wt_glyphs(shop_hit[0]) if shop_hit and shop_hit[0] else uid
        zh_name = strip_wt_glyphs(shop_hit[1]) if shop_hit and shop_hit[1] else en_name

        sources = [d]
        for sub in iter_dicts(d):
            bp = sub.get("blk")
            if isinstance(bp, str) and "weaponpresets" in bp.lower():
                pstem = Path(bp.replace("\\", "/")).stem.lower()
                pd = load_preset(pstem)
                if pd:
                    sources.append(pd)

        ac_bombs: dict[str, dict] = {}
        ac_rockets: dict[str, dict] = {}

        def register_weapon_ref(blk_path: str, external_flag: bool, pitch_deg: float):
            norm = blk_path.replace("\\", "/").lower()
            wstem = Path(norm).stem.lower()
            if "/containers/" in norm:
                for leaf in containers.get(wstem, []):
                    register_weapon_ref(leaf, True, pitch_deg)
                return
            if "/bombguns/" in norm and wstem in bombs:
                prev = ac_bombs.get(wstem)
                # Prefer external=True if available on external pylons; also record if internal bay exists
                if prev is None:
                    ac_bombs[wstem] = {"id": wstem, "external": external_flag, "has_internal_bay": not external_flag}
                else:
                    prev["external"] = prev["external"] or external_flag
                    prev["has_internal_bay"] = prev["has_internal_bay"] or (not external_flag)
            elif "/rocketguns/" in norm and wstem in rockets:
                if wstem not in ac_rockets:
                    ac_rockets[wstem] = {"id": wstem, "launcher_pitch_deg": round(pitch_deg, 3)}

        for src in sources:
            for sub in iter_dicts(src):
                bp = sub.get("blk")
                if not isinstance(bp, str):
                    continue
                low_bp = bp.lower()
                if not any(seg in low_bp for seg in ("bombguns", "rocketguns", "containers")):
                    continue
                ext = bool(sub.get("external", False))
                # Some rocket launchers define shootOffset or angles/pitch
                p_deg = 0.0
                angles = sub.get("angles")
                if isinstance(angles, list) and len(angles) >= 2 and isinstance(angles[1], (int, float)):
                    p_deg = float(angles[1])
                register_weapon_ref(bp, ext, p_deg)

        if not ac_bombs and not ac_rockets:
            continue

        utype = str(first(d.get("type")) or "")
        is_heli = (uid in heli_ids) or ("helicopter" in utype.lower())
        cockpit = d.get("cockpit") if isinstance(d.get("cockpit"), dict) else {}

        prefix = uid.split("_")[0].split("-")[0]
        nation = NATION_TAGS.get(prefix, "")
        display_zh = f"[{nation}] {zh_name}" if nation and not zh_name.startswith("[") else zh_name

        aircraft_list.append({
            "id": uid,
            "name_zh": display_zh,
            "name_en": en_name,
            "is_heli": is_heli,
            "tps_offset": vec3(d.get("tpsCameraOffset"), [0.0, 0.0, 0.0]),
            "tps_target_offset_at_zoom": vec3(d.get("tpsCameraTargetOffsetAtZoom"), [0.0, 0.0, 0.0]),
            "static_aim_pitch_deg": round(num(cockpit.get("staticAimPitch"), 0.0), 3),
            "has_auto_bay_door": bool(d.get("hasAutomaticBayDoor")),
            "ccip_bombs": bool(d.get("haveCCIPForBombs")),
            "ccip_rockets": bool(d.get("haveCCIPForRocket")),
            "ccrp_bombs": bool(d.get("haveCCRPForBombs")),
            "bombs": list(ac_bombs.values()),
            "rockets": list(ac_rockets.values()),
        })

    return aircraft_list


def main() -> int:
    global DM_ROOT, ACES_DIR, FM_DIR, PRESET_DIR, ROCKET_DIR, CONTAINER_DIR, OUT_PATH
    ap = argparse.ArgumentParser(description="提取战雷 CAS 航弹/火箭弹与 TPS 摄像机数据库")
    ap.add_argument("--datamine", type=Path, default=DM_ROOT, help="War-Thunder-Datamine 仓库路径")
    ap.add_argument("--out", type=Path, default=OUT_PATH, help="输出 cas_db.json 路径")
    args = ap.parse_args()

    DM_ROOT = args.datamine.resolve()
    ACES_DIR = DM_ROOT / "aces.vromfs.bin_u"
    FM_DIR = ACES_DIR / "gamedata" / "flightmodels"
    PRESET_DIR = FM_DIR / "weaponpresets"
    ROCKET_DIR = ACES_DIR / "gamedata" / "weapons" / "rocketGuns"
    CONTAINER_DIR = ACES_DIR / "gamedata" / "weapons" / "containers"
    OUT_PATH = args.out.resolve()

    dm_version = (DM_ROOT / "version").read_text("utf-8").strip() if (DM_ROOT / "version").exists() else "unknown"
    print(f"[1/5] 加载拆包本地化词表 (wt-datamine {dm_version})...")
    units_lang = load_csv("units.csv")
    weap_lang = load_csv("units_weaponry.csv")

    print("[2/5] 从 git 对象库只读流式读取 camera.blkx、wpcost.blkx 与 bombguns...")
    bomb_paths = git_list_bombguns(DM_ROOT)
    cam_path = "aces.vromfs.bin_u/config/camera.blkx"
    wpcost_path = "char.vromfs.bin_u/config/wpcost.blkx"
    blobs = git_read_blobs(DM_ROOT, [cam_path, wpcost_path] + bomb_paths)

    cam_cfg = extract_camera_config(blobs.pop(cam_path))
    wpcost_bytes = blobs.pop(wpcost_path, b"{}")
    wpcost_json = json.loads(wpcost_bytes.decode("utf-8"))
    heli_ids = {
        k.lower()
        for k, v in wpcost_json.items()
        if isinstance(v, dict) and "helicopter" in str(v.get("unitClass", "")).lower()
    }
    bombs = extract_bombs(blobs, weap_lang)
    print(f"      已提取无制导航弹/燃烧弹/减速弹: {len(bombs)} 种, 识别直升机 ID: {len(heli_ids)} 架")

    print("[3/5] 提取无制导空射火箭弹与发射巢映射...")
    rockets = extract_rockets(weap_lang)
    containers = build_container_map()
    print(f"      已提取无制导火箭弹: {len(rockets)} 种, 武器挂架/发射巢: {len(containers)} 种")

    print("[4/5] 扫描飞机与直升机挂载配置...")
    aircraft = extract_aircraft(bombs, rockets, containers, units_lang, heli_ids)
    heli_cas_cnt = sum(1 for a in aircraft if a["is_heli"])
    print(f"      已提取具备 CAS 航弹/火箭弹能力的载具: {len(aircraft)} 架 (其中武装直升机 {heli_cas_cnt} 架)")

    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "datamine_version": dm_version,
        "camera": cam_cfg,
        "bombs": bombs,
        "rockets": rockets,
        "aircraft": aircraft,
    }
    OUT_PATH.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    size_kb = OUT_PATH.stat().st_size / 1024.0
    print(f"[5/5] 已写出 {OUT_PATH} ({size_kb:.1f} KB)")

    # 关键样本断言自检
    ac_by_id = {a["id"]: a for a in aircraft}
    for req_ac in ("a_10a_early", "p-47d-28", "su_25", "b-29", "ah_64d", "ka_50"):
        assert req_ac in ac_by_id, f"缺失关键验证机型: {req_ac}"
        a = ac_by_id[req_ac]
        print(
            f"  ✔ 样本核验 {req_ac}: {a['name_zh']} | heli={a['is_heli']} | tps_offset={a['tps_offset']} "
            f"| zoom_target_off={a['tps_target_offset_at_zoom']} | 原生CCIP(弹={a['ccip_bombs']},箭={a['ccip_rockets']}) "
            f"| 可挂炸弹={len(a['bombs'])}种, 火箭弹={len(a['rockets'])}种"
        )
    assert ac_by_id["ah_64d"]["is_heli"] is True and ac_by_id["ka_50"]["is_heli"] is True
    assert ac_by_id["a_10a_early"]["is_heli"] is False
    for req_b in ("us_500lb_mk_82_ldgp", "us_500lb_mk_82_ldgp_snakeye", "su_fab_500m_62t"):
        assert req_b in bombs, f"缺失关键验证航弹: {req_b}"
        b = bombs[req_b]
        print(
            f"  ✔ 航弹核验 {req_b}: {b['name_zh']} | m={b['mass']}kg d={b['caliber']}m "
            f"dragCx={b['drag_cx']} CxK={b['cx_k']} brake={b['brake_time']}x{b['brake_cx_k']}"
        )
    for req_r in ("us_hydra_70_m247", "us_5_in_hvar", "su_s_8ko_rocket"):
        assert req_r in rockets, f"缺失关键验证火箭弹: {req_r}"
        r = rockets[req_r]
        print(
            f"  ✔ 火箭核验 {req_r}: {r['name_zh']} | m={r['mass']}->{r['mass_end']}kg "
            f"F={r['force0']}Nx{r['time_fire0']}s CxK={r['cx_k']} (podDrag={r['pod_drag_cx']})"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
