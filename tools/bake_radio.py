#!/usr/bin/env python3
"""Bake the F-22 radio-chatter clips with the local Qwen3-TTS models.

Reads ``tools/radio_lines.json`` (the single source of truth for who says
what), synthesises each line, then runs the same radio chain the ATC lesson
clips use: 200–2600 Hz band-pass, a carrier-hiss bed underneath, peak
normalised to −6 dBFS, 48 kHz stereo.  Two-voice exchanges (the opening
check-in) are baked as one clip with a fixed gap, like the 737's
controller-plus-readback assets.

Two voice kinds are supported per role:

* ``builtin`` – the 0.6B CustomVoice model's preset speakers (Ryan, Aiden …);
* ``clone``   – the 1.7B Base model cloning a reference clip.  The AWACS uses
  this with the existing C172 ATC clip as reference, so the controller on the
  combat net is literally the same voice as the lesson tower.

Examples:
    ~/qwen3_tts/bin/python tools/bake_radio.py --only r03_fox2 --out /tmp/x
    ~/qwen3_tts/bin/python tools/bake_radio.py                # every line
    ~/qwen3_tts/bin/python tools/bake_radio.py --only r02_tally --wing Aiden
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import librosa
import numpy as np
import soundfile as sf
import torch

REPO = Path(__file__).resolve().parent.parent
LINES_JSON = REPO / "tools" / "radio_lines.json"

# Radio chain, matching the ATC lesson clips (200 Hz – 2.6 kHz + carrier bed).
HIGHPASS_HZ = 200
LOWPASS_HZ = 2600
# The bed's RMS under the voice; the ATC clips sit about here.
HISS_RMS_DBFS = -35.0
BED_SEED = 0x5EED
# Output peak, same ballpark as the ATC clips (their peaks land near −7).
PEAK_DBFS = -6.0
OUT_SR = 48_000
# Trim head/tail silence but keep a little air so the bed does not butt-join.
TRIM_DBFS = -50.0
TRIM_PAD_S = 0.03


def db(x: float) -> float:
    return 20.0 * np.log10(max(x, 1e-12))


def load_manifest(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def trim_silence(y: np.ndarray, sr: int) -> np.ndarray:
    thresh = 10.0 ** (TRIM_DBFS / 20.0)
    loud = np.flatnonzero(np.abs(y) > thresh)
    if loud.size == 0:
        return y
    pad = int(TRIM_PAD_S * sr)
    lo = max(0, loud[0] - pad)
    hi = min(len(y), loud[-1] + pad)
    return y[lo:hi]


class VoiceBank:
    """Lazily load the TTS models the manifest asks for and synthesise one part."""

    def __init__(self, manifest: dict):
        self.manifest = manifest
        self.builtin_dir = Path(os.path.expanduser(manifest["builtin_model"]))
        self.models: dict[str, object] = {}

    def _load(self, spec: dict):
        if "clone" in spec:
            key = f"clone:{spec['clone']['model']}"
            if key not in self.models:
                from qwen_tts import Qwen3TTSModel

                path = Path(os.path.expanduser(spec["clone"]["model"]))
                dtype = (
                    torch.bfloat16
                    if spec["clone"].get("dtype", "bfloat16") == "bfloat16"
                    else torch.float32
                )
                print(f"loading clone model {path.name} …", flush=True)
                self.models[key] = Qwen3TTSModel.from_pretrained(
                    str(path), device_map="cpu", dtype=dtype
                )
            return self.models[key]
        key = "builtin"
        if key not in self.models:
            from qwen_tts import Qwen3TTSModel

            print(f"loading builtin model {self.builtin_dir.name} …", flush=True)
            self.models[key] = Qwen3TTSModel.from_pretrained(
                str(self.builtin_dir), device_map="cpu"
            )
        return self.models[key]

    def synth(self, part: dict, spec: dict, seed: int) -> tuple[np.ndarray, int]:
        model = self._load(spec)
        text = part["text"]
        torch.manual_seed(seed)
        if "clone" in spec:
            clone = spec["clone"]
            ref = str(REPO / clone["ref"])
            wavs, sr = model.generate_voice_clone(
                text=[text],
                language=["English"],
                ref_audio=[ref],
                ref_text=[clone["ref_text"]],
            )
        else:
            wavs, sr = model.generate_custom_voice(
                text=[text],
                speaker=[spec["speaker"]],
                language=["English"],
                do_sample=True,
            )
        return np.asarray(wavs[0], dtype=np.float32), int(sr)


def part_audio(bank: VoiceBank, part: dict, voices: dict, wing_override: str | None,
               seed: int) -> tuple[np.ndarray, int]:
    role = part["role"]
    if role == "wing" and wing_override:
        spec = {"speaker": wing_override}
    else:
        spec = voices[role]
    y, sr = bank.synth(part, spec, seed)
    y = trim_silence(y, sr)
    if sr != OUT_SR:
        y = librosa.resample(y, orig_sr=sr, target_sr=OUT_SR)
        sr = OUT_SR
    speed = float(part.get("speed", 1.0))
    if abs(speed - 1.0) > 1e-3:
        y = librosa.effects.time_stretch(np.ascontiguousarray(y), rate=speed)
    return y, sr


def speaker_name(role: str, voices: dict, wing_override: str | None) -> str:
    if role == "wing" and wing_override:
        return wing_override
    spec = voices[role]
    return spec.get("speaker") or spec.get("name") or "clone"


def build_voice(bank: VoiceBank, line: dict, voices: dict, wing_override: str | None,
                base_seed: int) -> tuple[np.ndarray, list[dict]]:
    chunks: list[np.ndarray] = []
    used: list[dict] = []
    for i, part in enumerate(line["parts"]):
        if "gap" in part:
            chunks.append(np.zeros(int(float(part["gap"]) * OUT_SR), dtype=np.float32))
            continue
        y, _ = part_audio(bank, part, voices, wing_override, base_seed + i)
        chunks.append(y)
        used.append({
            "role": part["role"],
            "speaker": speaker_name(part["role"], voices, wing_override),
            "text": part["text"],
            "seconds": round(len(y) / OUT_SR, 3),
        })
    return np.concatenate(chunks) if chunks else np.zeros(1, dtype=np.float32), used


def bandpass_voice(voice_path: Path, dest: Path) -> None:
    """Radio band-pass + gentle compression on the voice, at 48 kHz stereo."""
    filt = f"highpass=f={HIGHPASS_HZ}:poles=2,lowpass=f={LOWPASS_HZ}:poles=2"
    subprocess.run(
        [
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
            "-i", str(voice_path),
            "-af", f"{filt},acompressor=threshold=-18dB:ratio=3:attack=5:release=80",
            "-ar", str(OUT_SR), "-ac", "2", "-c:a", "pcm_f32le",
            str(dest),
        ],
        check=True,
    )


def build_bed(spec: dict, dest: Path, seconds: float, seed: int) -> np.ndarray:
    """Build the carrier bed for `seconds`, returned at its natural level.

    The shipped ``atc_static`` beds contain digital-silence gaps (they were
    recorded with squelch pauses), so looping them under a line leaves holes in
    the hiss.  A seeded band-limited noise gives the same spectral footprint
    with a continuous floor; keep ``kind: file`` in the manifest to loop a real
    recording instead.
    """
    filt = f"highpass=f={HIGHPASS_HZ}:poles=2,lowpass=f={LOWPASS_HZ}:poles=2"
    if spec.get("kind", "noise") == "file":
        subprocess.run(
            [
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                "-stream_loop", "-1", "-i", str(REPO / spec["file"]),
                "-t", f"{seconds:.3f}",
                "-af", filt,
                "-ar", str(OUT_SR), "-ac", "2", "-c:a", "pcm_f32le",
                str(dest),
            ],
            check=True,
        )
    else:
        n = int(seconds * OUT_SR)
        rng = np.random.default_rng(seed)
        # 0.1 keeps the float file well inside range before the filter pass.
        noise = (rng.standard_normal(n) * 0.1).astype(np.float32)
        src = dest.with_name(dest.stem + "_raw.wav")
        sf.write(str(src), noise, OUT_SR, subtype="FLOAT")
        subprocess.run(
            [
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
                "-i", str(src),
                "-af", filt,
                "-ar", str(OUT_SR), "-ac", "2", "-c:a", "pcm_f32le",
                str(dest),
            ],
            check=True,
        )
        src.unlink(missing_ok=True)
    y, _ = sf.read(str(dest), always_2d=True, dtype="float32")
    return y


def finish(voice_path: Path, hiss_path: dict, dst: Path, scratch: Path) -> tuple[float, float]:
    """Band-pass, level the voice to `PEAK_DBFS`, then lay the bed under it.

    Order matters: normalising the *mix* would drag the bed down by whatever
    headroom the voice needed (the TTS clips come out hot), so the voice is
    levelled first and the bed is added at a fixed RMS afterwards.
    """
    filtered = scratch / (dst.stem + "_band.wav")
    bandpass_voice(voice_path, filtered)
    v, _ = sf.read(str(filtered), always_2d=True, dtype="float32")
    peak = float(np.max(np.abs(v))) if v.size else 0.0
    if peak > 0.0:
        v *= 10.0 ** ((PEAK_DBFS - db(peak)) / 20.0)

    bed = scratch / (dst.stem + "_hiss.wav")
    h = build_bed(hiss_path, bed, len(v) / OUT_SR, BED_SEED)
    hrms = float(np.sqrt(np.mean(h**2))) if h.size else 0.0
    if hrms > 0.0:
        h *= 10.0 ** ((HISS_RMS_DBFS - db(hrms)) / 20.0)

    n = min(len(v), len(h))
    mix = v[:n] + h[:n]
    over = float(np.max(np.abs(mix)))
    if over > 0.95:
        mix *= 0.95 / over
    dst.parent.mkdir(parents=True, exist_ok=True)
    sf.write(str(dst), mix, OUT_SR, subtype="PCM_24")
    filtered.unlink(missing_ok=True)
    bed.unlink(missing_ok=True)
    return float(np.max(np.abs(mix))), float(np.sqrt(np.mean(mix**2)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--lines", type=Path, default=LINES_JSON)
    ap.add_argument("--out", type=Path, default=REPO / "assets" / "audio" / "radio")
    ap.add_argument("--only", default="", help="comma-separated line ids")
    ap.add_argument("--suffix", default="", help="appended to the output name")
    ap.add_argument("--wing", default=None, help="override the wingman speaker")
    ap.add_argument("--seed", type=int, default=20261027)
    ap.add_argument(
        "--sidecars",
        action="store_true",
        help="write a .wav.json next to each clip (provenance for a picks folder)",
    )
    args = ap.parse_args()

    spec = load_manifest(args.lines)
    bed_spec = spec.get("bed", {"kind": "file", "file": spec["hiss"]})
    voices = {role: ({"speaker": v} if isinstance(v, str) else v) for role, v in spec["voices"].items()}

    wanted = {s.strip() for s in args.only.split(",") if s.strip()}
    lines = [l for l in spec["lines"] if not wanted or l["id"] in wanted]
    if not lines:
        print(f"no lines matched --only={args.only!r}", file=sys.stderr)
        return 2

    bank = VoiceBank(spec)
    torch.set_num_threads(os.cpu_count() or 4)
    (REPO / ".tmp").mkdir(exist_ok=True)
    tmp_root = Path(tempfile.mkdtemp(prefix="radio_bake_", dir=REPO / ".tmp"))
    print(f"scratch: {tmp_root}", flush=True)

    try:
        for idx, line in enumerate(lines):
            lid = line["id"]
            seed = args.seed + idx * 17
            print(f"[{lid}] synthesising …", end="", flush=True)
            voice, used = build_voice(bank, line, voices, args.wing, seed)
            voice_path = tmp_root / f"{lid}_voice.wav"
            sf.write(str(voice_path), voice, OUT_SR, subtype="FLOAT")

            name = f"{lid}{args.suffix}.wav"
            dst = args.out / name
            peak, rms = finish(voice_path, bed_spec, dst, tmp_root)
            print(f"\r[{lid}] {name}  {len(voice)/OUT_SR:5.2f}s  "
                  f"peak {db(peak):6.1f} dBFS  rms {db(rms):6.1f} dBFS", flush=True)

            if not args.sidecars:
                continue
            meta = {
                "id": lid,
                "file": name,
                "note": line.get("note", ""),
                "seed": seed,
                "sample_rate": OUT_SR,
                "parts": used,
                "peak_dbfs": round(db(peak), 2),
                "rms_dbfs": round(db(rms), 2),
            }
            dst.with_suffix(".wav.json").write_text(
                json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)

    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
