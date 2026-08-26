# birdseye_tool

Interactive Rust rewrite of `skidpad_birdseye.py`: click reference points on a
drone/angled photo, edit their real-world (X, Y) coordinates in a live table,
and watch the metric bird's-eye rectification update in real time on the
other side of the window. Built for pulling cone-map-ready backgrounds out of
drone shots of prebuilt Formula Student tracks, feeding straight into your
`map_editor.py` background-image workflow.

## Status

Core homography math and the warp are unit-tested and were verified
end-to-end against a synthetic test photo with a known ground-truth
homography (see "Testing" below) — reprojection error comes out to 0.0 cm on
an exact 4-point fit, and the rectified output visually checks out (a
trapezoidal test rectangle straightens into a true axis-aligned rectangle).

The GUI (`app.rs`) compiles clean but I could not click-test it myself — no
display in the environment I built this in. Read through `image_panel()` /
`table_panel()` / `birdseye_panel()` once before you trust it blindly, and
expect a UI paper-cut or two on first run.

## Build & run

Needs a normal Rust toolchain (`rustup` is easiest — stable channel, this
targets edition 2021, MSRV isn't pinned aggressively). On Linux/WSL2 you'll
also need the usual GTK-less X11/GL dev headers for `winit`'s x11 backend
and OpenGL:

```bash
sudo apt install libx11-dev libxcursor-dev libxrandr-dev libxi-dev \
                  libgl1-mesa-dev libxkbcommon-dev
cargo build --release
```

WSLg (current WSL2) provides both an X server and Wayland compositor out of
the box, so the window should just appear. This build deliberately compiles
**without** the `wayland` winit backend (only `x11`) — see "A note on the
build environment" below for why; it makes no difference at runtime under
WSLg or any other X11/XWayland setup.

```bash
# GUI, optionally pre-loading an image
./target/release/birdseye_tool path/to/drone_photo.jpg

# Headless export from a saved project — no window, for GitLab CI
./target/release/birdseye_tool export project.json output.png --grid
```

## Using it

- **Left pane**: your photo. Click empty space to drop a new reference
  point — a small window pops up asking for its real-world X/Y in metres.
  Click-drag an existing marker to reposition it without retyping
  coordinates. Scroll to zoom, "Fit" resets the view.
- **Table** (top of the right side): every point, editable — label, world
  X/Y (drag or click-and-type), per-point reprojection error once ≥4 points
  exist, delete button. Error is color-coded on the image markers too:
  green < 5cm, yellow < 20cm, red beyond that, so a mis-clicked point is
  obvious at a glance — delete and redo it rather than fighting bad data.
- **Right pane**: the live bird's-eye warp, recomputed on every point edit
  (add/move/retype/delete). Scroll to zoom, drag to pan, "Fit" resets —
  same controls as the left pane. No grid overlay here; that's export-only
  now (see below), so the preview stays uncluttered while you calibrate.
- **Toolbar**: Streamlined single-row toolbar with native file dialogs
  (`rfd`):
  - **Open Image**: Opens native file dialog to choose drone photo (PNG,
    JPG, WEBP, etc.) with compact filename display and tooltip.
  - **Project Save/Load**: Save and load calibration projects (.json) via
    native file pickers.
  - **Drag and Drop**: Drag image files or `project.json` files directly into
    the window to load them instantly.
  - **px/m and margin controls**: Interactive sliders/drag-values for
    resolution and border padding.
  - **Export Birdseye**: Native save dialog to write the full-resolution
    rectified PNG (up to 8000px on long side), with optional "1m grid" baked in.

## Architecture

- `homography.rs` — normalized DLT. Builds the same 2n×9 constraint matrix
  the textbook derivation uses, but solves for the null space via
  eigendecomposition of AᵀA (always 9×9) rather than SVD of A directly —
  see the note below on why.
- `warp.rs` — inverse-mapped bilinear warp (canvas pixel → world metres →
  source pixel), parallelized over output rows with `rayon`. Same
  "clamp absurd canvas size" safety net as the Python version, for when a
  point set implies a wildly larger reprojection than expected.
- `project.rs` — serde JSON save/load.
- `app.rs` — eframe/egui UI, single window, no async.
- `main.rs` — GUI entry point + the `export` subcommand for headless/CI use.

### A note on the build environment (skip if you don't care)

I compiled and tested this in a sandbox with `rustc` 1.75 (Dec 2023) against
today's crates.io index — a lot of transitive dependencies now require
newer compilers, so getting a real `cargo build` to succeed took pinning
~40 transitive crate versions by hand to a mutually-compatible set from
early-2024 (matching egui 0.27's own contemporaneous lockfile). None of this
is baked into `Cargo.toml` — on your machine with an up-to-date `rustup`
toolchain, `cargo build` will just resolve the current versions of
everything normally and this whole paragraph is irrelevant. Mentioning it
only because if you *do* hit a similarly-ancient toolchain somewhere (an old
CI image, say), you'll know why.

### The bug I found and fixed while testing

First pass used `nalgebra::SVD` directly on the 2n×9 constraint matrix and
picked the last row of `V^T` as the null-space solution. That's correct in
general — *except* nalgebra computes a **thin** SVD, and for the minimal
case of exactly 4 correspondences (n=4 → an 8×9 matrix), thin SVD only
returns 8 of the 9 right-singular-vectors, silently dropping the exact one
representing the null space. It doesn't error, it just hands back a
plausible-looking but wrong homography (I caught it because a synthetic
4-point test gave 333cm RMS error instead of ~0). Fixed by solving via
eigendecomposition of AᵀA instead, which is always the full 9×9 regardless
of point count, so the null-space eigenvector is never truncated. Both a
4-point and an 8-point synthetic test now pass at < 1e-6 m error — run them
with `cargo test`.

## Testing

```bash
cargo test --release
```

Two unit tests in `homography.rs`: an exact 4-point minimal case and an
over-determined 8-point case generated from a known homography, both
asserting sub-micrometre reprojection error. If you change the DLT math,
these are your tripwire.

## Suggestions / things I'd add next

Roughly in the order I'd reach for them:

1. **RANSAC / robust fitting** once you're routinely clicking 6+ points.
   Right now it's plain least-squares — the color-coded error feedback is a
   manual substitute (spot the red point, delete, redo), which works but a
   genuine outlier-rejecting fit would be more forgiving for less careful
   clicking, especially on cones/gate posts that are easy to misjudge by a
   few pixels in a compressed photo.
2. **Lens undistortion pass** before the homography. A pure planar
   homography assumes an ideal pinhole; many drone cameras (DJI etc.) have
   enough barrel distortion that it'll visibly bow straight lines near the
   image edges on a wide-FOV shot. If you have the camera's intrinsics/
   distortion coefficients (often in the photo's EXIF or the drone's known
   calibration), undistorting first would meaningfully tighten accuracy,
   especially for track sections far from image center.
3. **Named landmark templates.** Instead of typing raw X/Y every time,
   a small dropdown of known layouts (skidpad left/right circle center,
   acceleration lane start gate, etc.) that auto-fills the world
   coordinates from your known geometry constants — fewer typos, faster
   clicking, and it'd generalize the tool beyond just the skidpad the way
   your other events (autocross, trackdrive) would need.
4. **Multi-photo stitching.** A single drone shot often won't cover a full
   track. Even a basic "load N photos, each gets its own homography from
   shared or independently-clicked reference points, composite onto one
   canvas" mode would cover that without needing real feature-based
   mosaicking.
5. **Background warp thread.** Right now recompute happens on the UI
   thread every frame something changes; fine at the current 3000px cap,
   but if you ever want full-res live preview on a big photo, move the warp
   to a worker thread and let the UI stay responsive while it catches up.
6. **Lens distortion calibration & profile storage**: Save drone camera calibration
   parameters (fx, fy, cx, cy, k1, k2, p1, p2) with project profiles.
7. **Multi-photo stitching**: Load N drone shots and mosaic onto one birdseye canvas.
8. **GeoJSON/CSV cone list export**: Direct export of surveyed reference points and
   detected cones to feed into Formula Student autonomous simulation pipelines.

None of these are load-bearing for a first real use — the current tool
already does the click → edit → live-preview → export loop you asked for.
