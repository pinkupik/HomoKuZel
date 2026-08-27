# HomoKuŽel

Interactive tool for perspective rectification. Load an image, click reference points with known real-world coordinates, and get a metric bird's-eye view in real time.

Built in Rust with [egui](https://github.com/emilk/egui).

## Features

- **Live preview** — the rectified output updates instantly as you add, move, or edit points
- **Per-point error feedback** — color-coded reprojection error (green < 5 cm, yellow < 20 cm, red beyond) so bad clicks are obvious at a glance
- **Native file dialogs** — open images, save/load projects (.json), export output via OS file pickers
- **Drag and drop** — drop image or project files directly into the window
- **Headless export** — `homokuzel export project.json output.png --grid` for scripting / CI
- **Cross-platform** — builds on Windows, macOS (Apple Silicon), and Linux

## Build

Requires a Rust toolchain ([rustup](https://rustup.rs/), stable channel).

On Linux you also need X11/GL dev headers:

```bash
sudo apt install libx11-dev libxcursor-dev libxrandr-dev libxi-dev \
                  libgl1-mesa-dev libxkbcommon-dev
```

```bash
cargo build --release
```

## Usage

```bash
# Launch the GUI
./target/release/homokuzel

# Launch with an image pre-loaded
./target/release/homokuzel photo.jpg

# Headless export from a saved project
./target/release/homokuzel export project.json output.png --grid
```

### GUI workflow

1. Open or drag-and-drop an image.
2. Left-click to place reference points. A dialog asks for the real-world X/Y coordinates (in metres). Tab between fields, Enter to confirm.
3. With 4+ points the homography is computed and the bird's-eye view appears in the right pane.
4. Adjust points as needed — error indicators highlight inaccurate ones.
5. Export the rectified image as PNG (optionally with a 1 m grid overlay).

### Controls

- **Scroll** to zoom, **drag** to pan (both panes)
- **Click** empty space to add a point
- **Drag** an existing marker to reposition it
- **Fit** button resets the view

## Project files

Projects are saved as JSON containing the image path, point coordinates, and export settings. They can be reloaded later or used with the headless `export` subcommand.

## Architecture

| File | Purpose |
|------|---------|
| `main.rs` | Entry point, CLI `export` subcommand |
| `app.rs` | GUI (eframe/egui) |
| `homography.rs` | Normalized DLT solver (eigendecomposition of AᵀA) |
| `warp.rs` | Inverse-mapped bilinear warp, parallelized with rayon |
| `project.rs` | JSON project save/load (serde) |

## Testing

```bash
cargo test
```

Two unit tests verify the homography solver against synthetic ground-truth data (4-point minimal case and 8-point overdetermined case), both asserting sub-micrometre reprojection error.

## License

MIT
