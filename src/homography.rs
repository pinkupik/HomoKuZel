//! Homography estimation via normalized DLT (Direct Linear Transform).
//!
//! Given N >= 4 correspondences between pixel coordinates (u, v) and world
//! coordinates (X, Y) in metres, solves for the 3x3 homography H such that
//!     [X, Y, 1]^T ~ H * [u, v, 1]^T
//! (up to scale). With N > 4 this is an overdetermined system and the
//! solution is the least-squares fit obtained via SVD -- the same
//! normalized-DLT approach cv2.findHomography uses internally, minus the
//! RANSAC outlier rejection (see suggestions in the README for adding that).

use nalgebra::{Matrix3, SymmetricEigen};

#[derive(Debug, Clone, Copy)]
pub struct Correspondence {
    pub pixel: (f64, f64),
    pub world: (f64, f64),
}

#[derive(Debug, Clone)]
pub struct HomographyResult {
    /// Maps homogeneous image pixel -> homogeneous world metres.
    pub h_img_to_world: Matrix3<f64>,
    /// Inverse: world metres -> image pixel. None if H was singular.
    pub h_world_to_img: Option<Matrix3<f64>>,
    /// Per-point reprojection error in metres (world space).
    pub reproj_errors: Vec<f64>,
    pub rms_error: f64,
    #[allow(dead_code)]
    pub mean_error: f64,
    pub max_error: f64,
}

/// Computes the similarity transform that normalizes a point set to have
/// centroid at the origin and average distance sqrt(2) from it. Returns the
/// 3x3 transform matrix and the normalized points.
fn normalize_points(pts: &[(f64, f64)]) -> (Matrix3<f64>, Vec<(f64, f64)>) {
    let n = pts.len() as f64;
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |acc, p| (acc.0 + p.0, acc.1 + p.1));
    let cx = sx / n;
    let cy = sy / n;

    let mean_dist: f64 = pts
        .iter()
        .map(|p| (((p.0 - cx).powi(2) + (p.1 - cy).powi(2)) as f64).sqrt())
        .sum::<f64>()
        / n;

    let scale = if mean_dist > 1e-12 { (2.0_f64).sqrt() / mean_dist } else { 1.0 };

    let t = Matrix3::new(
        scale, 0.0, -scale * cx,
        0.0, scale, -scale * cy,
        0.0, 0.0, 1.0,
    );

    let normalized = pts
        .iter()
        .map(|p| (scale * (p.0 - cx), scale * (p.1 - cy)))
        .collect();

    (t, normalized)
}

/// Solves for H (mapping `src` -> `dst`) using normalized DLT + SVD.
/// Requires at least 4 correspondences that are not all collinear.
pub fn solve_homography(correspondences: &[Correspondence]) -> anyhow::Result<HomographyResult> {
    let n = correspondences.len();
    if n < 4 {
        anyhow::bail!("need at least 4 correspondences, got {n}");
    }

    let src_pts: Vec<(f64, f64)> = correspondences.iter().map(|c| c.pixel).collect();
    let dst_pts: Vec<(f64, f64)> = correspondences.iter().map(|c| c.world).collect();

    let (t_src, src_norm) = normalize_points(&src_pts);
    let (t_dst, dst_norm) = normalize_points(&dst_pts);

    // Build the 2n x 9 DLT matrix A such that A * h = 0, h = vec(H^T).
    let mut a = nalgebra::DMatrix::<f64>::zeros(2 * n, 9);
    for (i, (s, d)) in src_norm.iter().zip(dst_norm.iter()).enumerate() {
        let (u, v) = *s;
        let (x, y) = *d;
        // Row for x-equation
        a.set_row(2 * i, &nalgebra::RowDVector::from_vec(vec![
            -u, -v, -1.0, 0.0, 0.0, 0.0, x * u, x * v, x,
        ]));
        // Row for y-equation
        a.set_row(2 * i + 1, &nalgebra::RowDVector::from_vec(vec![
            0.0, 0.0, 0.0, -u, -v, -1.0, y * u, y * v, y,
        ]));
    }

    let svd_or_eig_matrix = a.transpose() * &a; // 9x9, always full rank profile regardless of point count
    let eig = SymmetricEigen::new(svd_or_eig_matrix);
    let min_idx = eig
        .eigenvalues
        .iter()
        .enumerate()
        .min_by(|(_, x), (_, y)| x.partial_cmp(y).unwrap())
        .map(|(i, _)| i)
        .ok_or_else(|| anyhow::anyhow!("empty eigenvalue set"))?;
    let h_vec = eig.eigenvectors.column(min_idx).clone_owned();

    let h_norm = Matrix3::new(
        h_vec[0], h_vec[1], h_vec[2],
        h_vec[3], h_vec[4], h_vec[5],
        h_vec[6], h_vec[7], h_vec[8],
    );

    let t_dst_inv = t_dst
        .try_inverse()
        .ok_or_else(|| anyhow::anyhow!("normalization transform was singular"))?;

    let h = t_dst_inv * h_norm * t_src;
    // Normalize scale so H[2][2] == 1 where possible (purely cosmetic).
    let h = if h[(2, 2)].abs() > 1e-12 { h / h[(2, 2)] } else { h };

    let mut reproj_errors = Vec::with_capacity(n);
    for c in correspondences {
        let (u, v) = c.pixel;
        let p = h * nalgebra::Vector3::new(u, v, 1.0);
        let (px, py) = (p.x / p.z, p.y / p.z);
        let err = (((px - c.world.0).powi(2) + (py - c.world.1).powi(2)) as f64).sqrt();
        reproj_errors.push(err);
    }

    let rms_error = (reproj_errors.iter().map(|e| e * e).sum::<f64>() / n as f64).sqrt();
    let mean_error = reproj_errors.iter().sum::<f64>() / n as f64;
    let max_error = reproj_errors.iter().cloned().fold(0.0, f64::max);

    let h_world_to_img = h.try_inverse();

    Ok(HomographyResult {
        h_img_to_world: h,
        h_world_to_img,
        reproj_errors,
        rms_error,
        mean_error,
        max_error,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_exact_homography_from_4_points() {
        let correspondences = vec![
            Correspondence { pixel: (150.0, 480.0), world: (-5.0, -3.0) },
            Correspondence { pixel: (650.0, 480.0), world: (5.0, -3.0) },
            Correspondence { pixel: (720.0, 120.0), world: (5.0, 3.0) },
            Correspondence { pixel: (80.0, 120.0), world: (-5.0, 3.0) },
        ];
        let result = solve_homography(&correspondences).expect("solve failed");
        eprintln!("H = {}", result.h_img_to_world);
        eprintln!("errors = {:?}", result.reproj_errors);
        assert!(result.rms_error < 1e-6, "RMS error too high: {}", result.rms_error);
    }

    #[test]
    fn recovers_exact_homography_from_8_points() {
        // Same ground-plane rectangle as above, but with 8 correspondences
        // (over-determined) generated from a *known* homography rather than
        // hand-picked pixels, so we can check both near-zero residual AND
        // that clicking extra points doesn't destabilize the fit.
        let h_true = Matrix3::new(
            0.0125, 0.0006, -6.25,
            0.0002, -0.0128, 7.68,
            0.00001, 0.00002, 1.0,
        );
        let pixels = [
            (150.0, 480.0), (650.0, 480.0), (720.0, 120.0), (80.0, 120.0),
            (400.0, 500.0), (400.0, 100.0), (100.0, 300.0), (700.0, 300.0),
        ];
        let correspondences: Vec<Correspondence> = pixels
            .iter()
            .map(|&(u, v)| {
                let p = h_true * nalgebra::Vector3::new(u, v, 1.0);
                Correspondence { pixel: (u, v), world: (p.x / p.z, p.y / p.z) }
            })
            .collect();

        let result = solve_homography(&correspondences).expect("solve failed");
        eprintln!("errors = {:?}", result.reproj_errors);
        assert!(result.rms_error < 1e-6, "RMS error too high: {}", result.rms_error);
    }
}
