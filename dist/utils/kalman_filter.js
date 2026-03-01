import { Matrix, CholeskyDecomposition, solve } from "ml-matrix";
export class KalmanFilterXYAH {
    constructor() {
        const ndim = 4;
        const dt = 1.0;
        // Create Kalman filter model matrices
        this._motion_mat = Matrix.eye(2 * ndim, 2 * ndim);
        for (let i = 0; i < ndim; i++) {
            this._motion_mat.set(i, ndim + i, dt);
        }
        this._update_mat = Matrix.eye(ndim, 2 * ndim);
        // Motion and observation uncertainty are chosen relative to the current state estimate
        this._std_weight_position = 1.0 / 20;
        this._std_weight_velocity = 1.0 / 160;
    }
    initiate(measurement) {
        if (measurement.length < 4)
            throw new Error("Measurement must have at least 4 elements");
        const mean_pos = measurement;
        const mean_vel = new Array(4).fill(0);
        const mean = [...mean_pos, ...mean_vel];
        const std = [
            2 * this._std_weight_position * measurement[3],
            2 * this._std_weight_position * measurement[3],
            1e-2,
            2 * this._std_weight_position * measurement[3],
            10 * this._std_weight_velocity * measurement[3],
            10 * this._std_weight_velocity * measurement[3],
            1e-5,
            10 * this._std_weight_velocity * measurement[3],
        ];
        const covariance = Matrix.diag(std.map((s) => s * s));
        return { mean, covariance: covariance.to2DArray() };
    }
    predict(mean, covariance) {
        const std_pos = [
            this._std_weight_position * mean[3],
            this._std_weight_position * mean[3],
            1e-2,
            this._std_weight_position * mean[3],
        ];
        const std_vel = [
            this._std_weight_velocity * mean[3],
            this._std_weight_velocity * mean[3],
            1e-5,
            this._std_weight_velocity * mean[3],
        ];
        const motion_cov = Matrix.diag([...std_pos, ...std_vel].map((s) => s * s));
        const meanMat = Matrix.rowVector(mean);
        const covMat = new Matrix(covariance);
        // mean = np.dot(mean, self._motion_mat.T)
        // Note: Python's np.dot(1D, 2D) treats 1D as row vector if on left.
        // Matrix.mmul takes matrices.
        // mean (1x8) * motion_mat.T (8x8) -> 1x8
        const pred_mean = meanMat.mmul(this._motion_mat.transpose());
        // covariance = np.linalg.multi_dot((self._motion_mat, covariance, self._motion_mat.T)) + motion_cov
        // motion_mat * cov * motion_mat.T
        let pred_cov = this._motion_mat
            .mmul(covMat)
            .mmul(this._motion_mat.transpose());
        pred_cov = pred_cov.add(motion_cov);
        return {
            mean: pred_mean.to1DArray(),
            covariance: pred_cov.to2DArray(),
        };
    }
    project(mean, covariance) {
        const std = [
            this._std_weight_position * mean[3],
            this._std_weight_position * mean[3],
            1e-1,
            this._std_weight_position * mean[3],
        ];
        const innovation_cov = Matrix.diag(std.map((s) => s * s));
        const meanMat = Matrix.columnVector(mean); // 8x1
        const covMat = new Matrix(covariance); // 8x8
        // mean = np.dot(self._update_mat, mean)
        // update_mat (4x8) * mean (8x1) -> 4x1
        const proj_mean = this._update_mat.mmul(meanMat);
        // covariance = np.linalg.multi_dot((self._update_mat, covariance, self._update_mat.T))
        let proj_cov = this._update_mat
            .mmul(covMat)
            .mmul(this._update_mat.transpose());
        return {
            mean: proj_mean.to1DArray(),
            covariance: proj_cov.add(innovation_cov).to2DArray(),
        };
    }
    multi_predict(mean, covariance) {
        // Handle vectorized prediction by iterating (simpler than tensor lib)
        const N = mean.length;
        const predMeans = [];
        const predCovs = [];
        for (let i = 0; i < N; i++) {
            const res = this.predict(mean[i], covariance[i]);
            predMeans.push(res.mean);
            predCovs.push(res.covariance);
        }
        return { mean: predMeans, covariance: predCovs };
    }
    update(mean, covariance, measurement) {
        const proj = this.project(mean, covariance);
        const proj_mean = new Matrix([proj.mean]); // 1x4 (row)? No, project returns 1D array.
        // In python project returns 1D array.
        // Here we need vectors.
        // In Python update:
        // innovation = measurement - projected_mean
        // new_mean = mean + np.dot(innovation, kalman_gain.T)
        const proj_cov = new Matrix(proj.covariance); // S (4x4)
        const cov_mat = new Matrix(covariance); // P (8x8)
        // P * H.T
        const P_HT = cov_mat.mmul(this._update_mat.transpose()); // 8x4
        // K = P * H.T * S^-1
        // Solve S * K.T = (P * H.T).T
        // Transpose logic: (P HT).T is 4x8.
        // S * X = B -> X = S^-1 B.
        // We want K = X.T.
        // Using solve(A, B) -> X such that AX = B.
        // S * K^T = (P * H^T)^T ? No.
        // K = P H^T S^-1.
        // K^T = S^-T H P^T = S^-1 H P^T. (Since S is symmetric)
        // Solve S * Z = H P^T. Then Z = K^T.
        const B = this._update_mat.mmul(cov_mat.transpose()); // 4x8. (H P^T) which is transpose of P H^T.
        // solve(S, B) returns Z (4x8)
        const K_T = solve(proj_cov, B);
        const K = K_T.transpose(); // 8x4
        // innovation: measurement - projected_mean
        // measurement: 4, proj_mean: 4.
        const msg = Matrix.rowVector(measurement);
        const pm = Matrix.rowVector(proj.mean);
        const innovation = msg.sub(pm); // 1x4
        // new_mean = mean + np.dot(innovation, kalman_gain.T)
        // 1x8 + (1x4 * 4x8) -> 1x8
        const meanVec = Matrix.rowVector(mean);
        const correction = innovation.mmul(K_T);
        const new_mean = meanVec.add(correction);
        // new_covariance = covariance - K * S * K.T
        // Or cov - K * proj_cov * K.T
        const term = K.mmul(proj_cov).mmul(K_T);
        const new_cov = cov_mat.sub(term);
        return {
            mean: new_mean.to1DArray(),
            covariance: new_cov.to2DArray(),
        };
    }
    gating_distance(mean, covariance, measurements, only_position = false, metric = "maha") {
        let proj = this.project(mean, covariance);
        let m_mean = proj.mean;
        let m_cov = proj.covariance;
        let meas = measurements;
        if (only_position) {
            m_mean = m_mean.slice(0, 2);
            m_cov = new Matrix(m_cov).subMatrix(0, 1, 0, 1).to2DArray(); // top-left 2x2
            meas = meas.map((m) => m.slice(0, 2));
        }
        const d = meas.map((m) => {
            // m_mean is number[]
            return m.map((val, i) => val - m_mean[i]);
        }); // Nx4
        if (metric === "gaussian") {
            return d.map((row) => row.reduce((sum, val) => sum + val * val, 0));
        }
        else if (metric === "maha") {
            const covMat = new Matrix(m_cov);
            // Solve L * Y = D_transposed for Y, where cov = L * L.T?
            // Python: cholesky_factor = np.linalg.cholesky(covariance)
            // z = scipy.linalg.solve_triangular(cholesky_factor, d.T, lower=True)
            // return sum(z*z)
            // Should be roughly `d * cov^-1 * d.T`
            // Let's rely on inversion if stable, or cholesky if ml-matrix supports it easily.
            // ml-matrix has CholeskyDecomposition.
            try {
                const chol = new CholeskyDecomposition(covMat);
                // Lower triangular L.
                const L = chol.lowerTriangularMatrix;
                // Solve L * Z = D.T
                // Z = L^-1 * D.T
                const D_T = new Matrix(d).transpose(); // 4xN
                const Z = solve(L, D_T); // 4xN
                // sum columns of Z*Z
                const result = new Array(d.length).fill(0);
                for (let i = 0; i < d.length; i++) {
                    // for each measurement
                    for (let j = 0; j < Z.rows; j++) {
                        const val = Z.get(j, i);
                        result[i] += val * val;
                    }
                }
                return result;
            }
            catch (e) {
                // Fallback if not positive definite or error
                return new Array(d.length).fill(Infinity);
            }
        }
        else {
            throw new Error("Invalid distance metric");
        }
    }
}
