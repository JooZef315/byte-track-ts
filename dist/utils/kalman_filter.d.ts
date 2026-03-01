import { Matrix } from "ml-matrix";
export declare class KalmanFilterXYAH {
    _motion_mat: Matrix;
    _update_mat: Matrix;
    _std_weight_position: number;
    _std_weight_velocity: number;
    constructor();
    initiate(measurement: number[]): {
        mean: number[];
        covariance: number[][];
    };
    predict(mean: number[], covariance: number[][]): {
        mean: number[];
        covariance: number[][];
    };
    project(mean: number[], covariance: number[][]): {
        mean: number[];
        covariance: number[][];
    };
    multi_predict(mean: number[][], covariance: number[][][]): {
        mean: number[][];
        covariance: number[][][];
    };
    update(mean: number[], covariance: number[][], measurement: number[]): {
        mean: number[];
        covariance: number[][];
    };
    gating_distance(mean: number[], covariance: number[][], measurements: number[][], only_position?: boolean, metric?: string): number[];
}
//# sourceMappingURL=kalman_filter.d.ts.map