/**
 * Perform linear assignment using the Hungarian algorithm.
 * Equivalent to Python's scipy.optimize.linear_sum_assignment approach
 * used in the ultralytics tracker.
 */
export declare function linear_assignment(cost_matrix: number[][], thresh: number): {
    matches: number[][];
    unmatched_a: number[];
    unmatched_b: number[];
};
export declare function iou_distance(atracks: any[], btracks: any[]): number[][];
export declare function fuse_score(cost_matrix: number[][], detections: any[]): number[][];
//# sourceMappingURL=matching.d.ts.map