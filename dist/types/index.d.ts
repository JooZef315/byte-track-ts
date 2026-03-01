export declare enum TrackState {
    New = 0,
    Tracked = 1,
    Lost = 2,
    Removed = 3
}
export interface Detection {
    xywh: number[][];
    conf: number[];
    cls: number[];
    xywhr?: number[][];
}
export interface TrackerArgs {
    track_high_thresh: number;
    track_low_thresh: number;
    new_track_thresh: number;
    track_buffer: number;
    match_thresh: number;
    fuse_score: boolean;
}
export type TrackerConfig = TrackerArgs;
export type BBox = [number, number, number, number];
export type StateVector = number[];
export type CovarianceMatrix = number[][];
export declare const DEFAULT_ARGS: TrackerArgs;
//# sourceMappingURL=index.d.ts.map