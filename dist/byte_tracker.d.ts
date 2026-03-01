import type { TrackerConfig, Detection } from "./types/index.js";
import { KalmanFilterXYAH } from "./utils/kalman_filter.js";
import { STrack } from "./utils/s_track.js";
export declare class BYTETracker {
    tracked_stracks: STrack[];
    lost_stracks: STrack[];
    removed_stracks: STrack[];
    frame_id: number;
    args: TrackerConfig;
    max_time_lost: number;
    kalman_filter: KalmanFilterXYAH;
    constructor(args: TrackerConfig);
    get_kalmanfilter(): KalmanFilterXYAH;
    reset_id(): void;
    reset(): void;
    init_track(results: Detection | null, feats?: any): STrack[];
    get_dists(tracks: STrack[], detections: STrack[]): number[][];
    multi_predict(tracks: STrack[]): void;
    static joint_stracks(tlista: STrack[], tlistb: STrack[]): STrack[];
    static sub_stracks(tlista: STrack[], tlistb: STrack[]): STrack[];
    static remove_duplicate_stracks(stracksa: STrack[], stracksb: STrack[]): [STrack[], STrack[]];
    update(results: Detection, img?: any): number[][];
    slice_results(results: Detection, indices: number[]): Detection;
}
//# sourceMappingURL=byte_tracker.d.ts.map