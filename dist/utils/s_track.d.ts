import { BaseTrack } from "../basetrack.js";
import { KalmanFilterXYAH } from "./kalman_filter.js";
export declare class STrack extends BaseTrack {
    static shared_kalman: KalmanFilterXYAH;
    _tlwh: number[];
    kalman_filter: KalmanFilterXYAH | null;
    mean: number[] | null;
    covariance: number[][] | null;
    cls: number;
    idx: number;
    angle: number | null;
    tracklet_len: number;
    constructor(xywh: number[], score: number, cls: number);
    predict(): void;
    static multi_predict(stracks: STrack[]): void;
    static multi_gmc(stracks: STrack[], H?: number[][]): void;
    activate(kalman_filter: KalmanFilterXYAH, frame_id: number): void;
    re_activate(new_track: STrack, frame_id: number, new_id?: boolean): void;
    update(new_track: STrack, frame_id: number): void;
    convert_coords(tlwh: number[]): number[];
    get tlwh(): number[];
    get xyxy(): number[];
    static tlwh_to_xyah(tlwh: number[]): number[];
    get xywh(): number[];
    get xywha(): number[];
    get result(): number[];
}
//# sourceMappingURL=s_track.d.ts.map