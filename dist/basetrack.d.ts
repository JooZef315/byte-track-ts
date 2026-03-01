import { TrackState } from "./types/index.js";
export declare abstract class BaseTrack {
    static _count: number;
    track_id: number;
    is_activated: boolean;
    state: TrackState;
    history: any;
    features: any[];
    curr_feature: any;
    score: number;
    start_frame: number;
    frame_id: number;
    time_since_update: number;
    location: number[];
    abstract activate(...args: any[]): void;
    abstract predict(): void;
    abstract update(...args: any[]): void;
    get end_frame(): number;
    static next_id(): number;
    mark_lost(): void;
    mark_removed(): void;
    static reset_id(): void;
}
//# sourceMappingURL=basetrack.d.ts.map