import { TrackState } from "./types/index.js";
export class BaseTrack {
    constructor() {
        this.track_id = 0;
        this.is_activated = false;
        this.state = TrackState.New;
        this.history = {}; // OrderedDict equivalent or just object
        this.features = [];
        this.curr_feature = null;
        this.score = 0;
        this.start_frame = 0;
        this.frame_id = 0;
        this.time_since_update = 0;
        this.location = [Infinity, Infinity];
    }
    get end_frame() {
        return this.frame_id;
    }
    static next_id() {
        BaseTrack._count += 1;
        return BaseTrack._count;
    }
    mark_lost() {
        this.state = TrackState.Lost;
    }
    mark_removed() {
        this.state = TrackState.Removed;
    }
    static reset_id() {
        BaseTrack._count = 0;
    }
}
BaseTrack._count = 0;
