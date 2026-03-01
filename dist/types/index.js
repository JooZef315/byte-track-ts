export var TrackState;
(function (TrackState) {
    TrackState[TrackState["New"] = 0] = "New";
    TrackState[TrackState["Tracked"] = 1] = "Tracked";
    TrackState[TrackState["Lost"] = 2] = "Lost";
    TrackState[TrackState["Removed"] = 3] = "Removed";
})(TrackState || (TrackState = {}));
export const DEFAULT_ARGS = {
    track_high_thresh: 0.3,
    track_low_thresh: 0.1,
    new_track_thresh: 0.3,
    track_buffer: 60,
    match_thresh: 0.7,
    fuse_score: true,
};
