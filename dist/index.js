import { BYTETracker } from "./byte_tracker.js";
import { DEFAULT_ARGS } from "./types/index.js";
export class Tracker {
    constructor(args = DEFAULT_ARGS) {
        const merged = { ...DEFAULT_ARGS, ...args };
        this.tracker = new BYTETracker(merged);
    }
    update(detections, frame = null) {
        return this.tracker.update(detections, frame);
    }
    reset() {
        this.tracker.reset();
    }
}
