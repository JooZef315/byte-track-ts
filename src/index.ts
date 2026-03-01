import { BYTETracker } from "./byte_tracker.js";
import { DEFAULT_ARGS, type Detection, type TrackerArgs } from "./types/index.js";

export class Tracker {
  private tracker: BYTETracker;
  constructor(args: TrackerArgs = DEFAULT_ARGS) {
    const merged: TrackerArgs = { ...DEFAULT_ARGS, ...args };
    this.tracker = new BYTETracker(merged);
  }

  public update(detections: Detection, frame: any = null): number[][] {
    return this.tracker.update(detections, frame);
  }

  public reset(): void {
    this.tracker.reset();
  }
}

export type { Detection, TrackerArgs };
