import { type Detection, type TrackerArgs } from "./types/index.js";
export declare class Tracker {
    private tracker;
    constructor(args?: TrackerArgs);
    update(detections: Detection, frame?: any): number[][];
    reset(): void;
}
export type { Detection, TrackerArgs };
//# sourceMappingURL=index.d.ts.map