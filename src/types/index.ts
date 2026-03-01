export enum TrackState {
  New = 0,
  Tracked = 1,
  Lost = 2,
  Removed = 3,
}

export interface Detection {
  xywh: number[][]; // [x, y, w, h]
  conf: number[];
  cls: number[];
  xywhr?: number[][]; // Oriented boxes
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

export type BBox = [number, number, number, number]; // x, y, w, h (or similar)
export type StateVector = number[]; // 8-dim vector
export type CovarianceMatrix = number[][]; // 8x8 matrix

export const DEFAULT_ARGS: TrackerArgs = {
  track_high_thresh: 0.3,
  track_low_thresh: 0.1,
  new_track_thresh: 0.3,
  track_buffer: 60,
  match_thresh: 0.7,
  fuse_score: true,
};
