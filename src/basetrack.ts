import { TrackState } from "./types/index.js";

export abstract class BaseTrack {
  static _count: number = 0;

  track_id: number = 0;
  is_activated: boolean = false;
  state: TrackState = TrackState.New;
  history: any = {}; // OrderedDict equivalent or just object
  features: any[] = [];
  curr_feature: any = null;
  score: number = 0;
  start_frame: number = 0;
  frame_id: number = 0;
  time_since_update: number = 0;
  location: number[] = [Infinity, Infinity];

  // Abstract methods in Python, defined here as placeholders or abstract
  abstract activate(...args: any[]): void;
  abstract predict(): void;
  abstract update(...args: any[]): void;

  get end_frame(): number {
    return this.frame_id;
  }

  static next_id(): number {
    BaseTrack._count += 1;
    return BaseTrack._count;
  }

  mark_lost(): void {
    this.state = TrackState.Lost;
  }

  mark_removed(): void {
    this.state = TrackState.Removed;
  }

  static reset_id(): void {
    BaseTrack._count = 0;
  }
}
