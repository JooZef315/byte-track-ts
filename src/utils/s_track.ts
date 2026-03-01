import { BaseTrack } from "../basetrack.js";
import { TrackState } from "../types/index.js";
import { KalmanFilterXYAH } from "./kalman_filter.js";
import { xywh2ltwh } from "./ops.js";
import { Matrix } from "ml-matrix";

export class STrack extends BaseTrack {
  static shared_kalman = new KalmanFilterXYAH();

  _tlwh: number[];
  kalman_filter: KalmanFilterXYAH | null = null;
  mean: number[] | null = null; // 8-dim
  covariance: number[][] | null = null; // 8x8

  // attributes
  cls: number;
  idx: number; // detection index or identifier
  angle: number | null = null;
  tracklet_len: number = 0;

  constructor(xywh: number[], score: number, cls: number) {
    super();
    this._tlwh = xywh2ltwh([xywh.slice(0, 4)])[0]!;

    this.score = score;
    this.cls = cls;
    this.idx = xywh[xywh.length - 1]!;
    this.angle =
      xywh.length === 6 ? (xywh[4] !== undefined ? xywh[4] : null) : null;
  }

  predict(): void {
    if (!this.mean || !this.covariance) return;

    const mean_state = [...this.mean];
    if (this.state !== TrackState.Tracked) {
      mean_state[7] = 0;
    }
    const res = this.kalman_filter!.predict(mean_state, this.covariance);
    this.mean = res.mean;
    this.covariance = res.covariance;
  }

  static multi_predict(stracks: STrack[]): void {
    if (stracks.length <= 0) return;

    const multi_mean: number[][] = [];
    const multi_covariance: number[][][] = [];

    for (const track of stracks) {
      if (!track.mean || !track.covariance) continue;
      // track.mean and track.covariance are safe here due to check
      const m = [...track.mean];
      if (track.state !== TrackState.Tracked) {
        m[7] = 0;
      }
      multi_mean.push(m);
      multi_covariance.push(track.covariance);
    }

    if (multi_mean.length === 0) return;

    const res = STrack.shared_kalman.multi_predict(
      multi_mean,
      multi_covariance,
    );

    let idx = 0;
    for (let i = 0; i < stracks.length; i++) {
      const track = stracks[i]!;
      if (!track.mean || !track.covariance) continue;
      track.mean = res.mean[idx]!;
      track.covariance = res.covariance[idx]!;
      idx++;
    }
  }

  static multi_gmc(
    stracks: STrack[],
    H: number[][] = [
      [1, 0, 0],
      [0, 1, 0],
    ],
  ): void {
    if (stracks.length > 0) {
      // H is 2x3 usually.
      const R = new Matrix([
        [H[0]![0]!, H[0]![1]!],
        [H[1]![0]!, H[1]![1]!],
      ]);

      // Implementation of Kronecker product eye(4) x R:
      const R8x8 = Matrix.zeros(8, 8);
      for (let k = 0; k < 4; k++) {
        R8x8.setSubMatrix(R, k * 2, k * 2);
      }

      const t = [H[0]![2]!, H[1]![2]!];

      for (const track of stracks) {
        if (!track.mean || !track.covariance) continue;

        // mean = R8x8.dot(mean)
        const meanVec = Matrix.columnVector(track.mean);
        let mean = R8x8.mmul(meanVec);

        // mean[:2] += t
        mean.set(0, 0, mean.get(0, 0) + t[0]!);
        mean.set(1, 0, mean.get(1, 0) + t[1]!);

        // cov = R8x8.dot(cov).dot(R8x8.T)
        const covMat = new Matrix(track.covariance);
        const cov = R8x8.mmul(covMat).mmul(R8x8.transpose());

        track.mean = mean.to1DArray();
        track.covariance = cov.to2DArray();
      }
    }
  }

  activate(kalman_filter: KalmanFilterXYAH, frame_id: number): void {
    this.kalman_filter = kalman_filter;
    this.track_id = BaseTrack.next_id();

    const res = this.kalman_filter.initiate(this.convert_coords(this._tlwh));
    this.mean = res.mean;
    this.covariance = res.covariance;

    this.tracklet_len = 0;
    this.state = TrackState.Tracked;
    if (frame_id === 1) {
      this.is_activated = true;
    }
    this.frame_id = frame_id;
    this.start_frame = frame_id;
  }

  re_activate(
    new_track: STrack,
    frame_id: number,
    new_id: boolean = false,
  ): void {
    // assume new_track.tlwh is valid
    const res = this.kalman_filter!.update(
      this.mean!,
      this.covariance!,
      this.convert_coords(new_track.tlwh),
    );
    this.mean = res.mean;
    this.covariance = res.covariance;

    this.tracklet_len = 0;
    this.state = TrackState.Tracked;
    this.is_activated = true;
    this.frame_id = frame_id;
    if (new_id) {
      this.track_id = BaseTrack.next_id();
    }
    this.score = new_track.score;
    this.cls = new_track.cls;
    this.angle = new_track.angle;
    this.idx = new_track.idx;
  }

  update(new_track: STrack, frame_id: number): void {
    this.frame_id = frame_id;
    this.tracklet_len += 1;

    const new_tlwh = new_track.tlwh;
    const res = this.kalman_filter!.update(
      this.mean!,
      this.covariance!,
      this.convert_coords(new_tlwh),
    );
    this.mean = res.mean;
    this.covariance = res.covariance;

    this.state = TrackState.Tracked;
    this.is_activated = true;
    this.score = new_track.score;
    this.cls = new_track.cls;
    this.angle = new_track.angle;
    this.idx = new_track.idx;
  }

  convert_coords(tlwh: number[]): number[] {
    return STrack.tlwh_to_xyah(tlwh);
  }

  get tlwh(): number[] {
    if (!this.mean) return [...this._tlwh];
    const ret = this.mean.slice(0, 4);
    ret[2] = ret[2]! * ret[3]!;
    ret[0] = ret[0]! - ret[2] / 2;
    ret[1] = ret[1]! - ret[3]! / 2;
    return ret;
  }

  get xyxy(): number[] {
    const ret = this.tlwh;
    ret[2] = ret[2]! + ret[0]!; // x2 = x1 + w
    ret[3] = ret[3]! + ret[1]!; // y2 = y1 + h
    return ret;
  }

  static tlwh_to_xyah(tlwh: number[]): number[] {
    const ret = [...tlwh];
    ret[0]! += ret[2]! / 2;
    ret[1]! += ret[3]! / 2;
    ret[2]! /= ret[3]!;
    return ret;
  }

  get xywh(): number[] {
    const ret = this.tlwh; // copy
    ret[0]! += ret[2]! / 2;
    ret[1]! += ret[3]! / 2;
    return ret;
  }

  get xywha(): number[] {
    if (this.angle === null) {
      return this.xywh;
    }
    return [...this.xywh, this.angle];
  }

  get result(): number[] {
    const coords = this.angle === null ? this.xyxy : this.xywha;
    return [...coords, this.track_id, this.score, this.cls, this.idx];
  }
}
