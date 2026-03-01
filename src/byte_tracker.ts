import { BaseTrack } from "./basetrack.js";
import { TrackState } from "./types/index.js";
import type { TrackerConfig, Detection } from "./types/index.js";
import { KalmanFilterXYAH } from "./utils/kalman_filter.js";
import * as matching from "./utils/matching.js";
import { STrack } from "./utils/s_track.js";

export class BYTETracker {
  tracked_stracks: STrack[] = [];
  lost_stracks: STrack[] = [];
  removed_stracks: STrack[] = [];
  frame_id: number = 0;
  args: TrackerConfig;
  max_time_lost: number;
  kalman_filter: KalmanFilterXYAH;

  constructor(args: TrackerConfig) {
    this.args = args;
    this.max_time_lost = args.track_buffer;
    this.kalman_filter = this.get_kalmanfilter();
    this.reset_id();
  }

  get_kalmanfilter(): KalmanFilterXYAH {
    return new KalmanFilterXYAH();
  }

  reset_id(): void {
    BaseTrack.reset_id();
  }

  reset(): void {
    this.tracked_stracks = [];
    this.lost_stracks = [];
    this.removed_stracks = [];
    this.frame_id = 0;
    this.kalman_filter = this.get_kalmanfilter();
    this.reset_id();
  }

  init_track(results: Detection | null, feats: any = null): STrack[] {
    if (!results || !results.conf || results.conf.length === 0) return [];

    const bboxes = results.xywhr ? results.xywhr : results.xywh;

    const ret: STrack[] = [];
    for (let i = 0; i < bboxes.length; i++) {
      // detection index i is appended to box
      const box = bboxes[i]!;
      const boxWithIdx = [...box, i];
      const conf = results.conf[i]!;
      const cls = results.cls[i]!;
      ret.push(new STrack(boxWithIdx, conf, cls));
    }
    return ret;
  }

  get_dists(tracks: STrack[], detections: STrack[]): number[][] {
    let dists = matching.iou_distance(tracks, detections);
    if (this.args.fuse_score) {
      dists = matching.fuse_score(dists, detections);
    }
    return dists;
  }

  multi_predict(tracks: STrack[]): void {
    STrack.multi_predict(tracks);
  }

  // Helpers
  static joint_stracks(tlista: STrack[], tlistb: STrack[]): STrack[] {
    const exists = new Set<number>();
    const res: STrack[] = [];

    for (const t of tlista) {
      exists.add(t.track_id);
      res.push(t);
    }
    for (const t of tlistb) {
      if (!exists.has(t.track_id)) {
        exists.add(t.track_id);
        res.push(t);
      }
    }
    return res;
  }

  static sub_stracks(tlista: STrack[], tlistb: STrack[]): STrack[] {
    const track_ids_b = new Set(tlistb.map((t) => t.track_id));
    return tlista.filter((t) => !track_ids_b.has(t.track_id));
  }

  static remove_duplicate_stracks(
    stracksa: STrack[],
    stracksb: STrack[],
  ): [STrack[], STrack[]] {
    const pdist = matching.iou_distance(stracksa, stracksb);
    const pairs: [number, number][] = [];

    for (let i = 0; i < pdist.length; i++) {
      const row = pdist[i];
      if (!row) continue;
      for (let j = 0; j < row.length; j++) {
        const val = row[j];
        if (val === undefined) continue;
        if (val < 0.15) {
          pairs.push([i, j]);
        }
      }
    }

    const dupa = new Set<number>();
    const dupb = new Set<number>();

    for (const [p, q] of pairs) {
      if (p === undefined || q === undefined) continue;
      const timep = stracksa[p]!.frame_id - stracksa[p]!.start_frame;
      const timeq = stracksb[q]!.frame_id - stracksb[q]!.start_frame;
      if (timep > timeq) {
        dupb.add(q);
      } else {
        dupa.add(p);
      }
    }

    const resa = stracksa.filter((_, i) => !dupa.has(i));
    const resb = stracksb.filter((_, i) => !dupb.has(i));
    return [resa, resb];
  }

  update(results: Detection, img: any = null): number[][] {
    this.frame_id += 1;
    const activated_stracks: STrack[] = [];
    const refind_stracks: STrack[] = [];
    const lost_stracks: STrack[] = [];
    const removed_stracks: STrack[] = [];

    const scores = results.conf;

    const remain_inds: number[] = [];
    const inds_low: number[] = [];
    const inds_high: number[] = [];

    for (let i = 0; i < scores.length; i++) {
      if (scores[i]! >= this.args.track_high_thresh) remain_inds.push(i);
      if (scores[i]! > this.args.track_low_thresh) inds_low.push(i);
      if (scores[i]! < this.args.track_high_thresh) inds_high.push(i);
    }

    const inds_high_set = new Set(inds_high);
    const inds_second = inds_low.filter((i) => inds_high_set.has(i));

    const results_remain = this.slice_results(results, remain_inds);
    const results_second = this.slice_results(results, inds_second);

    const detections = this.init_track(results_remain);

    const unconfirmed: STrack[] = [];
    const tracked_stracks: STrack[] = [];

    for (const track of this.tracked_stracks) {
      if (!track.is_activated) unconfirmed.push(track);
      else tracked_stracks.push(track);
    }

    const strack_pool = BYTETracker.joint_stracks(
      tracked_stracks,
      this.lost_stracks,
    );
    this.multi_predict(strack_pool);

    // Step 2
    let matches: number[][], u_track: number[], u_detection: number[];
    if (strack_pool.length === 0) {
      matches = [];
      u_track = [];
      u_detection = detections.map((_, i) => i);
    } else {
      const dists = this.get_dists(strack_pool, detections);
      const res = matching.linear_assignment(dists, this.args.match_thresh);
      matches = res.matches;
      u_track = res.unmatched_a;
      u_detection = res.unmatched_b;
    }

    for (const [itracked, idet] of matches) {
      if (itracked === undefined || idet === undefined) continue;
      const track = strack_pool[itracked]!;
      const det = detections[idet]!;
      if (!track || !det) continue; // Safety guard for undefined access
      if (track.state === TrackState.Tracked) {
        track.update(det, this.frame_id);
        activated_stracks.push(track);
      } else {
        track.re_activate(det, this.frame_id, false);
        refind_stracks.push(track);
      }
    }

    const detections_second = this.init_track(results_second);
    const r_tracked_stracks = u_track
      .map((i) => strack_pool[i]!)
      .filter((t) => t.state === TrackState.Tracked);

    let matches2: number[][], u_track2: number[];
    if (r_tracked_stracks.length === 0) {
      matches2 = [];
      u_track2 = [];
    } else {
      const dists2 = matching.iou_distance(
        r_tracked_stracks,
        detections_second,
      );
      const res2 = matching.linear_assignment(dists2, 0.5);
      matches2 = res2.matches;
      u_track2 = res2.unmatched_a;
    }

    for (const [itracked, idet] of matches2) {
      if (itracked === undefined || idet === undefined) continue;
      const track = r_tracked_stracks[itracked]!;
      const det = detections_second[idet]!;
      if (!track || !det) continue;
      if (track.state === TrackState.Tracked) {
        track.update(det, this.frame_id);
        activated_stracks.push(track);
      } else {
        track.re_activate(det, this.frame_id, false);
        refind_stracks.push(track);
      }
    }

    for (const it of u_track2) {
      const track = r_tracked_stracks[it]!;
      if (track.state !== TrackState.Lost) {
        track.mark_lost();
        lost_stracks.push(track);
      }
    }

    // Unconfirmed
    const detections_unconfirmed = u_detection.map((i) => detections[i]!);
    let matches3: number[][], u_unconfirmed: number[], u_detection3: number[];

    if (unconfirmed.length === 0) {
      matches3 = [];
      u_unconfirmed = [];
      u_detection3 = detections_unconfirmed.map((_, i) => i);
    } else {
      const dists3 = this.get_dists(unconfirmed, detections_unconfirmed);
      const res3 = matching.linear_assignment(dists3, 0.7);
      matches3 = res3.matches;
      u_unconfirmed = res3.unmatched_a;
      u_detection3 = res3.unmatched_b;
    }

    for (const [itracked, idet] of matches3) {
      if (itracked === undefined || idet === undefined) continue;
      const track = unconfirmed[itracked];
      const det = detections_unconfirmed[idet];
      if (!track || !det) continue;
      track.update(det, this.frame_id);
      activated_stracks.push(track);
    }

    for (const it of u_unconfirmed) {
      if (it === undefined) continue;
      const track = unconfirmed[it]!;
      if (!track) continue; // strict guard
      track.mark_removed();
      removed_stracks.push(track);
    }

    // Step 4: Init new
    for (const inew of u_detection3) {
      const track = detections_unconfirmed[inew]!;
      if (track.score < this.args.new_track_thresh) continue;
      track.activate(this.kalman_filter, this.frame_id);
      activated_stracks.push(track);
    }

    // Step 5: Update state
    for (const track of this.lost_stracks) {
      if (this.frame_id - track.end_frame > this.max_time_lost) {
        track.mark_removed();
        removed_stracks.push(track);
      }
    }

    this.tracked_stracks = this.tracked_stracks.filter(
      (t) => t.state === TrackState.Tracked,
    );
    this.tracked_stracks = BYTETracker.joint_stracks(
      this.tracked_stracks,
      activated_stracks,
    );
    this.tracked_stracks = BYTETracker.joint_stracks(
      this.tracked_stracks,
      refind_stracks,
    );
    this.lost_stracks = BYTETracker.sub_stracks(
      this.lost_stracks,
      this.tracked_stracks,
    );
    this.lost_stracks.push(...lost_stracks);
    this.lost_stracks = BYTETracker.sub_stracks(
      this.lost_stracks,
      this.removed_stracks,
    );
    const [filtered_tracked, filtered_lost] =
      BYTETracker.remove_duplicate_stracks(
        this.tracked_stracks,
        this.lost_stracks,
      );
    this.tracked_stracks = filtered_tracked;
    this.lost_stracks = filtered_lost;

    this.removed_stracks.push(...removed_stracks);
    if (this.removed_stracks.length > 1000) {
      this.removed_stracks = this.removed_stracks.slice(
        this.removed_stracks.length - 1000,
      );
    }

    return this.tracked_stracks
      .filter((x) => x.is_activated)
      .map((x) => x.result);
  }

  slice_results(results: Detection, indices: number[]): Detection {
    // Helper to slice detection object
    const new_xywh = indices.map((i) => results.xywh[i]!);
    const new_conf = indices.map((i) => results.conf[i]!);
    const new_cls = indices.map((i) => results.cls[i]!);
    const ret: Detection = { xywh: new_xywh, conf: new_conf, cls: new_cls };
    if (results.xywhr) {
      // Capture xywhr in const to satisfy TS check inside callback
      const xywhr = results.xywhr;
      ret.xywhr = indices.map((i) => xywhr[i]!);
    }
    return ret;
  }
}
