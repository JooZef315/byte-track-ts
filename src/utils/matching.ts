/* eslint-disable camelcase */
import { batch_probiou, bbox_ioa } from "./ops.js";
import { linearSumAssignment } from "linear-sum-assignment";

/**
 * Perform linear assignment using the Hungarian algorithm.
 * Equivalent to Python's scipy.optimize.linear_sum_assignment approach
 * used in the ultralytics tracker.
 */
export function linear_assignment(
  cost_matrix: number[][],
  thresh: number,
): { matches: number[][]; unmatched_a: number[]; unmatched_b: number[] } {
  if (
    !cost_matrix ||
    cost_matrix.length === 0 ||
    !cost_matrix[0] ||
    cost_matrix[0].length === 0
  ) {
    const rows = cost_matrix ? cost_matrix.length : 0;
    const cols = rows > 0 && cost_matrix[0] ? cost_matrix[0].length : 0;
    return {
      matches: [],
      unmatched_a: Array.from({ length: rows }, (_, i) => i),
      unmatched_b: Array.from({ length: cols }, (_, i) => i),
    };
  }

  const rows = cost_matrix.length;
  const cols = cost_matrix[0]!.length;

  // Emulate lapjv(cost_limit=thresh) by adding dummy rows/cols with fixed costs.
  const costLimit = thresh;
  let maxCost = 0;
  for (let i = 0; i < rows; i++) {
    const row = cost_matrix[i]!;
    for (let j = 0; j < cols; j++) {
      const v = row[j]!;
      if (Number.isFinite(v) && v > maxCost) maxCost = v;
    }
  }
  const dummyCost = Number.isFinite(costLimit) ? costLimit : maxCost + 1;
  const bigCost = Number.isFinite(costLimit) ? costLimit + 1e-5 : maxCost + 2;

  const n = rows + cols;
  const augmented: number[][] = Array.from({ length: n }, () =>
    Array(n).fill(bigCost),
  );

  // Top-left: real costs, capped at bigCost if above threshold or not finite.
  for (let i = 0; i < rows; i++) {
    for (let j = 0; j < cols; j++) {
      const c = cost_matrix[i]![j]!;
      if (Number.isFinite(c) && c <= costLimit) {
        augmented[i]![j] = c;
      } else {
        augmented[i]![j] = bigCost;
      }
    }
  }

  // Track -> dummy column (one per track)
  for (let i = 0; i < rows; i++) {
    augmented[i]![cols + i] = dummyCost;
  }

  // Dummy row -> detection column (one per detection)
  for (let j = 0; j < cols; j++) {
    augmented[rows + j]![j] = dummyCost;
  }

  // Dummy-to-dummy block with zero cost
  for (let i = rows; i < n; i++) {
    for (let j = cols; j < n; j++) {
      augmented[i]![j] = 0;
    }
  }

  // Use Jonker-Volgenant (linear-sum-assignment) which is closer to lapjv.
  const result = linearSumAssignment(augmented, { maximaze: false });
  const rowAssignments = result.rowAssignments;

  const matches: number[][] = [];
  const matchedRows = new Set<number>();
  const matchedCols = new Set<number>();

  for (let i = 0; i < rows; i++) {
    const j = rowAssignments[i]!;
    if (j !== -1 && j < cols && cost_matrix[i]![j]! <= thresh) {
      matches.push([i, j]);
      matchedRows.add(i);
      matchedCols.add(j);
    }
  }

  const unmatched_a: number[] = [];
  for (let i = 0; i < rows; i++) {
    if (!matchedRows.has(i)) unmatched_a.push(i);
  }

  const unmatched_b: number[] = [];
  for (let j = 0; j < cols; j++) {
    if (!matchedCols.has(j)) unmatched_b.push(j);
  }

  return { matches, unmatched_a, unmatched_b };
}

export function iou_distance(atracks: any[], btracks: any[]): number[][] {
  let atlbrs: number[][] = [];
  let btlbrs: number[][] = [];

  if (atracks.length && btracks.length) {
    const a0 = atracks[0];
    const b0 = btracks[0];
    const arraysProvided = Array.isArray(a0) || Array.isArray(b0);

    if (arraysProvided) {
      atlbrs = atracks as number[][];
      btlbrs = btracks as number[][];
      if (atlbrs[0] && atlbrs[0].length === 5 && btlbrs[0]?.length === 5) {
        const ious: number[][] = batch_probiou(atlbrs, btlbrs);
        return ious.map((row: number[]) =>
          row.map((val: number) => 1 - val),
        );
      }
      const ious: number[][] = bbox_ioa(atlbrs, btlbrs, true);
      return ious.map((row: number[]) => row.map((val: number) => 1 - val));
    }

    const useObb = a0.angle !== undefined && a0.angle !== null;

    if (useObb) {
      atlbrs = atracks.map((t) => t.xywha);
      btlbrs = btracks.map((t) => t.xywha);
      const ious: number[][] = batch_probiou(atlbrs, btlbrs);
      return ious.map((row: number[]) => row.map((val: number) => 1 - val));
    } else {
      atlbrs = atracks.map((t) => t.xyxy);
      btlbrs = btracks.map((t) => t.xyxy);
      const ious: number[][] = bbox_ioa(atlbrs, btlbrs, true);
      return ious.map((row: number[]) => row.map((val: number) => 1 - val));
    }
  }

  const N = atracks.length;
  const M = btracks.length;
  return Array.from({ length: N }, () => Array(M).fill(1));
}

export function fuse_score(
  cost_matrix: number[][],
  detections: any[],
): number[][] {
  if (cost_matrix.length === 0) return cost_matrix;

  const result: number[][] = [];

  for (let i = 0; i < cost_matrix.length; i++) {
    const rowArr: number[] = [];
    const row = cost_matrix[i]!;
    for (let j = 0; j < row.length; j++) {
      const iou_sim = 1 - row[j]!;
      const det = detections[j];
      if (!det) {
        rowArr.push(1.0);
        continue;
      }
      const score = det.score;
      const fuse_sim = iou_sim * score;
      rowArr.push(1 - fuse_sim);
    }
    result.push(rowArr);
  }
  return result;
}
