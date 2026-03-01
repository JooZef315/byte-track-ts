/**
 * Convert bounding box format from [x, y, w, h] to [x1, y1, w, h] where x1, y1 are top-left coordinates.
 * @param x Input bounding box coordinates in xywh format.
 * @returns Bounding box coordinates in xyltwh format.
 */
export function xywh2ltwh(x: number[][]): number[][] {
  const y: number[][] = JSON.parse(JSON.stringify(x));
  for (let i = 0; i < y.length; i++) {
    y[i]![0] = x[i]![0]! - x[i]![2]! / 2; // top left x
    y[i]![1] = x[i]![1]! - x[i]![3]! / 2; // top left y
  }
  return y;
}

/**
 * Calculate the intersection over box2 area given box1 and box2.
 * @param box1 A 2D array of shape (N, 4) representing N bounding boxes in x1y1x2y2 format.
 * @param box2 A 2D array of shape (M, 4) representing M bounding boxes in x1y1x2y2 format.
 * @param iou Calculate the standard IoU if True else return inter_area/box2_area.
 * @param eps A small value to avoid division by zero.
 * @returns A 2D array of shape (N, M) representing the intersection over box2 area.
 */
export function bbox_ioa(
  box1: number[][],
  box2: number[][],
  iou: boolean = false,
  eps: number = 1e-7,
): number[][] {
  const N = box1.length;
  const M = box2.length;
  const ious: number[][] = Array.from({ length: N }, () => Array(M).fill(0));

  for (let i = 0; i < N; i++) {
    const b1_x1 = box1[i]![0];
    const b1_y1 = box1[i]![1];
    const b1_x2 = box1[i]![2];
    const b1_y2 = box1[i]![3];
    const box1_area = (b1_x2! - b1_x1!) * (b1_y2! - b1_y1!);

    for (let j = 0; j < M; j++) {
      const b2_x1 = box2[j]![0]!;
      const b2_y1 = box2[j]![1]!;
      const b2_x2 = box2[j]![2]!;
      const b2_y2 = box2[j]![3]!;
      const box2_area = (b2_x2 - b2_x1) * (b2_y2 - b2_y1);

      const inter_x1 = Math.max(b1_x1!, b2_x1);
      const inter_y1 = Math.max(b1_y1!, b2_y1);
      const inter_x2 = Math.min(b1_x2!, b2_x2);
      const inter_y2 = Math.min(b1_y2!, b2_y2);

      const inter_w = Math.max(0, inter_x2 - inter_x1);
      const inter_h = Math.max(0, inter_y2 - inter_y1);
      const inter_area = inter_w * inter_h;

      let union_area = box2_area;
      if (iou) {
        union_area = box2_area + box1_area - inter_area;
      }

      ious[i]![j] = inter_area / (union_area + eps);
    }
  }
  return ious;
}

function get_covariance_matrix(boxes: number[][]): {
  a: number[];
  b: number[];
  c: number[];
} {
  const N = boxes.length;
  const a = new Array(N);
  const b = new Array(N);
  const c = new Array(N);

  for (let i = 0; i < N; i++) {
    const w = boxes[i]![2]!;
    const h = boxes[i]![3]!;
    const r = boxes[i]![4] || 0; // angle

    const cos = Math.cos(r);
    const sin = Math.sin(r);
    const cos2 = cos * cos;
    const sin2 = sin * sin;

    // Gaussian bounding boxes
    const gw2 = (w * w) / 12;
    const gh2 = (h * h) / 12;

    const val_a = (w * w) / 12;
    const val_b = (h * h) / 12;

    const _a = val_a;
    const _b = val_b;

    a[i] = _a * cos2 + _b * sin2;
    b[i] = _a * sin2 + _b * cos2;
    c[i] = (_a - _b) * cos * sin;
  }
  return { a, b, c };
}

/**
 * Calculate the probabilistic IoU between oriented bounding boxes.
 * references: https://arxiv.org/pdf/2106.06072v1.pdf
 */
export function batch_probiou(
  obb1: number[][],
  obb2: number[][],
  eps: number = 1e-7,
): number[][] {
  const N = obb1.length;
  const M = obb2.length;
  const ious: number[][] = Array.from({ length: N }, () => Array(M).fill(0));

  const cov1 = get_covariance_matrix(obb1);
  const cov2 = get_covariance_matrix(obb2);

  for (let i = 0; i < N; i++) {
    const x1 = obb1[i]![0]!;
    const y1 = obb1[i]![1]!;
    const a1 = cov1.a[i]!;
    const b1 = cov1.b[i]!;
    const c1 = cov1.c[i]!;

    for (let j = 0; j < M; j++) {
      const x2 = obb2[j]![0]!;
      const y2 = obb2[j]![1]!;
      const a2 = cov2.a[j]!;
      const b2 = cov2.b[j]!;
      const c2 = cov2.c[j]!;

      const t1 =
        (((a1 + a2) * Math.pow(y1 - y2, 2) + (b1 + b2) * Math.pow(x1 - x2, 2)) /
          ((a1 + a2) * (b1 + b2) - Math.pow(c1 + c2, 2) + eps)) *
        0.25;

      const t2 =
        (((c1 + c2) * (x2 - x1) * (y1 - y2)) /
          ((a1 + a2) * (b1 + b2) - Math.pow(c1 + c2, 2) + eps)) *
        0.5;

      const t3_term =
        ((a1 + a2) * (b1 + b2) - Math.pow(c1 + c2, 2)) /
          (4 *
            Math.sqrt(
              Math.max(a1 * b1 - c1 * c1, 0) * Math.max(a2 * b2 - c2 * c2, 0),
            ) +
            eps) +
        eps;

      const t3 = Math.log(t3_term) * 0.5;

      const bd = Math.min(Math.max(t1 + t2 + t3, eps), 100.0);
      const hd = Math.sqrt(1.0 - Math.exp(-bd) + eps);
      ious[i]![j] = 1 - hd;
    }
  }
  return ious;
}
