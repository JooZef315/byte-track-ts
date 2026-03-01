/**
 * Convert bounding box format from [x, y, w, h] to [x1, y1, w, h] where x1, y1 are top-left coordinates.
 * @param x Input bounding box coordinates in xywh format.
 * @returns Bounding box coordinates in xyltwh format.
 */
export declare function xywh2ltwh(x: number[][]): number[][];
/**
 * Calculate the intersection over box2 area given box1 and box2.
 * @param box1 A 2D array of shape (N, 4) representing N bounding boxes in x1y1x2y2 format.
 * @param box2 A 2D array of shape (M, 4) representing M bounding boxes in x1y1x2y2 format.
 * @param iou Calculate the standard IoU if True else return inter_area/box2_area.
 * @param eps A small value to avoid division by zero.
 * @returns A 2D array of shape (N, M) representing the intersection over box2 area.
 */
export declare function bbox_ioa(box1: number[][], box2: number[][], iou?: boolean, eps?: number): number[][];
/**
 * Calculate the probabilistic IoU between oriented bounding boxes.
 * references: https://arxiv.org/pdf/2106.06072v1.pdf
 */
export declare function batch_probiou(obb1: number[][], obb2: number[][], eps?: number): number[][];
//# sourceMappingURL=ops.d.ts.map