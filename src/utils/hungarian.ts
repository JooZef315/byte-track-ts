/**
 * Hungarian Algorithm (Munkres/Kuhn-Munkres) for solving the linear sum assignment problem.
 * This is a pure TypeScript implementation equivalent to scipy.optimize.linear_sum_assignment.
 *
 * Given a cost matrix of shape (n, m), finds the assignment of rows to columns
 * that minimizes the total cost.
 *
 * Returns [row_indices, col_indices] such that cost_matrix[row_indices[i]][col_indices[i]]
 * are the assigned pairs.
 */

export function hungarianAlgorithm(
  costMatrix: number[][],
): [number[], number[]] {
  const nRows = costMatrix.length;
  if (nRows === 0) return [[], []];
  const nCols = costMatrix[0]!.length;
  if (nCols === 0) return [[], []];

  // Pad to square if needed
  const n = Math.max(nRows, nCols);
  const cost: number[][] = Array.from({ length: n }, (_, i) =>
    Array.from({ length: n }, (_, j) =>
      i < nRows && j < nCols ? costMatrix[i]![j]! : 0,
    ),
  );

  // u[i] and v[j] are potentials for rows and columns (1-indexed internally)
  const u = new Float64Array(n + 1);
  const v = new Float64Array(n + 1);
  // p[j] = row assigned to column j (1-indexed)
  const p = new Int32Array(n + 1);
  // way[j] = column that leads to the shortest augmenting path to j
  const way = new Int32Array(n + 1);

  for (let i = 1; i <= n; i++) {
    // Start augmenting path from row i
    p[0] = i;
    let j0 = 0; // virtual column 0
    const minv = new Float64Array(n + 1).fill(Infinity);
    const used = new Uint8Array(n + 1);

    do {
      used[j0] = 1;
      let i0 = p[j0]!;
      let delta = Infinity;
      let j1 = 0;

      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 - 1]![j - 1]! - u[i0]! - v[j]!;
        if (cur < minv[j]!) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j]! < delta) {
          delta = minv[j]!;
          j1 = j;
        }
      }

      for (let j = 0; j <= n; j++) {
        if (used[j]) {
          const pj = p[j]!;
          u[pj] = (u[pj] ?? 0) + delta;
          v[j] = (v[j] ?? 0) - delta;
        } else {
          minv[j] = (minv[j] ?? 0) - delta;
        }
      }

      j0 = j1;
    } while (p[j0]! !== 0);

    // Update augmenting path
    do {
      const j1 = way[j0]!;
      p[j0] = p[j1]!;
      j0 = j1;
    } while (j0 !== 0);
  }

  // Extract result — only include original (non-padded) assignments
  const rowInd: number[] = [];
  const colInd: number[] = [];

  for (let j = 1; j <= n; j++) {
    const row = p[j]! - 1; // convert to 0-indexed
    const col = j - 1;
    if (row < nRows && col < nCols) {
      rowInd.push(row);
      colInd.push(col);
    }
  }

  return [rowInd, colInd];
}
