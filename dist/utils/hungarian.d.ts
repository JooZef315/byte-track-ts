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
export declare function hungarianAlgorithm(costMatrix: number[][]): [number[], number[]];
//# sourceMappingURL=hungarian.d.ts.map