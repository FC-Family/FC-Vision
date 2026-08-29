/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Author       :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 * Date         :    Jul. 2024
 * E-mail       :    cfengag at connect dot ust dot hk.
 * Description  :    This file implements parallel solver for the Sequential Ordering Problem.
 * License      :    GNU General Public License <http://www.gnu.org/licenses/>.
 * Project      :    FC-Planner is free software: you can redistribute it and/or 
 *                   modify it under the terms of the GNU Lesser General Public 
 *                   License as published by the Free Software Foundation, 
 *                   either version 3 of the License, or (at your option) any 
 *                   later version.
 *                   FC-Planner is distributed in the hope that it will be useful,
 *                   but WITHOUT ANY WARRANTY; without even the implied warranty 
 *                   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 *                   See the GNU General Public License for more details.
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

#include "sop/sop.h"

namespace fc_vision
{

void SOP::setParams(const double& v, const double& w)
{
  vmax_ = v;
  wmax_ = w;

  return;
}

/*
* @brief Construct the cost matrix for the SOP problem.
* @param prior: the prior set of vectors -> N.
* @param updated: the updated set of vectors -> M.
* @return cost_mat: the cost matrix for the SOP problem -> MATRIX (N+M)x(N+M).
*/
Eigen::MatrixXi SOP::constructCostMat(const vector<Eigen::VectorXd>& prior, const vector<Eigen::VectorXd>& updated)
{
    int prior_num = prior.size(), updated_num = updated.size();
    Eigen::MatrixXi cost_mat(prior_num + updated_num, prior_num + updated_num);

    // Block A: Prior to Prior -> N x N
    Eigen::MatrixXi A = Eigen::MatrixXi::Zero(prior_num, prior_num);
    for (int i=0; i<prior_num; i++)
    {
      for (int j=0; j<prior_num; j++)
      {
        if (i == j) 
          continue;
        else if (i>j) 
          A(i, j) = -1;
        else
          A(i, j) = costCal(prior[i], prior[j]);
      }
    }

    // Block B: Prior to Updated -> N x M
    Eigen::MatrixXi B = Eigen::MatrixXi::Zero(prior_num, updated_num);
    for (int i=0; i<prior_num; i++)
    {
      for (int j=0; j<updated_num; j++)
      {
        B(i, j) = costCal(prior[i], updated[j]);
      }
    }

    // Block C: Updated to Prior -> M x N
    Eigen::MatrixXi C = Eigen::MatrixXi::Zero(updated_num, prior_num);
    for (int i=0; i<updated_num; i++)
    {
      for (int j=0; j<prior_num; j++)
      {
        if (j == 0) 
          C(i, j) = -1;
        else
          C(i, j) = costCal(updated[i], prior[j]);
      }
    }

    // Block D: Updated to Updated -> M x M
    Eigen::MatrixXi D = Eigen::MatrixXi::Zero(updated_num, updated_num);
    for (int i=0; i<updated_num; i++)
    {
      for (int j=0; j<updated_num; j++)
      {
        if (i == j) 
          continue;
        if (i > j) 
        {
          D(i, j) = costCal(updated[i], updated[j]);
          D(j, i) = D(i, j);
        }
      }
    }

    // Construct the cost matrix
    cost_mat.block(0, 0, prior_num, prior_num) = A;
    cost_mat.block(0, prior_num, prior_num, updated_num) = B;
    cost_mat.block(prior_num, 0, updated_num, prior_num) = C;
    cost_mat.block(prior_num, prior_num, updated_num, updated_num) = D;

    return cost_mat;
}

void SOP::sopSolve(const Eigen::MatrixXi& cost_mat, vector<int>& order)
{
  bool per = false;
  size_t hash_size = 10000; 
  int time_limit = 20, num_threads = 1;

  Digraph g;
  Digraph p;
  createGraphsFromMatrix(cost_mat, g, p);
  g.sort_edges();
  removeRedundantEdges(g, p);
  removeRedundantEdgeSuccessors(g, p);
  Solver::reset_best_solution();
  Solver::set_cost_matrix(g.dense_hungarian());
  Solver s = Solver(&g, &p);
  s.set_time_limit(time_limit, per);
  s.set_hash_size(hash_size);

  s.nearest_neighbor();
  s.solve_sop_parallel(num_threads);

  order = getSolution(s.best_solution_path());

  return;
}

} // namespace fc_vision