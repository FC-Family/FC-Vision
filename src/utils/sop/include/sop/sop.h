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

#ifndef _SOP_H_
#define _SOP_H_

#include "sop/digraph.h"
#include "sop/solver.h"
#include "sop/edge.h"
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <chrono>
#include <unordered_set>
#include <sys/types.h>
#include <dirent.h>
#include <Eigen/Dense>

using std::string;
using std::ifstream;
using std::vector;
using std::cout;
using std::unordered_set;
using namespace std;

namespace fc_vision
{

class SOP
{
  
  struct tour 
  {
    vector<Edge> path;
    int cost;
  };

  public:
    SOP(){
    }
    ~SOP(){
    }
    void setParams(const double& v, const double& w);
    Eigen::MatrixXi constructCostMat(const vector<Eigen::VectorXd>& prior, const vector<Eigen::VectorXd>& updated);
    void sopSolve(const Eigen::MatrixXi& cost_mat, vector<int>& order);

  private:
    double vmax_ = -1.0;
    double wmax_ = -1.0;

    int costCal(const Eigen::VectorXd& p1, const Eigen::VectorXd& p2);
    void createGraphsFromMatrix(const Eigen::MatrixXi& matrix, Digraph& g, Digraph& p);
    void removeRedundantEdges(Digraph& g, Digraph& p);
    void removeRedundantEdgeSuccessors(Digraph& g, Digraph& p);
    vector<int> getSolution(const vector<Edge>& path);
};

inline int SOP::costCal(const Eigen::VectorXd& p1, const Eigen::VectorXd& p2)
{
  double t_p = (p1.head(3) - p2.head(3)).norm() / vmax_;
  // double t_yaw = abs(p1(4) - p2(4)) / (0.5*wmax_);

  // double t_max = 100.0*max(t_p, t_yaw);
  double t_max = 100.0*t_p;

  return (int)t_max;
}

inline void SOP::createGraphsFromMatrix(const Eigen::MatrixXi& matrix, Digraph& g, Digraph& p)
{
    int rows = matrix.rows();
    g.set_size(rows);
    p.set_size(rows);

    for (int source = 0; source < rows; ++source) 
    {
      for (int dest = 0; dest < rows; ++dest) 
      {
        int weight = matrix(source, dest);
        if (weight < 0) 
        {
          p.add_edge(source, dest, 0);
        } 
        else if (source != dest) 
        {
          g.add_edge(source, dest, weight);
        }
      }
    }

    return;
}

inline void SOP::removeRedundantEdges(Digraph& g, Digraph& p)
{
    for(int i = 0; i < p.node_count(); ++i)
    {
      const vector<Edge>& preceding_nodes = p.adj_outgoing(i);
      unordered_set<int> expanded_nodes;
      for(int j = 0; j < preceding_nodes.size(); ++j)
      {
        vector<Edge> st;
        st.push_back(preceding_nodes[j]);
        while(!st.empty())
        {
          Edge dependence_edge = st.back();
          st.pop_back();
          if(dependence_edge.source != i)
          {
            g.remove_edge(dependence_edge.dest, i);
            expanded_nodes.insert(dependence_edge.dest);
          }
          for(const Edge& e : p.adj_outgoing(dependence_edge.dest))
          {
            if(expanded_nodes.find(e.dest) == expanded_nodes.end())
            {
              st.push_back(e);
              expanded_nodes.insert(e.dest);
            }
          }
        }
      } 
    }

    return;
}

inline void SOP::removeRedundantEdgeSuccessors(Digraph& g, Digraph& p)
{
    for(int i = 0; i < p.node_count(); ++i)
    {
      const vector<Edge>& preceding_nodes = p.adj_incoming(i);
      unordered_set<int> expanded_nodes;
      for(int j = 0; j < preceding_nodes.size(); ++j)
      {
        vector<Edge> st;
        st.push_back(preceding_nodes[j]);
        while(!st.empty())
        {
          Edge dependence_edge = st.back();
          st.pop_back();
          if(dependence_edge.source != i)
          {
            g.remove_edge(i, dependence_edge.dest);
            expanded_nodes.insert(dependence_edge.dest);
          }
          for(const Edge& e : p.adj_outgoing(dependence_edge.dest))
          {
            if(expanded_nodes.find(e.dest) == expanded_nodes.end())
            {
              st.push_back(e);
              expanded_nodes.insert(e.dest);
            }
          }
        }
      } 
    }

    return;
}

inline vector<int> SOP::getSolution(const vector<Edge>& path)
{
    vector<int> results;
    for(Edge e : path)
      results.push_back(e.dest);

    return results;
}

}

#endif