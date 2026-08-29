# 👁️ FC-Vision

## *SOP package*

✍️ Author: [Chen Feng](https://chen-albert-feng.github.io/AlbertFeng.github.io/)

### 📖 Usage

This package is built for implementing parallel solver for the Sequential Ordering Problem (SOP). 

Example:
```
...
#include "sop/sop.h"

unique_ptr<SOP> sop_= nullptr;
sop_->setParams(vmax_, wmax_);

vector<Eigen::VectorXd> prior, new_points;
Eigen::MatrixXi cost_mat = this->sop_->constructCostMat(prior, new_points);

vector<int> solution;
sop_->sopSolve(cost_mat, solution); // solved sqeuence

...
```