/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "api.hpp"

namespace rzq{
namespace mapf{


int RunMOCBS(
  basic::GridkConn& g, std::vector<long>& starts, std::vector<long>& goals,
  mapf::MOMAPFResult* res, double time_limit,
  std::vector<long> wait_cost, int ifTreeByTree)
{

  mapf::MOCBS_TB* planner;
  planner = new mapf::MOCBS_TEMOA();

  if (g.GetCostDim() == 1) {
    std::cout << "[WARNING] RunMOCBS, You are using MO-CBS to solve a single-objective problem." << std::endl;
    std::cout << "[WARNING] RunMOCBS, MO-CBS-N/TN is created" << std::endl;
  }
  planner->SetGrid(-1, starts.size(), g, wait_cost);
  planner->Search(starts, goals, time_limit, 0.0); 
  *res = planner->GetResult();

  delete planner;

  return 1;
};


int RunBBMOCBS(
  basic::GridkConn& g, std::vector<long>& starts, std::vector<long>& goals,
  mapf::MOMAPFResult* res, double time_limit,
  std::vector<long> wait_cost, int mode)
{

  mapf::NBMOCBS_EMOA* planner;
  if (mode == 0) {
    planner = new mapf::NBMOCBS_EMOA();
  }else{
    std::cout << "[ERROR] RunBBMOCBS, unknown planner mode" << mode << std::endl;
    throw std::runtime_error("[ERROR]");
  }

  if (g.GetCostDim() == 1) {
    std::cout << "[WARNING] RunBBMOCBS, You are using BB-MO-CBS to solve a single-objective problem." << std::endl;
    std::cout << "[WARNING] RunBBMOCBS, BBMOCBS_EMOA is created" << std::endl;
  }
  planner->SetGrid(-1, starts.size(), g, wait_cost);
  planner->Search(starts, goals, time_limit, 0.0); 
  *res = planner->GetResult();

  delete planner;

  return 1;
};


}
}