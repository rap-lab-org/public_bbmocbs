/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "mocbs_nb.hpp"

// #define __MY_COMPILE_KUNG_SORT__ false // for comparison, should never be used in practice.
#define __MY_COMPILE_KUNG_SORT__ true

// #define __MY_COMPILE_NONDOM_SORT__ false // for comparison, should nenver bve used in practice.
#define __MY_COMPILE_NONDOM_SORT__ true

namespace rzq{
namespace mapf{


std::ostream& operator<<(std::ostream& os, const CostIndexCombo& n)
{
  os << "{cvec:" << n.first << ",IDs:(";
  for (auto iy: n.second) {
    os << iy << ",";
  }
  os << ")}";
  return os;
};

int runMinkowskiSum(
  std::unordered_map<long, CostIndexCombo>& in1,
  std::unordered_map<long, basic::CostVector>& in2,
  std::unordered_map<long, CostIndexCombo> *out,
  int mode, int ri)
{
  out->clear();
  int temp_id = -1;
  for (auto item1: in1) {
    for (auto item2: in2) {
      // std::cout << " --- " << item1.second.first << ", " << item2.second << std::endl;
      (*out)[temp_id] = CostIndexCombo();
      out->at(temp_id).first = item1.second.first + item2.second;
      auto index_vec = item1.second.second;

      if (mode==0) {
        index_vec.push_back(item2.first);
      }else if (mode==1) {
        index_vec[ri] = item2.first;
      }else if (mode==2) {
        // do nothing
      }else {
        std::cout << " mode = " << mode << std::endl;
        throw std::runtime_error("[ERROR] runMinkowskiSum(), unknown mode.");
      }
      out->at(temp_id).second = index_vec;
      temp_id--;
    }
  }
  return 1;
};

int nondomSubsetKung(std::unordered_map<long, CostIndexCombo> *out) 
{
  if (out->size() == 0) {return 1;} // MO-CBS probably times out.

  std::list<CostIndexCombo> sorted;
  // lex sort via heap
  std::set< std::pair< basic::CostVector, long> > heap;
  for (auto& item: (*out)) {
    heap.insert( std::make_pair(item.second.first, item.first) );
  }
  while (!heap.empty()) {
    auto cvec = heap.begin()->first;
    long key = heap.begin()->second;
    sorted.push_back(out->at(key));
    heap.erase(heap.begin());
  }

  // compute non-dom subset

  /////  case M=2 ///////
  if (sorted.front().first.size() == 2) { 
    out->clear();
    long temp_id = 1;
    long g2min=1e10;
    for (auto item: sorted) {
      if (item.first[1] >= g2min) {
        // std::cout << " nondomSubsetKung filters " << item << std::endl;
        continue;
      }
      g2min = item.first[1];
      // std::cout << " nondomSubsetKung keeps " << item << " g2min = " << g2min << std::endl;
      (*out)[temp_id] = item;
      temp_id++;
    }
    return 1;
  }

  /////  case M=3 ///////
  if (sorted.front().first.size() == 3) { // M=3
    // This is not a full Kung's method for M=3! I do not implement the AVL-tree but simply use an array.
    // This is not tested yet!!!
    out->clear();
    long temp_id = 1;
    std::list<CostIndexCombo> front; // front is a set of vector of length 2.
    for (auto item: sorted) {
      // check dom for the project vector
      bool dom = false;
      for (auto item2: front) {
        if ( (item2.first[1] <= item.first[1]) && (item2.first[2] <= item.first[2]) ){
          // search::EpsDominance(vecProj(vec), vecProj(item.first)) ){
          dom = true;
          break;
        }
      }
      if (dom) {continue;}

      front.push_back(item);
      (*out)[temp_id] = item;
      temp_id++;      

    }
    return 1;
  }

  ///// case M!=2 and M!=3
  else {
    std::cout << "nondomSubsetKung() is only implemented for M=2,3" << " here M = " << sorted.front().first.size() << std::endl;
    throw std::runtime_error("[ERROR] nondomSubsetKung() M>3 not implemented!");
    return 0;
  }
};

int nondomSubsetNaive(std::unordered_map<long, CostIndexCombo> *out) 
{
  // double for-loop
  if (out->size() == 0) {return 1;} // MO-CBS probably times out.

  std::unordered_set<long> id_to_delete;
  for (auto& item1: (*out)) {
    if (id_to_delete.find(item1.first) != id_to_delete.end()){
      continue;
    }
    auto cvec1 = item1.second.first;
    for (auto& item2: (*out)) {
      if (item2.first == item1.first) {continue;}
      if (id_to_delete.find(item2.first) != id_to_delete.end()){
        continue;
      }
      auto cvec2 = item2.second.first;
      if (search::EpsDominance(cvec1, cvec2)) {
        id_to_delete.insert(item2.first);
      }
    }
  }

  for (auto k : id_to_delete) {
    out->erase(k);
  }
  return 1;
};

void NBCBSNode::computeSortedPF()
{
  // init a combo data structure.
  sorted_costIndex.clear();
  std::unordered_map<long, CostIndexCombo> *combo, *combo2;
  combo = new std::unordered_map<long, CostIndexCombo>();
  long temp_id = 1;
  for (auto& item: idvl_paths[0]) {
    long path_key = item.first;
    (*combo)[temp_id] = CostIndexCombo();
    combo->at(temp_id).first = idvl_costs[0][path_key];
    combo->at(temp_id).second = std::vector<long>({path_key});
    temp_id++;
  }
  for (long ri = 1; ri < idvl_costs.size(); ri++) { // assume there is more than one agent.
    combo2 = new std::unordered_map<long, CostIndexCombo>();
    runMinkowskiSum(*combo, idvl_costs[ri], combo2) ;

    if (__MY_COMPILE_NONDOM_SORT__) { // if this flag is false, all joint paths will be kept for testing purposes.
      if (__MY_COMPILE_KUNG_SORT__){
        nondomSubsetKung(combo2) ;
      }else{
        nondomSubsetNaive(combo2) ;
      }
    }
    
    // std::cout << " a3" << std::endl;
    delete combo;
    combo = combo2;
    combo2 = NULL;
  }

  // now, combo is a pointer to a hash
  // lex sort via heap
  std::set< std::pair< basic::CostVector, long> > heap;
  for (auto& item: (*combo)) {
    heap.insert( std::make_pair(item.second.first, item.first) );
  }
  while (!heap.empty()) {
    auto cvec = heap.begin()->first;
    // std::cout << " heap popped " << cvec << std::endl;
    long key = heap.begin()->second;
    sorted_costIndex.push_back(combo->at(key));
    heap.erase(heap.begin());
  }

  delete combo;
  return ;
};

void NBCBSNode::printPF() {
  int rank = 1;
  for (auto ix : sorted_costIndex) {
    std::cout << " rank:" << rank << ix << std::endl;
    rank++;
  }
  return ;
};


bool NBCBSNode::solFilter(std::unordered_map< long, basic::CostVector >& sol_costs)
{
  auto iter = sorted_costIndex.begin();
  bool ret_flag = false;
  while (iter != sorted_costIndex.end()) {
    // loop from the first
    bool erased = false;
    for (auto& item2: sol_costs) {
      if (search::EpsDominance(item2.second, iter->first)) {
        iter = sorted_costIndex.erase(iter);
        erased = true;
        ret_flag = true;
        break;
      }
    }
    if (!erased) {
      iter++;
    }
    else {
      break; // stop at the first non-dom one, which will be set as the representative, no need to run further filtering.
    }
  } // end while
  return ret_flag;
};

bool NBCBSNode::setRep()
{
  if (sorted_costIndex.size() == 0) {
    // this node should be pruned.
    return false;
  }else{
    rep_cvec = sorted_costIndex.begin()->first;
    rep_Ids = sorted_costIndex.begin()->second;
    for (int i = 0; i < idvl_costs.size(); i++) {
      rep_jpath[i] = idvl_paths[i][rep_Ids[i]];
      if (rep_jpath[i].size() == 0) {
        throw std::runtime_error("[ERROR] NBCBSNode::setRep zero size path.");
      }
    }
  }
  return true;
};

bool NBCBSNode::isEmptyPlans() {
  return (sorted_costIndex.size() == 0);
};

std::ostream& operator<<(std::ostream& os, const NBCBSNode& n)
{
  os << "NBCBSNode{id:" << n.id << ",parent:" << n.parent << ",cstr:" << n.cstr 
     << ",rep_cvec:" << n.rep_cvec << ",rep_path:" << n.rep_jpath << "}";
  return os;
};


/////////////////////////////////////////////////////////////////////////
////////////////////// NBMOCBS_EMOA /////////////////////
/////////////////////////////////////////////////////////////////////////

NBMOCBS_EMOA::NBMOCBS_EMOA() {
  // std::cout << "[INFO] Constructor - NBMOCBS_EMOA" << std::endl;
};

NBMOCBS_EMOA::~NBMOCBS_EMOA() {};

void NBMOCBS_EMOA::Search(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) {
  std::cout << "***BB-MO-CBS search starts***" << std::endl;
  // assign graph pointer here.
  for (int ri = 0; ri < _nAgent; ri++){
    SetGraphPtr(ri, _nAgent, &(_tempAgentGrids[ri]), _agentCosts[ri]);
  }
  // set epsilon, by default 0.
  _epsilon = eps;
  _result.others.resize(2); // to store the average runtime for computing Pareto-optimal front.
  _result.others[0] = 0; // total time
  _result.others[1] = 0; // total number of calls

  // copy
  if (starts.size() != _nAgent) {
    std::cout << "[ERROR] NBMOCBS_EMOA::Search input starts.size() = " << starts.size() 
              << " does not match _nAgent = " << _nAgent << std::endl;
    throw std::runtime_error("[ERROR] NBMOCBS_EMOA::Search input starts size mismatch!");
  }
  if (goals.size() != _nAgent) {
    std::cout << "[ERROR] NBMOCBS_EMOA::Search input goals.size() = " << starts.size() 
              << " does not match _nAgent = " << _nAgent << std::endl;
    throw std::runtime_error("[ERROR] NBMOCBS_EMOA::Search input goals size mismatch!");
  }
  _vo = starts;
  _vd = goals;
  _tlimit = time_limit;
  _t0 = std::chrono::steady_clock::now(); // for timing.

  // init
  _Init();
  _result.find_all_pareto = false;

  // main search loop
  size_t iterCount__ = 0;
  while ( true ) {
    iterCount__++;
    if (DEBUG_MOCBS > 0) {
      std::cout << "[DEBUG]----------------------- search iter : " 
        << iterCount__ << " ------------------------" << std::endl;
    }

    //// select high-level node, lexicographic order
    NBCBSNode *node = _PopNode();
    //// see if open depletes and there is no more node.
    if (!node) {
      _result.find_all_pareto = true;
      break; // terminates
    }

    //// filtering step.
    bool ret_flag = node->solFilter(_result.costs);
    if (ret_flag) { // at least one vector is filtered.
      if (!node->isEmptyPlans()) {
        bool flag = node->setRep();
        if (!flag) {
          std::cout << "[DEBUG] * NBMOCBS_EMOA::Search, fail to set Rep for node " << (*node) << std::endl;
          return ;
        }
        _open.insert( std::make_pair(node->rep_cvec, node->id) );
      }else{
        // do nothing, this node has empty plan.
      }
      continue;
    }

    //// detect and select conflict
    auto cft = _DetectAndSelectConflict(node);

    //// new solution
    if (!cft.IsValid()) {
      // find a conflict-free solution. add it to the result.
      _UpdateSolution(node);
      _open.insert( std::make_pair(node->rep_cvec, node->id) );
      continue; // end this iteration
    }

    _cfts.push_back(cft); // @2022-10-31 This is just to count duplicated conflicts.
                          // It can be removed without affecting the correctness of the algorithm.

    if ( _IfTimeout() ) {
      std::cout << "[INFO] * BBMOCBS_EMOA::Search Timeout, reach limit " << _tlimit << std::endl;
      break;
    }
    if (_result.n_expanded % (500/_nAgent) == 0) {
      std::cout << "[INFO] * BBMOCBS_EMOA::Search, after " << _result.n_expanded << " conflict splits " << std::endl;
    }

    //// split conflict (i.e. expand *node )
    auto cstrs = cft.Split();
    for (auto cstr: cstrs) { // loop over both constraints.
      _Lsearch(cstr.i, node, &cstr);
    }
    _result.n_expanded++; // # conflict resolved.
  } // end while loop

  auto tnow = std::chrono::steady_clock::now(); // for timing.
  _result.rt_search = std::chrono::duration<double>(tnow - _t0).count();  
  _result.n_branch = _result.n_branch / _result.n_expanded; // average branching factor.

  _result.others[0] = _result.others[0] / _result.others[1]; // get the averaged runtime for computeSortedPF().

  _CountDupConflict();

  return ;
};

bool NBMOCBS_EMOA::_Init() {

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] * NBMOCBS_EMOA::_Init start " << std::endl;
  }

  auto init_t0 = std::chrono::steady_clock::now(); // for timing.
  _result.n_expandLow = 0.0;

  //// create the first root node. output via a pointer to that root node.
  long nid = _GenNodeId();
  _nodes[nid].id = nid;
  NBCBSNode* out = &(_nodes[nid]); // set as output pointer
  out->idvl_paths.resize(_nAgent);
  out->idvl_costs.resize(_nAgent);

  //// plan individual pareto-optimal path for each agent.
  std::vector<CBSConstraint*> emptySetCstr;
  _result.n_initRoot = 1;
  for (int ri = 0; ri < _vo.size(); ri++) {
    _initRes[ri] = search::MOSPPResult();
    int ret_flag = _LsearchPlan( ri, emptySetCstr, &(_initRes[ri]) );
    _result.n_initRoot *= _initRes[ri].paths.size();
    // std::cout << " " << _initRes[ri].paths.size() << " ";

    if (ret_flag == 0) {
      // TODO, how to handle this exception
      std::cout << "[ERROR] * NBMOCBS_EMOA::_Init failed at robot " << ri << std::endl;
      throw std::runtime_error( "[ERROR] NBMOCBS_EMOA::_Init failed !" );
    }
    out->idvl_paths[ri] = _initRes[ri].paths;
    out->idvl_costs[ri] = _initRes[ri].costs;
  }

  auto tnow = std::chrono::steady_clock::now(); // for timing.    
  out->computeSortedPF();
  _result.others[0] += std::chrono::duration<double>(std::chrono::steady_clock::now()-tnow).count();
  _result.others[1] += 1.0;
  // no need to do solFilter, i.e., solution-filtering, at the beginning.
  bool flag = out->setRep();

  if (!flag) {
    std::cout << "[DEBUG] * NBMOCBS_EMOA::_Init fails. Fail to set Rep for root." << _nodes[nid] << std::endl;
    return false;
  }

  _open.insert( std::make_pair(out->rep_cvec, out->id) );
  _result.n_generated++; // update

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] * NBMOCBS_EMOA::_Init, node = " << _nodes[nid] << std::endl;
  }

  _result.n_branch = 0.0; // useless in node-based MO-CBS.
  _result.rt_initHeu = std::chrono::duration<double>(std::chrono::steady_clock::now() - init_t0).count();  

  return true;
};

NBCBSNode* NBMOCBS_EMOA::_PopNode() {
  if ( !_open.empty() ) {
    long nid = _open.begin()->second;
    NBCBSNode* out = &( _nodes[nid] );
    _open.erase(_open.begin());
    if (DEBUG_MOCBS > 0) {
      std::cout << "[DEBUG] ++ NBMOCBS_EMOA::_PopNode node = " << (*out) << std::endl;
    }
    return out; // a node is selected.
  }

  // now _open is empty, just terminate.
  return NULL;
};


void NBMOCBS_EMOA::_UpdateSolution(NBCBSNode* n) {

  if (_result.costs.size() == 0) {
    // runtime to reach the first feasible solution.
    auto tnow = std::chrono::steady_clock::now();
    _result.rt_firstSol = std::chrono::duration<double>(tnow - _t0).count();
    _result.firstSolCost = n->rep_cvec;
  }

  // add
  _result.solus[_sol_id_gen] = n->rep_jpath;
  _result.costs[_sol_id_gen] = n->rep_cvec;
  _sol_id_gen++;

  if (DEBUG_MOCBS > 0) {
    std::cout << "[DEBUG] *** NBMOCBS_EMOA::_UpdateSolution find sol node " << (*n) << std::endl;
  }
  return ;
};

bool NBMOCBS_EMOA::_GetAllCstr(int ri, NBCBSNode *in, std::vector<CBSConstraint*> *out) {
  out->clear();
  long cid = in->id;
  while (cid != -1) {
    NBCBSNode* n = &(_nodes[cid]);
    CBSConstraint* cstr_ptr = &(n->cstr);
    if (cstr_ptr->i == ri) { // a cstr imposed on agent-i.
      out->push_back( &(n->cstr) );
    }
    cid = n->parent;
  }
  return true;
};


int NBMOCBS_EMOA::_Lsearch(int ri, NBCBSNode* n, CBSConstraint* cp)
{
  std::vector<CBSConstraint*> cstrs;
  auto retFlag = _GetAllCstr(ri, n, &cstrs);
  // currently, this retFlag is useless.

  cstrs.push_back(cp); // currently, this pointer is still valid.
  if (DEBUG_MOCBS) {
    std::cout << "[DEBUG] **** NBMOCBS_EMOA::_GetAllCstr" << std::endl;
    for (auto & cstr : cstrs) {
      std::cout << "[DEBUG] **** ----- cstr " << (*cstr) << std::endl;
    }
  }
  search::MOSPPResult lowRes;

  auto lowFlag = _LsearchPlan(ri, cstrs, &lowRes);
  // handle different low flag?

  if (DEBUG_MOCBS) {
    std::cout << "[DEBUG] **** NBMOCBS_EMOA::_Lsearch, result:" << lowRes << std::endl;
  }

  //// generate a new HL node, only one.
  long nid = _GenNodeId();
  _nodes[nid] = *n; // make a copy
  _nodes[nid].id = nid;
  _nodes[nid].parent = n->id;
  _nodes[nid].cstr = *cp; // make a copy
  
  _nodes[nid].idvl_paths[ri].clear(); // clear old idvl paths of agent-ri.
  _nodes[nid].idvl_costs[ri].clear();

  //// Generate new HL nodes.
  for (const auto& ii : lowRes.paths) { // loop over all idvl paths of agent ri.
    int k = ii.first;

    // enforce unit time path
    std::vector<long> t2;
    std::vector<long> p2;
    UnitTimePath(lowRes.paths[k], lowRes.times[k], &p2, &t2); 
      // t2 is useless here. always start from t=0;
    _nodes[nid].idvl_paths[ri][k] = p2;
    _nodes[nid].idvl_costs[ri][k] = lowRes.costs[k];
  }

  //// solution filtering

  auto tnow = std::chrono::steady_clock::now(); // for timing.  
  _nodes[nid].computeSortedPF();
  _result.others[0] += std::chrono::duration<double>(std::chrono::steady_clock::now()-tnow).count();
  _result.others[1] += 1.0;
  // _nodes[nid].solFilter(_result.costs); // solution filtering, if no this, the impl is still correct.
  if (!_nodes[nid].isEmptyPlans()) {
    bool flag = _nodes[nid].setRep();
    if (!flag) {
      std::cout << "[DEBUG] * NBMOCBS_EMOA::Search, fail to set Rep for node " << _nodes[nid] << std::endl;
      return 0;
    }
    _open.insert( std::make_pair(_nodes[nid].rep_cvec, nid) );
    _result.n_generated++; // update, this also include root node.
    _result.n_branch++; // a new branch. not include root node.
    if (DEBUG_MOCBS) {
      std::cout << "[DEBUG] **** NBMOCBS_EMOA::_Lsearch, gen node:" << _nodes[nid] << std::endl;
    }
  }else{
    if (DEBUG_MOCBS) {
      std::cout << "[DEBUG] **** NBMOCBS_EMOA::_Lsearch, gen empty plan node:" << _nodes[nid] << std::endl;
    }
  }

  return 1;
};


CBSConflict NBMOCBS_EMOA::_DetectAndSelectConflict(NBCBSNode* n)
{
  return DetectConflict(&(n->rep_jpath));
};

//////////////////////////////////////////////////////////////////////////


void DetectAllConflict(std::vector<long>* p1, std::vector<long>* p2, int i, int j, std::vector<CBSConflict>* out) 
{
  // out->clear();
  if ( (p1->size() == 0) || (p2->size() == 0) ) {
    std::cout << "[WARNING] DetectConflict receive zero size path !" << std::endl;
    // throw std::runtime_error("[WARNING] DetectConflict receive zero size path !");
    return;
  }
  size_t l = p1->size() > p2->size() ? p1->size() : p2->size(); // find max.
  for (size_t idx = 0; idx < l; idx++) {
    CBSConflict cft;
    long v1 = idx < p1->size() ? p1->at(idx) : p1->back();
    long v2 = idx < p2->size() ? p2->at(idx) : p2->back();
    // std::cout << " idx = " << idx << " v1 = " << v1 << " v2 = " << v2 << std::endl;
    if ( v1 == v2 ) { // vertex conflict
      cft.i = i;
      cft.j = j;
      cft.vi = v1;
      cft.vj = v2;
      cft.ta = idx;
      cft.tb = idx;
      // break; // stop at the first conflict detected.
      out->push_back(cft) ;
    }
    size_t idy = idx + 1;
    if (idy >= l) {continue;}
    long v1b = idy < p1->size() ? p1->at(idy) : p1->back();
    long v2b = idy < p2->size() ? p2->at(idy) : p2->back();
    // std::cout << " --- v1b = " << v1b << " v2b = " << v2b << std::endl;
    if ( (v1 == v2b) && (v1b == v2) ) { // edge conflict
      cft.i = i;
      cft.j = j;
      cft.vi = v1;
      cft.vj = v2;
      cft.ta = idx;
      cft.tb = idy;
      // break; // stop at the first conflict detected.
      out->push_back(cft) ;
    }
  }
  // std::cout << " returned " << std::endl;
  return;
};


} // end namespace mapf
} // end namespace rzq
