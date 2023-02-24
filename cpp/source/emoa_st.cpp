
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#include "emoa_st.hpp"

#include <set>
#include <memory>
#include <chrono>

namespace rzq{
namespace search{

//################################################
// TOA* related //
//################################################

TOATree::TOATree() {
  
};

TOATree::~TOATree() {
  
};

void TOATree::Filter(basic::CostVector v) {
  int outFlag = 0;
  std::unordered_set<long> filtered_node;
  _root = _filter(_root, v, &filtered_node);
  if (filtered_node.size()>0) {
    _rebuildTree(&filtered_node);
  }
  return;
};

basic::AVLNode* TOATree::_filter(
  basic::AVLNode* n, const basic::CostVector& k, std::unordered_set<long> *filtered_node)
{
  if (n == NULL) {return n;}
  if ( _key[n->id] < k ) { // only need to look at the right sub-tree.
    n->right = _filter(n->right, k, filtered_node); 
  }else{
    n->left = _filter(n->left, k, filtered_node);
    n->right = _filter(n->right, k, filtered_node);
  }
  // no return above, will reach here if n != NULL.
  if (EpsDominance(k, _key[n->id])) {
    if (filtered_node) {
      filtered_node->insert(n->id);
    }
  }
  return n;
};

void TOATree::_rebuildTree(std::unordered_set<long> *skip_node) {
  std::vector<long> tree_node_ids;
  size_t current_size = _size;
  ToSortedVector(NULL, &tree_node_ids, skip_node);

  _size = 0;
  basic::AVLNode* new_root = _rebuildTreeMethod(tree_node_ids, 0, tree_node_ids.size()-1);
  _deleteAll(_root);
  _root = new_root;

  return ;
};

basic::AVLNode* TOATree::_rebuildTreeMethod(std::vector<long> &tree_node_ids, long start, long end) {
  if (start > end) {
    return NULL;
  }
  long mid = (start+end)/2;
  basic::AVLNode* n = basic::NewAVLNode(tree_node_ids[mid]) ;
  _size++;
  n->left  = _rebuildTreeMethod(tree_node_ids, start, mid-1);
  n->right = _rebuildTreeMethod(tree_node_ids, mid+1, end);
  n->h = 1 + basic::Max(basic::H(n->left), basic::H(n->right));
  if (DEBUG_AVLTREE) {_verifyTree(n);}
  return n;
};

void TOATree::_verifyNonDom(std::vector<long>& tree_node_ids){
  for (size_t i = 0; i < tree_node_ids.size(); i++) {
    for (size_t j = i+1; j < tree_node_ids.size(); j++) {
      if (EpsDominance(_key[tree_node_ids[i]], _key[tree_node_ids[j]]) || 
          EpsDominance(_key[tree_node_ids[j]], _key[tree_node_ids[i]]) )
      {
        std::cout << " vec1 = " << _key[tree_node_ids[i]] << " vec2 = " << _key[tree_node_ids[j]] << std::endl;
        throw std::runtime_error("[ERROR] TOATree::_verifyNonDom fails.");
      }
    }
  }
  return;
};

///////////////////////////////////////////////////

Frontier3d::Frontier3d() {
  return;
};

Frontier3d::~Frontier3d() {
  return;
};

bool Frontier3d::Check(basic::CostVector g) {
  basic::CostVector ref_vec;
  auto pg = _p(g);
  auto flag = _tree.FindMaxLess(pg, &ref_vec, true); // true means find the max CostVector that is <= l.g.
  if (flag == 0) { // fail to find a smaller vec than l.g.
    return false; // non-dominated.
  }
  return ref_vec[1] <= pg[1]; // these are projected vectors (2-d).
};

void Frontier3d::Update(LabelST l) {
  auto pg = _p(l.g);
  if (_tree.Size() == 0){
    _tree.Add( pg );
    label_ids.insert(l.id);
    return;
  }
  // non-empty tree.

  label_ids.insert(l.id);
  _tree.Filter(pg);
  _tree.Add(pg); // add at first

  return;
};

basic::CostVector Frontier3d::_p(basic::CostVector v) {
  // TODO, remove this, use _proj().
  basic::CostVector out;
  for (size_t i = 1; i < v.size(); i++){
    out.push_back(v[i]);
  }
  return out;
};

//////////////////////////////////////////////////

//################################################
// KOA* related //
//################################################

KOATree::KOATree() {
  return;
};

KOATree::~KOATree() {
  return;
};

bool KOATree::Check(basic::CostVector v) {
  return _check(_root, v);
};

bool KOATree::_check(basic::AVLNode* n, const basic::CostVector& k) {
  if (n == NULL) {return false;}

  if (EpsDominance(_key[n->id], k)) {
    return true;
  }

  if (_key[n->id] > k) {
    return _check(n->left, k);
  }else{
    if (_check(n->left, k)) {return true;}
    return _check(n->right, k);
  }
};

FrontierKd::FrontierKd() {
  return;
};

FrontierKd::~FrontierKd() {
  return;
};

bool FrontierKd::Check(basic::CostVector g) {
  auto pg = _p(g);
  auto temp = _tree.Check(pg);
  return temp;
};

void FrontierKd::Update(LabelST l) {
  auto pg = _p(l.g);
  if (_tree.Size() == 0){
    _tree.Add( pg );
    label_ids.insert(l.id);
    return;
  }
  // non-empty tree.

  label_ids.insert(l.id);
  _tree.Filter(pg);
  _tree.Add(pg);

  // NOTE: no need to remove label_ids, Note that Kung's algo has a notation error.

  return;
};


////////////////////////////////////


EMOAst::EMOAst() {};

EMOAst::~EMOAst() {};

void EMOAst::_PostProcRes() {
  for (auto lid : _sols.label_ids) {
    _res.paths[lid] = std::vector<long>();
    _res.times[lid] = std::vector<long>();
    bool ret_flag = _BuildPath(lid, &(_res.paths[lid]), &(_res.times[lid]) );
      // when this flag is used, remember to check here for safety.
    _res.costs[lid] = _label[lid].g;

    if ( long(_res.paths[lid].size()) <= _last_nc_t) {
      std::cout << "[ERROR] path length < _last_nc_t !!! " << " size = " 
        << _res.paths[lid].size() << " _last_nc_t = " << _last_nc_t << std::endl;
      throw std::runtime_error("[ERROR]");
    }
  }
};

bool EMOAst::_BuildPath(long lid, std::vector<long>* path, std::vector<long>* times) {
  std::vector<long> out, out2;
  out.push_back(_label[lid].v);
  // out2.push_back(_label[lid].t);
  while( _parent.find(lid) != _parent.end() ) {
    out.push_back(_label[_parent[lid]].v);
    // std::cout << " out push_back v = " << _label[_parent[lid]].v << " " << _label[lid] << ", par = " << _label[_parent[lid]] << std::endl;
    // out2.push_back(_label[_parent[lid]].t);
    lid = _parent[lid];
  }
  path->clear();
  times->clear();
  for (size_t i = 0; i < out.size(); i++) {
    path->push_back(out[out.size()-1-i]);
    // std::cout << " reverse, path push_back v = " << out[out.size()-1-i] << std::endl;
    times->push_back(i); // unit time action.
  }
  return true;
};

bool EMOAst::_FrontierCheck(LabelST& l) {
  auto str = _L2S(l);
  if (_alpha.find(str) == _alpha.end()) {return false;}
  return _alpha[str].Check(l.g);
};

bool EMOAst::_SolutionCheck(LabelST& l) {
  if (_sols.label_ids.size() == 0) {return false;}
  return _sols.Check(l.f);
};

void EMOAst::_UpdateFrontier(LabelST& l) {
  auto str = _L2S(l);
  if (_alpha.find(str) == _alpha.end()) {
    if (DEBUG_BOA > 2) {
      std::cout << "[DEBUG] new frontier-naive for label " << l << std::endl;
    }
    _alpha[str] = FrontierKd();
  }
  _alpha[str].Update(l);
  return ;
};

void EMOAst::_UpdateSol(LabelST& l) {
  _sols.Update(l);
  return;
};


/////////////////////////////////////////////////////////////////
////////////////// RunMOAst /////////////////////
/////////////////////////////////////////////////////////////////

int RunEMOAstGrid(
  basic::GridkConn& g, long vo, long vd, double time_limit, basic::CostVector& wait_cost,
  std::vector< std::vector<long> >& ncs, std::vector< std::vector<long> >& ecs, MOSPPResult* res) 
{
  // std::cout << " RunMOAstGrid " << std::endl;
  // std::cout << " set Wait vector = " << wait_cost << std::endl;
  EMOAst planner;
  planner.SetGrid(g);
  planner.SetWaitCost(wait_cost);
  for (auto nc: ncs) {
    planner.AddNodeCstr(nc[0], nc[1]);
  }
  for (auto ec: ecs) {
    planner.AddEdgeCstr(ec[0], ec[1], ec[2]);
  }
  int outFlag = planner.Search(vo, vd, time_limit);
  *res = planner.GetResult();

  return outFlag;
};


} // end namespace search
} // end namespace rzq
