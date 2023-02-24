
/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_SEARCH_EMOA_ST_H_
#define ZHONGQIANGREN_SEARCH_EMOA_ST_H_

#include "boa_st.hpp"

namespace rzq{
namespace search{

#define DEBUG_EMOA 0

/**
 * @brief an interface, depends on the impl of BOA*, TOA*, etc.
 */
class Frontier {
public:
  Frontier(){};
  virtual ~Frontier(){};
  virtual bool Check(basic::CostVector g) = 0; // check if l is dominated or not.
  virtual void Update(LabelST l) = 0; // update the frontier using l.
  std::unordered_set<long> label_ids;
};

/**
 * @brief
 */
class TOATree : public basic::AVLTree<basic::CostVector> 
{
public:
  TOATree() ;
  ~TOATree() ;
  virtual void Filter(basic::CostVector v) ;
  virtual void _rebuildTree(std::unordered_set<long> *skip_node);
protected:
  // virtual basic::AVLNode* _filter(basic::AVLNode* n, const basic::CostVector& k, int* outFlag=NULL);
  virtual basic::AVLNode* _filter(basic::AVLNode* n, const basic::CostVector& k, std::unordered_set<long> *a = NULL);
  virtual basic::AVLNode* _rebuildTreeMethod(std::vector<long> &tree_node_ids, long start, long end);
  virtual void _verifyNonDom(std::vector<long>&);
};

/**
 * @brief The frontier set at each node, for TOA* (Tri-objective A*).
 */
class Frontier3d : public Frontier
{
public:
  Frontier3d();
  virtual ~Frontier3d();
  virtual bool Check(basic::CostVector g) override ;
  virtual void Update(LabelST l) override ;
// protected:
  // project the 3d vector to 2d vector by removing the first component.
  virtual basic::CostVector _p(basic::CostVector v);
  TOATree _tree;
};

/**
 * @brief
 */
class KOATree : public TOATree
{
public:
  KOATree() ;
  virtual ~KOATree() ;
  virtual bool Check(basic::CostVector v) ;
  // void Filter(basic::CostVector v) ;
protected:
  virtual bool _check(basic::AVLNode* n, const basic::CostVector& k) ; // new impl @2021-08-31
};

/**
 * @brief The frontier set at each node, for TOA* (Tri-objective A*).
 */
class FrontierKd : public Frontier3d
{
public:
  FrontierKd();
  virtual ~FrontierKd();
  virtual bool Check(basic::CostVector g) override ;
  virtual void Update(LabelST l) override ;
// protected:
  // project the 3d vector to 2d vector by removing the first component.
  // virtual basic::CostVector _p(basic::CostVector v);
  KOATree _tree; // TODO use polymorphism, change to pointer...
  int _mode = 0;
};

///////////////////////

/**
 * @brief an interface / base class, use its pointer
 */
class EMOAst : public BOAst {
public:
  EMOAst() ;
  virtual ~EMOAst() ;

protected:

  virtual void _UpdateFrontier(LabelST& l) ;

  virtual void _UpdateSol(LabelST& l) ;

  virtual bool _FrontierCheck(LabelST& l) ;
  
  virtual bool _SolutionCheck(LabelST& l) ;

  // virtual bool _CollideCheck(long v1, long v2, long t);

  virtual bool _BuildPath(long lid, std::vector<long>* path, std::vector<long>* times) ;

  virtual void _PostProcRes();

  // std::unordered_map< std::string, FrontierNaive > _alpha; // map a vertex id (v) to alpha(v).
  std::unordered_map< std::string, FrontierKd > _alpha; // map a vertex id (v) to alpha(v).

  FrontierKd _sols; // solution labels.
};


int RunEMOAstGrid(
  basic::GridkConn& g, long vo, long vd, double time_limit, basic::CostVector& wait_cost,
  std::vector< std::vector<long> >& ncs, std::vector< std::vector<long> >& ecs, MOSPPResult* res) ;


} // end namespace search
} // end namespace rzq


#endif  // ZHONGQIANGREN_SEARCH_EMOA_ST_H_
