/*******************************************
 * Author: Zhongqiang Richard Ren. 
 * All Rights Reserved. 
 *******************************************/

#ifndef ZHONGQIANGREN_MAPF_MOCBSNB_H_
#define ZHONGQIANGREN_MAPF_MOCBSNB_H_

#include "graph.hpp"
#include "momapf_util.hpp"
#include "boa_st.hpp"
#include "mocbs.hpp"

#include <chrono>

namespace rzq{
namespace mapf{

/**
 * @brief This helps find the next Pareto-optimal in lex order.
 */
typedef std::pair< basic::CostVector, std::vector<long> > CostIndexCombo;
/**
 *
 */
std::ostream& operator<<(std::ostream& os, const CostIndexCombo& n) ;

/**
 * @brief take the Minkowski sum of the vectors in in1 and in2 and return the output via out.
 * mode = 0, init mode;
 * mode = 1, override agent-ri's paths.
 * mode = 2, subtract
 */
int runMinkowskiSum(
  std::unordered_map<long, CostIndexCombo>& in1,
  std::unordered_map<long, basic::CostVector>& in2,
  std::unordered_map<long, CostIndexCombo> *out,
  int mode=0, int ri=-1);

/**
 *
 */
int nondomSubsetKung(
  std::unordered_map<long, CostIndexCombo> *out) ;

/**
 * @brief high-level node for Binary Branching MO-CBS.
 */
struct NBCBSNode {
  long id = -1; // ID of this node.
  long parent = -1; // parent node ID.

  std::vector< PathSet > idvl_paths; // store each agent's individual paths.
  std::vector< std::unordered_map<long, basic::CostVector> > idvl_costs;
    // store each agent's individual Pareto-optimal cost vectors.
  
  std::vector<long> rep_Ids;
  PathSet rep_jpath; // representative joint path
  basic::CostVector rep_cvec; // representative cost vector
  
  CBSConstraint cstr;

  std::list<CostIndexCombo> sorted_costIndex;
  
  /**
   * @brief Compute a lex-sorted Pareto-optimal front.
   */
  void computeSortedPF();

  /**
   * @brief
   */
  void printPF();

  /**
   * @brief Filter sorted_costIndex with the given solution set.
   *   Return true if at least one vector is filtered.
   */
  bool solFilter(std::unordered_map< long, basic::CostVector >& sol_costs);

  /**
   * @brief Return false if no representative. Return true if the representative is set.
   */
  bool setRep();
  /**
   * @brief 
   */
  bool isEmptyPlans();
};

/**
 *
 */
std::ostream& operator<<(std::ostream& os, const NBCBSNode& n) ;



/**
 * Binary Branching MO-CBS with EMOA* as the low-level search.
 * NB = node-based, some outdated name for BB-MO-CBS.
 */
class NBMOCBS_EMOA : public MOCBS_TEMOA
{
public:
  /**
   *
   */
  NBMOCBS_EMOA();
  /**
   *
   */
  virtual ~NBMOCBS_EMOA();

  /**
   * @brief epsilon here is used as epsilon dominance.
   */
  virtual void Search(std::vector<long>& starts, std::vector<long>& goals, double time_limit, double eps) override ;

protected:
  /**
   *
   */
  virtual bool _Init() override;
  /**
   * @brief pop a node from OPEN, should override MO-CBS since high-level node data structure changes.
   */
  virtual NBCBSNode* _PopNode(); 
  /**
   * @brief node *in contains a conflict-free solution, use *in to update solution set.
   */
  virtual void _UpdateSolution(NBCBSNode* n) ;
  /**
   *
   */
  virtual bool _GetAllCstr(int ri, NBCBSNode* n, std::vector<CBSConstraint*> *out) ;
  /**
   *
   */
  virtual int _Lsearch(int ri, NBCBSNode* n, CBSConstraint* cp) ;
  /**
   * 
   */
  virtual CBSConflict _DetectAndSelectConflict(NBCBSNode* n) ;

  std::unordered_map<long, NBCBSNode> _nodes; // all nodes generated during the search.
  std::set< std::pair< basic::CostVector, long> > _open;
  int _sol_id_gen=1;
};


/**
 * 
 */
void DetectAllConflict(std::vector<long>* p1, std::vector<long>* p2, std::vector<CBSConflict>* out) ;


} // end namespace mapf
} // end namespace rzq

#endif  // ZHONGQIANGREN_MAPF_MOCBSNB_H_
