#ifndef NFD_DAEMON_FW_LSF_HPP
#define NFD_DAEMON_FW_LSF_HPP

#include "ns3/ndnSIM/NFD/daemon/fw/strategy.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/process-nack-traits.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/retx-suppression-exponential.hpp"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/vector.h"
#include <map>

namespace nfd {
namespace fw {

class LSF : public Strategy
                         , public ProcessNackTraits<LSF>
{
public:
  explicit
  LSF(Forwarder& forwarder, const Name& name = getStrategyName());

  static const Name&
  getStrategyName();

  void
  afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                       const shared_ptr<pit::Entry>& pitEntry) override;

  // void
  // afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
  //                  const shared_ptr<pit::Entry>& pitEntry) override;

  void 
  afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data) override;

  void
  afterContentStoreHit(const shared_ptr<pit::Entry>& pitEntry,
                                const FaceEndpoint& ingress, const Data& data) override; 

  void 
  initial(uint32_t num, std::map<std::string, std::vector<int>>& isr);

  void 
  updateISR(const FaceEndpoint& ingress,
                    const Interest& interest,
                    int type,
                    std::map<std::string, std::vector<int>>& isr);

  std::vector<double>
  caculateHopProb(const fib::NextHopList& nexthoplist,
									            const FaceEndpoint& ingress,
                              const Interest& interest,
                              const shared_ptr<pit::Entry>& pitEntry);

  int 
  getBestHop(const fib::NextHopList& nexthoplist,
									    const FaceEndpoint& ingress,
                      const Interest& interest,
                      const shared_ptr<pit::Entry>& pitEntry);

  void 
  probSend(const fib::NextHopList& nexthoplist,
									const FaceEndpoint& ingress,
                  const Interest& interest,
                  const shared_ptr<pit::Entry>& pitEntry,
                  int best_hop_index);

  std::map<std::string, std::vector<int>> 
  getISR(){
    return m_isr;
  }

  std::map<uint32_t, std::vector<int>>& 
  getHOP(){
    return m_hop;
  }

  void
  setHopList(uint32_t nonce, int nodeId,std::map<uint32_t, std::vector<int>>& hop,  bool isinitial);

  void
  updateHopList(int nodeId, const Interest& interest);

  void
  sendPosition();
  void 
  updatePos(const FaceEndpoint& ingress, const Interest& interest);
// int rouletteWheelSelection(const std::vector<double>& probabilities);

// int getBestProb(const std::vector<double>& problist);

// nfd::fib::NextHopList::const_iterator 
// getBestHop(const fib::NextHopList& nexthops,
// 									const FaceEndpoint& ingress,
//                                		const Interest& interest,
//                                		const shared_ptr<pit::Entry>& pitEntry);
private:
  std::vector<std::vector<ns3::Vector3D>> m_posMap;
  std::vector<std::vector<ns3::Vector3D>> m_volMap;
  std::map<std::string, std::vector<int>> m_isr;
  std::map<uint32_t, std::vector<int>> m_hop;
  std::vector<std::vector<double>> m_prob;
  double m_Rth;
  double m_probtime;
  ns3::NodeContainer m_nodes;
  uint32_t m_num;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  static const time::milliseconds RETX_SUPPRESSION_INITIAL;
  static const time::milliseconds RETX_SUPPRESSION_MAX;
  RetxSuppressionExponential m_retxSuppression;

  friend ProcessNackTraits<LSF>;

  // Forwarder& fw;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_LSF_HPP
