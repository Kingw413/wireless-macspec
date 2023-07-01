#ifndef NFD_DAEMON_FW_FLOOD_HPP
#define NFD_DAEMON_FW_FLOD_HPP

#include "ns3/ndnSIM/NFD/daemon/fw/strategy.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/process-nack-traits.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/retx-suppression-exponential.hpp"
#include "ns3/node-container.h"

namespace nfd {
namespace fw {

/** \brief a forwarding strategy that forwards Interest to all FIB nexthops
 *
 *  \note This strategy is not EndpointId-aware.
 */
class FLOOD : public Strategy
                        , public ProcessNackTraits<FLOOD>
{
public:
  explicit
  FLOOD(Forwarder& forwarder, const Name& name = getStrategyName());

  static const Name&
  getStrategyName();

  void
  afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                       const shared_ptr<pit::Entry>& pitEntry) override;
  void 
  afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data) override;
  void
  afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                   const shared_ptr<pit::Entry>& pitEntry) override;
  bool
  isInRegion(nfd::fib::NextHop hop);

  std::map<uint32_t, std::vector<int>>& 
  getHOP(){
    return m_hop;
  }

  void
  setHopList(uint32_t nonce, std::map<uint32_t, std::vector<int>>&, std::map<uint32_t, std::vector<int>>& hop, int hopId, int next_hopId);

  void
  updateHopList(nfd::face::Face& inface, nfd::face::Face& outface, const Interest& interest);

  void
  getHopCounts(const Interest& interest,
                           		 ns3::Ptr<ns3::Node> node);
private:
  friend ProcessNackTraits<FLOOD>;
  RetxSuppressionExponential m_retxSuppression;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  static const time::milliseconds RETX_SUPPRESSION_INITIAL;
  static const time::milliseconds RETX_SUPPRESSION_MAX;

private: 
ns3::NodeContainer m_nodes;
double m_Rth;
std::map<uint32_t, std::vector<int>> m_hop;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_MULTICAST_STRATEGY_HPP
