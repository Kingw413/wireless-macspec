#ifndef NFD_DAEMON_FW_PRFS_HPP
#define NFD_DAEMON_FW_PRFS_HPP

#include "ns3/ndnSIM/NFD/daemon/fw/strategy.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/process-nack-traits.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/retx-suppression-exponential.hpp"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/vector.h"
namespace nfd {
namespace fw {

class PRFS : public Strategy
                         , public ProcessNackTraits<PRFS>
{
public:
  explicit
  PRFS(Forwarder& forwarder, const Name& name = getStrategyName());

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

  nfd::fib::NextHopList::const_iterator 
  getBestHop(const fib::NextHopList& nexthops,
									    const FaceEndpoint& ingress,
                      const Interest& interest,
                      const shared_ptr<pit::Entry>& pitEntry,
                      int road_direction);

  // bool
  // isNext(const nfd::face::Face& face, int type);
  bool
  clarifyDirection(int node, int remote_node);
  double 
  caculateDR(ns3::Vector3D nodePos, ns3::Vector3D remotePos, ns3::Vector3D direction);

private: 
double m_Rth;
  ns3::NodeContainer m_nodes;
  uint32_t m_num;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  static const time::milliseconds RETX_SUPPRESSION_INITIAL;
  static const time::milliseconds RETX_SUPPRESSION_MAX;
  RetxSuppressionExponential m_retxSuppression;

  friend ProcessNackTraits<PRFS>;

  Forwarder& fw;


};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_PRFS_HPP