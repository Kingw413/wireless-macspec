#ifndef NFD_DAEMON_FW_LSF_HPP
#define NFD_DAEMON_FW_LSF_HPP

#include "ns3/ndnSIM/NFD/daemon/fw/strategy.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/process-nack-traits.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/retx-suppression-exponential.hpp"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/vector.h"

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

  void
  afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                   const shared_ptr<pit::Entry>& pitEntry) override;

  void
  sendPosition();
  void 
  initialPosition();

  void 
  updatePos(const FaceEndpoint& ingress, const Interest& interest);

private:
  std::vector<std::vector<ns3::Vector3D>> m_posMap;
  std::vector<std::vector<ns3::Vector3D>> m_volMap;
  ns3::NodeContainer m_nodes = ns3::NodeContainer::GetGlobal();
  double m_probtime;
  uint32_t num;

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
