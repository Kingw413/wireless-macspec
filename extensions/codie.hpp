#ifndef NFD_DAEMON_FW_CODIE_HPP
#define NFD_DAEMON_FW_CODIE_HPP

#include "ns3/ndnSIM/NFD/daemon/fw/strategy.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/process-nack-traits.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/retx-suppression-exponential.hpp"
#include "ns3/node-container.h"

namespace nfd {
namespace fw {

class CODIE : public Strategy
                         , public ProcessNackTraits<CODIE>
{
public:
  explicit
  CODIE(Forwarder& forwarder, const Name& name = getStrategyName());

  static const Name&
  getStrategyName();

  void
  afterReceiveInterest(const FaceEndpoint& ingress, const Interest& interest,
                       const shared_ptr<pit::Entry>& pitEntry) override;

  void
  afterReceiveNack(const FaceEndpoint& ingress, const lp::Nack& nack,
                   const shared_ptr<pit::Entry>& pitEntry) override;

  void
  afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                           const FaceEndpoint& ingress, const Data& data) override;
 bool
 isInRegion(const nfd::face::Face& face);

private:
   double m_Rth;
   ns3::NodeContainer m_nodes;
   uint32_t m_num;
   std::vector<std::map<std::string, int>> m_h;
   std::vector<std::map<std::string, int>> m_ddl;

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  static const time::milliseconds RETX_SUPPRESSION_INITIAL;
  static const time::milliseconds RETX_SUPPRESSION_MAX;
  RetxSuppressionExponential m_retxSuppression;

  friend ProcessNackTraits<CODIE>;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_BEST_ROUTE_STRATEGY2_HPP
