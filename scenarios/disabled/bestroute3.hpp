#ifndef NFD_DAEMON_FW_BestTest_HPP
#define NFD_DAEMON_FW_BestTest_HPP

#include "ns3/ndnSIM/NFD/daemon/fw/strategy.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/process-nack-traits.hpp"
#include "ns3/ndnSIM/NFD/daemon/fw/retx-suppression-exponential.hpp"

namespace nfd {
namespace fw {

class BestTest : public Strategy
                         , public ProcessNackTraits<BestTest>
{
public:
  explicit
  BestTest(Forwarder& forwarder, const Name& name = getStrategyName());

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

PUBLIC_WITH_TESTS_ELSE_PRIVATE:
  static const time::milliseconds RETX_SUPPRESSION_INITIAL;
  static const time::milliseconds RETX_SUPPRESSION_MAX;
  RetxSuppressionExponential m_retxSuppression;

  friend ProcessNackTraits<BestTest>;
};

} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_BEST_ROUTE_STRATEGY2_HPP
