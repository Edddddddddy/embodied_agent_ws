from embodied_offline_agent.latency import OfflineLatency


def test_report_never_claims_unmeasured_targets():
    report = OfflineLatency().report(0, 0)
    assert report["asr_target_met"] is False
    assert report["e2e_target_met"] is False
