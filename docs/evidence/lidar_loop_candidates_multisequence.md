# OpenLORIS LiDAR 回环候选多序列验证

- 数据与公平性检查：**PASS**
- 序列数：2
- 决策：**ready_for_shadow_scan_match_integration**
- 平均 Recall@10：48.13%
- 平均 Precision@10：4.28%

| sequence | scans | eligible query | ring-key R@10 | rerank R@10 | P@10 | event recall |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| corridor1-1 | 542 | 191 | 33.51% | 24.08% | 2.79% | 100.00% |
| corridor1-2 | 232 | 51 | 62.75% | 45.10% | 5.77% | 100.00% |

> 结论边界：当前只允许进入 shadow scan-matcher 集成，不允许把候选直接写入位姿图。
