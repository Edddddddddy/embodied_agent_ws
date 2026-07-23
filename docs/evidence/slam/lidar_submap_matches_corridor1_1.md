# corridor1-1 LiDAR shadow scan matching

- 数据契约：**PASS**
- 已评分 pair：3763
- 真回环候选 pair：105

| Profile | Accepted | Precision | Conditional pair recall | Query recall | Median translation error |
| --- | ---: | ---: | ---: | ---: | ---: |
| cpp_default | 203 | 10.34% | 20.00% | 25.00% | 0.424 m |
| balanced_shadow | 32 | 3.12% | 0.95% | 1.56% | 0.250 m |
| conservative_shadow | 1 | 0.00% | 0.00% | 0.00% | n/a |
| cpp_ranked_single | 119 | 9.24% | 10.48% | 17.19% | 0.475 m |
| cpp_temporal | 8 | 12.50% | 0.95% | 1.56% | 1.254 m |
| cpp_sequence_multi_hypothesis | 6 | 33.33% | 1.90% | 3.12% | 0.954 m |

- balanced shadow 事件恢复：1/2

> 边界：配准结果只在 shadow 报告中评分，没有向 Ceres/GTSAM 位姿图写入边；真值仅用于离线标注。
