# corridor1-1 LiDAR shadow scan matching

- 数据契约：**PASS**
- 已评分 pair：3763
- 真回环候选 pair：105

| Profile | Accepted | Precision | Conditional pair recall | Query recall | Median translation error |
| --- | ---: | ---: | ---: | ---: | ---: |
| cpp_default | 333 | 8.41% | 26.67% | 29.69% | 0.688 m |
| balanced_shadow | 86 | 10.47% | 8.57% | 6.25% | 0.658 m |
| conservative_shadow | 1 | 0.00% | 0.00% | 0.00% | n/a |

- balanced shadow 事件恢复：1/2

> 边界：配准结果只在 shadow 报告中评分，没有向 Ceres/GTSAM 位姿图写入边；真值仅用于离线标注。
