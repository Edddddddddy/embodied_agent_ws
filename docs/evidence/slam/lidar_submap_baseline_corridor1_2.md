# corridor1-2 LiDAR shadow scan matching

- 数据契约：**PASS**
- 已评分 pair：1075
- 真回环候选 pair：62

| Profile | Accepted | Precision | Conditional pair recall | Query recall | Median translation error |
| --- | ---: | ---: | ---: | ---: | ---: |
| cpp_default | 74 | 33.78% | 40.32% | 40.62% | 1.436 m |
| balanced_shadow | 0 | 0.00% | 0.00% | 0.00% | n/a |
| conservative_shadow | 0 | 0.00% | 0.00% | 0.00% | n/a |

- balanced shadow 事件恢复：0/2

> 边界：配准结果只在 shadow 报告中评分，没有向 Ceres/GTSAM 位姿图写入边；真值仅用于离线标注。
