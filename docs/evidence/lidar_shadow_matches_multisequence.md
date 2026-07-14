# OpenLORIS LiDAR 影子扫描匹配多序列验证

- 数据契约：**PASS**
- 序列数：2
- 发布决策：**shadow_only_improve_geometric_verification**
- 平均 accepted precision：21.10%
- 平均 conditional recall：33.49%

| sequence | scored | true pair | accepted | precision | conditional recall | median trans. error | median yaw error |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| corridor1-1 | 3763 | 105 | 333 | 8.41% | 26.67% | 0.688 m | 2.08° |
| corridor1-2 | 1075 | 62 | 74 | 33.78% | 40.32% | 1.436 m | 1.89° |

> 结论边界：当前结果只作为 shadow evidence；直接图边写入始终关闭。
