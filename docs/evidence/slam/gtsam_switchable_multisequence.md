# GTSAM 可切换回环约束多序列消融

- 门禁：**PASS**
- switch prior sigma：1.0
- suppression threshold：0.5
- 加权 ATE RMSE：Gaussian 1.6214 m / Cauchy 1.0892 m / Switchable+Cauchy 0.9196 m
- Switchable+Cauchy 相对 Gaussian：-43.28%
- Switchable+Cauchy 相对 Cauchy：-15.57%
- 被压低回环：80/859

| sequence | nodes | constraints | Gaussian ATE | Cauchy ATE | Switch+Cauchy ATE | vs Gaussian | vs Cauchy | switch off/all | min switch |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| corridor1-1 | 1834 | 2751 | 1.8235 | 1.2236 | 1.0323 | -43.39% | -15.63% | 80/858 | 0.0004 |
| corridor1-2 | 488 | 491 | 0.1484 | 0.1484 | 0.1484 | -0.00% | -0.00% | 0/1 | 0.9976 |

> 结论边界：switch 是后端对“前端已接受边”的潜变量权重，不是真值标签；本报告不能证明
> 前端回环 precision 提升，也不能外推到未评测地图。在线配置仍默认关闭。
