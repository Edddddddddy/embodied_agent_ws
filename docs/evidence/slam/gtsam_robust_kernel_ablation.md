# GTSAM 鲁棒核固定图消融证据

- 数据：OpenLORIS `corridor1-1`，派生 SLAM-only bag SHA256
  `54ab373b4d340021ec11c2f817058458b01e6f2cdab840b515556a3640303d84`。
- 固定位姿图 SHA256：`792ecb7d1871b506274423631fdf16da60b52e46aa18eaa1287cddbd4cadbc65`。
- 图规模：1834 个连续节点、2751 条去重约束、0 条悬空边；每组均匹配 1828 个真值位姿。
- 公平性：四组的图哈希、节点数、约束数和匹配位姿数完全相同。

| variant | kernel | 作用范围 | 鲁棒化边 | ATE RMSE | ATE P95 | 1 s RPE RMSE | 终点误差 | ATE vs Gaussian |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| gaussian | none | 无 | 0 | 1.8235 m | 4.7794 m | 0.2103 m | 5.4486 m | 0.00% |
| huber_all | Huber, k=1.345 | 全部边 | 2751 | 1.4081 m | 3.1571 m | 0.1784 m | 3.6858 m | -22.78% |
| huber_loop | Huber, k=1.345 | 非局部边启发式 | 858 | 1.4626 m | 3.3582 m | 0.1740 m | 3.9235 m | -19.79% |
| cauchy_loop | Cauchy, k=1.0 | 非局部边启发式 | 858 | **1.2236 m** | **2.3968 m** | **0.1595 m** | **2.4741 m** | **-32.90%** |

这里的 `loop_only` 使用 `|source_id-target_id| >= 20` 识别“非局部边”，它是后端防护启发式，
不是 Karto 原生 closure 标签。因此可以说“同一已接受图上，Cauchy 对离群非局部约束更不敏感”，
不能说“回环检测 precision 得到提升”。前端质量仍应以原生 closure callback 和独立真值残差评价。
