# corridor1-2 LiDAR 回环候选检索

- 数据契约：**PASS**
- 长回访 query：51
- 事件：2
- 环键检索 MRR：0.1931
- 完整相似度重排 MRR：0.1500

| K | ring-key recall | similarity-rerank recall | ring-key precision | rerank precision |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 7.84% | 5.88% | 3.57% | 2.68% |
| 5 | 33.33% | 27.45% | 4.91% | 3.82% |
| 10 | 62.75% | 45.10% | 5.77% | 3.63% |

- 最大 K 事件召回：2/2 (100.00%)

> 边界：候选检索发生在 scan matcher 之前；候选命中不等于约束被接受，真值只用于离线评分。
