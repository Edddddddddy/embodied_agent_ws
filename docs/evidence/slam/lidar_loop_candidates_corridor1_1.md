# corridor1-1 LiDAR 回环候选检索

- 数据契约：**PASS**
- 长回访 query：191
- 事件：2
- 环键检索 MRR：0.1251
- 完整相似度重排 MRR：0.0930

| K | ring-key recall | similarity-rerank recall | ring-key precision | rerank precision |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 6.81% | 3.66% | 3.28% | 1.78% |
| 5 | 19.37% | 18.32% | 2.95% | 2.70% |
| 10 | 33.51% | 24.08% | 2.79% | 2.40% |

- 最大 K 事件召回：2/2 (100.00%)

> 边界：候选检索发生在 scan matcher 之前；候选命中不等于约束被接受，真值只用于离线评分。
