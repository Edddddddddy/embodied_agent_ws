# 同一条自然语言可能拆成多个动作；该上下文用于跨队列、执行器和监控关联整批命令。
string batch_id
uint32 batch_index
uint32 batch_size
string source_text
string nlu_intent
float32 nlu_confidence
