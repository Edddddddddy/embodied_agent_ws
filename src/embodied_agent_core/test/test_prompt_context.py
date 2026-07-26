from pathlib import Path

import pytest

from embodied_agent_core.memory import ConversationMemory
from embodied_agent_core.prompt_context import (
    PromptBudgetExceeded,
    PromptContextAssembler,
    RagQueryRouter,
    SparseKnowledgeRetriever,
    conservative_token_estimate,
    prompt_context_from_parameters,
)


class _UserContext:
    @staticmethod
    def system_prompt(base: str) -> str:
        return base + "\n用户偏好：慢速。"


def _knowledge(tmp_path: Path) -> Path:
    path = tmp_path / "manual.md"
    path.write_text(
        """
# 离线部署

离线语音使用 sherpa-onnx Zipformer ASR、llama.cpp Qwen3 GGUF 和 Sherpa-TTS。
llama-server 应限制上下文和并发，并在启动前检查 health。

# 安全控制

运动命令必须绕过 RAG，由本地 NLU 生成候选动作，再经过 C++ ActionGuard。
急停会清空普通队列并发布最高优先级 STOP。
""".strip(),
        encoding="utf-8",
    )
    return path


def test_adaptive_router_keeps_robot_commands_on_zero_retrieval_path():
    router = RagQueryRouter(policy="adaptive")

    assert router.route("向前走一秒") == "control_bypass"
    assert router.route("先去门口再回到起点") == "control_bypass"
    assert router.route("离线语音怎么部署") == "knowledge"
    assert router.route("你好") == "conversation"


def test_sparse_retriever_returns_bounded_cited_chinese_context(tmp_path):
    retriever = SparseKnowledgeRetriever.from_paths(
        [_knowledge(tmp_path)], chunk_chars=120, overlap_chars=20
    )

    result = retriever.retrieve(
        "llama.cpp 离线模型怎么部署",
        top_k=2,
        max_chars=90,
        min_score=0.01,
    )

    assert result.route == "knowledge"
    assert result.hits
    assert result.retrieved_chars <= 90
    assert result.hits[0].source_id.startswith("manual.md@")
    assert "llama" in result.hits[0].content
    assert result.corpus_version.startswith("sha256-")


def test_corpus_version_changes_when_equal_length_content_changes(tmp_path):
    path = tmp_path / "manual.md"
    path.write_text("# 部署\n使用甲方案。", encoding="utf-8")
    first = SparseKnowledgeRetriever.from_paths([path]).retrieve("部署")
    path.write_text("# 部署\n使用乙方案。", encoding="utf-8")
    second = SparseKnowledgeRetriever.from_paths([path]).retrieve("部署")

    assert first.corpus_version != second.corpus_version


def test_same_filename_with_different_content_has_distinct_source_ids(tmp_path):
    first_path = tmp_path / "first" / "manual.md"
    second_path = tmp_path / "second" / "manual.md"
    first_path.parent.mkdir()
    second_path.parent.mkdir()
    first_path.write_text("# 部署\n甲方案。", encoding="utf-8")
    second_path.write_text("# 部署\n乙方案。", encoding="utf-8")

    first = SparseKnowledgeRetriever.from_paths([first_path]).retrieve("部署")
    second = SparseKnowledgeRetriever.from_paths([second_path]).retrieve("部署")

    assert first.hits[0].source_id != second.hits[0].source_id


def test_prompt_assembler_injects_evidence_only_for_knowledge_questions(tmp_path):
    memory = ConversationMemory(str(tmp_path / "memory.json"), max_turns=2)
    retriever = SparseKnowledgeRetriever.from_paths([_knowledge(tmp_path)])
    assembler = PromptContextAssembler(
        memory=memory,
        base_system_prompt="只输出可信回答。",
        retriever=retriever,
        enabled=True,
        top_k=1,
        max_context_chars=300,
        min_score=0.01,
    )

    question = assembler.build("离线语音怎么部署？", _UserContext())
    command = assembler.build("向前走一秒", _UserContext())

    assert question.retrieval.hits
    assert "[本地知识检索上下文]" in question.model_user_text
    assert question.retrieval.source_ids[0] in question.model_user_text
    assert question.actions_allowed is False
    assert question.model_output_for_history(
        assistant_text="本地部署。",
        model_output=(
            '<speech>本地部署。</speech>'
            '<action>{"name":"move","arguments":{}}</action>'
        ),
    ) == "<speech>本地部署。</speech>"
    assert question.citation_metrics("依据无关来源回答。")["citation_valid"] is False
    assert question.citation_metrics(
        f"依据 {question.retrieval.source_ids[0]} 回答。"
    )["citation_valid"] is True
    assert question.messages[0] == {
        "role": "system",
        "content": "只输出可信回答。\n用户偏好：慢速。",
    }
    assert command.retrieval.route == "control_bypass"
    assert command.actions_allowed is True
    assert command.retrieval.hits == ()
    assert command.citation_metrics("好的。")["citation_valid"] is True
    assert command.model_user_text == "向前走一秒"


def test_prompt_assembler_can_run_without_rag_assets(tmp_path):
    memory = ConversationMemory(str(tmp_path / "memory.json"), max_turns=1)
    assembler = PromptContextAssembler(
        memory=memory,
        base_system_prompt="system",
        retriever=None,
        enabled=False,
    )

    result = assembler.build("什么是 RAG？", _UserContext())

    assert result.retrieval.route == "conversation"
    assert result.messages[-1]["content"] == "什么是 RAG？"


def test_knowledge_miss_instructs_model_to_abstain(tmp_path):
    memory = ConversationMemory(str(tmp_path / "memory.json"), max_turns=1)
    assembler = PromptContextAssembler(
        memory=memory,
        base_system_prompt="system",
        retriever=SparseKnowledgeRetriever.from_paths([_knowledge(tmp_path)]),
        min_score=1000.0,
    )

    result = assembler.build("天气预报怎么查询", _UserContext())

    assert result.retrieval.route == "knowledge_no_hit"
    assert result.retrieval.hits == ()
    assert "没有找到可引用的本地证据" in result.model_user_text


def test_prompt_metrics_do_not_copy_query_or_knowledge_body(tmp_path):
    memory = ConversationMemory(str(tmp_path / "memory.json"), max_turns=1)
    assembler = PromptContextAssembler(
        memory=memory,
        base_system_prompt="system",
        retriever=SparseKnowledgeRetriever.from_paths([_knowledge(tmp_path)]),
        min_score=0.01,
    )

    result = assembler.build("离线部署有什么要求", _UserContext())
    metrics = result.metrics()

    assert metrics["route"] == "knowledge"
    assert metrics["retrieval_top_k"] >= 1
    assert set(metrics) == {
        "route",
        "retrieval_ms",
        "retrieval_top_k",
        "retrieved_chars",
        "source_ids",
        "corpus_version",
        "context_window_tokens",
        "llm_max_tokens",
        "prompt_safety_reserve_tokens",
        "prompt_input_budget_tokens",
        "prompt_original_tokens",
        "prompt_estimated_tokens",
        "prompt_budget_met",
        "history_messages_dropped",
        "history_turns_dropped",
        "rag_chars_trimmed",
        "rag_context_trimmed",
    }


def test_conservative_estimator_and_injected_counter_need_no_tokenizer(tmp_path):
    assert conservative_token_estimate("中文 abc") >= 6
    calls = []

    def counter(text):
        calls.append(text)
        return len(text)

    assembler = PromptContextAssembler(
        memory=ConversationMemory(str(tmp_path / "memory.json")),
        base_system_prompt="system",
        token_counter=counter,
        llm_context_window_tokens=128,
        llm_max_tokens=16,
        prompt_safety_reserve_tokens=16,
    )

    result = assembler.build("你好", _UserContext())

    assert calls
    assert result.budget.input_budget_tokens == 96
    assert result.budget.budget_met is True


def test_budget_discards_oldest_complete_turns_before_current_rag(tmp_path):
    memory = ConversationMemory(str(tmp_path / "memory.json"), max_turns=5)
    memory.append_turn("最旧问题" * 12, "最旧回答" * 12)
    memory.append_turn("最近问题", "最近回答")
    assembler = PromptContextAssembler(
        memory=memory,
        base_system_prompt="系统安全约束",
        retriever=SparseKnowledgeRetriever.from_paths([_knowledge(tmp_path)]),
        min_score=0.01,
        top_k=1,
        max_context_chars=180,
        token_counter=len,
        llm_context_window_tokens=420,
        llm_max_tokens=20,
        prompt_safety_reserve_tokens=20,
    )

    result = assembler.build("离线语音怎么部署？", _UserContext())
    contents = [message["content"] for message in result.messages]

    assert not any("最旧问题" in content for content in contents)
    assert not any("最旧回答" in content for content in contents)
    assert any("最近问题" in content for content in contents)
    assert result.model_user_text.startswith("离线语音怎么部署？")
    assert result.budget.history_messages_dropped == 2
    assert result.budget.history_turns_dropped == 1
    assert result.budget.rag_context_trimmed is False
    assert result.budget.budget_met is True


def test_small_context_trims_history_then_rag_but_keeps_system_and_question(tmp_path):
    memory = ConversationMemory(str(tmp_path / "memory.json"), max_turns=3)
    memory.append_turn("历史用户问题" * 10, "历史模型回答" * 10)
    assembler = PromptContextAssembler(
        memory=memory,
        base_system_prompt="不可覆盖的系统安全约束",
        retriever=SparseKnowledgeRetriever.from_paths([_knowledge(tmp_path)]),
        min_score=0.01,
        top_k=2,
        max_context_chars=400,
        token_counter=len,
        llm_context_window_tokens=110,
        llm_max_tokens=20,
        prompt_safety_reserve_tokens=20,
    )

    question = "离线语音怎么部署？"
    result = assembler.build(question, _UserContext())

    assert result.messages[0]["role"] == "system"
    assert "不可覆盖的系统安全约束" in result.messages[0]["content"]
    assert result.messages[-1] == {"role": "user", "content": question}
    assert len(result.messages) == 2
    assert result.model_user_text == question
    assert result.retrieval.route == "knowledge"
    assert result.retrieval.hits == ()
    assert result.budget.history_turns_dropped == 1
    assert result.budget.rag_chars_trimmed > 0
    assert result.budget.rag_context_trimmed is True
    assert result.budget.budget_met is True


def test_budget_never_truncates_system_or_current_question_on_overflow(tmp_path):
    system = "系统约束" * 30
    question = "当前用户问题" * 20
    assembler = PromptContextAssembler(
        memory=ConversationMemory(str(tmp_path / "memory.json")),
        base_system_prompt=system,
        token_counter=len,
        llm_context_window_tokens=80,
        llm_max_tokens=16,
        prompt_safety_reserve_tokens=16,
    )

    result = assembler.build(question, _UserContext())

    assert result.messages[0]["content"].startswith(system)
    assert result.messages[-1]["content"] == question
    assert result.budget.budget_met is False
    with pytest.raises(PromptBudgetExceeded, match="configured budget"):
        result.require_budget()


def test_no_hit_knowledge_turn_keeps_abstention_rule_when_budget_is_tight(tmp_path):
    assembler = PromptContextAssembler(
        memory=ConversationMemory(str(tmp_path / "memory.json")),
        base_system_prompt="系统约束",
        retriever=SparseKnowledgeRetriever.from_paths([_knowledge(tmp_path)]),
        min_score=999.0,
        token_counter=len,
        llm_context_window_tokens=110,
        llm_max_tokens=20,
        prompt_safety_reserve_tokens=20,
    )

    result = assembler.build("未知模块如何部署？", _UserContext())

    assert result.retrieval.route == "knowledge_no_hit"
    assert result.retrieval.hits == ()
    assert "材料不足时请明确说明，不要编造" in result.model_user_text
    assert result.messages[-1]["content"] == result.model_user_text
    assert result.budget.budget_met is False
    with pytest.raises(PromptBudgetExceeded):
        result.require_budget()


def test_parameter_factory_loads_packaged_prompt_and_default_knowledge(tmp_path):
    share = tmp_path / "share"
    (share / "prompts").mkdir(parents=True)
    (share / "knowledge").mkdir()
    (share / "prompts" / "system_prompt_zh.txt").write_text(
        "系统安全约束", encoding="utf-8"
    )
    (share / "knowledge" / "robot_runtime_zh.md").write_text(
        "# 部署\n离线运行使用 llama.cpp。", encoding="utf-8"
    )
    values = {
        "system_prompt_path": "",
        "rag_enabled": True,
        "rag_cloud_context_policy": "allow_custom",
        "rag_knowledge_paths": [""],
        "rag_chunk_chars": 600,
        "rag_chunk_overlap_chars": 80,
        "rag_query_policy": "adaptive",
        "rag_top_k": 2,
        "rag_max_context_chars": 300,
        "rag_min_score": 0.01,
        "llm_context_window_tokens": 4096,
        "llm_max_tokens": 192,
        "prompt_safety_reserve_tokens": 256,
    }
    assembler = prompt_context_from_parameters(
        memory=ConversationMemory(str(tmp_path / "memory.json")),
        param=values.__getitem__,
        package_share=share,
    )

    result = assembler.build("离线怎么部署", _UserContext())

    assert result.messages[0]["content"].startswith("系统安全约束")
    assert result.retrieval.hits


def test_parameter_factory_requires_opt_in_before_custom_cloud_context(tmp_path):
    share = tmp_path / "share"
    (share / "prompts").mkdir(parents=True)
    (share / "knowledge").mkdir()
    (share / "prompts" / "system_prompt_zh.txt").write_text(
        "系统安全约束", encoding="utf-8"
    )
    (share / "knowledge" / "robot_runtime_zh.md").write_text(
        "# 部署\n公开手册。", encoding="utf-8"
    )
    custom = tmp_path / "private.md"
    custom.write_text("# 内部\n敏感部署说明。", encoding="utf-8")
    values = {
        "system_prompt_path": "",
        "rag_enabled": True,
        "rag_cloud_context_policy": "builtin_only",
        "rag_knowledge_paths": [str(custom)],
        "rag_chunk_chars": 600,
        "rag_chunk_overlap_chars": 80,
        "rag_query_policy": "adaptive",
        "rag_top_k": 2,
        "rag_max_context_chars": 300,
        "rag_min_score": 0.01,
        "llm_context_window_tokens": 4096,
        "llm_max_tokens": 192,
        "prompt_safety_reserve_tokens": 256,
    }

    with pytest.raises(ValueError, match="allow_custom"):
        prompt_context_from_parameters(
            memory=ConversationMemory(str(tmp_path / "memory.json")),
            param=values.__getitem__,
            package_share=share,
        )
