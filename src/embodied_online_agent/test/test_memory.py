import json

from embodied_online_agent.memory import ConversationMemory


def test_memory_persists_and_bounds_turns(tmp_path):
    path = tmp_path / "memory.json"
    memory = ConversationMemory(str(path), max_turns=2)
    memory.append_turn("u1", "a1")
    memory.append_turn("u2", "a2")
    memory.append_turn("u3", "a3")

    loaded = ConversationMemory(str(path), max_turns=2)
    assert loaded.messages() == [
        {"role": "user", "content": "u2"},
        {"role": "assistant", "content": "a2"},
        {"role": "user", "content": "u3"},
        {"role": "assistant", "content": "a3"},
    ]
    assert json.loads(path.read_text(encoding="utf-8")) == loaded.messages()


def test_corrupt_memory_recovers_empty(tmp_path):
    path = tmp_path / "memory.json"
    path.write_text("not-json", encoding="utf-8")
    assert ConversationMemory(str(path)).messages() == []


def test_prompt_messages_include_loaded_history_as_stable_prefix(tmp_path):
    memory = ConversationMemory(str(tmp_path / "memory.json"), max_turns=2)
    memory.append_turn("上一问", "上一答")

    messages = memory.prompt_messages("系统提示", "当前问题")

    assert messages == [
        {"role": "system", "content": "系统提示"},
        {"role": "user", "content": "上一问"},
        {"role": "assistant", "content": "上一答"},
        {"role": "user", "content": "当前问题"},
    ]
    messages[1]["content"] = "外部修改"
    assert memory.messages()[0]["content"] == "上一问"


def test_memory_can_store_exact_model_protocol_for_kv_prefix_reuse(tmp_path):
    memory = ConversationMemory(str(tmp_path / "memory.json"), max_turns=1)

    memory.append_turn(
        "向前走一秒",
        "好的。",
        model_output=(
            '<speech>好的。</speech><action>{"name":"move",'
            '"arguments":{"linear_x":0.2,"duration_s":1.0}}</action>'
        ),
    )

    assert memory.messages()[1]["content"].startswith("<speech>好的。</speech><action>")
