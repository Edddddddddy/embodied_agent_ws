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

