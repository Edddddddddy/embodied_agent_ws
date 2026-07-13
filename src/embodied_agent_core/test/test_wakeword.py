from embodied_agent_core.wakeword import WakeWordGate


class Clock:
    value = 0.0

    def __call__(self):
        return self.value


def test_wake_word_is_removed_and_followup_is_allowed():
    clock = Clock()
    gate = WakeWordGate(["小智"], active_timeout_s=10.0, clock=clock)
    assert gate.process("小智，向前走") == "向前走"
    clock.value = 5.0
    assert gate.process("然后停下") == "然后停下"
    clock.value = 16.0
    assert gate.process("再挥手") is None


def test_disabled_gate_accepts_text():
    assert WakeWordGate([], enabled=False).process("直接执行") == "直接执行"


def test_configured_homophone_alias_activates_gate():
    gate = WakeWordGate(["小智"], aliases=["小志", "小治", "晓智"])
    assert gate.process("小志向前走一秒") == "向前走一秒"


def test_longest_wake_phrase_is_removed_first():
    gate = WakeWordGate(["小智", "你好小智"])
    assert gate.process("你好小智，停下") == "停下"
