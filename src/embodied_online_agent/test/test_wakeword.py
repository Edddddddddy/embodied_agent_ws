from embodied_online_agent.wakeword import WakeWordGate


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

