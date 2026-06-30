import threading

from embodied_offline_agent.double_buffer import DoubleBuffer


def test_drop_oldest_keeps_latest_two():
    buffer = DoubleBuffer[int](drop_oldest=True)
    assert buffer.put(1)
    assert buffer.put(2)
    assert buffer.put(3)
    assert buffer.get() == 2
    assert buffer.get() == 3
    assert buffer.stats.dropped == 1
    assert buffer.stats.high_watermark == 2


def test_lossless_close_drains_before_stop():
    buffer = DoubleBuffer[str](drop_oldest=False)
    values = []

    def consume():
        try:
            while True:
                values.append(buffer.get())
        except StopIteration:
            pass

    thread = threading.Thread(target=consume)
    thread.start()
    buffer.put("a")
    buffer.put("b")
    buffer.close()
    thread.join(timeout=1.0)
    assert values == ["a", "b"]
    assert not thread.is_alive()


def test_abort_discards_and_wakes_consumer():
    buffer = DoubleBuffer[int](drop_oldest=False)
    buffer.put(1)
    buffer.abort()
    try:
        buffer.get()
        assert False, "abort must stop the consumer"
    except StopIteration:
        pass
