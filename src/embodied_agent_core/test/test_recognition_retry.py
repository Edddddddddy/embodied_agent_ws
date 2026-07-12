from embodied_agent_core.recognition_retry import RecognitionRetryTracker


def test_failed_recognition_retries_and_wraps_without_lockout():
    tracker = RecognitionRetryTracker(max_attempts=2)

    assert tracker.failed("小紫").attempt == 1
    assert tracker.failed("晓子").attempt == 2
    assert tracker.failed("小治").attempt == 1


def test_success_resets_retry_counter_and_feedback_is_json():
    tracker = RecognitionRetryTracker(max_attempts=3)
    tracker.failed("噪声")
    tracker.succeeded()

    payload = tracker.failed("还是噪声").as_dict()
    assert payload["status"] == "retry"
    assert payload["reason"] == "wake_word_not_detected"
    assert payload["attempt"] == 1
    assert "再说一次" in payload["prompt"]
