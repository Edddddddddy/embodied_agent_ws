#!/usr/bin/env bash

# 真实语音入口共享的部署 profile。这里只计算稳定默认值；现场显式环境变量仍由调用方覆盖。
apply_voice_control_profile_defaults() {
  local scenario="${1:?scenario is required}"
  local profile="${2:?profile is required}"

  PROFILE_SESSION_TIMEOUT=60
  PROFILE_COMMAND_QUEUE_SIZE=8
  PROFILE_COMMAND_MAX_AGE=30
  PROFILE_DUPLICATE_WINDOW_S=1.2
  PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.82
  PROFILE_SPEECH_START_THRESHOLD=0.018
  PROFILE_VAD_SPEECH_START_MS=96
  PROFILE_SPEECH_END_SILENCE_S=0.7
  PROFILE_MIN_UTTERANCE_MS=100
  PROFILE_MAX_UTTERANCE_S=12.0
  PROFILE_ASR_COMMIT_DELAY_MS=300
  unset PROFILE_AEC_ENABLED || true

  if [[ "$scenario" == "navigation" ]]; then
    PROFILE_SESSION_TIMEOUT=150
    PROFILE_COMMAND_QUEUE_SIZE=6
    PROFILE_COMMAND_MAX_AGE=180
    PROFILE_SPEECH_END_SILENCE_S=0.75
  elif [[ "$scenario" != "control" ]]; then
    echo "unknown voice profile scenario=$scenario; expected control or navigation" >&2
    return 2
  fi

  case "$profile" in
    normal) ;;
    quiet)
      PROFILE_SESSION_TIMEOUT=$([[ "$scenario" == "navigation" ]] && echo 180 || echo 75)
      PROFILE_COMMAND_QUEUE_SIZE=$([[ "$scenario" == "navigation" ]] && echo 8 || echo 10)
      PROFILE_SPEECH_START_THRESHOLD=0.014
      PROFILE_VAD_SPEECH_START_MS=64
      PROFILE_SPEECH_END_SILENCE_S=$([[ "$scenario" == "navigation" ]] && echo 0.65 || echo 0.6)
      PROFILE_MIN_UTTERANCE_MS=80
      ;;
    low_gain)
      PROFILE_SESSION_TIMEOUT=$([[ "$scenario" == "navigation" ]] && echo 180 || echo 75)
      PROFILE_COMMAND_QUEUE_SIZE=$([[ "$scenario" == "navigation" ]] && echo 8 || echo 10)
      PROFILE_SPEECH_START_THRESHOLD=0.0012
      PROFILE_SPEECH_END_SILENCE_S=$([[ "$scenario" == "navigation" ]] && echo 0.85 || echo 0.8)
      PROFILE_MIN_UTTERANCE_MS=120
      PROFILE_ASR_COMMIT_DELAY_MS=450
      PROFILE_AEC_ENABLED=false
      ;;
    noisy_room)
      PROFILE_SESSION_TIMEOUT=$([[ "$scenario" == "navigation" ]] && echo 120 || echo 45)
      PROFILE_COMMAND_QUEUE_SIZE=5
      if [[ "$scenario" == "control" ]]; then PROFILE_COMMAND_MAX_AGE=20; fi
      PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.78
      PROFILE_SPEECH_START_THRESHOLD=0.026
      PROFILE_VAD_SPEECH_START_MS=160
      PROFILE_SPEECH_END_SILENCE_S=$([[ "$scenario" == "navigation" ]] && echo 0.9 || echo 0.85)
      PROFILE_MIN_UTTERANCE_MS=180
      PROFILE_MAX_UTTERANCE_S=10.0
      ;;
    *)
      echo "unknown VOICE_CONTROL_PROFILE=$profile; expected normal, quiet, low_gain, or noisy_room" >&2
      return 2
      ;;
  esac
}
