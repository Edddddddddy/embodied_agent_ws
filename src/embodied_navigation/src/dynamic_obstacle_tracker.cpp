#include "embodied_navigation/dynamic_obstacle_tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace embodied_navigation
{

namespace
{

constexpr double kMinimumDeltaS = 1e-4;
constexpr double kMinimumVariance = 1e-9;
constexpr double kMinimumLikelihood = 1e-12;

struct MotionEstimate
{
  Point2d position;
  Point2d velocity;
};

class MotionEstimator
{
public:
  virtual ~MotionEstimator() = default;
  virtual MotionEstimate update(const Point2d & observation, double timestamp_s) = 0;
  virtual MotionEstimate predict(double timestamp_s) const = 0;
};

class CurrentOnlyEstimator final : public MotionEstimator
{
public:
  CurrentOnlyEstimator(const Point2d & observation, double timestamp_s)
  : position_(observation), timestamp_s_(timestamp_s) {}

  MotionEstimate update(const Point2d & observation, double timestamp_s) override
  {
    position_ = observation;
    timestamp_s_ = timestamp_s;
    return {position_, {0.0, 0.0}};
  }

  MotionEstimate predict(double) const override
  {
    return {position_, {0.0, 0.0}};
  }

private:
  Point2d position_;
  double timestamp_s_{0.0};
};

class SmoothedConstantVelocityEstimator final : public MotionEstimator
{
public:
  SmoothedConstantVelocityEstimator(
    const Point2d & observation, double timestamp_s, double smoothing)
  : position_(observation), timestamp_s_(timestamp_s), smoothing_(std::clamp(smoothing, 0.0, 1.0))
  {
  }

  MotionEstimate update(const Point2d & observation, double timestamp_s) override
  {
    const double delta_s = timestamp_s - timestamp_s_;
    if (delta_s > kMinimumDeltaS) {
      const Point2d measured_velocity{
        (observation.x - position_.x) / delta_s,
        (observation.y - position_.y) / delta_s};
      velocity_.x = smoothing_ * measured_velocity.x + (1.0 - smoothing_) * velocity_.x;
      velocity_.y = smoothing_ * measured_velocity.y + (1.0 - smoothing_) * velocity_.y;
    }
    position_ = observation;
    timestamp_s_ = timestamp_s;
    return {position_, velocity_};
  }

  MotionEstimate predict(double timestamp_s) const override
  {
    const double delta_s = std::max(0.0, timestamp_s - timestamp_s_);
    return {
      {position_.x + velocity_.x * delta_s, position_.y + velocity_.y * delta_s}, velocity_};
  }

private:
  Point2d position_;
  Point2d velocity_;
  double timestamp_s_{0.0};
  double smoothing_{0.65};
};

struct AxisState
{
  double position{0.0};
  double velocity{0.0};
  double p00{1.0};
  double p01{0.0};
  double p10{0.0};
  double p11{1.0};

  void predict(double delta_s, double process_noise, double velocity_decay = 1.0)
  {
    if (delta_s <= 0.0) {
      return;
    }
    const double old_p00 = p00;
    const double old_p01 = p01;
    const double old_p10 = p10;
    const double old_p11 = p11;
    const double decay = std::clamp(velocity_decay, 0.0, 1.0);
    position += velocity * delta_s;
    velocity *= decay;

    // CV 状态为 [position, velocity]；Q 使用白噪声加速度离散模型，协方差会随预测时长增长。
    const double q = std::max(0.0, process_noise);
    const double dt2 = delta_s * delta_s;
    const double dt3 = dt2 * delta_s;
    const double dt4 = dt2 * dt2;
    p00 = old_p00 + delta_s * (old_p01 + old_p10) + dt2 * old_p11 + q * dt4 / 4.0;
    p01 = decay * (old_p01 + delta_s * old_p11) + q * dt3 / 2.0;
    p10 = decay * (old_p10 + delta_s * old_p11) + q * dt3 / 2.0;
    p11 = decay * decay * old_p11 + q * dt2;
  }

  double update(double measurement, double measurement_variance)
  {
    const double variance = std::max(measurement_variance, kMinimumVariance);
    const double innovation = measurement - position;
    const double innovation_variance = std::max(p00 + variance, kMinimumVariance);
    const double gain_position = p00 / innovation_variance;
    const double gain_velocity = p10 / innovation_variance;
    const double old_p00 = p00;
    const double old_p01 = p01;
    position += gain_position * innovation;
    velocity += gain_velocity * innovation;
    p00 = std::max((1.0 - gain_position) * old_p00, kMinimumVariance);
    p01 = (1.0 - gain_position) * old_p01;
    p10 = p01;
    p11 = std::max(p11 - gain_velocity * old_p01, kMinimumVariance);
    constexpr double two_pi = 6.28318530717958647692;
    return std::max(
      std::exp(-0.5 * innovation * innovation / innovation_variance) /
      std::sqrt(two_pi * innovation_variance),
      kMinimumLikelihood);
  }
};

AxisState mix_axis(
  const std::array<AxisState, 2> & sources, const std::array<double, 2> & weights)
{
  AxisState mixed;
  mixed.position = weights[0] * sources[0].position + weights[1] * sources[1].position;
  mixed.velocity = weights[0] * sources[0].velocity + weights[1] * sources[1].velocity;
  mixed.p00 = mixed.p01 = mixed.p10 = mixed.p11 = 0.0;
  for (std::size_t index = 0; index < sources.size(); ++index) {
    const double position_delta = sources[index].position - mixed.position;
    const double velocity_delta = sources[index].velocity - mixed.velocity;
    mixed.p00 += weights[index] * (sources[index].p00 + position_delta * position_delta);
    mixed.p01 += weights[index] * (sources[index].p01 + position_delta * velocity_delta);
    mixed.p10 += weights[index] * (sources[index].p10 + velocity_delta * position_delta);
    mixed.p11 += weights[index] * (sources[index].p11 + velocity_delta * velocity_delta);
  }
  return mixed;
}

class KalmanEstimator final : public MotionEstimator
{
public:
  KalmanEstimator(const Point2d & observation, double timestamp_s, const TrackerConfig & config)
  : timestamp_s_(timestamp_s), measurement_variance_(config.measurement_noise_variance),
    process_noise_(config.process_noise_variance)
  {
    x_.position = observation.x;
    y_.position = observation.y;
    x_.p00 = y_.p00 = std::max(measurement_variance_, kMinimumVariance);
  }

  MotionEstimate update(const Point2d & observation, double timestamp_s) override
  {
    const double delta_s = std::max(0.0, timestamp_s - timestamp_s_);
    x_.predict(delta_s, process_noise_);
    y_.predict(delta_s, process_noise_);
    x_.update(observation.x, measurement_variance_);
    y_.update(observation.y, measurement_variance_);
    timestamp_s_ = std::max(timestamp_s_, timestamp_s);
    return estimate(x_, y_);
  }

  MotionEstimate predict(double timestamp_s) const override
  {
    AxisState predicted_x = x_;
    AxisState predicted_y = y_;
    const double delta_s = std::max(0.0, timestamp_s - timestamp_s_);
    predicted_x.predict(delta_s, process_noise_);
    predicted_y.predict(delta_s, process_noise_);
    return estimate(predicted_x, predicted_y);
  }

private:
  static MotionEstimate estimate(const AxisState & x, const AxisState & y)
  {
    return {{x.position, y.position}, {x.velocity, y.velocity}};
  }

  AxisState x_;
  AxisState y_;
  double timestamp_s_{0.0};
  double measurement_variance_{0.01};
  double process_noise_{0.2};
};

class ImmEstimator final : public MotionEstimator
{
public:
  ImmEstimator(const Point2d & observation, double timestamp_s, const TrackerConfig & config)
  : timestamp_s_(timestamp_s), measurement_variance_(config.measurement_noise_variance),
    process_noise_{config.imm_stationary_process_noise, config.imm_maneuver_process_noise},
    stay_probability_(std::clamp(config.imm_stay_probability, 0.5, 0.999)),
    stationary_velocity_decay_(std::clamp(config.imm_stationary_velocity_decay, 0.0, 1.0))
  {
    for (auto & model : models_) {
      model.x.position = observation.x;
      model.y.position = observation.y;
      model.x.p00 = model.y.p00 = std::max(measurement_variance_, kMinimumVariance);
    }
  }

  MotionEstimate update(const Point2d & observation, double timestamp_s) override
  {
    const double delta_s = std::max(0.0, timestamp_s - timestamp_s_);
    const double switch_probability = 1.0 - stay_probability_;
    const std::array<std::array<double, 2>, 2> transition{{
      {{stay_probability_, switch_probability}},
      {{switch_probability, stay_probability_}}}};
    std::array<double, 2> prior_probability{};
    for (std::size_t target = 0; target < 2; ++target) {
      for (std::size_t source = 0; source < 2; ++source) {
        prior_probability[target] += probabilities_[source] * transition[source][target];
      }
    }

    // IMM 的关键不是末端加权，而是更新前先按 Markov 转移概率交互状态和协方差。
    std::array<ModelState, 2> mixed_models;
    const std::array<AxisState, 2> source_x{{models_[0].x, models_[1].x}};
    const std::array<AxisState, 2> source_y{{models_[0].y, models_[1].y}};
    for (std::size_t target = 0; target < 2; ++target) {
      const double denominator = std::max(prior_probability[target], kMinimumLikelihood);
      const std::array<double, 2> weights{{
        probabilities_[0] * transition[0][target] / denominator,
        probabilities_[1] * transition[1][target] / denominator}};
      mixed_models[target].x = mix_axis(source_x, weights);
      mixed_models[target].y = mix_axis(source_y, weights);
    }
    models_ = mixed_models;

    // 两个模型看到同一观测；创新似然决定本帧更信任低运动还是机动模型。
    std::array<double, 2> posterior{};
    for (std::size_t model = 0; model < 2; ++model) {
      const double velocity_decay = model == 0 ? stationary_velocity_decay_ : 1.0;
      models_[model].x.predict(delta_s, process_noise_[model], velocity_decay);
      models_[model].y.predict(delta_s, process_noise_[model], velocity_decay);
      const double likelihood_x = models_[model].x.update(
        observation.x, measurement_variance_);
      const double likelihood_y = models_[model].y.update(
        observation.y, measurement_variance_);
      posterior[model] = prior_probability[model] * likelihood_x * likelihood_y;
    }
    const double normalizer = posterior[0] + posterior[1];
    if (normalizer > kMinimumLikelihood) {
      probabilities_[0] = posterior[0] / normalizer;
      probabilities_[1] = posterior[1] / normalizer;
    } else {
      probabilities_ = {{0.5, 0.5}};
    }
    timestamp_s_ = std::max(timestamp_s_, timestamp_s);
    return mixed_estimate(models_, probabilities_);
  }

  MotionEstimate predict(double timestamp_s) const override
  {
    auto predicted = models_;
    const double delta_s = std::max(0.0, timestamp_s - timestamp_s_);
    for (std::size_t model = 0; model < predicted.size(); ++model) {
      const double velocity_decay = model == 0 ? stationary_velocity_decay_ : 1.0;
      predicted[model].x.predict(delta_s, process_noise_[model], velocity_decay);
      predicted[model].y.predict(delta_s, process_noise_[model], velocity_decay);
    }
    return mixed_estimate(predicted, probabilities_);
  }

private:
  struct ModelState
  {
    AxisState x;
    AxisState y;
  };

  static MotionEstimate mixed_estimate(
    const std::array<ModelState, 2> & models, const std::array<double, 2> & probabilities)
  {
    MotionEstimate result;
    for (std::size_t index = 0; index < models.size(); ++index) {
      result.position.x += probabilities[index] * models[index].x.position;
      result.position.y += probabilities[index] * models[index].y.position;
      result.velocity.x += probabilities[index] * models[index].x.velocity;
      result.velocity.y += probabilities[index] * models[index].y.velocity;
    }
    return result;
  }

  std::array<ModelState, 2> models_;
  std::array<double, 2> probabilities_{{0.5, 0.5}};
  double timestamp_s_{0.0};
  double measurement_variance_{0.01};
  std::array<double, 2> process_noise_{{0.01, 1.0}};
  double stay_probability_{0.94};
  double stationary_velocity_decay_{0.2};
};

std::unique_ptr<MotionEstimator> make_motion_estimator(
  const Point2d & observation, double timestamp_s, const TrackerConfig & config)
{
  switch (config.motion_model) {
    case MotionModel::CurrentOnly:
      return std::make_unique<CurrentOnlyEstimator>(observation, timestamp_s);
    case MotionModel::ConstantVelocity:
      return std::make_unique<SmoothedConstantVelocityEstimator>(
        observation, timestamp_s, config.velocity_smoothing);
    case MotionModel::Kalman:
      return std::make_unique<KalmanEstimator>(observation, timestamp_s, config);
    case MotionModel::Imm:
      return std::make_unique<ImmEstimator>(observation, timestamp_s, config);
  }
  throw std::invalid_argument("unsupported motion model");
}

}  // namespace

MotionModel motion_model_from_string(const std::string & value)
{
  if (value == "current_only") {
    return MotionModel::CurrentOnly;
  }
  if (value == "constant_velocity" || value == "cv") {
    return MotionModel::ConstantVelocity;
  }
  if (value == "kalman") {
    return MotionModel::Kalman;
  }
  if (value == "imm") {
    return MotionModel::Imm;
  }
  throw std::invalid_argument(
          "motion_model must be one of: current_only, constant_velocity, kalman, imm");
}

const char * to_string(MotionModel model) noexcept
{
  switch (model) {
    case MotionModel::CurrentOnly:
      return "current_only";
    case MotionModel::ConstantVelocity:
      return "constant_velocity";
    case MotionModel::Kalman:
      return "kalman";
    case MotionModel::Imm:
      return "imm";
  }
  return "unknown";
}

class DynamicObstacleTracker::Impl
{
public:
  explicit Impl(TrackerConfig config)
  : config_(std::move(config)) {}

  const std::vector<TrackedObstacle> & update(
    const std::vector<Point2d> & observations, double timestamp_s)
  {
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this, timestamp_s](const auto & track) {
          return timestamp_s - track.output.last_seen_s > config_.track_timeout_s;
        }),
      tracks_.end());

    std::vector<MotionEstimate> predictions;
    std::vector<Point2d> predicted_positions;
    predictions.reserve(tracks_.size());
    predicted_positions.reserve(tracks_.size());
    for (const auto & track : tracks_) {
      predictions.push_back(track.estimator->predict(timestamp_s));
      predicted_positions.push_back(predictions.back().position);
    }
    const auto assignments = assign_gated_observations(
      predicted_positions, observations, config_.association_distance_m,
      config_.association_strategy);
    std::vector<bool> observation_used(observations.size(), false);
    for (std::size_t track_index = 0U; track_index < tracks_.size(); ++track_index) {
      auto & track = tracks_[track_index];
      const auto & predicted = predictions[track_index];
      if (!assignments[track_index]) {
        // 短时遮挡期间继续发布模型预测，但 last_seen 不前移，TTL 到期后仍会删除轨迹。
        track.output.position = predicted.position;
        track.output.velocity = predicted.velocity;
        const double age_s = std::max(0.0, timestamp_s - track.output.last_seen_s);
        track.output.confidence = std::max(
          0.0, track.output.confidence * (1.0 - age_s / std::max(config_.track_timeout_s, 1e-3)));
        continue;
      }

      const std::size_t observation_index = *assignments[track_index];
      const MotionEstimate estimate = track.estimator->update(
        observations[observation_index], timestamp_s);
      track.output.position = estimate.position;
      track.output.velocity = estimate.velocity;
      track.output.last_seen_s = timestamp_s;
      ++track.output.observation_count;
      track.output.confidence = std::min(
        1.0, 0.45 + 0.12 * static_cast<double>(track.output.observation_count));
      observation_used[observation_index] = true;
    }

    for (std::size_t index = 0; index < observations.size(); ++index) {
      if (observation_used[index]) {
        continue;
      }
      InternalTrack track;
      track.output.id = "track_" + std::to_string(next_track_id_++);
      track.output.position = observations[index];
      track.output.radius = config_.default_radius_m;
      track.output.last_seen_s = timestamp_s;
      track.output.observation_count = 1U;
      track.estimator = make_motion_estimator(observations[index], timestamp_s, config_);
      tracks_.push_back(std::move(track));
    }

    output_cache_.clear();
    output_cache_.reserve(tracks_.size());
    for (const auto & track : tracks_) {
      output_cache_.push_back(track.output);
    }
    return output_cache_;
  }

  const std::vector<TrackedObstacle> & tracks() const
  {
    return output_cache_;
  }

  void clear()
  {
    tracks_.clear();
    output_cache_.clear();
    next_track_id_ = 1U;
  }

private:
  struct InternalTrack
  {
    TrackedObstacle output;
    std::unique_ptr<MotionEstimator> estimator;
  };

  TrackerConfig config_;
  std::vector<InternalTrack> tracks_;
  std::vector<TrackedObstacle> output_cache_;
  std::size_t next_track_id_{1U};
};

DynamicObstacleTracker::DynamicObstacleTracker(TrackerConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

DynamicObstacleTracker::~DynamicObstacleTracker() = default;
DynamicObstacleTracker::DynamicObstacleTracker(DynamicObstacleTracker &&) noexcept = default;
DynamicObstacleTracker & DynamicObstacleTracker::operator=(
  DynamicObstacleTracker &&) noexcept = default;

const std::vector<TrackedObstacle> & DynamicObstacleTracker::update(
  const std::vector<Point2d> & observations, double timestamp_s)
{
  return impl_->update(observations, timestamp_s);
}

const std::vector<TrackedObstacle> & DynamicObstacleTracker::tracks() const
{
  return impl_->tracks();
}

void DynamicObstacleTracker::clear()
{
  impl_->clear();
}

}  // namespace embodied_navigation
