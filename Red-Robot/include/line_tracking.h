// Line tracking through crossings, for the 6-sensor line follower.
//
// Pure logic (no Arduino), so it can be unit-tested on a computer.
//
// Problem it solves: at a crossing, the crossing tape reaches one outer sensor
// while the center sensors are still on our line. A plain weighted average
// blends both lines and steers toward the crossing, and an "outer sensor sees
// tape -> pivot" rule turns onto it. Instead we:
//   1. split the readings into groups of consecutive on-line sensors,
//   2. follow only the group closest to where our line was last seen,
//   3. if that group is wider than one tape line (a crossing merged into it),
//      keep only the `line_width` sensors nearest our line,
//   4. report on which side the other line was (for corners, see
//      `branch_side`), and when the line has truly moved to an edge.

#ifndef RED_ROBOT_LINE_TRACKING_H_
#define RED_ROBOT_LINE_TRACKING_H_

#include <math.h>

namespace line_tracking {

constexpr int kNumSensors = 6;
// Robot's left to right, in sensor spacings from the center.
constexpr float kPositions[kNumSensors] = {-2.5f, -1.5f, -0.5f,
                                           0.5f,  1.5f,  2.5f};

struct Result {
  bool visible = false;  // Some line is under the sensors.
  float position = 0.0f;  // Position of the tracked line (-2.5 .. +2.5).
  int groups = 0;         // Separate groups of on-line sensors.
  int width = 0;          // On-line sensors in the tracked group.
  bool narrowed = false;  // Tracked group was wider than one line.
  int branch_side = 0;    // -1/+1: another line seen left/right of ours.
  bool at_left_edge = false;   // Our line is only at the left edge.
  bool at_right_edge = false;  // Our line is only at the right edge.
};

// Weighted average position of sensors lo..hi, ignoring values below
// `noise_floor`. Writes the summed weight to `strength`.
inline float WeightedPosition(const float norm[], int lo, int hi,
                              float noise_floor, float* strength) {
  float sum = 0.0f;
  float weighted = 0.0f;
  for (int i = lo; i <= hi; ++i) {
    const float w = norm[i] < noise_floor ? 0.0f : norm[i];
    sum += w;
    weighted += w * kPositions[i];
  }
  *strength = sum;
  return sum > 0.0f ? weighted / sum : 0.0f;
}

// Distance from `x` to the span [a, b] (0 if inside).
inline float DistanceToSpan(float x, float a, float b) {
  if (x < a) return a - x;
  if (x > b) return x - b;
  return 0.0f;
}

// `norm`: 0 = floor, 1 = line, robot's left to right.
// `last_position`: where our line was on the previous step.
// `line_width`: how many sensors one tape line normally covers.
inline Result Track(const float norm[kNumSensors], float on_threshold,
                    float noise_floor, float lost_sum, float last_position,
                    int line_width) {
  Result r;
  bool on[kNumSensors];
  for (int i = 0; i < kNumSensors; ++i) on[i] = norm[i] >= on_threshold;

  // Groups of consecutive on-line sensors.
  int first[kNumSensors];
  int last[kNumSensors];
  int n = 0;
  for (int i = 0; i < kNumSensors;) {
    if (!on[i]) {
      ++i;
      continue;
    }
    first[n] = i;
    while (i < kNumSensors && on[i]) ++i;
    last[n] = i - 1;
    ++n;
  }
  r.groups = n;

  if (n == 0) {
    // Only partial readings (line between sensors, or faint): plain average.
    float strength = 0.0f;
    const float p =
        WeightedPosition(norm, 0, kNumSensors - 1, noise_floor, &strength);
    r.visible = strength >= lost_sum;
    r.position = r.visible ? p : last_position;
    return r;
  }

  // Follow the group closest to where our line was.
  int best = 0;
  float best_distance = 1e9f;
  for (int g = 0; g < n; ++g) {
    const float d = DistanceToSpan(last_position, kPositions[first[g]],
                                   kPositions[last[g]]);
    if (d < best_distance) {
      best_distance = d;
      best = g;
    }
  }
  int lo = first[best];
  int hi = last[best];
  r.width = hi - lo + 1;

  // Wider than one line: another line merged in. Keep the window of
  // `line_width` sensors nearest our line; the extra coverage is the branch.
  if (line_width > 0 && r.width > line_width) {
    int best_start = lo;
    float best_offset = 1e9f;
    for (int k = lo; k + line_width - 1 <= hi; ++k) {
      const float center =
          (kPositions[k] + kPositions[k + line_width - 1]) / 2.0f;
      const float offset = fabsf(center - last_position);
      if (offset < best_offset) {
        best_offset = offset;
        best_start = k;
      }
    }
    const int extra_left = best_start - lo;
    const int extra_right = hi - (best_start + line_width - 1);
    if (extra_left > extra_right) r.branch_side = -1;
    if (extra_right > extra_left) r.branch_side = 1;
    lo = best_start;
    hi = best_start + line_width - 1;
    r.narrowed = true;
  }

  // Add one partial (not on-line) neighbor each side for sub-sensor detail.
  const int ext_lo = (lo > 0 && !on[lo - 1]) ? lo - 1 : lo;
  const int ext_hi = (hi < kNumSensors - 1 && !on[hi + 1]) ? hi + 1 : hi;
  float strength = 0.0f;
  r.position = WeightedPosition(norm, ext_lo, ext_hi, noise_floor, &strength);
  r.visible = true;

  // Another separate group marks a branch on its side (widest one wins).
  if (n >= 2 && r.branch_side == 0) {
    int widest = -1;
    for (int g = 0; g < n; ++g) {
      if (g == best) continue;
      if (widest < 0 ||
          last[g] - first[g] > last[widest] - first[widest]) {
        widest = g;
      }
    }
    const float other =
        (kPositions[first[widest]] + kPositions[last[widest]]) / 2.0f;
    r.branch_side = other < r.position ? -1 : 1;
  }

  // Our line has truly moved to an edge: it's the only group, it touches the
  // edge, and the center sensors no longer see it.
  const bool center_off = !on[2] && !on[3];
  r.at_left_edge = n == 1 && !r.narrowed && first[best] == 0 && center_off;
  r.at_right_edge =
      n == 1 && !r.narrowed && last[best] == kNumSensors - 1 && center_off;
  return r;
}

}  // namespace line_tracking

#endif  // RED_ROBOT_LINE_TRACKING_H_
