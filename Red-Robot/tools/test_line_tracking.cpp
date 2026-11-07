// Host-side test for include/line_tracking.h (crossing handling).
// Compares the old LF-2 behavior with the new tracker on crossing cases.
// Run from the Red-Robot folder:
//   clang++ -std=c++17 -Wall -Iinclude tools/test_line_tracking.cpp -o /tmp/tlt && /tmp/tlt

#include <cstdio>
#include <cmath>
#include "line_tracking.h"
using namespace line_tracking;
constexpr float kOn = 0.5f, kNoise = 0.1f, kLost = 0.3f; constexpr int kW = 2;
int fails = 0;
// Old behavior: plain average of all sensors + "outer sees tape -> pivot".
const char* Old(const float n[6], float* pos) {
  float s; *pos = WeightedPosition(n, 0, 5, kNoise, &s);
  if (n[0] >= kOn && n[5] < kOn) return "EDGE_L";
  if (n[5] >= kOn && n[0] < kOn) return "EDGE_R";
  return s >= kLost ? "PID" : "LOST";
}
void Check(const char* name, const float n[6], float last, float want_pos, int want_branch,
           bool want_left_edge, bool want_right_edge, bool want_visible = true) {
  Result r = Track(n, kOn, kNoise, kLost, last, kW);
  float old_pos; const char* old_mode = Old(n, &old_pos);
  bool ok = r.visible == want_visible && fabsf(r.position - want_pos) < 0.05f &&
            r.branch_side == want_branch && r.at_left_edge == want_left_edge &&
            r.at_right_edge == want_right_edge;
  if (!ok) ++fails;
  printf("%-4s %-46s new: pos=%+.2f groups=%d width=%d narrowed=%d branch=%+d edgeL=%d edgeR=%d | old: %-6s pos=%+.2f\n",
         ok ? "PASS" : "FAIL", name, r.position, r.groups, r.width, r.narrowed, r.branch_side,
         r.at_left_edge, r.at_right_edge, old_mode, old_pos);
}
int main() {
  { float n[6]={0,0,1,1,0,0};     Check("1 straight, centered", n, 0.0f, 0.0f, 0, false, false); }
  { float n[6]={0,.6f,1,.3f,0,0}; Check("2 normal drift left", n, -0.5f, -0.66f, 0, false, false); }
  { float n[6]={1,0,1,1,0,0};     Check("3 angled crossing hits far LEFT", n, 0.0f, 0.0f, -1, false, false); }
  { float n[6]={0,0,1,1,0,1};     Check("4 angled crossing hits far RIGHT", n, 0.0f, 0.0f, +1, false, false); }
  { float n[6]={1,1,0,1,1,0};     Check("5 crossing left, main off-center right", n, 1.0f, 1.0f, -1, false, false); }
  { float n[6]={0,1,1,1,0,0};     Check("6 crossing merged into line (3 wide)", n, 0.0f, 0.0f, -1, false, false); }
  { float n[6]={1,1,1,1,1,1};     Check("7 perpendicular crossing (all on)", n, 0.1f, 0.0f, 0, false, false); }
  { float n[6]={1,1,1,1,0,0};     Check("8 corner: branch appears left", n, 0.0f, 0.0f, -1, false, false); }
  { float n[6]={1,1,0,0,0,0};     Check("9 corner: only branch left -> edge pivot", n, 0.0f, -2.0f, 0, true, false); }
  { float n[6]={1,0,0,0,0,0};     Check("10 line truly at far left -> edge pivot", n, -1.5f, -2.5f, 0, true, false); }
  { float n[6]={0,0,0,0,0,0};     Check("11 no line", n, 0.7f, 0.7f, 0, false, false, false); }
  { float n[6]={0,0,.4f,.4f,0,0}; Check("12 faint line between sensors", n, 0.0f, 0.0f, 0, false, false); }
  { float n[6]={0,1,1,0,0,0};     Check("13 real curve left (no crossing)", n, -0.5f, -1.0f, 0, false, false); }
  printf("\n%d failure(s)\n", fails);
  return fails;
}
