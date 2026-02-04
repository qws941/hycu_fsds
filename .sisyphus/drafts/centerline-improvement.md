# Draft: FSDS Centerline Generation Improvement

## Requirements (confirmed)
- User reports ~30-40% centerline generation failure rate
- "No current centerline" message appears frequently
- Issue occurs even when cones are detected (L:3 R:4)
- Problem especially pronounced with single-side cone detection (L:0 R:5)

## Code Analysis

### build_centerline() (lines 469-531)
**Current Logic:**
```python
if left_cones and right_cones:
    all_x = sorted(set([c['x'] for c in left_cones + right_cones]))
    for x in all_x[:8]:
        left_at_x = [c for c in left_cones if abs(c['x'] - x) < 3.0]  # x-matching
        right_at_x = [c for c in right_cones if abs(c['x'] - x) < 3.0]
        
        if left_at_x and right_at_x:  # both sides matched
            # create centerline point
        elif left_at_x:  # left only
            target_y = left_y + single_side_offset  # offset = 1.0m
        elif right_at_x:  # right only
            target_y = right_y - single_side_offset
```

**Identified Issues:**
1. `safe_track_width` declared but NOT USED (line 476)
2. Fallback logic at lines 501-507 only runs when centerline is empty after the loop
3. `single_side_offset = 1.0` may be too small for 3-4m track width

### Unused Variable
```python
safe_track_width = max(3.0, min(self.last_valid_track_width, 4.0))  # DECLARED
# But never used in build_centerline() - the single_side_offset is hardcoded to 1.0
```

### Track Width Update (line 775)
```python
if left_cones and right_cones:
    left_y = np.mean([c['y'] for c in left_cones[:3]])
    right_y = np.mean([c['y'] for c in right_cones[:3]])
    raw_width = abs(left_y - right_y)
    self.last_valid_track_width = np.clip(raw_width, self.track_width_min, self.track_width_max)
```
- Only updated when BOTH sides have cones
- Initial value: 4.0m

### Fallback When Empty Centerline (lines 913-929)
```python
if not centerline:
    if last_valid_centerline_snapshot:
        centerline = last_valid_centerline_snapshot
        self.current_target_speed = 3.5  # Reduced speed
    else:
        cmd.brake = 1.0  # Full brake
```

## Root Cause Analysis

### Problem 1: X-Matching May Exclude All Cones
When `all_x` contains x-coordinates from both sides, but they're spread out:
- Left cones at x = [5.0, 7.0, 9.0]
- Right cones at x = [6.5, 8.5, 10.5]
- For x = 5.0: no right cones within 3.0m → left_at_x only
- For x = 6.5: no left cones within 3.0m → right_at_x only

This creates centerline points from single sides, which IS working.

### Problem 2: Single-Side Detection (L:0 R:5 or L:5 R:0)
When only one side has cones:
```python
elif left_cones:
    for c in left_cones[:5]:
        target_y = c['y'] + single_side_offset
```
This SHOULD work. So why the 30-40% failure?

### Hypothesis: Empty Return Before elif Branches
Looking more carefully:
```python
if left_cones and right_cones:
    # ... loop with x-matching ...
    if not centerline:  # fallback if loop produced nothing
        # create from individual cones
elif left_cones:  # THIS ONLY RUNS IF right_cones IS EMPTY
    ...
elif right_cones:  # THIS ONLY RUNS IF left_cones IS EMPTY
    ...
```

**THE BUG**: When `left_cones = True` and `right_cones = True`, but the x-matching loop produces nothing AND the fallback also fails, it returns empty!

Wait, the fallback at 501-507:
```python
if not centerline:
    for c in left_cones[:5]:
        target_y = c['y'] + single_side_offset
        centerline.append(...)
    for c in right_cones[:5]:
        target_y = c['y'] - single_side_offset
        centerline.append(...)
```
This should catch it... unless there's an exception or edge case.

## Need to Verify

1. Check if `single_side_offset` is appropriate (1.0m seems small for 3-4m track)
2. Check if there are edge cases where centerline is still empty after all fallbacks
3. Check what happens with very close cones (clustering issues?)

## Technical Decisions (pending)
- [ ] Should `safe_track_width` be used instead of `single_side_offset`?
- [ ] Should we use track_width / 2 for offset calculation?
- [ ] Add logging to identify WHY centerline is empty?

## Research Findings (Explore Agent)

### CRITICAL FINDING #1: Single-Side Offset Calculation Error
**competition_driver.py (line 476):**
```python
safe_track_width = max(3.0, min(self.last_valid_track_width, 4.0))  # DECLARED
single_side_offset = 1.0  # HARDCODED - NOT using safe_track_width!
```
**Should be**: `single_side_offset = safe_track_width / 2.0` (= 1.5~2.0m)

### CRITICAL FINDING #2: Offset Direction Correct but Magnitude Wrong
```python
# Left cone at y=+2.0, offset=1.0 → target_y = 3.0 (inside track)
# Right cone at y=-2.0, offset=1.0 → target_y = -3.0 (inside track)
```
Direction is correct, but 1.0m offset is too small for 3-4m wide tracks.

### CRITICAL FINDING #3: Duplicate build_centerline() Implementations
- `cone_detector.py`: Uses `last_valid_track_width / 2` correctly (lines 170, 173)
- `competition_driver.py`: Uses hardcoded `single_side_offset = 1.0` (lines 494, 498)
**The driver is NOT using the modular cone_detector.py version!**

### CRITICAL FINDING #4: Perception State Machine Logic
Line 750-751 has logic issue:
```python
has_strong_perception = has_both_sides or total_cones >= 1  # Any 1 cone = strong
has_weak_perception = total_cones >= 1 and not has_strong_perception  # ALWAYS FALSE
```
This means weak perception handling never activates.

### CRITICAL FINDING #5: Fallback Only on Complete Empty
Lines 501-507 fallback only triggers when centerline is completely empty.
If main loop produces 1-2 points instead of expected 5+, no correction occurs.

## Root Cause Summary

| Issue | Impact | Expected Fix Gain |
|-------|--------|-------------------|
| Hardcoded 1.0m offset | Path drifts from center | +10-15% |
| State machine logic error | Partial detection ignored | +5-10% |
| Short centerline (1-2 points) not triggering fallback | Unstable steering | +2-3% |

**Combined expected improvement: 30-40% failure → 10-15% failure**

## Open Questions
1. 수정 범위: 작은 변경(offset만)? 또는 전체 로직 개선?
2. 테스트 전략: TDD 방식 or 시뮬레이터에서 실시간 검증?
3. cone_detector.py의 모듈식 함수를 드라이버에서 재사용할지?
