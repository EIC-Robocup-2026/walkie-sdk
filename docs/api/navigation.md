# Navigation

Robot navigation controller. Access via `bot.nav`.

## Return values

`go_to()` returns one of five strings:

| Value | Meaning |
|---|---|
| `"SUCCEEDED"` | Nav2 confirmed the robot reached the goal |
| `"FAILED"` | Nav2 aborted and the robot is beyond `goal_tolerance` (or no tolerance was set) |
| `"CLOSE_ENOUGH"` | Nav2 aborted but `final_distance_remaining ≤ goal_tolerance` |
| `"CANCELED"` | Navigation was explicitly canceled via `cancel()` |
| `"IN_PROGRESS"` | Non-blocking call — navigation is still running |

## Examples

### Basic navigation

```python
result = bot.nav.go_to(x=2.0, y=1.0, heading=0.0)
# "SUCCEEDED" or "FAILED"
```

### Tolerance-based result

Useful in competition when an obstacle causes Nav2 to abort, but the robot stopped
close enough to the goal to continue with the task:

```python
result = bot.nav.go_to(x=3.0, y=2.0, heading=0.0, goal_tolerance=0.4)

if result in ("SUCCEEDED", "CLOSE_ENOUGH"):
    print("Close enough — proceeding")
elif result == "FAILED":
    print(f"Too far: {bot.nav.final_distance_remaining:.2f} m remaining")
```

### Inspecting failure reason

Nav2 (Humble+) includes an `error_code` and `error_msg` in the action result:

```python
result = bot.nav.go_to(x=99.0, y=99.0, heading=0.0)
if result == "FAILED":
    print(bot.nav.nav_error_code)   # e.g. 3 (INVALID_PATH)
    print(bot.nav.nav_error_msg)    # e.g. "Failed to find a valid plan"
```

### Rich result dict

```python
bot.nav.go_to(x=3.0, y=2.0, heading=0.0, goal_tolerance=0.5)
print(bot.nav.last_result)
# {
#   "status": "CLOSE_ENOUGH",
#   "final_distance": 0.3,
#   "error_code": 6,
#   "error_msg": "...",
#   "recoveries": 1,
# }
```

### Non-blocking with feedback

```python
def on_feedback(fb):
    print(f"Distance remaining: {bot.nav.distance_remaining:.2f} m")

bot.nav.go_to(x=5.0, y=3.0, heading=1.57, blocking=False, feedback_callback=on_feedback)

while bot.nav.is_navigating:
    time.sleep(0.5)

print(bot.nav.status)  # final status
```

### Emergency stop

```python
bot.nav.stop()   # publishes zero velocity immediately
```

## API Reference

::: walkie_sdk.modules.navigation.Navigation
    options:
      show_source: false
