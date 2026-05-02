# NAO 1v1 Soccer

Two NAO robots compete for a red ball in a field with two neon-yellow goals. Vision-based opponent detection, ball velocity tracking, and strategic positioning determine play.

---

## Hardware Setup

| Component   | Specs                                                                    |
| ----------- | ------------------------------------------------------------------------ |
| **Robots**  | NAO v6 (white body, dark grey joints)                                    |
| **Ball**    | Red rubber, ~1 ft diameter (CVS Sport Design)                            |
| **Goals**   | Neon yellow half-circle rims (Athletic Works, Walmart) w/ black net band |
| **Field**   | 15–20 ft long, bounded by desks or walls                                 |
| **Cameras** | NAO top camera (640×480, ~61° FOV)                                       |

---

## Vision Detection

| Object           | Method                                    | Threshold                                    |
| ---------------- | ----------------------------------------- | -------------------------------------------- |
| **Opponent NAO** | White blob detection (HSV)                | H: 0–180°, S: 0–50, V: 180–255               |
| **Goal rims**    | Neon yellow blob detection                | H: 40–70°, S: 100–255, V: 100–255            |
| **Red ball**     | Red blob detection + `ALRedBallDetection` | H: 0–10° or 170–180°, S: 100–255, V: 100–255 |

Opponent **distance estimation** uses known NAO 6 height (573 mm) and blob bounding-box height:

```
distance_m = (NAO_HEIGHT * focal_length_px) / blob_height_pixels
```

Opponent **azimuth** is calculated from blob x-position in image space.

---

## Game Flow

### **Phase 1: Init** (`_calibrate_goals`)

- Robot scans field, detects both neon-yellow goal rims
- Records goal positions (left, right)
- Blocks until both goals locked

### **Phase 2: Lineup** (`_lineup`)

- Robot walks to center field at slow speed (10% fwd)
- Waits for human "go" command via voice recognition
- Listener thread monitors for `"go"` / `"fetch"` → enters PLAYING

### **Phase 3: Play** (`_movement_loop`)

- Two-layer control: strategy + sonar safety

**Strategy layer** (every 100 ms):

1. **Ball moving?** (distance delta >5cm/frame)
   - YES → **INTERCEPT** mode: run to predicted ball trajectory (both robots race)
   - NO → check distance to opponent

2. **Opponent closer to ball?** (opp_dist < ball_dist + 0.3 m)
   - YES → **STALK** mode: face opponent, slow forward (20%), ready to block
   - NO → **CHARGE** mode: pursue ball at full speed (55%)

3. **Ball within KICK range?** (<0.25 m)
   - YES → **KICK** mode: stop, posture, resume

4. **No ball visible?**
   - SEARCH: pivot in place, owl-style sweep (no forward movement, just rotate)

**Sonar safety layer** (always active, overrides strategy):

- DANGER (<0.32 m) → hard pivot + creep forward (15% speed)
- OBSTACLE (0.32–0.55 m) → curve hard, slow (35% speed)
- WARN (0.55–0.80 m) → blended curve, blended speed
- CLEAR (>0.80 m) → gentle wandering arc + strategy azimuth

---

## Voice Commands

| Command            | Effect                                                    |
| ------------------ | --------------------------------------------------------- |
| `"go"` / `"fetch"` | Exit LINEUP, enter PLAYING                                |
| `"stop"`           | Pause motion (sonar danger still auto-resumes for safety) |

---

## Key Constants (Tunable)

| Parameter               | Value  | Meaning                                |
| ----------------------- | ------ | -------------------------------------- |
| `WALK_VX`               | 0.55   | Charge/search forward speed (norm 0–1) |
| `WARN_DIST`             | 0.80 m | Start preemptive curve                 |
| `OBS_DIST`              | 0.55 m | Hard curve threshold                   |
| `DANGER_DIST`           | 0.32 m | Pivot threshold                        |
| `KICK_DIST`             | 0.25 m | Ball range for kick posture            |
| `BALL_MOTION_THRESHOLD` | 0.05 m | Distance delta to detect motion        |

---

## Ball Motion Detection

Ball velocity is estimated by comparing azimuth and distance across frames:

- If distance delta >5 cm/frame: ball is moving
- Velocity = (delta_azimuth, delta_distance)
- Predicted intercept azimuth = current_azi + velocity_azi × 0.5

---

## Startup Sequence

```
1. NAO wakes, initializes proxies (motion, sonar, camera, ASR, ball detection)
2. Sonar and camera spin up
3. ASR subscribes to vocabulary: ["go", "stop", "fetch"]
4. Vision thread starts scanning for goals
5. Goal calibration blocks until both yellow rims locked
6. Robot walks to center at slow speed
7. Waits for human "go" command
8. On "go": enters PLAYING state
9. Movement loop runs: detect opponent/ball, decide strategy, execute via moveToward
10. Sonar safety layer always active, overriding strategy if collision risk
11. Say "stop" to pause; say "go" to resume
12. Ctrl-C to shutdown gracefully
```

---

## Future Refinements

- **Perpendicular defensive positioning**: Use goal positions to calculate optimal blocking point on the opponent→goal line
- **Goal-aware kicking**: Direct kicks toward detected goal position, not just raw ball location
- **2v2 team detection**: Distinguish teammate from opponent via color markers or team assignments
- **Odometry-based positioning**: Track own position via wheel encoders for field-relative strategy
- **Opponent prediction**: Estimate opponent's next move based on their heading and ball position
