# NAO Soccer-Dog Demo

A unified NAO behaviour that demonstrates three core capabilities simultaneously:
**voice recognition**, **object detection**, and **locomotion with dynamic arc correction**.

---

## Concept

The robot always moves forward, avoids obstacles autonomously via sonar, and chases a red ball when one is visible.  
Voice commands exist purely as a human fallback — a way to stop or resume the robot if the internal avoidance ever fails or the operator needs to intervene.

---

## Capabilities Demonstrated

| Capability | Mechanism |
|---|---|
| Voice recognition | `ALSpeechRecognition` — 3-word vocabulary, confidence-gated |
| Object / obstacle detection | `ALSonar` — two ultrasonic sensors, ~0–1 m range |
| Ball detection | `ALRedBallDetection` — camera-based, gives azimuth + distance |
| Locomotion | `ALMotion.moveToward` — continuous velocity with live theta steering |

---

## Behaviour Priority Stack

Every 100 ms the movement loop evaluates in this order:

```
1. DANGER   sonar < 0.32 m  →  stop, Roomba-spin toward open side until clear, resume
2. BALL     visible          →  steer azimuth toward ball; kick posture at 0.25 m
3. OBSTACLE sonar < 0.55 m  →  hard curve toward open side + 35% speed
4. WARN     sonar < 0.80 m  →  preemptive blend curve (sharpens as distance shrinks)
5. CLEAR                    →  gentle alternating arc (direction flips after each Roomba turn)
```

Obstacle avoidance and voice listening run as independent parallel threads. The movement thread
continuously adjusts the arc path — preemptive curving begins at 0.80 m, well before a hard
obstacle is reached. Voice is a separate channel that can interrupt at any point, but the robot
does not depend on it to navigate safely.

After a Roomba spin the arc direction flips, biasing the next run toward the side with more open space.

---

## Voice Commands

| Word | Effect |
|---|---|
| `go` / `fetch` | Resume movement after a manual stop |
| `stop` | Pause all motion (sonar danger still auto-resumes the spin) |

---

## Setup

**Requirements**
- NAO robot on the local network
- Python 2.7 with the NAOqi Python SDK (`naoqi-sqk`)
- Paths in `dog.py` assume the SDK at `~/Desktop/naoqi-sqk`

**Run**
```bash
/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python2.7 dog.py
```

Robot IP and port are set at the top of `dog.py`:
```python
ROBOT_IP   = "172.16.0.29"
ROBOT_PORT = 9559
```

---

## Tuning Constants

| Constant | Default | Meaning |
|---|---|---|
| `WARN_DIST` | 0.80 m | Begin preemptive curve |
| `OBS_DIST` | 0.55 m | Hard curve + slow |
| `DANGER_DIST` | 0.32 m | Stop and spin |
| `CLEAR_DIST` | 1.00 m | Exit spin threshold |
| `WALK_VX` | 0.55 | Forward speed (0–1) |
| `ARC_THETA` | 0.12 | Gentle arc magnitude |
| `TURN_THETA` | 0.75 | Spin / hard-curve rate |
| `KICK_DIST` | 0.25 m | Ball distance to trigger kick |
| `VOICE_CONFIDENCE` | 0.38 | Minimum ASR confidence |

---

## Planned: Opponent Responses

Next layer inserts between priority 2 (ball) and priority 3 (obstacle):
- Detect opponent (person / second robot) via face detection or landmark
- Decide: intercept path, shield ball, retreat
