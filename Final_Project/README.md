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

Every 100 ms the movement loop evaluates in this order. The robot **never stops to turn** —
theta and speed are continuously scaled by distance so avoidance is a smooth steering response
from far out, not a reactive emergency stop.

```
1. BALL     visible          →  azimuth steers theta; sonar overrides if obstacle on same side
                                kick posture fires at 0.25 m (only intentional stop)
2. DANGER   sonar < 0.32 m  →  near-pivot: max theta toward open side, 15% forward speed
3. OBSTACLE sonar < 0.55 m  →  hard curve toward open side, 35% forward speed
4. WARN     sonar < 0.80 m  →  blended curve — sharpens and slows as distance shrinks
5. CLEAR                    →  gentle alternating arc; direction flips after each obstacle recovery
```

Obstacle avoidance and voice listening run as independent parallel threads. Voice is a separate
channel that can interrupt at any point, but the robot does not depend on it to navigate safely.

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
| `DANGER_DIST` | 0.32 m | Near-pivot: max theta, 15% forward speed |
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
