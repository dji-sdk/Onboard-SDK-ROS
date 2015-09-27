### Task workflow

navigation in local coordinate(x,y,z) -> navigation by waypoints(lati,long,altitude)

### Dispatate basic functions

#### For the coordinate part

1. conversion between radiu GPS to angle GPS
2. conversion between GPS to local coordinate (what about yaw angle?

#### For the flying part

---position control ---

1. calculate the flight directionin in NED coordinate (angle between origin and destination)

~~2. calculate the flight direction in body frame (drone yaw as an offset of what we get in previous step)~~


---velocity control---

3. calculate the ratio of velocity in pitch/roll direction (same as the position direction)

4. separte the overall velocity into these two direction

---accleration control---

3. calculate the ratio of pitch accleration and roll accleration (same as position one)

4. calculate the ratio of pitch and yaw offset angle for given acceleration according to accleration = g/tan(theta)

---

Therefore, there are three control loops able to be implemented.
~~What's more, we can also do all things above in body frame. Aha, 6 control loops in total.~~


The detailed control methodology: still TBD (nested PID, step by step PID, position estimation, or just use open loop, whatever, etc.)
