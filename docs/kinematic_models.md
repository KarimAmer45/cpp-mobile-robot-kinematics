# Kinematic Models

This project uses a planar robot frame: `x` forward, `y` left, and positive
`yaw` counter-clockwise. Wheel angular velocities are expressed in radians per
second.

## Differential Drive

Given wheel radius `r`, track width `b`, left wheel angular speed `wl`, and
right wheel angular speed `wr`:

```text
vx = r * (wr + wl) / 2
wz = r * (wr - wl) / b
```

The inverse model is:

```text
wl = (vx - wz * b / 2) / r
wr = (vx + wz * b / 2) / r
```

## Mecanum Drive

For a rectangular mecanum base, `lx` is half the wheelbase and `ly` is half the
track width. The coupling term is `k = lx + ly`.

```text
front_left  = ( vx - vy - k * wz) / r
front_right = ( vx + vy + k * wz) / r
rear_left   = ( vx + vy - k * wz) / r
rear_right  = ( vx - vy + k * wz) / r
```

The forward model averages the wheel contributions back into a body twist.

## Ackermann Steering

Ackermann steering maps commanded rear-axle speed `v` and steering angle `a`
to body angular velocity:

```text
wz = v * tan(a) / wheel_base
```

Pure in-place rotation is not represented by the Ackermann model because the
steered vehicle has a non-zero turning radius when it is moving.

