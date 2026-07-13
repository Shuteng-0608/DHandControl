"""Shared MH6 palm motor calibration in actual hardware ID order."""

PALM_MOTOR_CALIBRATION = {
    1: {"outward": 0, "neutral": 247, "inward": 1000},
    2: {"outward": 630, "neutral": 500, "inward": 120},
    3: {"outward": 536, "neutral": 500, "inward": 401},
}

PALM_MOTOR_SAFE_LIMITS = {
    motor_id: (min(points.values()), max(points.values()))
    for motor_id, points in PALM_MOTOR_CALIBRATION.items()
}

PALM_NEUTRAL_MOTORS = [
    PALM_MOTOR_CALIBRATION[motor_id]["neutral"]
    for motor_id in (1, 2, 3)
]
