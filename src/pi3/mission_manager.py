"""Mission Manager: which competition phase is active.

    current_mission() -> {"mission": "OBSTACLE_CHALLENGE", "reason": "..."}

Reports only. No steering, no camera, no state transitions - the Competition
State Machine handles behaviour within a mission, this just says which one.

Set MISSION below before a run. Automatic detection can replace the constant
later without changing anything that calls this.
"""

import sys
from enum import Enum


class Mission(Enum):
    OPEN_CHALLENGE = "OPEN_CHALLENGE"
    OBSTACLE_CHALLENGE = "OBSTACLE_CHALLENGE"
    PARKING = "PARKING"


# Selected by hand for now. Change this line, not the code below.
MISSION = Mission.OBSTACLE_CHALLENGE


def current_mission():
    return {"mission": MISSION.value, "reason": "Selected manually in config"}


def selftest():
    result = current_mission()
    assert set(result) == {"mission", "reason"}, result
    assert result["mission"] in {mission.value for mission in Mission}
    assert result["reason"]

    # every mission name must survive a round trip, so a typo in MISSION fails here
    for mission in Mission:
        assert Mission(mission.value) is mission

    print("selftest ok  mission:", result["mission"])


if __name__ == "__main__":
    selftest() if "--selftest" in sys.argv else print(current_mission())
