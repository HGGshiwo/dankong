#pragma once

//@JSON_ENABLE
enum MotionState { CRAWL, WALK, RUN_LOW, RUN_FAST };

//@JSON_ENABLE
struct SetMotionState {
    MotionState state;
};