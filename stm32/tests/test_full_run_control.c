#include <assert.h>
#include "full_run_control.h"

int main(void)
{
    FullRunControl s = {0};
    FullRun_Begin(&s, 123);
    assert(!FullRun_Allowed(&s) && s.base_rpm == 0);
    FullRun_ButtonTick(&s, 100, 1, 0, 0);
    assert(s.base_rpm == 5 && s.up_count == 1 && FullRun_Allowed(&s));
    FullRun_Begin(&s, 123); /* a repeated handshake cannot reset a live run */
    assert(s.base_rpm == 5);
    for (int i = 0; i < 10; ++i) FullRun_ButtonTick(&s, 110, 1, 0, 0);
    assert(s.base_rpm == 20);
    FullRun_ButtonTick(&s, 200, 0, 1, 0);
    assert(s.base_rpm == 15 && s.down_count == 1);
    FullRun_ButtonTick(&s, 300, 0, 0, 1);
    FullRun_ButtonTick(&s, 1100, 0, 0, 1);
    assert(s.instant_stop && s.stop_count == 1 && !FullRun_Allowed(&s));
    FullRun_ButtonTick(&s, 1200, 1, 0, 1); /* held stop wins */
    assert(s.instant_stop);
    FullRun_ButtonTick(&s, 1300, 1, 0, 0);
    assert(!s.instant_stop && s.base_rpm == 15 && FullRun_Allowed(&s));
    FullRun_InstantStop(&s);
    s.fault = 1;
    FullRun_ButtonTick(&s, 1400, 1, 0, 0);
    assert(s.instant_stop && s.fault && !FullRun_Allowed(&s));
    FullRun_Begin(&s, 456); /* restart always needs a new UP */
    assert(!s.fault && !s.instant_stop && s.base_rpm == 0 && !FullRun_Allowed(&s));
    FullRun_ButtonTick(&s, 1500, 1, 1, 0);
    assert(s.base_rpm == 0); /* simultaneous DOWN wins */
    return 0;
}
