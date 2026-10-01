/*
 * What the host build has to supply that the board supplies for real.
 *
 * The fault record lives in retained RAM on the target; on a development
 * machine there is nothing to retain, so it is a plain object in ordinary
 * memory. What it is *not* is a stub that always answers "no fault": the rule
 * is the same one the ports use - the magic says whether a record is worth
 * reading - because the simulator's `fault` session plants a record and then
 * boots, and a stand-in that could never report one would make the boot's
 * fault lines unreachable. A machine that has just started with nothing in
 * this object reports nothing, which is the case every other session runs.
 */

#include "ak_fault.h"

ak_fault_record_t ak_fault;

int ak_fault_present(void)
{
    return ak_fault.magic == AK_FAULT_MAGIC;
}

void ak_fault_clear(void)
{
    ak_fault.magic = 0;
    ak_fault.count = 0;
}
