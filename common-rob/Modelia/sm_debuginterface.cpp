#include "sm_debuginterface.h"
#include "sm_statemachinebase.h"

SM_DebugInterface::SM_DebugInterface()
    : m_active_start(true),
      m_active_stop(true),
      m_active_on_entry(true),
      m_active_on_exit(true),
      m_active_interrupt_evitement(true)
{
}

SM_DebugInterface::~SM_DebugInterface()
{
}

// ________________________________________________
void SM_DebugInterface::SM_Start(SM_StateMachineBase* origin, float match_time)
{
    (void)origin;
    (void)match_time;
}

// ________________________________________________
void SM_DebugInterface::SM_Stop(SM_StateMachineBase* origin, float match_time)
{
    (void)origin;
    (void)match_time;
}

// ________________________________________________
void SM_DebugInterface::SM_EntryState(SM_StateMachineBase* origin, unsigned short state, float match_time)
{
    (void)origin;
    (void)state;
    (void)match_time;
}

// ________________________________________________
void SM_DebugInterface::SM_ExitState(SM_StateMachineBase* origin, unsigned short state, float match_time)
{
    (void)origin;
    (void)state;
    (void)match_time;
}

// ________________________________________________
void SM_DebugInterface::SM_InterruptedByEvitement(SM_StateMachineBase* origin, unsigned short state, float match_time)
{
    (void)origin;
    (void)state;
    (void)match_time;
}
