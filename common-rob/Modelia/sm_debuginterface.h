#ifndef SM_DEBUGINTERFACE_H
#define SM_DEBUGINTERFACE_H

class SM_StateMachineBase;

class SM_DebugInterface
{
public:
    SM_DebugInterface();
    virtual ~SM_DebugInterface();

    virtual void SM_Start(SM_StateMachineBase* origin, float match_time);
    virtual void SM_Stop(SM_StateMachineBase* origin, float match_time);
    virtual void SM_EntryState(SM_StateMachineBase* origin, unsigned short state, float match_time);
    virtual void SM_ExitState(SM_StateMachineBase* origin, unsigned short state, float match_time);
    virtual void SM_InterruptedByEvitement(SM_StateMachineBase* origin, unsigned short state, float match_time);

    bool m_active_start;
    bool m_active_stop;
    bool m_active_on_entry;
    bool m_active_on_exit;
    bool m_active_interrupt_evitement;
};

#endif // SM_DEBUGINTERFACE_H
