#!/usr/bin/env python

import table_1_smach
import table_2_smach

def main():

    

    table1_sm = smach.StateMachine()
    with table1_sm:
        smach.StateMachine.add("IDLE", table_1_smach.Idle(), transitions={"table1_process_start": "IN_PROCESS"})
        smach.StateMachine.add("IN_PROCESS", table_1_smach.InProcess, transitions={"process_done": "WAITING_FOR_PICKUP"})
        smach.StateMachine.add("WAITING_FOR_PICKUP", table_1_smach.WaitingForPickup, transitions={"waiting_before_table1": "IDLE"})

    table2_sm = smach.StateMachine()
    with table2_sm:
        smach.StateMachine.add("IDLE", table_2_smach.Idle(), transitions={"waiting_table2": "WAITING_FOR_DROPOFF"})
        smach.StateMachine.add("WAITING_FOR_DROPOFF", table_1_smach.InProcess, transitions={"drop_off2_complete": "INPROCESS"})
        smach.StateMachine.add("WAITING_FOR_PICKUP", table_1_smach.WaitingForPickup, transitions={"waiting_before_table1": "IDLE"})

    table1_sm.execute()
    table2_sm.execute()

if __name__ == '__main__':
    main()
