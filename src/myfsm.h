#ifndef myFSM_H
#define myFSM_H

#include "ThirdParty/fsm.h"

enum Triggers { BlockAhead, ClearAhead };
enum States   { KeepLane, Follow, ChangeLeft, ChangeRight };
const char* StateNames[] = { "Keeping lane", "Stuck behind grandma", "Changing lane left", "Changing lane right" };
void fsm_info(States from, States to, Triggers trigger) { if (from != to) { std::cout << StateNames[to] << '\n'; } }

FSM::Fsm<States, States::KeepLane, Triggers> fsm;

#endif  // myFSM_H