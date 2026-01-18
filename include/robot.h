//robot module declarations for robot lifecycle  state machine handlers and helper utilities

#ifndef ROBOT_H
#define ROBOT_H

#include "common.h"    //shared core types robot and the survivor and shareddata config  point3d
#include "ipc.h"     //ipc resource definitions used by robot processes

                                                                                 //robot initialization
void robot_init_all(SharedData* shared, const Config* config);                      //init all robots and place them on the map
void robot_init(Robot* robot, int id, Point3D start_pos, const Config* config);     //init a single robot with id and starting position

                                                                               //robot process entry points
void robot_process_main(int robot_id, SharedData* shared, IPCResources* ipc,
                        const Config* config);                                     //main function for a robot process/thread
void robot_main_loop(Robot* robot, SharedData* shared, IPCResources* ipc,
                     const Config* config);                                        //main state machine loop

                                                                               //state handlers
void robot_handle_idle(Robot* robot, SharedData* shared, const Config* config);    //idle behavior waiting  selecting next action
void robot_handle_exploring(Robot* robot, SharedData* shared, IPCResources* ipc,
                            const Config* config);                                 //exploration behavior searching or sensing
void robot_handle_approaching(Robot* robot, SharedData* shared, IPCResources* ipc,
                              const Config* config);                               //approach target behavior moving to survivor
void robot_handle_rescuing(Robot* robot, SharedData* shared, IPCResources* ipc,
                           const Config* config);                                  //rescue behavior perform rescue + update shared state
void robot_handle_returning(Robot* robot, SharedData* shared, IPCResources* ipc,
                            const Config* config);                                 //return behavior go back to base or entry
void robot_handle_refilling(Robot* robot, SharedData* shared, const Config* config); //refill behavior restock energy or resources

                                                                               //helper functions
int robot_find_target_survivor(Robot* robot, SharedData* shared);                  //select a survivor to target returns survivor id or -1
void robot_sense(Robot* robot, SharedData* shared, IPCResources* ipc,
                 const Config* config);                                            //sense environment and update internal/shared info
void robot_plan(Robot* robot, SharedData* shared, const Config* config);           //compute path/plan to current target
void robot_plan_return_to_base(Robot* robot, SharedData* shared,
                               const Config* config);                              //compute return path to entry/base
int robot_execute_step(Robot* robot, SharedData* shared, IPCResources* ipc,
                       const Config* config);                                      //execute one step of plan move or act

                                                                                   //movement helpers
int robot_move(Robot* robot, SharedData* shared, Point3D new_pos); 
                                                                                //attempt to move robot to a new position
void robot_random_move(Robot* robot, SharedData* shared);                          //random move fallback when no plan exists

                                                                                   //survivor interaction
int robot_check_survivor(Robot* robot, SharedData* shared);                        //check if robot is on/near a survivor cell
int robot_rescue_survivor(Robot* robot, SharedData* shared, IPCResources* ipc);    //perform rescue and update shared data

                                                                                     //communication helpers
void robot_broadcast_rescue(Robot* robot, int survivor_id, IPCResources* ipc);     //broadcast rescue event to other robots
void robot_process_messages(Robot* robot, SharedData* shared, IPCResources* ipc);  //receive and handle incoming messages

                                                                                      //state name utility
const char* robot_state_name(int state);                                           //convert state enum/int to human readable string

#endif 
