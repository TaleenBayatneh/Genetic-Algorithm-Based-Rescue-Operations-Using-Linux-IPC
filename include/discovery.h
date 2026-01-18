//discovery module is were we declarations for scanning the map detecting survivors and syncing exploration knowledge

#ifndef DISCOVERY_H
#define DISCOVERY_H

#include "common.h"    //shared core types robot,shareddata,config,point3d
#include "ipc.h"     //ipc definitions for message passing and synchronization

                                                                                       //discovery initialization
void discovery_init(SharedData* shared, const Config* config);                         //initialize discovery-related shared state

                                                                                       //discovery scanning and sensing
void discovery_scan(Robot* robot, SharedData* shared, const Config* config);  
                                                                                        //scan around robot and update discovered or visited data
int discovery_get_cell(Robot* robot, SharedData* shared, int x, int y, int z); 
                                                                                        //get a cell value with discovery rules known and unknown handling
int discovery_detect_survivors(Robot* robot, SharedData* shared, const Config* config);//detect nearby survivors and update shared survivor info

                                                                                        //ipc message handling for discovery data
void discovery_process_messages(Robot* robot, SharedData* shared, int msg_id);          //process discovery-related messages (knowledge sharing)

                                                                                        //exploration targeting
Point3D discovery_find_exploration_target(Robot* robot, SharedData* shared,
                                         const Config* config);                         //choose next exploration target location

                                                                                      //progress and synchronization
double discovery_get_progress(SharedData* shared);
                                                                                       //compute how much of the map has been explored
void discovery_sync_knowledge(Robot* robot, SharedData* shared);                       //sync discovered cells/targets between robots

#endif
