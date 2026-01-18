#ifndef IPC_H
#define IPC_H

#include "common.h"

// IPC Keys
// Define unique keys for shared memory, message queue, and semaphore
#define SHM_KEY 0x1234 // Key for Shared Memory
#define MSG_KEY 0x5678 // Key for Message Queue
#define SEM_KEY 0x9ABC // Key for Semaphore

// IPC Resources structure
// Stores all IPC identifiers and a pointer to shared data
typedef struct {
    int shm_id;            // Shared memory ID
    int msg_id;            // Message queue ID
    int sem_id;            // Semaphore ID
    SharedData* shared_data; // Pointer to shared memory data
} IPCResources;

// IPC Initialization and Cleanup
int ipc_init(IPCResources* ipc, const Config* config); // Initialize all IPC resources
void ipc_cleanup(IPCResources* ipc);                   // Cleanup all IPC resources

// Shared Memory Operations
int shm_create(size_t size);                // Create shared memory segment
SharedData* shm_attach(int shm_id);        // Attach process to shared memory to allow it see the shared data
int shm_detach(SharedData* shared);        // Detach process from shared memory
int shm_destroy(int shm_id);               // Remove shared memory

// Message Queue Operations
int msgq_create(void);                                      // Create message queue
int msgq_destroy(int msg_id);                               // Remove message queue


// Semaphore Operations
// Semaphores are used to synchronize access to shared resources
int sem_create(int num_sems);                              // Create a group of semaphores for the resourses i have 
int sem_init_value(int sem_id, int sem_num, int value);    // Initialize semaphore value
int sem_wait_op(int sem_id, int sem_num);                  // Wait (decrement) semaphore if it 0 so process wait to see it >0 then enter
int sem_signal_op(int sem_id, int sem_num);                // Signal (increment) semaphore incremant by one so the other procees can enter 
int sem_destroy(int sem_id);                              // Remove semaphore

#endif
