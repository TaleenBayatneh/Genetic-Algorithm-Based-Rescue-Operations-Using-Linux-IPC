#include "ipc.h"
#include "utils.h"
#include "robot.h"

// function to initial resourses ,shared memory,semaphore,message queue.
int ipc_init(IPCResources* ipc, const Config* config) {
    // Check if the pointers are valid before doing anything
    // If either 'ipc' or 'config' is NULL, return error
    if (!ipc || !config) return -1;
    
    // Log the start of the IPC initialization process
    utils_log(LOG_INFO, "Initializing IPC resources...");
    
    //Create shared memory segment 
    // Allocate a shared memory block with size of SharedData structure
    ipc->shm_id = shm_create(sizeof(SharedData));
    // If creation failed, print error and exit
    if (ipc->shm_id < 0) {
        utils_log(LOG_ERROR, "Failed to create shared memory");
        return -1;
    }
    
    // Attach shared memory to the process 
    // Map the shared memory into this process's address space
    ipc->shared_data = shm_attach(ipc->shm_id);
    // If attach fails, destroy the shared memory and return error
    if (!ipc->shared_data) {
        utils_log(LOG_ERROR, "Failed to attach shared memory");
        shm_destroy(ipc->shm_id);//destroy the shared memory
        return -1;
    }
    
    // Create message queue 
    // Create a message queue for inter-process communication
    ipc->msg_id = msgq_create();
    // If creation failed, clean up shared memory and exit
    if (ipc->msg_id < 0) {
        utils_log(LOG_ERROR, "Failed to create message queue");
        shm_detach(ipc->shared_data);
        shm_destroy(ipc->shm_id);
        return -1;
    }
    
    // Create semaphore set 
    // Create a set of semaphores used for synchronization
    ipc->sem_id = sem_create(NUM_SEMS);
    // If creation failed, clean up both message queue and shared memory
    if (ipc->sem_id < 0) {
        utils_log(LOG_ERROR, "Failed to create semaphores");
        msgq_destroy(ipc->msg_id);//destroy message queue 
        shm_detach(ipc->shared_data);
        shm_destroy(ipc->shm_id);//destroy shared memory
        return -1;
    }
    
    // Initialize all semaphore values 
    // SEM_MUTEX: general mutual exclusion lock
    sem_init_value(ipc->sem_id, SEM_MUTEX, 1);
    
    // SEM_MAP_ACCESS: controls access to the shared map data
    sem_init_value(ipc->sem_id, SEM_MAP_ACCESS, 1);
    
    // SEM_SURVIVOR: protects survivor-related shared variables
    sem_init_value(ipc->sem_id, SEM_SURVIVOR, 1);
    
    
    utils_log(LOG_INFO, "IPC resources initialized (SHM=%d, MSG=%d, SEM=%d)",
              ipc->shm_id, ipc->msg_id, ipc->sem_id);
    
    return 0;
}

// Used with semctl() to set or get semaphore values,semctl() can accept different types of data, and this union allows passing 
// one of several types in the same memory space depending on the operation.
union semun {
    int val;                  // set Single semaphore value
    struct semid_ds *buf;     // used when getting or setting semaphore metadata.
    unsigned short *array;    // Array of values for initializing semaphore set
};

//function to clean the resourses
void ipc_cleanup(IPCResources* ipc) {
    // If the pointer is NULL, there is nothing to clean up
    if (!ipc) return;
    
    utils_log(LOG_INFO, "Cleaning up IPC resources...");
    
    // Detach shared memory 
    // If shared_data pointer is valid, detach it from the process
    if (ipc->shared_data) {
        shm_detach(ipc->shared_data);   
        ipc->shared_data = NULL;        // Set pointer to NULL
    }
    
    // Remove shared memory segment 
    // If SHM ID is valid (>= 0), remove the shared memory from the system
    if (ipc->shm_id >= 0) {
        shm_destroy(ipc->shm_id);       // destroy it
        ipc->shm_id = -1;               // Reset ID to invalid value
    }
    
    // Remove message queue 
    // If message queue ID is valid, delete it
    if (ipc->msg_id >= 0) {
        msgq_destroy(ipc->msg_id);      // Destroy message queue
        ipc->msg_id = -1;               // Reset ID
    }
    
    // Remove semaphores 
    // If semaphore set ID is valid, remove the semaphore set
    if (ipc->sem_id >= 0) {
        sem_destroy(ipc->sem_id);       // Delete semaphore set
        ipc->sem_id = -1;               // Reset ID
    }
    
    utils_log(LOG_INFO, "IPC resources cleaned up");
}

///////////////////////// Shared Memory functions ///////////////////////////

int shm_create(size_t size) {
    // create a new shared memory segment with a unique key (SHM_KEY)
    // IPC_CREAT: create if not exist
    // IPC_EXCL: fail if it already exists
    // 0666: read/write permission for all users
    int shm_id = shmget(SHM_KEY, size, IPC_CREAT | IPC_EXCL | 0666);
    
    // If shmget() failed (returns -1)
    if (shm_id < 0) {
        // If the error is "EEXIST", it means the shared memory already exists
        if (errno == EEXIST) {
            //Shared memory segment already exists, so we remove it and recreate it 

            // Try to get the existing shared memory ID
            shm_id = shmget(SHM_KEY, size, 0666);
            
            // If we successfully got it, delete it using IPC_RMID
            if (shm_id >= 0) {
                shmctl(shm_id, IPC_RMID, NULL);  // remove existing shared memory 
            }
            
            // Create a new clean shared memory segment without IPC_EXCL
            shm_id = shmget(SHM_KEY, size, IPC_CREAT | 0666);
        }
    }
    
    return shm_id;
}

SharedData* shm_attach(int shm_id) {
    // Attach the shared memory segment identified by shm_id to the process
    // shmat() returns a pointer to the shared memory area
    // 0: default flags (read/write)
    void* ptr = shmat(shm_id, NULL, 0);

    // If shmat fails, it returns (void*)-1
    if (ptr == (void*)-1) {
        return NULL;  // Return NULL to indicate failure
    }

    // Cast the generic pointer to SharedData* before returning
    return (SharedData*)ptr;
}

int shm_detach(SharedData* shared) {
    // Check if the pointer is valid
    if (!shared) return -1;  // If NULL, return error

    // Detach the shared memory segment from the process
    // shmdt() returns 0 on success, -1 on failure
    return shmdt(shared);
}

int shm_destroy(int shm_id) {
    // Remove the shared memory segment identified by shm_id
    // shmctl() with IPC_RMID tells the system to delete the segment
    return shmctl(shm_id, IPC_RMID, NULL);  
}


////////////////////////////////////// Message Queue functions ////////////////////////////
int msgq_create(void) {
    // create a new message queue with unique key (MSG_KEY)
    // IPC_CREAT: create if it does not exist
    // IPC_EXCL: fail if already exists
    // 0666: read/write permission for all users
    int msg_id = msgget(MSG_KEY, IPC_CREAT | IPC_EXCL | 0666);

    // If creation failed
    if (msg_id < 0) {
        // If the failure reason is "EEXIST", the queue already exists
        if (errno == EEXIST) {
            // Get the existing message queue
            msg_id = msgget(MSG_KEY, 0666);
            
            // If found, remove it
            if (msg_id >= 0) {
                msgctl(msg_id, IPC_RMID, NULL);  // remove existing queue
            }

            // Create a new clean message queue without IPC_EXCL
            msg_id = msgget(MSG_KEY, IPC_CREAT | 0666);
        }
    }
    return msg_id;
}


int msgq_destroy(int msg_id) {
    // Removethe message queue identified by msg_id
    // msgctl() with IPC_RMID tells the system to delete the queue
    return msgctl(msg_id, IPC_RMID, NULL);  
}


////////////////////////////////////// Semaphore functions //////////////////////////////
int sem_create(int num_sems) {
    // create a new set of semaphores with unique key (SEM_KEY)
    // IPC_CREAT: create if it does not exist
    // IPC_EXCL: fail if already exists
    // 0666: read/write permission for all users
    int sem_id = semget(SEM_KEY, num_sems, IPC_CREAT | IPC_EXCL | 0666);

    // If creation failed
    if (sem_id < 0) {
        // If the failure reason is "EEXIST", the semaphore set already exists
        if (errno == EEXIST) {
            // Get the existing semaphore set
            sem_id = semget(SEM_KEY, num_sems, 0666);
            
            // If found, remove it
            if (sem_id >= 0) {
                semctl(sem_id, 0, IPC_RMID);  // Delete the existing semaphore set
            }

            // Create a new semaphore set without IPC_EXCL
            sem_id = semget(SEM_KEY, num_sems, IPC_CREAT | 0666);
        }
    }
    return sem_id;
}

int sem_init_value(int sem_id, int sem_num, int value) {
    // Initialize a single semaphore in a set with a specific value
    union semun arg;        // Create union to pass data to semctl
    arg.val = value;         // Set the desired initial value
    
    // Use semctl() with SETVAL to assign the value to semaphore number 'sem_num'
    return semctl(sem_id, sem_num, SETVAL, arg);
}

int sem_wait_op(int sem_id, int sem_num) {
    // Prepare a semaphore operation structure
    // sem_num: the semaphore number in the set
    // -1: decrement the semaphore (wait)
    // 0: default flags (blocking)
    struct sembuf op = {sem_num, -1, 0};

    // Perform the semaphore operation using semop()
    return semop(sem_id, &op, 1);
}

int sem_signal_op(int sem_id, int sem_num) {
    // Prepare a semaphore operation structure
    // sem_num: the semaphore number in the set
    // 1: increment the semaphore (signal to allow other  process to enter)
    // 0: default flags (blocking)
    struct sembuf op = {sem_num, 1, 0};

    // Perform the semaphore operation using semop()
    return semop(sem_id, &op, 1);
}


int sem_destroy(int sem_id) {
    // Remove the semaphore set identified by sem_id
    // semctl() with IPC_RMID marks the set for deletion
    return semctl(sem_id, 0, IPC_RMID);  
}


