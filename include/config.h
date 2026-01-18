#ifndef CONFIG_H
#define CONFIG_H

#include "common.h"

// function to initialize configuration with default values 
void config_init_defaults(Config* config);

//function to Load configuration from file 
int config_load(Config* config, const char* filename);

// function to Print configuration 
void config_print(const Config* config);

//function to Validate configuration 
int config_validate(const Config* config);

#endif
