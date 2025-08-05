#ifndef SD_CARD_H
#define SD_CARD_H

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "sd_card.h"

sd_card_t *sd_get_by_name(const char *name);
FATFS *sd_get_fs_by_name(const char *name);
bool run_format();
bool run_mount();
bool run_unmount();
void run_get_size();
void run_ls();
void run_cat();
void read_file(const char *filename);

#endif

