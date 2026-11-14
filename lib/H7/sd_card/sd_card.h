#ifndef AIRBRAKES_SD_CARD_H
#define AIRBRAKES_SD_CARD_H

#include "fatfs.h"

FRESULT sd_card_init();

FRESULT sd_card_open_log_txt();

FRESULT sd_card_printf(const char *format, ...);

FRESULT sd_card_close();

#endif