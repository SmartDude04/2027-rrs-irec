#include "sd_card.h"

#include <stdarg.h>
#include <stdio.h>

static FATFS sd_filesystem;
static FIL log_file;

static int sd_mounted = 0;
static int log_open = 0;

static char write_buffer[200];

FRESULT sd_card_init(void)
{
    if (sd_mounted)
    {
        return FR_OK;
    }

    const FRESULT result = f_mount(&sd_filesystem, SDPath, 1);

    if (result == FR_OK)
    {
        sd_mounted = 1;
    }

    return result;
}

FRESULT sd_card_open_log_txt(void)
{
    if (!sd_mounted)
    {
        return FR_NOT_READY;
    }

    if (log_open)
    {
        return FR_OK;
    }

    char path[sizeof(SDPath) + sizeof("log.txt")];
    const int path_length = snprintf(path, sizeof(path), "%slog.txt", SDPath);

    if (path_length < 0 || (size_t)path_length >= sizeof(path))
    {
        return FR_INVALID_PARAMETER;
    }

    FRESULT result = f_open(&log_file, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (result != FR_OK)
    {
        return result;
    }

    result = f_lseek(&log_file, f_size(&log_file));
    if (result != FR_OK)
    {
        (void)f_close(&log_file);
        return result;
    }

    log_open = 1;
    return FR_OK;
}

FRESULT sd_card_printf(const char *format, ...)
{
    if (!log_open)
    {
        return FR_INVALID_OBJECT;
    }

    if (format == NULL)
    {
        return FR_INVALID_PARAMETER;
    }

    va_list args;
    va_start(args, format);
    const int length = vsnprintf(write_buffer, sizeof(write_buffer),
                                 format, args);
    va_end(args);

    if (length < 0 || (size_t)length >= sizeof(write_buffer))
    {
        return FR_INVALID_PARAMETER;
    }

    if (length == 0)
    {
        return FR_OK;
    }

    UINT bytes_written = 0;
    const FRESULT result = f_write(&log_file, write_buffer, (UINT)length, &bytes_written);

    if (result != FR_OK)
    {
        return result;
    }

    if (bytes_written != (UINT)length)
    {
        return FR_DISK_ERR;
    }

    // Flush file data and metadata while keeping the file open
    return f_sync(&log_file);
}

FRESULT sd_card_close(void)
{
    if (!log_open)
    {
        return FR_OK;
    }

    const FRESULT result = f_close(&log_file);

    if (result == FR_OK)
    {
        log_open = 0;
    }

    return result;
}