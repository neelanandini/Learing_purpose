/*
 * Debug_Print.c
 *
 *  Created on: 10-Mar-2022
 *      Author: Pooja Chandgude
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "Debug_Print.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/



/*******************************************************************************
 * Private Variables
 ******************************************************************************/
static mutex_t s_consoleMutex;


/*******************************************************************************
 * Code
 ******************************************************************************/
static void lock_console(bool lock)
{
    static bool s_initMutex = false;

    if (!s_initMutex)
    {
        OSIF_MutexCreate(&s_consoleMutex);
        s_initMutex = true;
    }

    if (lock)
    {
        OSIF_MutexLock(&s_consoleMutex, 100);
    }
    else
    {
        OSIF_MutexUnlock(&s_consoleMutex);
    }
}

int __read_console(__file_handle handle, unsigned char * buffer, size_t * count)
{
    uint32_t rest;
    bool done = false;
    int i = 0;

    while (done == false)
    {
        lock_console(true);
        ESCI_DRV_Receive(CONSOLE_INST, &buffer[i], 1);
        while (ESCI_DRV_GetReceiveStatus(CONSOLE_INST, &rest) != STATUS_SUCCESS);
        lock_console(false);

        if (buffer[i++] == '\r')
        {
            buffer[i-1]='\n';
            done = true;
        }
    }

    buffer[i]=0;
    *count = (size_t)i;
    return 0;
}


int __write_console(__file_handle handle, unsigned char * buffer, size_t * count)
{
    uint32_t rest;
    lock_console(true);
    ESCI_DRV_Send(CONSOLE_INST, buffer, *count);
    while (ESCI_DRV_GetSendStatus(CONSOLE_INST, &rest) != STATUS_SUCCESS);
    lock_console(false);

    return 0;
}


int __close_console(__file_handle handle)
{
    return 0;
}


