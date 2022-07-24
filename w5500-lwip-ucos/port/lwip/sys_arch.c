/*
 * Copyright (c) 2017 Simon Goldschmidt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Simon Goldschmidt <goldsimon@gmx.de>
 *
 */

/* lwIP includes. */
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"
#include <ucos_ii.h>
#include <lwipopts.h>

static void *mailbox_mem[LWIP_UCOSII_MAILBOX_SIZE * LWIP_UCOSII_MAX_NUM_OF_MAILBOX];
static OS_MEM *p_mbox_mem;

/* Initialize this module (see description in sys.h) */
void sys_init(void)
{
    INT8U err;

    p_mbox_mem = OSMemCreate(mailbox_mem, LWIP_UCOSII_MAX_NUM_OF_MAILBOX, LWIP_UCOSII_MAILBOX_SIZE * sizeof(void *), &err);
}

u32_t sys_now(void)
{
    return OSTimeGet() * OS_TICKS_PER_SEC / 1000;
}

u32_t sys_jiffies(void)
{
    return OSTimeGet();
}

#if SYS_LIGHTWEIGHT_PROT

sys_prot_t
sys_arch_protect(void)
{
    OS_CPU_SR cpu_sr;
    OS_ENTER_CRITICAL();

    return cpu_sr;
}

void sys_arch_unprotect(sys_prot_t pval)
{
    OS_CPU_SR cpu_sr = pval;
    OS_EXIT_CRITICAL();
}

#endif /* SYS_LIGHTWEIGHT_PROT */

void sys_arch_msleep(u32_t delay_ms)
{
    OSTimeDlyHMSM(0, 0, 0, delay_ms);
}

#if !LWIP_COMPAT_MUTEX

/* Create a new mutex*/
err_t sys_mutex_new(sys_mutex_t *mutex)
{
    LWIP_ASSERT("mutex != NULL", mutex != NULL);

    mutex->mut = OSSemCreate(1);

    if (mutex->mut == NULL)
    {
        SYS_STATS_INC(mutex.err);
        return ERR_MEM;
    }
    SYS_STATS_INC_USED(mutex);
    return ERR_OK;
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
    INT8U err;
    LWIP_ASSERT("mutex != NULL", mutex != NULL);
    LWIP_ASSERT("mutex->mut != NULL", mutex->mut != NULL);

    OSSemPend(mutex->mut, 0, &err);

    LWIP_ASSERT("failed to take the mutex", err == OS_ERR_NONE);
}

void sys_mutex_unlock(sys_mutex_t *mutex)
{
    INT8U ret;
    LWIP_ASSERT("mutex != NULL", mutex != NULL);
    LWIP_ASSERT("mutex->mut != NULL", mutex->mut != NULL);

    ret = OSSemPost(mutex->mut);

    LWIP_ASSERT("failed to give the mutex", ret == OS_ERR_NONE);
}

void sys_mutex_free(sys_mutex_t *mutex)
{
    INT8U err;

    LWIP_ASSERT("mutex != NULL", mutex != NULL);
    LWIP_ASSERT("mutex->mut != NULL", mutex->mut != NULL);

    SYS_STATS_DEC(mutex.used);
    OSSemDel(mutex->mut, OS_DEL_ALWAYS, &err);
    mutex->mut = NULL;
}

#endif /* !LWIP_COMPAT_MUTEX */

err_t sys_sem_new(sys_sem_t *sem, u8_t initial_count)
{
    LWIP_ASSERT("sem != NULL", sem != NULL);

    sem->sem = OSSemCreate(initial_count);

    if (sem->sem == NULL)
    {
        return ERR_MEM;
    }

    SYS_STATS_INC(sem.used);

    return ERR_OK;
}

void sys_sem_signal(sys_sem_t *sem)
{
    INT8U ret;
    LWIP_ASSERT("sem != NULL", sem != NULL);
    LWIP_ASSERT("sem->sem != NULL", sem->sem != NULL);

    ret = OSSemPost(sem->sem);

    LWIP_ASSERT("sys_sem_signal: sane return value", ret == OS_ERR_NONE);
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout_ms)
{
    INT8U err;
    LWIP_ASSERT("sem != NULL", sem != NULL);
    LWIP_ASSERT("sem->sem != NULL", sem->sem != NULL);

    OSSemPend(sem->sem, timeout_ms, &err);

    if (err == OS_ERR_TIMEOUT)
    {
        return SYS_ARCH_TIMEOUT;
    }

    LWIP_ASSERT("taking semaphore failed", err == OS_ERR_NONE);

    /* Old versions of lwIP required us to return the time waited.
       This is not the case any more. Just returning != SYS_ARCH_TIMEOUT
       here is enough. */
    return 1;
}

void sys_sem_free(sys_sem_t *sem)
{
    INT8U err;

    LWIP_ASSERT("sem != NULL", sem != NULL);
    LWIP_ASSERT("sem->sem != NULL", sem->sem != NULL);

    SYS_STATS_DEC(sem.used);
    OSSemDel(sem->sem, OS_DEL_ALWAYS, &err);
    sem->sem = NULL;
}

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    LWIP_ASSERT("mbox != NULL", mbox != NULL);
    LWIP_ASSERT("size > 0", size > 0);

    INT8U err;
    mbox->pblk = OSMemGet(p_mbox_mem, &err);

    if (err != OS_ERR_NONE)
    {
        SYS_STATS_INC(mbox.err);
        return ERR_MEM;
    }

    mbox->mbx = OSQCreate((void **)mbox->pblk, size);

    if (mbox->mbx == NULL)
    {
        SYS_STATS_INC(mbox.err);
        return ERR_MEM;
    }
    SYS_STATS_INC_USED(mbox);
    return ERR_OK;
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
    INT8U ret;
    LWIP_ASSERT("mbox != NULL", mbox != NULL);
    LWIP_ASSERT("mbox->mbx != NULL", mbox->mbx != NULL);

    ret = OSQPost(mbox->mbx, msg);

    LWIP_ASSERT("mbox post failed", ret == OS_ERR_NONE);
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
    INT8U ret;
    LWIP_ASSERT("mbox != NULL", mbox != NULL);
    LWIP_ASSERT("mbox->mbx != NULL", mbox->mbx != NULL);

    ret = OSQPost(mbox->mbx, msg);
    if (ret == OS_ERR_NONE)
    {
        return ERR_OK;
    }
    else
    {
        LWIP_ASSERT("mbox trypost failed", ret == OS_ERR_Q_FULL);
        SYS_STATS_INC(mbox.err);
        return ERR_MEM;
    }
}

err_t sys_mbox_trypost_fromisr(sys_mbox_t *mbox, void *msg)
{
    return sys_mbox_trypost(mbox, msg);
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout_ms)
{
    INT8U err;
    void *msg_dummy;
    LWIP_ASSERT("mbox != NULL", mbox != NULL);
    LWIP_ASSERT("mbox->mbx != NULL", mbox->mbx != NULL);

    if (!msg)
    {
        msg = &msg_dummy;
    }

    *msg = OSQPend(mbox->mbx, timeout_ms * OS_TICKS_PER_SEC / 1000, &err);
    if (err == OS_ERR_TIMEOUT)
    {
        /* timed out */
        *msg = NULL;
        return SYS_ARCH_TIMEOUT;
    }
    LWIP_ASSERT("mbox fetch failed", err == OS_ERR_NONE);

    /* Old versions of lwIP required us to return the time waited.
       This is not the case any more. Just returning != SYS_ARCH_TIMEOUT
       here is enough. */
    return 1;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    INT8U err;
    void *msg_dummy;
    LWIP_ASSERT("mbox != NULL", mbox != NULL);
    LWIP_ASSERT("mbox->mbx != NULL", mbox->mbx != NULL);

    if (!msg)
    {
        msg = &msg_dummy;
    }

    *msg = OSQPend(mbox->mbx, 1, &err);
    if (err == OS_ERR_TIMEOUT)
    {
        /* timed out */
        *msg = NULL;
        return SYS_MBOX_EMPTY;
    }
    LWIP_ASSERT("mbox fetch failed", err == OS_ERR_NONE);

    return 0;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
    LWIP_ASSERT("mbox != NULL", mbox != NULL);
    LWIP_ASSERT("mbox->mbx != NULL", mbox->mbx != NULL);

    INT8U err;

    OSQDel(mbox->mbx, OS_DEL_ALWAYS, &err);

    if (err != OS_ERR_NONE)
    {
        SYS_STATS_DEC(mbox.used);
    }
    err = OSMemPut(p_mbox_mem, mbox->pblk);
}

sys_thread_t
sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
    INT8U err;
    sys_thread_t lwip_thread;
    OS_STK *ptos;
    OS_STK *pbos;
    static OS_STK thread_stk[LWIP_UCOSII_MAX_NUM_OF_TASK][LWIP_UCOSII_TASK_STK_SIZE];

    static uint8_t index_of_freeStk = 0;


    if (index_of_freeStk < LWIP_UCOSII_MAX_NUM_OF_TASK)
    {
        pbos = &thread_stk[index_of_freeStk][0];
        ptos = &thread_stk[index_of_freeStk][LWIP_UCOSII_TASK_STK_SIZE -1];
    }
    else
    {
        LWIP_ASSERT("invalid stacksize", stacksize > 0);
        lwip_thread.id = 0xFF;
        return lwip_thread;
    }

    LWIP_ASSERT("invalid stacksize", stacksize > 0);
    LWIP_ASSERT("invalid prio", prio > 0);

    err = OSTaskCreateExt(thread, arg, ptos, prio, prio, pbos, LWIP_UCOSII_TASK_STK_SIZE, NULL, 0);

    LWIP_ASSERT("task creation failed", err == OS_ERR_NONE);

    lwip_thread.id = (uint8_t)prio;
    return lwip_thread;
}

#if LWIP_NETCONN_SEM_PER_THREAD
#error Not support LWIP_NETCONN_SEM_PER_THREAD

sys_sem_t *
sys_arch_netconn_sem_get(void)
{
    void *ret;
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    LWIP_ASSERT("task != NULL", task != NULL);

    ret = pvTaskGetThreadLocalStoragePointer(task, 0);
    return ret;
}

void sys_arch_netconn_sem_alloc(void)
{
    void *ret;
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    LWIP_ASSERT("task != NULL", task != NULL);

    ret = pvTaskGetThreadLocalStoragePointer(task, 0);
    if (ret == NULL)
    {
        sys_sem_t *sem;
        err_t err;
        /* need to allocate the memory for this semaphore */
        sem = mem_malloc(sizeof(sys_sem_t));
        LWIP_ASSERT("sem != NULL", sem != NULL);
        err = sys_sem_new(sem, 0);
        LWIP_ASSERT("err == ERR_OK", err == ERR_OK);
        LWIP_ASSERT("sem invalid", sys_sem_valid(sem));
        vTaskSetThreadLocalStoragePointer(task, 0, sem);
    }
}

void sys_arch_netconn_sem_free(void)
{
    void *ret;
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    LWIP_ASSERT("task != NULL", task != NULL);

    ret = pvTaskGetThreadLocalStoragePointer(task, 0);
    if (ret != NULL)
    {
        sys_sem_t *sem = ret;
        sys_sem_free(sem);
        mem_free(sem);
        vTaskSetThreadLocalStoragePointer(task, 0, NULL);
    }
}

#endif /* LWIP_NETCONN_SEM_PER_THREAD */
