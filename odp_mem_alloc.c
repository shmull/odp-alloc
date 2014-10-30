/*
 * Copyright (c) 2014, Shmulik Ladkani
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _ODP_ALLOCATOR_H_
#define _ODP_ALLOCATOR_H_

#include <odp_shared_memory.h>
#include <odp_buffer_pool.h>
#include <stdio.h>

#define BUF_ALIGNMENT 8
#define POOL_SIZE (256*1024)

#define MIN_ALLOC_SHIFT 7
#define MIN_ALLOC_SZ (1 << MIN_ALLOC_SHIFT)
#define MAX_ALLOC_SZ (16 * 1024)

#define sz_2_pool_idx(sz) ((sz) == 0 ? 0 : \
	((unsigned long)((sz) - 1) >> MIN_ALLOC_SHIFT) == 0 ? 0 : \
	32 - __builtin_clzl((unsigned long)((sz) - 1) >> MIN_ALLOC_SHIFT))

#define NUM_POOLS sz_2_pool_idx(MAX_ALLOC_SZ)
static odp_buffer_pool_t pool[NUM_POOLS];

int odp_mem_init(void)
{
    odp_shm_t shm;
    char name[64];
    size_t sz;
    int i;
    
    /* compile time assert. ensure BUF_ALIGNMENT >= sizeof(odp_buffer_t) */
    { sizeof(int [1 - 2 * !!(BUF_ALIGNMENT < sizeof(odp_buffer_t))]); }

    /* initialize pool array as non existing pools */
    for (i = 0; i < NUM_POOLS; i++)
	pool[i] = ODP_BUFFER_POOL_INVALID;

    for (i = 0, sz = MIN_ALLOC_SZ; i < NUM_POOLS; i++, sz <<= 1)
    {
	/* initialize every other pool */
	if (i % 2)
	    continue;
	sprintf(name, "odp_mem_alloc%d", sz);

	shm = odp_shm_reserve(name, POOL_SIZE, BUF_ALIGNMENT, 0);
	if (shm == ODP_SHM_INVALID)
	    goto Err;

	pool[i] = odp_buffer_pool_create(name, odp_shm_addr(shm), POOL_SIZE,
	    sz + BUF_ALIGNMENT, BUF_ALIGNMENT, ODP_BUFFER_TYPE_RAW);
	if (pool[i] == ODP_BUFFER_POOL_INVALID)
	    goto Err;
    }
    return 0;

Err:
    for (i = 0; i < NUM_POOLS; i++)
	pool[i] = ODP_BUFFER_POOL_INVALID;
    return -1;
}

void *odp_mem_alloc(size_t size)
{
    odp_buffer_t buf_hdl = ODP_BUFFER_INVALID;
    void *p;
    int i;

    i = sz_2_pool_idx(size);
    for (; buf_hdl == ODP_BUFFER_INVALID && i < NUM_POOLS; i++)
    {
	if (pool[i] == ODP_BUFFER_POOL_INVALID)
	    continue;
	buf_hdl = odp_buffer_alloc(pool[i]);
    }
    if (buf_hdl == ODP_BUFFER_INVALID)
	return NULL;

    p = odp_buffer_addr(buf_hdl);
    *(odp_buffer_t *)p = buf_hdl;
    return p + BUF_ALIGNMENT;
}

void odp_mem_free(void *p)
{
    odp_buffer_t *orig;

    if (!p)
	return;
    orig = p - BUF_ALIGNMENT;
    odp_buffer_free(*orig);
}

#endif
