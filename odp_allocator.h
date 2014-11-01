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

#include <new>
#include <typeinfo>
#include <string>
#include <algorithm>

#include <odp_shared_memory.h>
#include <odp_buffer_pool.h>
#include <odp_atomic.h>
#include <odp_hints.h>

#include "odp_mem_alloc.h"

/* ODP Allocator for standard STL uses.
 * 
 * Currently, an odp_allocator instance cannot have a global lifetime.
 * Meaning, no global STL data structures using this allocator.
 *
 * User must initialize the ODP library and also call odp_mem_init() prior 
 * using an odp_allocator.
 */

template <class T, size_t ODP_POOL_SIZE = (256<<10)>
class odp_allocator {
public:
    typedef T value_type;
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    template <class U> struct rebind {
	typedef odp_allocator<U> other;
    };
 
    odp_allocator() {}
    template <class U> odp_allocator(const odp_allocator<U> &) {}

    pointer address(reference r) const { return &r; }
    const_pointer address(const_reference r) const { return &r; }
    
    void construct(pointer p, const_reference v) { ::new((void *)p)T(v); }
    void destroy(pointer p) { p->~T(); }

    size_type max_size() const {
	return ODP_POOL_SIZE / value_type_size; /* heuristic :) */
    }

    pointer allocate(size_type n, const void * = 0) throw() {
	static const size_type arr_alloc_align =
	    std::max(value_type_align, sizeof(void*));

	if (odp_unlikely(!n))
	    return 0;
	if (odp_unlikely(n > max_size()))
	    throw std::bad_alloc();
	if (n == 1) {
	    /* single object allocation using the dedicated buffer pool */
	    return (pointer)obj_pool.buffer_alloc();
	}
	
	/* For array of objects, use the general purpose odp_mem_alloc */
	void *p = odp_mem_alloc(n * value_type_size + sizeof(void*) +
	    arr_alloc_align);
	if (!p)
	    throw std::bad_alloc();

	size_type delta = arr_alloc_align -
	    ((uintptr_t)p + sizeof(void*)) % arr_alloc_align;
	pointer user_ptr = (pointer)((uintptr_t)p + sizeof(void*) + delta);
	*((void **)user_ptr - 1) = p;
	return user_ptr;
    }

    void deallocate(pointer user_ptr, size_type n) {
	if (odp_unlikely(!n))
	    return;
	if (n == 1)
	    return obj_pool.buffer_free(user_ptr);
	odp_mem_free(*((void **)user_ptr - 1));
    }
    
private:
    class odp_pool;

    /* Single pool for all instances of the exact same allocator type.
     * No per-object data in the allocator.
     *
     * The odp_pool is a dedicated odp_buffer_pool for single object
     * allocations.
     */
    static odp_pool obj_pool;
    static const size_type value_type_size;
    static const size_type value_type_align;

    class odp_pool {
    private:
	odp_buffer_pool_t handle;
	odp_atomic_int_t initialized;

    public:
	odp_pool() : handle(ODP_BUFFER_POOL_INVALID) {
	    /* Don't initialize the odp_buffer_pool yet, as the actual
	     * odp_pool object has global lifetime, and odp lib might not be
	     * initialized yet
	     */
	    odp_atomic_init_int(&initialized);
	}

	void *buffer_alloc() throw() {
	    odp_buffer_t buf_hdl;
	    void *p;

	    if (odp_unlikely(!odp_atomic_load_int(&initialized))) {
		if (!odp_atomic_fetch_inc_int(&initialized))
		    initialize();
	    }

	    if (odp_unlikely(handle == ODP_BUFFER_POOL_INVALID))
		throw std::bad_alloc();

	    if ((buf_hdl = odp_buffer_alloc(handle)) == ODP_BUFFER_INVALID)
		throw std::bad_alloc();

	    p = odp_buffer_addr(buf_hdl);
	    *(odp_buffer_t *)p = buf_hdl;
	    return (uint8_t *)p + value_type_align;
	}

	void buffer_free(void *p) {
	    void *orig = (uint8_t *)p - value_type_align;
	    odp_buffer_free(*(odp_buffer_t *)orig);
	}

    private:
	odp_pool(const odp_pool&);
	odp_pool& operator=(const odp_pool&);

	void initialize() throw() {
	    static const size_type odpbuf_alloc_align =
		std::max(value_type_align, sizeof(odp_buffer_t));
	    const std::string sname =
		std::string("odp_allocator") + typeid(value_type).name();
	    const char* name = sname.c_str();
	    odp_shm_t shm;
	    
	    shm = odp_shm_reserve(name, ODP_POOL_SIZE, odpbuf_alloc_align, 0);
	    if (shm == ODP_SHM_INVALID)
		throw std::bad_alloc();

	    size_type buf_size = value_type_size + odpbuf_alloc_align;

	    handle = odp_buffer_pool_create(name, odp_shm_addr(shm),
		ODP_POOL_SIZE, buf_size, odpbuf_alloc_align,
		ODP_BUFFER_TYPE_RAW);
	    if (handle == ODP_BUFFER_POOL_INVALID)
		throw std::bad_alloc();
	}
    };
};

template <class T, size_t ODP_POOL_SIZE>
const typename odp_allocator<T, ODP_POOL_SIZE>::size_type
odp_allocator<T, ODP_POOL_SIZE>::value_type_size =
    sizeof(odp_allocator::value_type);

template <class T, size_t ODP_POOL_SIZE>
const typename odp_allocator<T, ODP_POOL_SIZE>::size_type
odp_allocator<T, ODP_POOL_SIZE>::value_type_align =
    __alignof__(odp_allocator::value_type);

template <class T, size_t ODP_POOL_SIZE>
typename odp_allocator<T, ODP_POOL_SIZE>::odp_pool
odp_allocator<T, ODP_POOL_SIZE>::obj_pool;

#endif
