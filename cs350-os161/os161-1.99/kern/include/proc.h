/*
 * Copyright (c) 2013
 *	The President and Fellows of Harvard College.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _PROC_H_
#define _PROC_H_

/*
 * Definition of a process.
 *
 * Note: curproc is defined by <current.h>.
 */

#include <spinlock.h>
#include <thread.h> /* required for struct threadarray */
#include <array.h>
#include <opt-A2.h>

struct addrspace;
struct vnode;
#ifdef UW
struct semaphore;
#endif // UW


#if OPT_A2
struct lock;
struct cv;

struct array *proc_table; // an array of proc
struct lock *proc_table_lock;
struct cv *proc_table_cv;

struct proc_info {
	pid_t self_pid;
	pid_t parent_pid;
	int alive_or_not; // 1 indicates alive and 0 indicates zombie state
	int exit_code;
};


#endif /* OPT_A2 */


/*
 * Process structure.
 */
struct proc {
	char *p_name;			/* Name of this process */
	struct spinlock p_lock;		/* Lock for this structure */
	struct threadarray p_threads;	/* Threads in this process */

	/* VM */
	struct addrspace *p_addrspace;	/* virtual address space */

	/* VFS */
	struct vnode *p_cwd;		/* current working directory */

#ifdef UW
  /* a vnode to refer to the console device */
  /* this is a quick-and-dirty way to get console writes working */
  /* you will probably need to change this when implementing file-related
     system calls, since each process will need to keep track of all files
     it has opened, not just the console. */
  struct vnode *console;                /* a vnode for the console device */
#endif

	/* add more material here as needed */
#if OPT_A2
   pid_t pid;
   pid_t parent_pid;
#endif /* OPT_A2 */

};

/* This is the process structure for the kernel and for kernel-only threads. */
extern struct proc *kproc;

/* Semaphore used to signal when there are no more processes */
#ifdef UW
extern struct semaphore *no_proc_sem;
#endif // UW

/* Call once during system startup to allocate data structures. */
void proc_bootstrap(void);

/* Create a fresh process for use by runprogram(). */
struct proc *proc_create_runprogram(const char *name);

/* Destroy a process. */
void proc_destroy(struct proc *proc);

/* Attach a thread to a process. Must not already have a process. */
int proc_addthread(struct proc *proc, struct thread *t);

/* Detach a thread from its process. */
void proc_remthread(struct thread *t);

/* Fetch the address space of the current process. */
struct addrspace *curproc_getas(void);

/* Change the address space of the current process, and return the old one. */
struct addrspace *curproc_setas(struct addrspace *);


// proc_table
#if OPT_A2
//create: return 0 for success and ENOMEM for failure
int proc_table_create(void);

//add: add a proc to current proc_table
void proc_table_add(struct proc *new_proc);

//remove: remove a proc from the current proc_table
void proc_table_remove(pid_t pid);

//find: find the proc with the given pid, return the proc if it is found, and NULL otherwise.
struct proc_info *find_proc_info(pid_t pid);

//clear: clear all dead children for the given process. 
void proc_clear_dead_children(pid_t parent_pid);
#endif /* OPT_A2 */


#endif /* _PROC_H_ */
