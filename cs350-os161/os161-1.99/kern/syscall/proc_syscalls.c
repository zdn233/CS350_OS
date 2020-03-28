#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/fcntl.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include <opt-A2.h>
#include <syscall.h>
#include <mips/trapframe.h>
#include <synch.h>
#include <vfs.h>

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */


  #if OPT_A2
  lock_acquire(proc_table_lock);
  //clear all dead children
  proc_clear_dead_children(p->pid);
  struct proc_info *parent_proc_info = find_proc_info(p->parent_pid);
  //parent is in zombie state or dead, safe to delte curproc
  if (parent_proc_info == NULL) {
    proc_table_remove(p->pid);
  } else if (!parent_proc_info->alive_or_not) {
    proc_table_remove(p->pid);
  } else {
    struct proc_info *myinfo = find_proc_info(p->pid);
    myinfo->alive_or_not = 0; // turns to zombie state now
    myinfo->exit_code = exitcode;
    //tell parent waitpid can proceed if there is any (parent has already called waitpid)
    cv_broadcast(proc_table_cv, proc_table_lock);
  }
  lock_release(proc_table_lock);
  #endif /* OPT_A2 */


  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  #if OPT_A2
  *retval = curproc->pid;
  return(0);
  #else
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  *retval = 1;
  return(0);
  #endif /* OPT_A2 */
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */


  #if OPT_A2
  lock_acquire(proc_table_lock);
  struct proc_info *child_proc_info = find_proc_info(pid);
  if (child_proc_info == NULL) return ESRCH;
  if (child_proc_info->parent_pid != curproc->pid) return ECHILD;
  
  while(child_proc_info->alive_or_not) { //now alive_or_not == 1 (child alive state)
    cv_wait(proc_table_cv, proc_table_lock);
  }

  KASSERT(!child_proc_info->alive_or_not); //now alive_or_not == 0 (child zombie state)
  exitstatus = _MKWAIT_EXIT(child_proc_info->exit_code);

  //child info is retrieved and safe to delete now
  proc_table_remove(pid);

  lock_release(proc_table_lock);
  #endif /* OPT_A2 */


  if (options != 0) {
    return(EINVAL);
  }
  /* for now, just pretend the exitstatus is 0 */
  //exitstatus = 0;
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}

#if OPT_A2
int 
sys_fork(struct trapframe *tf, pid_t *retval) {
  //create child process
  struct proc *child_proc = proc_create_runprogram("child");
  if (child_proc == NULL) return ENOMEM;

  //create new address space and let it associated with the new child process
  as_copy(curproc_getas(), &child_proc->p_addrspace);
  if (child_proc->p_addrspace == NULL) {
    proc_destroy(child_proc);
    return ENOMEM;
  }

  //copy parent trapframe to OS heap, might need to handle error for no mem space
  struct trapframe *copy_parent_tf = (struct trapframe *)kmalloc(sizeof(struct trapframe));
  memcpy(copy_parent_tf, tf, sizeof(struct trapframe));
  //have parent trapframe ready for thread_fork, begin thread_fork
  thread_fork("new child thread", child_proc, &enter_forked_process, copy_parent_tf, 0);

  //get child pid and returns 0 (success)
  *retval = child_proc->pid;
  return 0;
}


int
sys_execv(userptr_t program, char **args) {
  struct addrspace *oldas;
  struct addrspace *newas;
  struct vnode *v;
  vaddr_t entrypoint, stackptr;
  int result;

  oldas = curproc_getas();

  //copy program name from userspace to kernel space
  char *kprogram = kmalloc((sizeof(char)) * (strlen((const char *) program) + 1));
  if (kprogram == NULL) return ENOMEM;
  result = copyinstr(program, kprogram, strlen((const char *) program) + 1, NULL);
  if (result) {
    kfree(kprogram);
    return result;
  }

  //copy args from userspace to kernel space
  int count = 0;
  int total_args_len = 0; // used for alignment later
  while (args[count] != NULL) {
    total_args_len += strlen((const char *) args[count]) + 1;
    count++;
  }

  char **kargs = kmalloc((sizeof(char *)) * (count + 1));
  if (kargs == NULL) {
    kfree(kprogram);
    return ENOMEM;
  }

  for (int i = 0; i < count; ++i) {
    kargs[i] = kmalloc((sizeof(char)) * (strlen(args[i]) + 1));
    if (kargs[i] == NULL) {
      for (int j = 0; j < i; ++j) {
        kfree(kargs[i]);
      }
      kfree(kargs);
      kfree(kprogram);
      return ENOMEM;
    }
    result = copyinstr((const_userptr_t) args[i], kargs[i], strlen(args[i]) + 1, NULL);
    if (result) {
      for (int j = 0; j < i; ++j) {
        kfree(kargs[i]);
      }
      kfree(kargs);
      kfree(kprogram);
      return result;
    }
  }
  kargs[count] = NULL;

  /* Open the file. */
  result = vfs_open(kprogram, O_RDONLY, 0, &v);
  if (result) {
    return result;
  }

  // /* We should be a new process. */
  // KASSERT(curproc_getas() == NULL);

  /* Create a new address space. */
  newas = as_create();
  if (newas ==NULL) {
    vfs_close(v);
    return ENOMEM;
  }

  /* Switch to it and activate it. */
  curproc_setas(newas);
  as_activate();

  /* Load the executable. */
  result = load_elf(v, &entrypoint);
  if (result) {
    /* p_addrspace will go away when curproc is destroyed */
    vfs_close(v);
    return result;
  }

  /* Done with the file now. */
  vfs_close(v);

  /* Define the user stack in the address space */
  result = as_define_stack(newas, &stackptr);
  if (result) {
    /* p_addrspace will go away when curproc is destroyed */
    return result;
  }

  // Copy args back to the user stack
  int num_for_string = DIVROUNDUP(total_args_len, 4);
  int required = ROUNDUP(count + num_for_string + 1, 2);
  int start_for_string = required - count - 1;
  vaddr_t array_of_args_stackaddr[count + 1];
  vaddr_t bot_of_stack = stackptr;
  
  //copy strings first
  stackptr = stackptr - (start_for_string * 4);
  array_of_args_stackaddr[count] = 0;
  for(int i = 0; i < count; ++i) {
    array_of_args_stackaddr[i] = stackptr;
    result = copyoutstr(kargs[i], (userptr_t) stackptr, strlen(kargs[i]) + 1, NULL);
    if (result) {
      return result;
    }
    stackptr = stackptr + strlen(kargs[i]) + 1;
  }

  //copy stack addresses next
  stackptr = bot_of_stack;
  stackptr = stackptr - (required * 4);
  for (int i = 0; i <= count; ++i) {
    result = copyout(&array_of_args_stackaddr[i], (userptr_t) stackptr, sizeof(vaddr_t));
    if (result) {
      return result;
    }
    stackptr += 4;
  }

  //change stackptr pointing to the top of the stack
  stackptr = bot_of_stack;
  stackptr = stackptr - (required * 4);

  //destroy old address space
  as_destroy(oldas);

  /* Warp to user mode. */
  enter_new_process(count /*argc*/, (userptr_t) stackptr /*userspace addr of argv*/,
        stackptr, entrypoint);
  
  /* enter_new_process does not return. */
  panic("enter_new_process returned\n");
  return EINVAL;
}

#endif /* OPT_A2 */

