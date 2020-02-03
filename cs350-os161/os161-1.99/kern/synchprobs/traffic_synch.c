#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */


#define typeNum 4
#define empty 0
static struct lock *intersection_lk;
static volatile int curnumLeft;
static volatile Direction curtype;
static volatile bool blocked;

/* There are 4 types: 0 = all traffic from North
                      1 = all traffic from East
                      2 = all traffic from South
                      3 = all traffic from West
*/

static volatile int wait_on_type[typeNum];
static struct cv *cv_pos[typeNum];

/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */

void
intersection_sync_init(void)
{
  intersection_lk = lock_create("intersection lock");
  if (intersection_lk == NULL) {
    panic("Unable to create intersection lock");
  }
  for (int i = 0; i < typeNum; i++) {
    wait_on_type[i] = 0;
  }
  curnumLeft = 1;
  curtype = 10;
  blocked = false;

  cv_pos[0] = cv_create("all traffic from north"); // type 0
  if (cv_pos[0] == NULL) {
    panic("Unable to create cv_N");
    lock_destroy(intersection_lk);
    return;
  }
  cv_pos[1] = cv_create("all traffic from south"); // type 1
  if (cv_pos[1] == NULL) {
    panic("Unable to create cv_S");
    lock_destroy(intersection_lk);
    return;
  }
  cv_pos[2] = cv_create("all traffic from east"); // type 2
  if (cv_pos[2] == NULL) {
    panic("Unable to create cv_E");
    lock_destroy(intersection_lk);
    return;
  }
  cv_pos[3] = cv_create("all traffic from west"); // type 3
  if (cv_pos[3] == NULL) {
    panic("Unable to create cv_W");
    lock_destroy(intersection_lk);
    return;
  }
  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  KASSERT(intersection_lk != NULL);
  lock_destroy(intersection_lk);

  for (int i = 0; i < typeNum; ++i) {
    KASSERT(wait_on_type[i] == 0);
    KASSERT(cv_pos[i] != NULL);
    cv_destroy(cv_pos[i]);
  }
}

/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  (void)destination;

  KASSERT(intersection_lk != NULL);
  lock_acquire(intersection_lk);
  if (curtype == 10) { // first traffic entered.
    curtype = origin;
  }
  if (origin != curtype || blocked) {
    wait_on_type[origin] += 1;
    cv_wait(cv_pos[origin],intersection_lk);
  }
  blocked = true;
  lock_release(intersection_lk);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  (void)origin;  
  (void)destination;
  
  lock_acquire(intersection_lk);
  curnumLeft -= 1;
  if (curnumLeft <= 0) {
    if (wait_on_type[0] <= 0 && wait_on_type[1] <= 0 && wait_on_type[2] <= 0 && wait_on_type[3] <= 0) {
      curnumLeft = 1;
      blocked = false;
      curtype = 10;
      lock_release(intersection_lk);
      return;
    }
    curnumLeft = 0;
    blocked = true;
    curtype = (curtype + 1) % 4;
    while(wait_on_type[curtype] <= 0 ) {
      curtype = (curtype + 1) % 4;
    }
    curnumLeft = wait_on_type[curtype];
    wait_on_type[curtype] = 0;
    cv_broadcast(cv_pos[curtype], intersection_lk);
  }
  lock_release(intersection_lk);
}
