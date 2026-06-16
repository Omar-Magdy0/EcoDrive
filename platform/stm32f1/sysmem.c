/**
  ******************************************************************************
  * @file      sysmem.c
  * @author    MCD Application Team
  * @brief     This file implements __sbrk function for memory allocation
  ******************************************************************************
  */

#include <errno.h>
#include <stddef.h>
#include <sys/types.h>

/**
 * Increase program data space.
 * Stub only, does not adjust heap.
 */
caddr_t _sbrk(int incr)
{
  extern char end asm("end");
  static char *heap_end;
  char *prev_heap_end;
  
  if (heap_end == NULL)
    heap_end = &end;

  prev_heap_end = heap_end;

  if (heap_end + incr - &end > 20480) /* limit heap to about 20K */
  {
    errno = ENOMEM;
    return (caddr_t) -1;
  }

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}
