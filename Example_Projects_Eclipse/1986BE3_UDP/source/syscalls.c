// Parts of this file are from the newlib sources, issued under GPL.
// Copyright (c) 2014 Liviu Ionescu

int errno;
void *__dso_handle __attribute__ ((weak));

#include <errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>

#include "MDR1986VE3.h"
#include "MDR32F9Qx_uart.h"

// This is the standard default implementation for the routine to
// process args. It returns a single empty arg.
// For semihosting applications, this is redefined to get the real
// args from the debugger. You can also use it if you decide to keep
// some args in a non-volatile memory.

void __attribute__((weak))
__initialize_args(int* p_argc, char*** p_argv)
{
  // By the time we reach this, the data and bss should have been initialised.

  // The strings pointed to by the argv array shall be modifiable by the
  // program, and retain their last-stored values between program startup
  // and program termination. (static, no const)
  static char name[] = "";

  // The string pointed to by argv[0] represents the program name;
  // argv[0][0] shall be the null character if the program name is not
  // available from the host environment. argv[argc] shall be a null pointer.
  // (static, no const)
  static char* argv[2] =
    { name, NULL };

  *p_argc = 1;
  *p_argv = &argv[0];
  return;
}

// If you need the empty definitions, remove the -ffreestanding option.
#if __STDC_HOSTED__ == 1

char* __env[1] =
  { 0 };
char** environ = __env;

int __attribute__((weak))
_chown(const char* path __attribute__((unused)),
    uid_t owner __attribute__((unused)), gid_t group __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_close(int fildes __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_execve(char* name __attribute__((unused)), char** argv __attribute__((unused)),
    char** env __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_fork(void)
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_fstat(int fildes __attribute__((unused)),
    struct stat* st __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_getpid(void)
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_gettimeofday(struct timeval* ptimeval __attribute__((unused)),
    void* ptimezone __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_isatty(int file __attribute__((unused)))
{
  errno = ENOSYS;
  return 0;
}

int __attribute__((weak))
_kill(int pid __attribute__((unused)), int sig __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_link(char* existing __attribute__((unused)),
    char* _new __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_lseek(int file __attribute__((unused)), int ptr __attribute__((unused)),
    int dir __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_open(char* file __attribute__((unused)), int flags __attribute__((unused)),
    int mode __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_read(int file __attribute__((unused)), char* ptr __attribute__((unused)),
    int len __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_readlink(const char* path __attribute__((unused)),
    char* buf __attribute__((unused)), size_t bufsize __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_stat(const char* file __attribute__((unused)),
    struct stat* st __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_symlink(const char* path1 __attribute__((unused)),
    const char* path2 __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

clock_t __attribute__((weak))
_times(struct tms* buf __attribute__((unused)))
{
  errno = ENOSYS;
  return ((clock_t) -1);
}

int __attribute__((weak))
_unlink(char* name __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_wait(int* status __attribute__((unused)))
{
  errno = ENOSYS;
  return -1;
}

int __attribute__((weak))
_write(int file __attribute__((unused)), char* ptr, int len )
{
  for ( int i = 0; i < len; i++ ) {
    UART_SendData( DEBUG_UART, ( uint8_t ) ptr[ i ]);
    while ( UART_GetFlagStatus( DEBUG_UART, UART_FLAG_TXFF ) == SET );
  }
  return ( len );
}

#endif // __STDC_HOSTED__ == 1
