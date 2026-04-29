/* Bare-metal newlib stubs.
 * We don't use printf in the firmware, so these are no-ops to satisfy linker.
 */
#include <stdint.h>
#include <stddef.h>
#include <sys/stat.h>

int _write(int file, const char *ptr, int len)   { (void)file; (void)ptr; return len; }
int _read(int file, char *ptr, int len)          { (void)file; (void)ptr; (void)len; return 0; }
int _close(int file)                             { (void)file; return -1; }
int _isatty(int file)                            { (void)file; return 1; }
int _lseek(int file, int ptr, int dir)           { (void)file; (void)ptr; (void)dir; return 0; }
int _fstat(int file, struct stat *st)            { (void)file; (void)st; return 0; }
int _getpid(void)                                { return 1; }
int _kill(int pid, int sig)                      { (void)pid; (void)sig; return -1; }
void _exit(int status)                           { (void)status; while (1) {} }

extern char _end;
static char *heap_end = NULL;
void *_sbrk(int incr) {
    if (heap_end == NULL) heap_end = &_end;
    char *prev = heap_end;
    heap_end += incr;
    return prev;
}
