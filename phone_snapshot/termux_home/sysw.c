#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/ptrace.h>
#include <sys/wait.h>
int main(int argc, char **argv) {
  pid_t p = fork();
  if (!p) { ptrace(PTRACE_TRACEME, 0, 0, 0); execv(argv[1], argv + 1); perror("exec"); return 127; }
  int st;
  while (waitpid(p, &st, 0) > 0) {
    if (WIFEXITED(st)) { printf("exit %d\n", WEXITSTATUS(st)); break; }
    if (WIFSIGNALED(st)) { printf("killed by %d\n", WTERMSIG(st)); break; }
    if (WIFSTOPPED(st)) {
      int sig = WSTOPSIG(st);
      if (sig == SIGSYS) { siginfo_t si; ptrace(PTRACE_GETSIGINFO, p, 0, &si); printf("SIGSYS syscall=%d\n", si.si_syscall); }
      ptrace(PTRACE_CONT, p, 0, sig == SIGTRAP ? 0 : sig);
    }
  }
  return 0;
}
