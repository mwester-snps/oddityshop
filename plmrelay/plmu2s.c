/* m.c
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Make an Insteon 2413u look like a 2413s to a Universal Devices ISY
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/select.h>

/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */

/* TTY definitions for the ISY and PLM devices */
#define PLMTTY "/dev/ttyUSB0"
#define ISYTTY "/dev/ttyUSB1"

/* 100 ms max wait in main loop if no input */
#define UWAIT 100000

/* Standard way to check and exit if error... */
#define CHKERR(a,s) {if ((a)<0) {perror((s)); exit(1);}}

/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */

/* Main state machine variables */
int state = 0;
unsigned char matchsequence[] = {0x02, 0x06, 0x02, 0x00, 0x00};
unsigned char replacement = 0x11;

/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */

static int open_tty(char *ttyname) {
  struct termios ts;
  int i, ttyfd;
  
  ttyfd = open(ttyname, O_RDWR | O_NDELAY);
  CHKERR(ttyfd, ttyname);

  i = tcgetattr(ttyfd, &ts);
  CHKERR(i, "tcgetattr");

  cfmakeraw(&ts);

  ts.c_iflag &= ~(IXON | IXOFF);
  ts.c_iflag |= IGNBRK | IGNPAR;
  ts.c_cflag |= CLOCAL;
  ts.c_cflag &= ~CRTSCTS;

  i = cfsetispeed(&ts, B19200);
  CHKERR(i, "cfsetispeed");

  i = cfsetospeed(&ts, B19200);
  CHKERR(i, "cfsetospeed");

  i = tcsetattr(ttyfd, TCSANOW, &ts);
  CHKERR(i, "tcsetattr");

  i = fcntl(ttyfd, F_GETFL);
  CHKERR(i, "fcntl F_GETFL");
  i = fcntl(ttyfd, i & ~O_NONBLOCK);
  CHKERR(i, "fcntl F_SETFL");

  tcflush(ttyfd, TCIOFLUSH);

  return ttyfd;
}

/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */

void blog(char d, unsigned char *b, int n) {
  int i;
  if (n > 0) {
    putchar(d);
    for (i = 0; i < n; i++) {
      printf(" 0x%02X", (int) b[i]);
    }
    putchar('\n');
  }
}

/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */

void munge(unsigned char *b, int n) {
  int i;
  for (i = 0; i < n; i++) {
    if (matchsequence[state] == b[i]) {
      state++;
      if (matchsequence[state] == 0x00) {
	b[i] = replacement;
	state = 0;
      }
    } else {
      state = 0;
    }
  }
}

/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */

void bsend(int fd, unsigned char *b, int n) {
  int w = 0;
  while (w < n) {
    int i = write(fd, b + w, n - w);
    w += i;
  }
}

/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */

int main(int argc, char *const *argv) {
  fd_set fds;
  struct timeval tval;
  int i, n, nfds;
  unsigned char buf[8192];

  int plm = open_tty(PLMTTY);
  int isy = open_tty(ISYTTY);

  nfds = (plm > isy) ? (plm + 1) : (isy + 1);

  while (1) {

    tval.tv_sec = 0;
    tval.tv_usec = UWAIT;

    FD_ZERO(&fds);
    FD_SET(plm, &fds);
    FD_SET(isy, &fds);

    i = select(nfds, &fds, NULL, NULL, &tval);
    CHKERR(i, "select");

    if (FD_ISSET(plm, &fds)) {
      n = read(plm, buf, sizeof(buf));
      munge(buf, n);
      bsend(isy, buf, n);
      blog('>', buf, n);
    }

    if (FD_ISSET(isy, &fds)) {
      n = read(isy, buf, sizeof(buf));
      bsend(plm, buf, n);
      blog('<', buf, n);
    }

  }

}

#ifdef USE_BTREE
/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */
static void execute_rollback_list(Rbtree *pRbtree, BtRollbackOp *pList)
{
  BtRollbackOp *pTmp;
  RbtCursor cur;
  int res;

  cur.pRbtree = pRbtree;
  cur.wrFlag = 1;
  while( pList ){
    switch( pList->eOp ){
      case ROLLBACK_INSERT:
        cur.pTree  = sqliteHashFind( &pRbtree->tblHash, 0, pList->iTab );
        assert(cur.pTree);
        cur.iTree  = pList->iTab;
        cur.eSkip  = SKIP_NONE;
        memRbtreeInsert( &cur, pList->pKey,
            pList->nKey, pList->pData, pList->nData );
        break;
      case ROLLBACK_DELETE:
        cur.pTree  = sqliteHashFind( &pRbtree->tblHash, 0, pList->iTab );
        assert(cur.pTree);
        cur.iTree  = pList->iTab;
        cur.eSkip  = SKIP_NONE;
        memRbtreeMoveto(&cur, pList->pKey, pList->nKey, &res);
        assert(res == 0);
        memRbtreeDelete( &cur );
        break;
      case ROLLBACK_CREATE:
        btreeCreateTable(pRbtree, pList->iTab);
        break;
      case ROLLBACK_DROP:
        memRbtreeDropTable(pRbtree, pList->iTab);
        break;
      default:
        assert(0);
    }
    sqliteFree(pList->pKey);
    sqliteFree(pList->pData);
    pTmp = pList->pNext;
    sqliteFree(pList);
    pList = pTmp;
  }
}
/*   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   *   */
#endif
