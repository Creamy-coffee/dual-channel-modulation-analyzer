#include "serial_deadline.h"
#include <assert.h>
#include <stdio.h>
typedef struct {int64_t ms;int mode,written,cancel_at,queue_polls;} fake;
static int64_t now(void *p){return ((fake *)p)->ms;}
static int cancel(void *p){fake *f=p;return f->cancel_at && f->ms>=f->cancel_at;}
static int output(void *p,const unsigned char *data,size_t n){
 fake *f=p;int k;(void)data;
 if(f->mode==1){errno=EAGAIN;return -1;}
 if(f->mode==2){errno=EINTR;return -1;}
 if(f->mode==3)return 0;
 if(f->mode==4){errno=EIO;return -1;}
 k=n>2?2:(int)n;f->written+=k;return k;
}
static int queue(void *p){fake *f=p;f->queue_polls++;return f->mode==5?10:0;}
static int wait_ms(void *p,int ms){((fake *)p)->ms+=ms;return 0;}
static int run(fake *f){const unsigned char b[]={1,2,3,4,5};serial_deadline_ops o={f,now,cancel,output,queue,wait_ms};return serial_send_bounded(&o,b,sizeof(b),100);}
int main(void){
 int m;fake f={0};assert(run(&f)==0 && f.written==5 && f.queue_polls==1);
 for(m=1;m<=5;m++){
  fake t={0};t.mode=m;errno=0;
  assert(run(&t)==-1);
  if(m==4)assert(errno==EIO);else assert(errno==ETIMEDOUT && t.ms==100);
 }
 {fake t={0};t.mode=1;t.cancel_at=15;assert(run(&t)==-1 && errno==EINTR && t.ms==15);}
 puts("SERIAL_DEADLINE_PASS partial_write EAGAIN EINTR zero_progress io_error drain_timeout cancel");return 0;
}
