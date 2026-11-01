#include "harmonic_receiver.h"
#include <stdio.h>
int main(int argc,char **argv){
 if(argc!=2)return 2;
 FILE *f=fopen(argv[1],"rb");if(!f)return 2;
 unsigned char p[216];
 while(fread(p,1,sizeof p,f)==sizeof p){
  v32_harmonic_measurement m;
  if(!v32_harmonic_decode(p,sizeof p,&m)){puts("0");continue;}
  printf("1 %u %d %u %u\n",m.version,m.thd_valid,m.thd_q24,m.flags);
 }
 int bad=ferror(f);fclose(f);return bad?2:0;
}
