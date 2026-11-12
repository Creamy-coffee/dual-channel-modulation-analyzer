#include "iq_measure.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define PI 3.14159265358979323846
int main(void) {
 int16_t i[1024],q[1024];size_t k;int t;double fs=65000000.0/2048;
 iq_bandwidth b;iq_bandwidth_control c={0};
 for(k=0;k<1024;k++){double a=2*PI*600*k/fs;i[k]=(int16_t)(1000*cos(a));q[k]=(int16_t)(1000*sin(a));}
 if(iq_bandwidth_estimate(i,q,1024,fs,3,6000,&b)||!b.valid||fabs(b.radius_hz-600)>6*fs/1024)return 1;
 for(t=0;t<5;t++)if(iq_bandwidth_choose(&c,&b,2,fs,0)!=-1)return 2;
 if(iq_bandwidth_choose(&c,&b,2,fs,0)!=0)return 3;
 b.radius_hz=1800;b.edge_fraction=.02;b.valid=1;
 if(iq_bandwidth_choose(&c,&b,0,65000000.0/8192,0)!=1)return 4;
 if(iq_bandwidth_choose(&c,&b,1,65000000.0/4096,0)!=2)return 5;
 memset(&c,0,sizeof(c));b.radius_hz=100;b.edge_fraction=0;
 /* Periodic wide probe still occurs without edge power in the narrow data. */
 for(t=0;t<15;t++)if(iq_bandwidth_choose(&c,&b,0,65000000.0/8192,0)!=-1)return 6;
 if(iq_bandwidth_choose(&c,&b,0,65000000.0/8192,0)!=2)return 7;
 memset(i,0,sizeof(i));memset(q,0,sizeof(q));
 if(iq_bandwidth_estimate(i,q,1024,fs,3,6000,&b)||b.valid)return 8;
 memset(&c,0,sizeof(c));for(t=0;t<10;t++)if(iq_bandwidth_choose(&c,&b,2,fs,0)!=-1)return 9;
 /* Two off-centre sidebands must be retained even with a strong AM carrier. */
 for(k=0;k<1024;k++){i[k]=(int16_t)(1000+600*cos(2*PI*4000*k/fs));q[k]=0;}
 if(iq_bandwidth_estimate(i,q,1024,fs,3,6000,&b)||!b.valid||b.radius_hz<3900)return 10;
 for(k=0;k<1024;k++){i[k]=(int16_t)(1000+20*cos(2*PI*4000*k/fs));q[k]=0;}
 if(iq_bandwidth_estimate(i,q,1024,fs,3,6000,&b)||!b.valid||b.radius_hz<3900)return 11;
 puts("IQ_BANDWIDTH_PASS tone, AM sidebands, margin, six-block dwell, expansion, periodic probe, no signal");return 0;
}
