/* Exercise the real receiver callback and DSP across acknowledged rate changes. */
#define main iq_demod_program_main
#include "../linux/iq_demod.c"
#undef main
static void sample_frame(receiver *r,iq_frame_t *h,double hz) {
 unsigned char data[1068]={0};unsigned k;
 double fs=(double)h->fs_hz/h->decimation;
 for(k=0;k<256;k++) {
  double a=2*PI*hz*((h->frame_id-1)*256+k)/fs;
  int16_t v=(int16_t)(800+300*cos(a));
  data[40+4*k]=(unsigned char)v;data[41+4*k]=(unsigned char)((uint16_t)v>>8);
 }
 frame(r,data,sizeof(data),h);
}
int main(void) {
 receiver r; iq_decoder_t d;unsigned char dbuf[IQ_DECODER_REC_CAP];int k;
 iq_frame_t h={1,1,65000000,2048,256,0,10,0};
 memset(&r,0,sizeof(r));r.mode=1;r.rate=2000;r.floor=3;r.json=tmpfile();
 if(!r.json)return 1;r.decoder=&d;r.auto_bandwidth=1;r.pending_rate=-1;r.waiting_rate=-1;
 if(iq_decoder_init(&d,dbuf,sizeof(dbuf)))return 2;
 for(k=0;k<40 && r.pending_rate<0;k++){h.frame_id=k+1;sample_frame(&r,&h,500);}
 if(r.pending_rate!=0)return 3;
 /* Simulate command transmission. Queued old-config frames must be ignored. */
 r.waiting_rate=0;r.pending_cfg=r.cfg;r.pending_rate=-1;reset_dsp(&r);
 h.frame_id++;sample_frame(&r,&h,500);
 if(r.used || r.waiting_rate!=0)return 4;
 h.cfg_id=11;h.decimation=8192;h.frame_id=1;sample_frame(&r,&h,500);
 if(r.waiting_rate!=-1||r.fs!=65000000.0/8192||r.used!=128||r.errors)return 5;
 r.auto_bandwidth=0;
 for(k=2;k<=6;k++){h.frame_id=k;sample_frame(&r,&h,500);}
 if(!r.results || r.errors)return 6;
 h.frame_id+=2;sample_frame(&r,&h,500);
 if(!r.errors)return 7;
 fclose(r.json);puts("IQ_RATE_RECEIVER_PASS auto decision, old-frame exclusion, rate acknowledgement, resync, gap reset");return 0;
}
