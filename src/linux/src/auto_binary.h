#ifndef V32_AUTO_BINARY_H
#define V32_AUTO_BINARY_H

#include <math.h>
#include <string.h>

#define AUTO_BINARY_SAMPLES 4096

typedef struct {
    int baud, fsk;
    int bit_count;
    float edges[33], sample_hz, low, high;
    char bits[33];
} auto_binary_result_t;

static inline auto_binary_result_t auto_binary_levels(const float *x, int n, float fs,
                                                       int fsk, const int *rates, int nrates)
{
    auto_binary_result_t result={0};
    double mean=0., squares=0.;
    int valid=0;
    for(int k=0;k<n;k++){
        if(!isfinite(x[k]))continue;
        mean+=x[k];squares+=(double)x[k]*x[k];valid++;
    }
    if(valid<n-2 || valid<12)return result;
    mean/=valid;
    const float spread=(float)sqrt(fmax(0.,squares/valid-mean*mean));
    float low=(float)mean-spread,high=(float)mean+spread;
    int sizes[2]={0};
    double error=0.;
    for(int pass=0;pass<4;pass++){
        double sums[2]={0.,0.};
        const float mid=(low+high)*.5f;
        sizes[0]=sizes[1]=0;error=0.;
        for(int k=0;k<n;k++){
            if(!isfinite(x[k]))continue;
            const int state=x[k]>mid;
            const float delta=x[k]-(state?high:low);
            sums[state]+=x[k];sizes[state]++;error+=(double)delta*delta;
        }
        if(sizes[0]<valid/12 || sizes[1]<valid/12)return result;
        low=(float)(sums[0]/sizes[0]);high=(float)(sums[1]/sizes[1]);
    }
    const float separation=high-low;
    if(separation<(fsk?60.f:6.f) || sqrt(error/valid)>.16f*separation ||
       (!fsk && low>.32f*high))return result;

    int runs[AUTO_BINARY_SAMPLES/3], states[AUTO_BINARY_SAMPLES/3];
    int count=0, edge=-1;
    const float mid=(high+low)*.5f, hysteresis=.08f*separation;
    int state=x[0]>mid;
    for(int k=1;k<n;k++){
        if(!isfinite(x[k]) || (state?x[k]>=mid-hysteresis:x[k]<=mid+hysteresis))continue;
        if(edge>=0){
            if(count==(int)(sizeof runs/sizeof runs[0]))return result;
            runs[count]=k-edge;states[count++]=state;
        }
        edge=k;state^=1;
    }
    if(count<8)return result;

    float best=.18f;
    for(int r=0;r<nrates;r++){
        const float samples=fs/rates[r];
        if(samples<3.f)continue;
        int single=0, matches=0;
        float total_error=0.f;
        for(int k=0;k<count;k++){
            const float length=runs[k]/samples;
            const int symbols=(int)(length+.5f);
            const float err=fabsf(length-symbols);
            if(symbols>=1 && err<=.27f){matches++;single+=symbols==1;}
            total_error+=symbols<1?1.f:fminf(err,1.f);
        }
        const float score=total_error/count;
        if(matches*10<count*9 || single<2 || score>=best)continue;
        best=score;result.baud=rates[r];
    }
    if(!result.baud)return result;
    result.fsk=fsk;
    result.sample_hz=fs;
    result.low=low;
    result.high=high;
    if(fsk){
        /* The DDC settles after each transition. Measure each state's
         * frequency in the middle half of complete runs. */
        double sum[2]={0.,0.};
        int used[2]={0,0},end=edge;
        for(int k=count-1;k>=0;k--){
            const int start=end-runs[k];
            const int trim=(runs[k]+3)/4;
            const int state=states[k];
            for(int j=start+trim;j<end-trim;j++){
                if(!isfinite(x[j]))continue;
                sum[state]+=x[j];used[state]++;
            }
            end=start;
        }
        if(used[0]>=4 && used[1]>=4){
            result.low=(float)(sum[0]/used[0]);
            result.high=(float)(sum[1]/used[1]);
        }
    }
    int written=0;
    int run_end=edge;
    result.edges[0]=(float)run_end;
    for(int k=count-1;k>=0 && written<32;k--){
        const int symbols=(int)(runs[k]*result.baud/fs+.5f);
        if(symbols<1 || fabsf(runs[k]*result.baud/fs-symbols)>.27f)break;
        for(int j=1;j<=symbols && written<32;j++){
            result.bits[written++]=(char)('0'+states[k]);
            result.edges[written]=run_end-(float)j*runs[k]/symbols;
        }
        run_end-=runs[k];
    }
    if(written<8)return (auto_binary_result_t){0};
    result.bit_count=written;
    for(int k=0;k<written/2;k++){
        const char bit=result.bits[k];
        result.bits[k]=result.bits[written-1-k];result.bits[written-1-k]=bit;
    }
    for(int k=0;k<(written+1)/2;k++){
        const float edge_at=result.edges[k];
        result.edges[k]=result.edges[written-k];result.edges[written-k]=edge_at;
    }
    return result;
}

static inline auto_binary_result_t auto_binary_analyze(const float *i,const float *q,
                                                         int n,float fs,const int *rates,int nrates,
                                                         float *features)
{
    auto_binary_result_t none={0};
    if(n<512 || !isfinite(fs) || fs<=0.f)return none;
    if(n>AUTO_BINARY_SAMPLES){i+=n-AUTO_BINARY_SAMPLES;q+=n-AUTO_BINARY_SAMPLES;n=AUTO_BINARY_SAMPLES;}
    float *amp=features;
    double sum=0.,squares=0.;
    for(int k=0;k<n;k++){
        amp[k]=hypotf(i[k],q[k]);
        if(!isfinite(amp[k]))return none;
        sum+=amp[k];squares+=(double)amp[k]*amp[k];
    }
    const double mean=sum/n;
    if(mean<4.)return none;
    const double cv=sqrt(fmax(0.,squares/n-mean*mean))/mean;
    if(cv>.24)return auto_binary_levels(amp,n,fs,0,rates,nrates);
    if(cv>.16)return none;
    /* Each write is behind the original amplitude samples still needed. */
    int used=0;
    for(int k=1;k<n;k+=2){
        if(amp[k-1]<mean*.4 || amp[k]<mean*.4)return none;
        const float dot=i[k]*i[k-1]+q[k]*q[k-1];
        const float cross=q[k]*i[k-1]-i[k]*q[k-1];
        const float angle=atan2f(cross,dot);
        if(!isfinite(angle) || fabsf(angle)>2.8f)return none;
        features[used++]=angle*fs/6.28318530718f;
    }
    return auto_binary_levels(features,used,fs*.5f,1,rates,nrates);
}
#endif
