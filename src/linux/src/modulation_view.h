#ifndef V32_MODULATION_VIEW_H
#define V32_MODULATION_VIEW_H
#include <math.h>
#include <stdio.h>
#include <stddef.h>
/* Presentation only: never estimates modulation, fills missing data, or sends commands. */
static inline double mod_view_auto_ms(double rate,unsigned periods)
{
    if(!isfinite(rate) || rate<=0. || !periods)return 16.;
    return fmin(1024.,fmax(.5,1000.*periods/rate));
}

static inline int mod_view_count(int available,double fs,double milliseconds)
{
    if(available<2 || !isfinite(fs) || fs<=0 || !isfinite(milliseconds) || milliseconds<=0)return 0;
    double want=ceil(fs*milliseconds/1000.0);
    int n=want>available?available:(int)want;
    return n<2?2:n;
}

/* A few invalid samples at the end must not replace a recent good window
 * with a tiny fragment. Search recent runs without bridging invalid data. */
static inline int mod_view_recent_contiguous(const float *src,int count,int wanted,int lookback,int *end)
{
    int best=0,run=0;
    *end=count;
    const int first=count>wanted+lookback?count-wanted-lookback:0;
    for(int k=count-1;k>=first;k--){
        if(!isfinite(src[k])){run=0;continue;}
        run++;
        if(run>best){best=run;*end=k+run;}
        if(run==wanted){
            /* Keep earlier samples of this same run for phase drift removal
               and cycle averaging. Limit work to a bounded recent record. */
            while(k>0 && run<4096 && isfinite(src[k-1])){k--;run++;}
            return run;
        }
    }
    return best;
}

/* Estimate the phase origin and carrier drift over the contiguous capture.
 * Crop the result only after that fit so changing zoom cannot change phase. */
static inline int mod_view_relative_phase(const float *src,int available,double fs,
                                          int from_frequency,float *phase)
{
    int count=0;
    while(count<available && isfinite(src[available-1-count]))count++;
    if(count<3 || (from_frequency && !(fs>0.)))return 0;
    src+=available-count;
    double unwrapped=0.;
    phase[0]=0.f;
    for(int k=1;k<count;k++){
        /* Each frequency sample comes from the preceding phase interval. */
        unwrapped+=from_frequency?360.*((double)src[k]-src[0])/fs:
                   remainderf(src[k]-src[k-1],360.f);
        phase[k]=(float)unwrapped;
    }
    double sum=0.,cross=0.,energy=0.;
    const double middle=(count-1)*.5;
    for(int k=0;k<count;k++){
        const double t=k-middle;
        sum+=phase[k];cross+=t*phase[k];energy+=t*t;
    }
    for(int k=0;k<count;k++)phase[k]-=(float)(sum/count+(k-middle)*cross/energy);
    return count;
}

/* Dimensionless angle-modulation index: FM uses Δf/fm; PM uses Δφ in rad. */
static inline double mod_view_angle_index(int mode,double deviation,double rate)
{
    if(!isfinite(deviation) || !isfinite(rate) || deviation<0. || rate<=0.)return NAN;
    if(mode==2)return deviation/rate;
    if(mode==7)return deviation*0.017453292519943295769;
    return NAN;
}

/* Average measured points at the same position in recent modulation periods.
 * Keep the current time axis, and fall back if the record has a gap. */
static inline int mod_view_average_periods(const float *src,int available,float *out,
                                           int count,double period)
{
    const int start=available-count;
    if(count<2 || !(period>=8.) || start<3.*period)return 0;
    const int repeats=(int)fmin(15.,floor(start/period));
    const int first=(int)(start-repeats*period);
    for(int k=first;k<available;k++)if(!isfinite(src[k]))return 0;
    for(int k=0;k<count;k++){
        const int at=start+k;
        double sum=src[at];
        for(int c=1;c<=repeats;c++){
            const double t=at-c*period;
            const int j=(int)t;
            const double fraction=t-j;
            sum+=src[j]+fraction*((double)src[j+1]-src[j]);
        }
        out[k]=(float)(sum/(repeats+1));
    }
    return repeats+1;
}

/* The caller provides count+4 phase values for the centered derivative.
 * The first displayed phase is phase_out[2], sharing the frequency time axis. */
static inline int mod_view_average_angle(const float *phase,int available,float *phase_out,
                                         float *freq_out,int count,double fs,double rate)
{
    if(count<2 || !(rate>0.))return 0;
    const int periods=mod_view_average_periods(phase,available,phase_out,count+4,fs/rate);
    if(!periods)return 0;
    for(int k=0;k<count;k++)
        freq_out[k]=(float)((phase_out[k]-8.*phase_out[k+1]+8.*phase_out[k+3]-phase_out[k+4])*fs/4320.);
    return periods;
}

static inline void mod_view_time(char *out,size_t len,double seconds)
{
    if(!isfinite(seconds) || seconds<0)snprintf(out,len,"--");
    else if(seconds<.001)snprintf(out,len,"%.2f us",seconds*1e6);
    else if(seconds<1)snprintf(out,len,"%.2f ms",seconds*1e3);
    else snprintf(out,len,"%.3f s",seconds);
}
static inline int mod_view_range(const float *y,int n,int nonnegative,float *lo,float *hi)
{
    float a=INFINITY,b=-INFINITY;
    for(int i=0;i<n;i++)if(isfinite(y[i])){if(y[i]<a)a=y[i];if(y[i]>b)b=y[i];}
    if(!isfinite(a)||!isfinite(b))return 0;
    if(nonnegative){*lo=0;*hi=b>1e-6f?b*1.12f:1.f;}
    else {float d=b-a;if(d<1e-6f)d=fmaxf(fabsf(a)*.1f,1.f);*lo=a-d*.08f;*hi=b+d*.08f;}
    return 1;
}
/* Baseband magnitude bounds around zero, never a reconstructed RF waveform. */
static inline void mod_view_bounds(const float *amplitude,float *upper,float *lower,int n)
{
    for(int i=0;i<n;i++){
        if(!isfinite(amplitude[i]) || amplitude[i]<0){upper[i]=lower[i]=NAN;continue;}
        upper[i]=.5f+.5f*amplitude[i];lower[i]=.5f-.5f*amplitude[i];
    }
}
static inline void mod_view_resample(const float *src,int count,float *out,int n,float lo,float hi)
{
    if(count<2 || n<2 || !(hi>lo)){for(int i=0;i<n;i++)out[i]=NAN;return;}
    if(count>n){
        const int buckets=n/2;
        for(int k=0;k<buckets;k++){
            int a=(int)((long long)k*count/buckets),b=(int)((long long)(k+1)*count/buckets);
            int imin=a,imax=a,hole=0;
            for(int j=a;j<b;j++){if(!isfinite(src[j]))hole=1;else {if(src[j]<src[imin])imin=j;if(src[j]>src[imax])imax=j;}}
            int first=imin<imax?imin:imax,last=imin<imax?imax:imin;
            out[2*k]=hole?NAN:(src[first]-lo)/(hi-lo);out[2*k+1]=hole?NAN:(src[last]-lo)/(hi-lo);
        }
        if(n&1)out[n-1]=isfinite(src[count-1])?(src[count-1]-lo)/(hi-lo):NAN;
    }else{
        for(int i=0;i<n;i++){
            double t=(double)i*(count-1)/(n-1);int a=(int)t,b=a+1<count?a+1:a;float f=(float)(t-a);
            if(f<1e-6f)out[i]=isfinite(src[a])?(src[a]-lo)/(hi-lo):NAN;
            else out[i]=isfinite(src[a])&&isfinite(src[b])?(src[a]+(src[b]-src[a])*f-lo)/(hi-lo):NAN;
        }
    }
}
/* Monotone cubic interpolation for analog display. Preserve the measured
 * extrema and leave invalid intervals blank. Wide windows keep min/max buckets. */
static inline void mod_view_smooth(const float *src,int count,float *out,int n,float lo,float hi)
{
    if(count<3 || count>=n || n<2 || !(hi>lo)){
        mod_view_resample(src,count,out,n,lo,hi);
        return;
    }
    for(int k=0;k<n;k++){
        const double t=(double)k*(count-1)/(n-1);
        const int a=(int)t;
        const float f=(float)(t-a), x=src[a];
        if(f<1e-6f){out[k]=(x-lo)/(hi-lo);continue;}
        const float y=src[a+1];
        if(!isfinite(x) || !isfinite(y)){out[k]=NAN;continue;}
        const float d=y-x;
        const float left=a && isfinite(src[a-1])?x-src[a-1]:d;
        const float right=a+2<count && isfinite(src[a+2])?src[a+2]-y:d;
        const float m0=left*d>0.f?2.f*left*d/(left+d):0.f;
        const float m1=d*right>0.f?2.f*d*right/(d+right):0.f;
        const float v=x+f*(m0+f*(3.f*d-2.f*m0-m1+f*(m0+m1-2.f*d)));
        out[k]=(fminf(fmaxf(v,fminf(x,y)),fmaxf(x,y))-lo)/(hi-lo);
    }
}

/* Compare identical-length excerpts within one ADC frame. Average across
 * several cycles so a single noisy zero crossing cannot determine the result. */
typedef struct {
    int dense_start, sparse_start;
    float dense_hz, sparse_hz;
} mod_raw_density_t;

static inline int mod_raw_density(const float *x,int n,int span,double fs,
                                  float midpoint,float hysteresis,mod_raw_density_t *out)
{
    if(span<8 || n<span || hysteresis<=0.f || fs<=0.)return 0;
    const int cycles=4;
    float crossings[4], pending=-1.f;
    int armed=0, seen=0;
    float fast=0.f, slow=INFINITY;
    for(int k=1;k<n;k++){
        if(x[k]<=midpoint-hysteresis){armed=1;pending=-1.f;}
        if(armed && x[k-1]<=midpoint && x[k]>midpoint)
            pending=k-1+(midpoint-x[k-1])/(x[k]-x[k-1]);
        if(!armed || pending<0.f || x[k]<midpoint+hysteresis)continue;
        armed=0;
        const int slot=seen%cycles;
        if(seen>=cycles){
            const float elapsed=pending-crossings[slot];
            const int start=(int)((pending+crossings[slot])*.5f)-span/2;
            if(elapsed>=cycles*4 && elapsed<=span && start>=0 && start+span<=n){
                const float hz=(float)(fs*cycles/elapsed);
                if(hz>fast){fast=hz;out->dense_start=start;}
                if(hz<slow){slow=hz;out->sparse_start=start;}
            }
        }
        crossings[slot]=pending;
        seen++;
    }
    out->dense_hz=fast;
    out->sparse_hz=slow;
    return isfinite(slow) && fast>0.f;
}

#endif
