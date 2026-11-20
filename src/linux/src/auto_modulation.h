#ifndef V32_AUTO_MODULATION_H
#define V32_AUTO_MODULATION_H
#include <math.h>

enum auto_mod_kind { AUTO_MOD_WAIT, AUTO_MOD_CARRIER, AUTO_MOD_AM, AUTO_MOD_ANGLE };

static inline int auto_mod_should_show_binary(enum auto_mod_kind kind,float tone_quality,
                                               float modulation_hz,int ready,
                                               unsigned votes,int use_psk,
                                               int binary_fsk,int binary_levels)
{
    const int coherent_angle=kind==AUTO_MOD_ANGLE && tone_quality>=.85f && modulation_hz>0.f;
    /* Continuous-phase MFSK can also look like a clean periodic angle tone.
     * A stable 3/4-level frequency fit is stronger evidence than that tone
     * fit; keep the analog guard for two-level and non-FSK candidates. */
    const int multilevel_fsk=binary_fsk && binary_levels>=3;
    return ready && votes>=3 && !use_psk && (!coherent_angle || multilevel_fsk);
}

static inline int auto_mod_has_phase_steps(const float *i,const float *q,int n,float fs)
{
    if(n<64 || !(fs>0.f))return 0;
    int valid=0,steps=0;
    float peak=0.f;
    for(int k=0;k<n;k++)peak=fmaxf(peak,hypotf(i[k],q[k]));
    const float gate=fmaxf(2.f,peak*.08f);
    for(int k=2;k+1<n;k++){
        if(hypotf(i[k-2],q[k-2])<gate || hypotf(i[k-1],q[k-1])<gate ||
           hypotf(i[k],q[k])<gate || hypotf(i[k+1],q[k+1])<gate)continue;
        const float dot0=i[k-2]*i[k-1]+q[k-2]*q[k-1];
        const float cross0=q[k-1]*i[k-2]-i[k-1]*q[k-2];
        const float dot1=i[k-1]*i[k]+q[k-1]*q[k];
        const float cross1=q[k]*i[k-1]-i[k]*q[k-1];
        const float dot2=i[k]*i[k+1]+q[k]*q[k+1];
        const float cross2=q[k+1]*i[k]-i[k+1]*q[k];
        const float before=atan2f(cross0,dot0),center=atan2f(cross1,dot1),after=atan2f(cross2,dot2);
        const float baseline=before+.5f*remainderf(after-before,6.28318530718f);
        const float impulse=fabsf(remainderf(center-baseline,6.28318530718f));
        /* A smooth angle-modulated phase increment stays near the linear
         * interpolation of its neighbors. PSK impulses and abrupt FSK tone
         * transitions produce larger local curvature. */
        if(impulse>.30f)steps++;
        valid++;
    }
    return valid>=16 && steps*100>=valid;
}

typedef struct {
    float amplitude_mean, frequency_mean, amplitude_pp, frequency_pp;
    float trace_fs, modulation_hz, depth_pct, deviation_hz, phase_deviation_deg;
    float tone_quality;
    int trace_count;
    enum auto_mod_kind kind;
} auto_mod_result_t;


#define AUTO_MOD_INPUT_MAX 8192
#define AUTO_MOD_TRACE_MAX 1024
#define AUTO_MOD_AVERAGE_AT 1024
#define AUTO_MOD_AM_FIT_MIN .86f

/* Find the first strong repeat after a correlation trough. */
static inline int auto_mod_period(const float *x, int n)
{
    float before=1.f, previous=1.f;
    int have_trough=0;
    for(int lag=1;lag<=n/2;lag++){
        double xy=0.,xx=0.,yy=0.;
        for(int i=lag;i<n;i++){
            const float a=x[i],b=x[i-lag];
            xy+=a*b;xx+=a*a;yy+=b*b;
        }
        const float corr=xx>0. && yy>0. ? (float)(xy/sqrt(xx*yy)):0.f;
        if(corr<0.2f)have_trough=1;
        if(have_trough && lag>=5 && previous>.82f && previous>before && previous>=corr)
            return lag-1;
        before=previous;previous=corr;
    }
    return 0;
}

/* Frequency is already demodulated here. Use full-rate samples for fast
 * modulation and averaged history for slower repetition. */
static inline double auto_mod_frequency_rate(const float *frequency,int count,double fs)
{
    enum { WINDOW=256 };
    float centered[WINDOW];
    int contiguous=0;
    while(contiguous<count && isfinite(frequency[count-1-contiguous]))contiguous++;
    for(int stride=1;stride<=8;stride*=2){
        const int n=contiguous/stride<WINDOW?contiguous/stride:WINDOW;
        if(n<16)break;
        const float *recent=frequency+count-n*stride;
        double mean=0., energy=0.;
        for(int j=0;j<n;j++){
            double sum=0.;
            for(int k=0;k<stride;k++)sum+=recent[j*stride+k];
            centered[j]=(float)(sum/stride);
            mean+=centered[j];
        }
        mean/=n;
        for(int j=0;j<n;j++){
            centered[j]-=(float)mean;
            energy+=(double)centered[j]*centered[j];
        }
        if(energy/n<25.)return 0.;
        const int period=auto_mod_period(centered,n);
        if(period>0)return fs/(stride*period);
    }
    return 0.;
}

static inline auto_mod_result_t auto_mod_analyze_window(const float *i,const float *q,
                                                       int n,float fs,float *amp,float *df,
                                                       int interval)
{
    auto_mod_result_t r={0};
    if(n<64 || n>AUTO_MOD_INPUT_MAX || !(fs>0.f))return r;
    const int stride=n>=AUTO_MOD_AVERAGE_AT?8:1;
    const int count=n/stride;
    float peak=0.f;
    for(int k=0;k<n;k++){
        const float magnitude=hypotf(i[k],q[k]);
        if(isfinite(magnitude) && magnitude>peak)peak=magnitude;
    }
    const float gate=fmaxf(2.f,peak*.08f);
    float lo=INFINITY,hi=0.f,flo=INFINITY,fhi=-INFINITY;
    double sum=0.,sq=0.,f_sum=0.,f_sq=0.;
    int frequency_count=0;
    for(int b=0;b<count;b++){
        double a_total=0., f_total=0.;
        int a_valid=0,f_valid=0;
        for(int k=b*stride;k<(b+1)*stride;k++){
            const float a=hypotf(i[k],q[k]);
            if(!isfinite(a))continue;
            a_total+=a;a_valid++;
            if(k<interval || a<gate || hypotf(i[k-interval],q[k-interval])<gate)continue;
            const float cross=i[k-interval]*q[k]-q[k-interval]*i[k];
            const float dot=i[k-interval]*i[k]+q[k-interval]*q[k];
            const float f=atan2f(cross,dot)*fs/(6.28318530718f*interval);
            if(!isfinite(f) || fabsf(f)>=.47f*fs/interval)continue;
            f_total+=f;f_valid++;
        }
        const float a=a_valid?(float)(a_total/a_valid):0.f;
        amp[b]=a;lo=fminf(lo,a);hi=fmaxf(hi,a);sum+=a;sq+=(double)a*a;
        df[b]=NAN;
        if(f_valid){
            const float f=(float)(f_total/f_valid);
            df[b]=f;flo=fminf(flo,f);fhi=fmaxf(fhi,f);
            f_sum+=f;f_sq+=(double)f*f;frequency_count++;
        }
    }
    r.trace_count=count;
    r.trace_fs=fs/stride;
    r.amplitude_mean=(float)(sum/count);r.amplitude_pp=hi-lo;
    if(r.amplitude_mean<4.f || frequency_count<count*.70f)return r;
    r.frequency_mean=(float)(f_sum/frequency_count);r.frequency_pp=fhi-flo;
    const float ar=(float)sqrt(fmax(0.,sq/count-(sum/count)*(sum/count)))/r.amplitude_mean;
    const float fr=(float)sqrt(fmax(0.,f_sq/frequency_count-(f_sum/frequency_count)*(f_sum/frequency_count)));
    float centered[AUTO_MOD_TRACE_MAX];
    int ap=0,fp=0;
    if(ar>.022f){
        for(int b=0;b<count;b++)centered[b]=amp[b]-r.amplitude_mean;
        ap=auto_mod_period(centered,count);
    }
    if(fr>fmaxf(15.f,.0008f*fs)){
        for(int b=0;b<count;b++)centered[b]=isfinite(df[b])?df[b]-r.frequency_mean:0.f;
        fp=auto_mod_period(centered,count);
    }
    /* A periodic phase increment reveals angle modulation. With no
       reference message, its integral also describes a valid PM signal. */
    if(ap && !fp)r.kind=AUTO_MOD_AM;
    else if(fp && ar<.10f && (!ap || ar<.035f))r.kind=AUTO_MOD_ANGLE;
    else if(!ap && !fp && ar<.025f && fr<fmaxf(35.f,.004f*fs))r.kind=AUTO_MOD_CARRIER;
    const int period=r.kind==AUTO_MOD_AM?ap:r.kind==AUTO_MOD_ANGLE?fp:0;
    if(period){
        r.modulation_hz=r.trace_fs/period;
        /* Use the median of block extrema so one phase spike does not
           become the reported peak deviation or modulation depth. */
        float mins[64],maxs[64];
        const float *signal=r.kind==AUTO_MOD_AM?amp:df;
        int blocks=0;
        for(int end=count;end>=period && blocks<64;end-=period){
            float min=INFINITY,max=-INFINITY;
            int samples=0;
            for(int j=end-period;j<end;j++){
                if(!isfinite(signal[j]))continue;
                min=fminf(min,signal[j]);max=fmaxf(max,signal[j]);samples++;
            }
            if(samples<period*.8f)continue;
            int a=blocks,b=blocks;
            while(a>0 && mins[a-1]>min){mins[a]=mins[a-1];a--;}
            while(b>0 && maxs[b-1]>max){maxs[b]=maxs[b-1];b--;}
            mins[a]=min;maxs[b]=max;blocks++;
        }
        if(blocks>=2){
            const float low=mins[blocks/2],high=maxs[blocks/2];
            if(r.kind==AUTO_MOD_AM && high+low>0.f)
                r.depth_pct=100.f*(high-low)/(high+low);
            if(r.kind==AUTO_MOD_ANGLE)r.deviation_hz=(high-low)*.5f;
        }
        if(r.kind==AUTO_MOD_ANGLE && period>=4){
            /* Integrate one complete, valid period, removing its mean
               frequency so residual tuning does not inflate phase swing. */
            for(int end=count;end>=period;end-=period){
                const int start=end-period;
                double avg=0.;
                int j=start;
                for(;j<end && isfinite(df[j]);j++)avg+=df[j];
                if(j!=end)continue;
                avg/=period;
                double phase=0.,lo_phase=0.,hi_phase=0.;
                for(j=start;j<end;j++){
                    phase+=360.*(df[j]-avg)/r.trace_fs;
                    lo_phase=fmin(lo_phase,phase);
                    hi_phase=fmax(hi_phase,phase);
                }
                r.phase_deviation_deg=(float)((hi_phase-lo_phase)*.5);
                break;
            }
        }
    }
    /* Reject ripple near the measured unmodulated carrier floor. */
    if(r.kind==AUTO_MOD_AM && r.depth_pct<5.f)r.kind=AUTO_MOD_WAIT;
    return r;
}

static inline auto_mod_result_t auto_mod_analyze(const float *i,const float *q,
                                                int n,float fs,float *amp,float *df)
{
    const auto_mod_result_t long_view=auto_mod_analyze_window(i,q,n,fs,amp,df,1);
    if(n<AUTO_MOD_AVERAGE_AT || !long_view.trace_count || long_view.kind==AUTO_MOD_AM)
        return long_view;

    /* Confirm fast angle modulation at the original I/Q rate. The eight-point
       averages used for slow modulation can alias a message near Nyquist. */
    enum { FAST_SAMPLES=512 };
    float fast_amp[FAST_SAMPLES],fast_df[FAST_SAMPLES];
    auto_mod_result_t fast=auto_mod_analyze_window(i+n-FAST_SAMPLES,q+n-FAST_SAMPLES,
                                                  FAST_SAMPLES,fs,fast_amp,fast_df,1);
    if(fast.kind!=AUTO_MOD_ANGLE && long_view.amplitude_mean>=4.f &&
       fabsf(long_view.frequency_mean)+long_view.frequency_pp<fs*.10f){
        /* A wider phase difference improves the SNR of narrow FM. Its
           unambiguous deviation is smaller, so only try it near center. */
        fast=auto_mod_analyze_window(i+n-FAST_SAMPLES,q+n-FAST_SAMPLES,
                                     FAST_SAMPLES,fs,fast_amp,fast_df,4);
    }
    if(fast.kind!=AUTO_MOD_ANGLE || fast.modulation_hz<=long_view.trace_fs*.25f)
        return long_view;
    for(int k=0;k<fast.trace_count;k++){
        amp[k]=fast_amp[k];
        df[k]=fast_df[k];
    }
    return fast;
}

static inline int auto_mod_angle_interval(const auto_mod_result_t *r,float fs)
{
    return r->kind==AUTO_MOD_ANGLE && r->modulation_hz<=fs/12.f &&
           fabsf(r->frequency_mean)+r->deviation_hz<fs*.10f &&
           r->deviation_hz<fs/35.f?4:1;
}

typedef struct {
    double quality,rate,peak,dc;
} auto_mod_tone_t;

/* Fit a DC offset and both quadratures near the autocorrelation estimate. */
static inline auto_mod_tone_t auto_mod_fit_tone(const float *values,int count,double fs,
                                               double initial)
{
    auto_mod_tone_t fit={.quality=-1.,.rate=initial};
    if(initial<=0. || fs<=0. || count*initial/fs<2.)return fit;
    double mean=0.,energy=0.;
    int valid=0;
    for(int k=0;k<count;k++){
        const float value=values[k];
        if(isfinite(value)){mean+=value;energy+=(double)value*value;valid++;}
    }
    if(valid<count*.9)return fit;
    mean/=valid;
    energy-=valid*mean*mean;
    if(energy<=0.)return fit;
    fit.quality=0.;

    for(int pass=0;pass<3;pass++){
        const double center=fit.rate;
        const double spacing=initial*(pass==0?.015:pass==1?.003:.0006);
        for(int step=-4;step<=4;step++){
            const double hz=center+spacing*step;
            const double angle=6.28318530718*hz/fs;
            const double c=cos(angle),s=sin(angle);
            double a=1.,b=0.,ac=0.,bs=0.,aa=0.,bb=0.,ab=0.,ay=0.,by=0.;
            for(int k=0;k<count;k++){
                if(isfinite(values[k])){
                    const double y=values[k]-mean;
                    ac+=a;bs+=b;aa+=a*a;bb+=b*b;ab+=a*b;ay+=a*y;by+=b*y;
                }
                const double next=a*c-b*s;
                b=a*s+b*c;a=next;
            }
            aa-=ac*ac/valid;bb-=bs*bs/valid;ab-=ac*bs/valid;
            const double det=aa*bb-ab*ab;
            if(det<=0.)continue;
            const double ia=(ay*bb-by*ab)/det,iq=(by*aa-ay*ab)/det;
            const double quality=(ia*ay+iq*by)/energy;
            if(quality>fit.quality){
                fit=(auto_mod_tone_t){quality,hz,hypot(ia,iq),mean-(ia*ac+iq*bs)/valid};
            }
        }
    }
    return fit;
}

/* A phase difference measures frequency averaged over one interval. */
static inline double auto_mod_instant_peak(double peak,double rate,double fs,int interval)
{
    const double half_step=3.141592653589793*rate*interval/fs;
    return peak*half_step/sin(half_step);
}

/* Fit the repeating message using the original I/Q sample rate. */
static inline void auto_mod_refine_tone(auto_mod_result_t *r,const float *i,const float *q,
                                        int n,float fs)
{
    const int am=r->kind==AUTO_MOD_AM;
    if((!am && r->kind!=AUTO_MOD_ANGLE) || r->modulation_hz<=0.f)return;
    /* A periodic digital phase code can create a strong spectral line in the
     * frequency discriminator. Abrupt carrier-phase steps identify that
     * trajectory as PSK rather than coherent FM/PM. */
    if(!am && auto_mod_has_phase_steps(i,q,n,fs)){
        r->kind=AUTO_MOD_WAIT;
        r->tone_quality=0.f;
        return;
    }
    const double initial=r->modulation_hz;
    const int interval=auto_mod_angle_interval(r,fs);
    int count=(int)ceil((am || interval==4?12.:5.)*fs/initial);
    if(count>n-interval)count=n-interval;
    if(count>2048)count=2048;
    if(count*initial/fs<2.){
        /* Too little captured trajectory to validate the analog hypothesis. */
        r->kind=AUTO_MOD_WAIT;
        return;
    }
    const float gate=fmaxf(2.f,r->amplitude_mean*.1f);
    float values[2048];
    for(int k=0;k<count;k++){
        const int j=n-count+k;
        const float a=hypotf(i[j],q[j]);
        values[k]=a;
        if(!am){
            const float cross=i[j-interval]*q[j]-q[j-interval]*i[j];
            const float dot=i[j-interval]*i[j]+q[j-interval]*q[j];
            values[k]=a>=gate && hypotf(i[j-interval],q[j-interval])>=gate?
                atan2f(cross,dot)*fs/(6.28318530718f*interval):NAN;
        }
    }
    const auto_mod_tone_t fit=auto_mod_fit_tone(values,count,fs,initial);
    if(fit.quality<0.)return;
    r->tone_quality=(float)fit.quality;
    if(am){
        const double depth=fit.dc>0.?100.*fit.peak/fit.dc:0.;
        /* Shallow peaks must have a coherent message above residual ripple. */
        if(r->depth_pct<12.f && (fit.quality<.8 || depth<5.)){
            r->kind=AUTO_MOD_WAIT;
            return;
        }
        /* A digital envelope can have a periodic autocorrelation without
         * being a sinusoidal AM message. Do not keep the provisional analog
         * label when the original-rate tone fit rejects that hypothesis. */
        if(fit.quality<AUTO_MOD_AM_FIT_MIN || fit.dc<=0.){
            r->kind=AUTO_MOD_WAIT;
            return;
        }
        r->depth_pct=(float)depth;
    }else{
        /* FSK and PSK phase jumps can also look periodic to a coarse
         * frequency discriminator. Only retain FM/PM when a coherent tone
         * actually explains the trajectory. */
        if(fit.quality<.85){
            r->kind=AUTO_MOD_WAIT;
            return;
        }
        const double peak=auto_mod_instant_peak(fit.peak,fit.rate,fs,interval);
        r->deviation_hz=(float)peak;
        r->phase_deviation_deg=(float)(peak/fit.rate*180./3.141592653589793);
    }
    r->modulation_hz=(float)fit.rate;
}

/* Average consecutive phase increments while keeping their full sample rate.
 * The sum telescopes to a four-sample phase difference and reduces jitter. */
static inline void auto_mod_average_frequency(const float *src,int available,
                                               float *out,int count,int interval)
{
    for(int k=0;k<count;k++){
        const int end=available-count+k+1;
        float sum=0.f;
        if(end<interval){out[k]=NAN;continue;}
        for(int j=end-interval;j<end;j++)sum+=src[j];
        out[k]=sum/interval;
    }
}

static inline auto_mod_tone_t auto_mod_fit_narrow_fm(const float *frequency,int available,
                                                     double fs,double initial)
{
    float recent[2048];
    const int count=available-3<2048?available-3:2048;
    if(count<16 || !(fs>0.))return (auto_mod_tone_t){0};
    auto_mod_average_frequency(frequency,available,recent,count,4);
    /* Estimate the period after reducing phase-step noise. On weak FM,
       the raw phase increments can correlate at a multiple of the period. */
    const double filtered=auto_mod_frequency_rate(recent,count,fs);
    const double seed=filtered>0. && filtered<=fs/12.?filtered:initial;
    if(!(seed>0.) || seed>fs/12.)return (auto_mod_tone_t){0};
    const int fit_count=(int)ceil(fmin(count,12.*fs/seed));
    auto_mod_tone_t fit=auto_mod_fit_tone(recent+count-fit_count,fit_count,fs,seed);
    if(fit.quality<.93 && initial>0. && initial<=fs/12. && initial!=seed){
        const int fallback_count=(int)ceil(fmin(count,12.*fs/initial));
        fit=auto_mod_fit_tone(recent+count-fallback_count,fallback_count,fs,initial);
    }
    if(fit.quality<.93)return (auto_mod_tone_t){0};
    fit.peak=auto_mod_instant_peak(fit.peak,fit.rate,fs,4);
    return fit;
}

/* A long history finds slow modulation. At the standard I/Q rate the
 * 8-point averages put a 1 kHz message at Nyquist, so use a recent full-rate
 * window to measure AM when it has enough coherent cycles. Keep the long
 * history's result for modulation too slow for the short window. */
static inline void auto_mod_refine_am(auto_mod_result_t *r,const float *i,const float *q,
                                      int n,float fs,float *amplitude,float *frequency)
{
    if(r->kind!=AUTO_MOD_AM || n<AUTO_MOD_AVERAGE_AT)return;
    const int count=AUTO_MOD_AVERAGE_AT-1;
    const auto_mod_result_t recent=auto_mod_analyze(i+n-count,q+n-count,count,fs,
                                                   amplitude,frequency);
    if(recent.kind==AUTO_MOD_AM && recent.depth_pct>0.f){
        r->depth_pct=recent.depth_pct;
        r->modulation_hz=recent.modulation_hz;
    }
}
#endif
