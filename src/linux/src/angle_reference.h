#ifndef V32_ANGLE_REFERENCE_H
#define V32_ANGLE_REFERENCE_H

#include <math.h>

enum angle_reference_kind { ANGLE_REF_UNKNOWN, ANGLE_REF_FM, ANGLE_REF_PM };

typedef struct {
    double sine, cosine, quality;
} angle_reference_tone_t;

static inline void angle_reference_detrend(double *values,int count)
{
    const double middle=(count-1)*.5;
    double sum=0.,cross=0.,time_power=0.;
    for(int k=0;k<count;k++){
        const double t=k-middle;
        sum+=values[k];cross+=t*values[k];time_power+=t*t;
    }
    for(int k=0;k<count;k++)values[k]-=sum/count+(k-middle)*cross/time_power;
}

static inline angle_reference_tone_t angle_reference_fit(double *values,int count,
                                                        const double *sine,const double *cosine)
{
    angle_reference_detrend(values,count);
    double ss=0.,cc=0.,sc=0.,sy=0.,cy=0.,yy=0.;
    for(int k=0;k<count;k++){
        const double s=sine[k],c=cosine[k],y=values[k];
        ss+=s*s;cc+=c*c;sc+=s*c;sy+=s*y;cy+=c*y;yy+=y*y;
    }
    const double det=ss*cc-sc*sc;
    if(det<=0. || yy<=0.)return (angle_reference_tone_t){0};
    const double a=(sy*cc-cy*sc)/det,b=(cy*ss-sy*sc)/det;
    return (angle_reference_tone_t){a,b,(a*sy+b*cy)/yy};
}

/* Use the longest simultaneous RAW frame that still resolves the carrier. */
static inline int angle_reference_decimation(double adc_fs,double carrier_hz)
{
    int decim=1;
    if(carrier_hz>0. && adc_fs>0.)
        while(decim<32768 && adc_fs/(2*decim)>=4.*carrier_hz)decim*=2;
    return decim;
}

/* Fit the carrier-phase and reference tones using exactly the same RAW
 * samples and time origin. A frequency deviation is the derivative of the
 * phase deviation, so its tone has a 90-degree quadrature rotation. */
static inline enum angle_reference_kind angle_reference_classify(const float *carrier,
    const float *reference,int points,double fs,double carrier_hz,double modulation_hz)
{
    enum { MAX_BLOCKS=128 };
    if(points<64 || !(carrier_hz>0.) || !(modulation_hz>0.) || fs<4.*carrier_hz ||
       points*modulation_hz/fs<.8 || 16.*fs/carrier_hz>points/24.)return ANGLE_REF_UNKNOWN;

    const int block=(int)fmax(ceil(16.*fs/carrier_hz),ceil((double)points/MAX_BLOCKS));
    const int count=points/block;
    if(count<24 || block*modulation_hz/fs>1./12.)return ANGLE_REF_UNKNOWN;

    double phase[MAX_BLOCKS],baseband[MAX_BLOCKS],sine[MAX_BLOCKS],cosine[MAX_BLOCKS];
    const double osc_step=6.283185307179586*carrier_hz/fs;
    const double win_step=6.283185307179586/(block-1);
    const double oc=cos(osc_step),os=sin(osc_step),wc=cos(win_step),ws=sin(win_step);
    double oi=1.,oq=0.,last_phase=0.,unwrapped=0.;
    for(int k=0;k<count;k++){
        double i=0.,q=0.,r=0.,norm=0.,wi=1.,wq=0.;
        for(int j=0;j<block;j++){
            const int at=k*block+j;
            const double weight=.5-.5*wi;
            const double v=weight*carrier[at];
            i+=v*oi;q-=v*oq;r+=weight*reference[at];norm+=weight;
            const double next_oi=oi*oc-oq*os,next_wi=wi*wc-wq*ws;
            oq=oi*os+oq*oc;oi=next_oi;
            wq=wi*ws+wq*wc;wi=next_wi;
        }
        if(!isfinite(r) || !(hypot(i,q)>=6.*norm))return ANGLE_REF_UNKNOWN;
        const double wrapped=atan2(q,i);
        if(k>0){
            const double delta=remainder(wrapped-last_phase,6.283185307179586);
            if(fabs(delta)>2.8)return ANGLE_REF_UNKNOWN;
            unwrapped+=delta;
        }
        last_phase=wrapped;
        phase[k]=unwrapped;
        baseband[k]=r/norm;
        const double angle=6.283185307179586*modulation_hz*(k*block+(block-1)*.5)/fs;
        sine[k]=sin(angle);cosine[k]=cos(angle);
    }

    angle_reference_detrend(sine,count);angle_reference_detrend(cosine,count);
    const angle_reference_tone_t ref=angle_reference_fit(baseband,count,sine,cosine);
    const angle_reference_tone_t sig=angle_reference_fit(phase,count,sine,cosine);
    const double ref_amplitude=hypot(ref.sine,ref.cosine),sig_amplitude=hypot(sig.sine,sig.cosine);
    if(ref_amplitude<8. || sig_amplitude<.035 || ref.quality<.95 || sig.quality<.90)
        return ANGLE_REF_UNKNOWN;

    const double scale=ref_amplitude*sig_amplitude;
    const double pm=fabs(sig.sine*ref.sine+sig.cosine*ref.cosine)/scale;
    const double fm=fabs(sig.sine*ref.cosine-sig.cosine*ref.sine)/scale;
    if(fm>=.94 && fm-pm>=.5)return ANGLE_REF_FM;
    if(pm>=.94 && pm-fm>=.5)return ANGLE_REF_PM;
    return ANGLE_REF_UNKNOWN;
}

#endif
