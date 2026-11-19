#ifndef SCOPE_PROCESSING_H
#define SCOPE_PROCESSING_H
#include <math.h>
#include <stdbool.h>

/* Display repeatability gate for weak sine fits. This does not establish
 * absolute voltage accuracy. Values and mean are in ADC counts. */
static inline bool scope_weak_fit_repeatable(const float *values,int count,float mean)
{
    if(count<5 || !(mean>0.f))return false;
    float lo=values[0],hi=values[0];
    for(int i=1;i<count;i++){
        lo=fminf(lo,values[i]);hi=fmaxf(hi,values[i]);
    }
    return hi-lo<=.06f*mean;
}
/* Values stay in ADC counts; multiply/divide have independent display units. */
static inline float scope_math_sample(float a,float b,int mode)
{
    switch(mode){case 1:return a+b;case 2:return a-b;case 3:return a*b;
    case 4:return fabsf(b)<2.f?NAN:a/b;default:return NAN;}
}
/* A genuine crossing with complete pre/post samples; never wrap a frame. */
static inline int scope_trigger_start(const float *a,int n,int span,float level,int falling)
{
    if(!a || n<8 || span<8 || span>n || !isfinite(level))return -1;
    float lo=a[0],hi=a[0];
    for(int i=1;i<n;i++){if(a[i]<lo)lo=a[i];if(a[i]>hi)hi=a[i];}
    if(hi-lo<=1.f)return -1;
    float hysteresis=fmaxf(1.f,(hi-lo)*.05f);int armed=0,pre=span/5;
    for(int i=1;i<n;i++){
        if(!isfinite(a[i-1]) || !isfinite(a[i])){armed=0;continue;}
        if(falling?a[i-1]>=level+hysteresis:a[i-1]<=level-hysteresis)armed=1;
        int crossing=falling?(a[i-1]>level && a[i]<=level):(a[i-1]<level && a[i]>=level);
        if(armed && crossing){
            armed=0;
            int confirmed=0;
            /* Confirmation must exist inside the displayed post-trigger
             * record, not in samples beyond its right edge. */
            for(int j=i;j<n && j<i-pre+span;j++){
                if(!isfinite(a[j]))break;
                if(falling?a[j]<=level-hysteresis:a[j]>=level+hysteresis){confirmed=1;break;}
                if(falling?a[j]>level:a[j]<level)break;
            }
            if(confirmed && i>=pre && i-pre+span<=n)return i-pre;
        }
    }
    return -1;
}
#endif
