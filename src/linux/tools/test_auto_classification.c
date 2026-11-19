#include "auto_binary.h"
#include "auto_modulation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

enum { N=8192 };
enum signal_kind { SIG_AM, SIG_ASK, SIG_OOK, SIG_FSK2, SIG_FSK3, SIG_FSK4, SIG_BPSK, SIG_QPSK };
#define PI 3.14159265358979323846

static const double FS=31738.28125;
static const int BAUD=1000;
static const enum auto_mod_kind EXPECTED_ANALOG[]={
    AUTO_MOD_AM,AUTO_MOD_WAIT,AUTO_MOD_WAIT,AUTO_MOD_WAIT,
    AUTO_MOD_WAIT,AUTO_MOD_WAIT,AUTO_MOD_CARRIER,AUTO_MOD_WAIT
};

static void make_signal(enum signal_kind kind,float *i,float *q)
{
    static const char binary[]="0010111010010110010101101";
    static const char symbols[]="012302132031210230123102";
    static const double tones2[]={-1200.,1200.};
    static const double tones3[]={-1500.,0.,1500.};
    static const double tones4[]={-1800.,-600.,600.,1800.};
    double phase=0.,am_phase=0.;
    int previous=-1;
    for(int k=0;k<N;k++){
        const int symbol=(int)((double)k*BAUD/FS);
        double amplitude=40.,frequency=120.;
        int state=binary[symbol%24]-'0';
        if(kind==SIG_AM){
            amplitude=40.*(1.+.5*sin(2.*PI*200.*k/FS));
        }else if(kind==SIG_ASK || kind==SIG_OOK){
            amplitude=kind==SIG_OOK?(state?60.:0.):(state?60.:20.);
        }else if(kind==SIG_FSK2){
            frequency+=tones2[state];
        }else if(kind==SIG_FSK3 || kind==SIG_FSK4){
            const int value=symbols[symbol%24]-'0';
            frequency+=kind==SIG_FSK3?tones3[value%3]:tones4[value%4];
        }else if(kind==SIG_BPSK || kind==SIG_QPSK){
            const int value=symbols[symbol%24]-'0';
            const int order=kind==SIG_BPSK?2:4;
            if(symbol!=previous)am_phase+=2.*PI*(value%order)/order;
            previous=symbol;
        }
        phase+=2.*PI*frequency/FS;
        i[k]=(float)(amplitude*cos(phase+am_phase));
        q[k]=(float)(amplitude*sin(phase+am_phase));
    }
}

static void check(enum signal_kind test)
{
    static float i[N],q[N],amp[AUTO_MOD_TRACE_MAX],df[AUTO_MOD_TRACE_MAX];
    static float features[AUTO_BINARY_SAMPLES];
    static const int rates[]={100,250,500,1000,2000};
    make_signal(test,i,q);
    auto_mod_result_t analog=auto_mod_analyze(i,q,N,(float)FS,amp,df);
    auto_mod_refine_tone(&analog,i,q,N,(float)FS);
    const auto_binary_result_t digital=auto_binary_analyze(i,q,N,(float)FS,rates,5,features);
    assert(analog.kind==EXPECTED_ANALOG[test]);
    if(test==SIG_AM)assert(analog.tone_quality>=AUTO_MOD_AM_FIT_MIN && fabsf(analog.depth_pct-50.f)<2.f);
    if(test>=SIG_ASK && test<=SIG_FSK4){
        assert(digital.baud==BAUD);
        assert(digital.fsk==(test>=SIG_FSK2));
        assert(digital.ask==(test==SIG_ASK));
        assert(digital.levels==(test==SIG_FSK3?3:test==SIG_FSK4?4:2));
        assert(digital.bit_count>=8);
    }
    printf("case %d: analog=%d tone=%.3f depth=%.1f%% rate=%.1fHz digital=%d fsk=%d ask=%d levels=%d bits=%s\n",
           test,analog.kind,analog.tone_quality,analog.depth_pct,analog.modulation_hz,
           digital.baud,digital.fsk,digital.ask,digital.levels,digital.bits);
}

int main(void)
{
    for(enum signal_kind kind=SIG_AM;kind<=SIG_QPSK;kind++)check(kind);
    puts("AUTO_CLASSIFICATION_PROBE_DONE");
    return 0;
}
