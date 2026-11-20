#include "auto_binary.h"
#include "auto_modulation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

enum { N=8192 };
enum signal_kind { SIG_AM, SIG_ASK, SIG_OOK, SIG_FSK2, SIG_FSK3, SIG_FSK4, SIG_BPSK, SIG_QPSK, SIG_FM, SIG_PM };
#define PI 3.14159265358979323846

static const double FS=31738.28125;
static const int BAUD=1000;
static const enum auto_mod_kind EXPECTED_ANALOG[]={
    AUTO_MOD_AM,AUTO_MOD_WAIT,AUTO_MOD_WAIT,AUTO_MOD_WAIT,
    AUTO_MOD_WAIT,AUTO_MOD_WAIT,AUTO_MOD_CARRIER,AUTO_MOD_WAIT,AUTO_MOD_ANGLE,AUTO_MOD_ANGLE
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
            amplitude=kind==SIG_OOK?(state?60.:0.):(state?20.3:10.9)+
                2.2*sin(2.*PI*7500.*k/FS);
        }else if(kind==SIG_FSK2){
            frequency+=tones2[state];
        }else if(kind==SIG_FSK3 || kind==SIG_FSK4){
            const int value=symbols[symbol%24]-'0';
            frequency+=kind==SIG_FSK3?tones3[value%3]:tones4[value%4];
        }else if(kind==SIG_FM){
            frequency+=900.*sin(2.*PI*200.*k/FS);
        }else if(kind==SIG_PM){
            am_phase=2.0*sin(2.*PI*200.*k/FS);
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

static void check_high_index_angle(void)
{
    static float i[N],q[N],amp[AUTO_MOD_TRACE_MAX],df[AUTO_MOD_TRACE_MAX];
    const double fs=15869.140625;
    for(int kind=0;kind<2;kind++){
        double phase=0.;
        for(int k=0;k<N;k++){
            const double message=sin(2.*PI*1000.*k/fs);
            const double deviation=kind==0?2000.*message:0.;
            const double phase_mod=kind==1?(PI/3.)*message:0.;
            phase+=2.*PI*(120.+deviation)/fs;
            i[k]=(float)(40.*cos(phase+phase_mod));
            q[k]=(float)(40.*sin(phase+phase_mod));
        }
        assert(!auto_mod_has_phase_steps(i,q,N,(float)fs));
        auto_mod_result_t analog=auto_mod_analyze(i,q,N,(float)fs,amp,df);
        auto_mod_refine_tone(&analog,i,q,N,(float)fs);
        assert(analog.kind==AUTO_MOD_ANGLE);
    }
}

static void check_fast_fsk_phase_steps(void)
{
    static float i[N],q[N],amp[AUTO_MOD_TRACE_MAX],df[AUTO_MOD_TRACE_MAX];
    static const char symbols[]="312102301231020123021320";
    static const double tones[]={-1500.,-500.,500.,1500.};
    const double fs=7934.5703125;
    double phase=0.;
    for(int k=0;k<N;k++){
        const int symbol=(int)((double)k*500./fs);
        const int state=symbols[symbol%24]-'0';
        phase+=2.*PI*(120.+tones[state])/fs;
        i[k]=(float)(40.*cos(phase));
        q[k]=(float)(40.*sin(phase));
    }
    assert(auto_mod_has_phase_steps(i,q,N,(float)fs));
    auto_mod_result_t analog=auto_mod_analyze(i,q,N,(float)fs,amp,df);
    auto_mod_refine_tone(&analog,i,q,N,(float)fs);
    assert(analog.kind!=AUTO_MOD_ANGLE);
}

static void check(enum signal_kind test)
{
    static float i[N],q[N],amp[AUTO_MOD_TRACE_MAX],df[AUTO_MOD_TRACE_MAX];
    static float features[AUTO_BINARY_SAMPLES];
    static const int rates[]={100,250,500,1000,1500,2000,2500,3000,3500,4000};
    make_signal(test,i,q);
    auto_mod_result_t analog=auto_mod_analyze(i,q,N,(float)FS,amp,df);
    auto_mod_refine_tone(&analog,i,q,N,(float)FS);
    const auto_binary_result_t digital=auto_binary_analyze(i,q,N,(float)FS,rates,10,features);
    if(test==SIG_BPSK || test==SIG_QPSK)
        assert(auto_mod_has_phase_steps(i,q,N,(float)FS));
    if(test==SIG_FM || test==SIG_PM)
        assert(!auto_mod_has_phase_steps(i,q,N,(float)FS));
    assert(analog.kind==EXPECTED_ANALOG[test]);
    if(test==SIG_AM)assert(analog.tone_quality>=AUTO_MOD_AM_FIT_MIN && fabsf(analog.depth_pct-50.f)<2.f);
    if(test==SIG_FM || test==SIG_PM){
        assert(analog.kind==AUTO_MOD_ANGLE && analog.tone_quality>=.85f);
        assert(!auto_mod_should_show_binary(analog.kind,analog.tone_quality,
                                             analog.modulation_hz,1,3,0,0,0));
    }
    if(test>=SIG_ASK && test<=SIG_FSK4){
        assert(digital.baud==BAUD);
        assert(digital.fsk==(test>=SIG_FSK2));
        assert(digital.ask==(test==SIG_ASK));
        assert(digital.levels==(test==SIG_FSK3?3:test==SIG_FSK4?4:2));
        assert(digital.bit_count>=8);
        assert(auto_mod_should_show_binary(analog.kind,analog.tone_quality,
                                            analog.modulation_hz,1,3,0,
                                            digital.fsk,digital.levels));
    }
    printf("case %d: analog=%d tone=%.3f depth=%.1f%% rate=%.1fHz digital=%d fsk=%d ask=%d levels=%d bits=%s\n",
           test,analog.kind,analog.tone_quality,analog.depth_pct,analog.modulation_hz,
           digital.baud,digital.fsk,digital.ask,digital.levels,digital.bits);
}

int main(void)
{
    assert(auto_mod_should_show_binary(AUTO_MOD_ANGLE,.90f,500.f,1,3,0,1,4));
    assert(auto_mod_should_show_binary(AUTO_MOD_ANGLE,.90f,500.f,1,3,0,1,3));
    assert(!auto_mod_should_show_binary(AUTO_MOD_ANGLE,.90f,500.f,1,3,0,1,2));
    for(enum signal_kind kind=SIG_AM;kind<=SIG_PM;kind++)check(kind);
    check_high_index_angle();
    check_fast_fsk_phase_steps();
    puts("AUTO_CLASSIFICATION_PROBE_DONE");
    return 0;
}
