#ifndef RECORD_HISTORY_H
#define RECORD_HISTORY_H
/* Bounded RAM archive. Caller serializes access. Every entry remains a separate
 * acquisition; adjacent frame IDs NEVER imply continuous ADC sampling. */
#include "v32cap3.h"
#include <stdlib.h>
#include <string.h>
#define RECORD_HISTORY_BUDGET (8u*1024u*1024u)
typedef struct {
    v32_frame_t frame;       /* Original signed ADC samples, no display smoothing. */
    uint64_t serial;         /* Local identity independent of wrapped FPGA IDs. */
    uint32_t received_ms;    /* Host monotonic arrival, not ADC sample time. */
    uint32_t adc_fs_hz;      /* Nominal rate validated by the CAP3 parser. */
    uint32_t discontinuity;  /* Always 1: CAP3 records are not continuous history. */
} record_entry_t;
typedef struct {
    record_entry_t *entries;
    size_t capacity,count,next;
    uint64_t total;
} record_history_t;
static int record_history_init(record_history_t *h,size_t budget)
{
    memset(h,0,sizeof *h);
    h->capacity=budget/sizeof(record_entry_t);
    if(!h->capacity)return 0;
    h->entries=calloc(h->capacity,sizeof(record_entry_t));
    if(!h->entries){h->capacity=0;return 0;}
    return 1;
}
static void record_history_push(record_history_t *h,const v32_frame_t *f,uint32_t now)
{
    if(!h->entries || !f || f->n<1 || f->n>V32_POINTS)return;
    record_entry_t *e=&h->entries[h->next];
    e->frame=*f;e->serial=++h->total;e->received_ms=now;
    e->adc_fs_hz=V32_ADC_FS_HZ;e->discontinuity=1;
    h->next=(h->next+1)%h->capacity;
    if(h->count<h->capacity)h->count++;
}
/* age=0 is newest. Copy-out avoids dangling pointers when the writer wraps. */
static int record_history_copy(const record_history_t *h,size_t age,record_entry_t *out)
{
    if(!out || age>=h->count)return 0;
    *out=h->entries[(h->next+h->capacity-1-age)%h->capacity];
    return 1;
}
static void record_history_free(record_history_t *h)
{
    free(h->entries);memset(h,0,sizeof *h);
}
#endif
