#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "thermal.h"
static float input[THERMAL_PIXELS];
static thermal_frame_t frame;
static void uniform(float t) { for(int i=0;i<THERMAL_PIXELS;i++) input[i]=t; }
int main(int argc,char **argv)
{
    uniform(NAN); assert(!thermal_validate(input,&frame));
    uniform(32); input[20]=INFINITY;
    assert(thermal_validate(input,&frame)); assert(frame.repaired==1);
    assert(frame.min==32 && frame.max==32 && frame.pixels[20]==32);
    uniform(32); input[0]=input[1]=input[32]=NAN;
    assert(!thermal_validate(input,&frame)); /* corner has no original valid neighbour */
    uniform(32); input[100]=NAN;
    assert(thermal_validate(input,&frame)); assert(frame.pixels[100]==32);
    uniform(32); for(int i=0;i<5;i++) input[i*40]=NAN;
    assert(!thermal_validate(input,&frame));
    uniform(32); assert(thermal_validate(input,&frame));
    float lo,hi; thermal_range(&frame,MODE_AUTO,&lo,&hi);
    assert(lo==32 && hi==32.5f && frame.max==32);
    thermal_range(&frame,MODE_BODY,&lo,&hi); assert(lo==30 && hi==45);
    thermal_range(&frame,MODE_HOME,&lo,&hi); assert(lo==-20 && hi==100);
    for(int y=0;y<24;y++) for(int x=0;x<32;x++) input[y*32+x]=20+x;
    assert(thermal_validate(input,&frame));
    uint8_t *memory=malloc(IMAGE_BYTES+32); assert(memory);
    memset(memory,0xa5,IMAGE_BYTES+32);
    thermal_render(memory+16,&frame,MODE_AUTO,false,50);
    for(int i=0;i<16;i++) assert(memory[i]==0xa5 && memory[IMAGE_BYTES+16+i]==0xa5);
    if(argc>1) {
        FILE *f=fopen(argv[1],"wb"); assert(f); fprintf(f,"P6\n480 320\n255\n");
        for(int i=0;i<IMAGE_W*IMAGE_H;i++) {
            uint8_t *p=memory+16+i*3; uint8_t rgb[]={p[2],p[1],p[0]}; fwrite(rgb,1,3,f);
        }
        fclose(f);
    }
    thermal_render(memory+16,&frame,MODE_BODY,true,1000);
    frame.valid=false; thermal_render(memory+16,&frame,MODE_AUTO,true,0);
    free(memory);
    puts("PASS: invalid frames, isolated repair, cluster rejection, actual extrema, ranges, render bounds");
}
