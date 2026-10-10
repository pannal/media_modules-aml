#!/usr/bin/env python3
"""Run actual driver-helper/policy seams; no kernel, firmware or module build.

The C integration fixture extracts the current preparation prefix, duration/type
block, full force check, status prefix and full reset callback. Queue ownership,
firmware reads and native output are intentionally not modeled.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import tempfile

DEFAULT_MODULE = str(Path(__file__).resolve().parents[1])

def function(source, name):
    match = re.search(r'^(?:static (?:inline )?)?(?:int|void|u32|bool) ' + re.escape(name) + r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise AssertionError('Missing production function: ' + name)
    start = source.index('{', match.start())
    clean = re.sub(r"/\*.*?\*/|//[^\n]*|\"(?:\\.|[^\"\\])*\"", lambda m: " " * len(m.group()), source, flags=re.S)
    depth = 1
    index = start + 1
    while depth:
        depth += (clean[index] == '{') - (clean[index] == '}')
        index += 1
    return source[match.start():index]

def replace_once(source, old, new):
    if old not in source:
        raise AssertionError('Mutation/extraction anchor absent: ' + old)
    return source.replace(old, new, 1)

STUBS = r'''
#include <assert.h>
#include <string.h>
#include <stdio.h>
typedef unsigned int u32;
typedef unsigned long u64;
typedef unsigned long ulong;
typedef unsigned int bool;
#define false 0
#define true 1
#define VIDTYPE_VIU_NV21 0x100
#define VIDTYPE_VIU_NV12 0x200
#define VIDTYPE_PROGRESSIVE 0x400
#define VIDTYPE_VIU_FIELD 0x800
#define VIDTYPE_INTERLACE_TOP 0x1000
#define VIDTYPE_INTERLACE_BOTTOM 0x2000
#define V4L2_PIX_FMT_NV12 12
#define V4L2_PIX_FMT_NV12M 13
#define FRAME_REPEAT_NONE 0
#define AV_SCRATCH_C 0
#define MREG_SEQ_INFO 1
static u32 sequence_register;
#define READ_VREG(x) ((x)==MREG_SEQ_INFO ? sequence_register : 0)
#define pr_info(x) ((void)0)
struct aml_vcodec_ctx { u32 cap_pix_fmt; };
struct vdec_s { void *private; };
struct vdec_mpeg12_hw_s {
 u32 frame_width, frame_height, frame_dur, frame_prog, seqinfo, dec_control;
 u32 is_used_v4l, i_only, frame_rpt_state, stat, first_i_frame_ready, reject_picture;
 u32 notify_work;
 struct { u32 bit_rate; } gvs;
 struct mpeg12_keep_prog keep_prog;
 struct aml_vcodec_ctx *v4l2_ctx;
};
struct vframe_s { u32 duration, duration_pulldown, type; };
struct vdec_info { u32 frame_width, frame_height, frame_dur, error_count, status, bit_rate; int frame_rate; };
struct output { u32 fields, duration[3], type[3]; };
static struct vdec_s dummy_vdec;
static struct vdec_s *hw_to_vdec(struct vdec_mpeg12_hw_s *hw) { dummy_vdec.private=hw; return &dummy_vdec; }
static void user_data_ready_notify(struct vdec_mpeg12_hw_s *hw, u32 pts, bool valid) { (void)hw; (void)pts; (void)valid; }
static int error_skip(struct vdec_mpeg12_hw_s *hw, u32 info, struct vframe_s *vf) { (void)info; (void)vf; return hw->reject_picture; }
static void vdec_schedule_work(u32 *work) { (*work)++; }
'''

TESTS = r'''
static void init(struct vdec_mpeg12_hw_s *hw, u32 enabled, u32 duration)
{
 memset(hw, 0, sizeof(*hw));
 local_init_fixture(hw); assert(hw->keep_prog.last_rff==2);
 hw->frame_width=720; hw->frame_height=480; hw->frame_dur=duration;
 hw->frame_prog=PICINFO_PROG; hw->seqinfo=SEQINFO_EXT_AVAILABLE; hw->first_i_frame_ready=1;
 hw->dec_control=enabled ? DEC_CONTROL_FLAG_KEEP_PROG_FRAME : 0;
}
static struct output picture(struct vdec_mpeg12_hw_s *hw, u32 flags, u32 seq, u32 duration)
{
 struct pic_info_t pic;
 u32 code;
 memset(&pic,0,sizeof(pic));
 pic.width=hw->frame_width; pic.height=hw->frame_height;
 for (code=0;code<9;code++) if(frame_rate_tab[code]==duration) break;
 assert(code<9);
 capture_fixture(hw,&pic,flags,(seq & ~0xf0U)|(code<<4));
 /* replaced with production picture metadata assignments when present */
 PICTURE_METADATA
 return prepare_fixture(hw,&pic);
}
static void helper_cases(void)
{
 struct mpeg12_keep_prog s, other;
 u32 i, sum, duration;
 mpeg12_keep_prog_reset(&s);
 assert(s.last_rff==2 && !s.active && !s.output_duration);
 assert(mpeg12_keep_prog_duration(&s,1,1,0,0,3203)==0);
 assert(mpeg12_keep_prog_duration(&s,1,1,0,1,3203)==0);
 assert(mpeg12_keep_prog_duration(&s,1,1,0,0,3203)==0);
 assert(mpeg12_keep_prog_duration(&s,1,1,0,1,3203)==4003);
 assert(s.active && s.remainder==3 && s.output_duration==4003);
 sum=4003;
 for(i=0;i<3;i++) sum+=mpeg12_keep_prog_duration(&s,1,1,0,i%2,3203);
 assert(sum==16015 && s.remainder==0);
 /* An unalternated current picture clears film immediately. */
 assert(mpeg12_keep_prog_duration(&s,1,1,0,0,3203)==0);
 assert(!s.active && !s.output_duration && !s.remainder);
 mpeg12_keep_prog_reset(&s);
 for(i=0;i<4;i++) duration=mpeg12_keep_prog_duration(&s,1,1,0,i%2,3200);
 assert(duration==4000 && s.active);
 assert(mpeg12_keep_prog_duration(&s,1,0,0,0,3200)==0 && !s.active);
 for(i=0;i<4;i++) duration=mpeg12_keep_prog_duration(&s,1,1,0,i%2,3200);
 assert(duration==4000);
 assert(mpeg12_keep_prog_duration(&s,1,1,1,0,3200)==0 && !s.active);
 for(i=0;i<4;i++) assert(mpeg12_keep_prog_duration(&s,1,1,0,i%2,3840)==0);
 for(i=0;i<4;i++) duration=mpeg12_keep_prog_duration(&s,1,1,0,i%2,3200);
 assert(duration==4000);
 assert(mpeg12_keep_prog_duration(&s,1,1,0,0,3203)==0 && !s.active);
 assert(mpeg12_keep_prog_duration(&s,0,1,0,1,3203)==0 && !s.base_duration);
 for(i=0;i<4;i++) assert(mpeg12_keep_prog_duration(&s,1,1,0,i%2,0)==0);
 for(i=0;i<4;i++) assert(mpeg12_keep_prog_duration(&s,1,1,0,i%2,0xffffffffU)==0);
 mpeg12_keep_prog_reset(&other);
 assert(!other.active && !other.alternations);
 for(i=0;i<4;i++) duration=mpeg12_keep_prog_duration(&s,1,1,0,i%2,3206);
 assert(duration==4007 && s.remainder==2);
 assert(mpeg12_keep_prog_duration(&other,1,1,0,0,3206)==0);
 assert(!other.active && s.active);
}
static void integration_cases(void)
{
 struct vdec_mpeg12_hw_s hw;
 struct output out;
 struct vdec_info status;
 struct vdec_s decoder;
 struct pic_info_t queued, later;
 u32 i;
 const u32 progseq=SEQINFO_EXT_AVAILABLE|SEQINFO_PROG;
 init(&hw,0,3200);
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST,SEQINFO_EXT_AVAILABLE,3200);
 assert(out.fields==1 && out.duration[0]==4800);
 assert(!hw.keep_prog.active);
 hw.seqinfo=progseq;
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST|PICINFO_TOP_FIRST,progseq,3200);
 assert(out.fields==1 && out.duration[0]==9600);
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST,progseq,3200);
 assert(out.duration[0]==6400);
 hw.frame_prog=0; hw.seqinfo=SEQINFO_EXT_AVAILABLE;
 out=picture(&hw,PICINFO_TOP_FIRST|PICINFO_RPT_FIRST,SEQINFO_EXT_AVAILABLE,3200);
 assert(out.fields==3 && out.duration[0]==1599 && out.duration[1]==1599 && out.duration[2]==1599);
 assert((out.type[0]&VIDTYPE_INTERLACE_TOP) && (out.type[1]&VIDTYPE_INTERLACE_BOTTOM));
 assert(hw.dec_control&DEC_CONTROL_FLAG_FORCE_SEQ_INTERLACE);
 /* OFF retains native zero-cache duration initialization and notify. */
 init(&hw,0,0);
 out=picture(&hw,PICINFO_PROG,SEQINFO_EXT_AVAILABLE|(4U<<4),3203);
 assert(out.fields==1 && out.duration[0]==3203 && hw.frame_dur==3203 && hw.notify_work==1);
 /* Startup capture must not depend on the still-zero cached duration. */
 init(&hw,1,0);
 assert(hw.frame_dur==0 && hw.notify_work==0);
 out=picture(&hw,PICINFO_PROG,SEQINFO_EXT_AVAILABLE|(4U<<4),3203);
 assert(out.fields==1 && out.duration[0]==3203 && hw.frame_dur==3203 && hw.notify_work==1);
 out=picture(&hw,PICINFO_PROG,SEQINFO_EXT_AVAILABLE|(4U<<4),3203);
 assert(out.duration[0]==3203 && hw.notify_work==1);
 /* A queued picture keeps its captured rate after later register/state change. */
 init(&hw,1,0);
 memset(&queued,0,sizeof(queued)); memset(&later,0,sizeof(later));
 queued.width=later.width=720; queued.height=later.height=480;
 capture_fixture(&hw,&queued,PICINFO_PROG|PICINFO_RPT_FIRST,SEQINFO_EXT_AVAILABLE|(4U<<4));
 capture_fixture(&hw,&later,PICINFO_PROG,SEQINFO_EXT_AVAILABLE|SEQINFO_PROG|(3U<<4));
 assert(queued.frame_dur==3203 && later.frame_dur==3840);
 hw.frame_dur=later.frame_dur; hw.seqinfo=later.seqinfo; sequence_register=later.seqinfo;
 out=prepare_fixture(&hw,&queued);
 assert(out.fields==1 && out.duration[0]==4804 && hw.frame_dur==3203 && hw.notify_work==1);
 assert(hw.seqinfo==later.seqinfo);
 /* Native duration initializes before admission; film cadence remains unaccepted. */
 init(&hw,1,0); hw.reject_picture=1;
 out=picture(&hw,PICINFO_PROG,SEQINFO_EXT_AVAILABLE|(4U<<4),3203);
 assert(hw.frame_dur==3203 && hw.notify_work==1 && !hw.keep_prog.active);
 /* Keep on uses the queued picture's flag, preserves latest hardware flag. */
 init(&hw,1,3203); hw.frame_prog=0;
 for(i=0;i<3;i++) {
  out=picture(&hw,PICINFO_PROG|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3203);
  assert(out.fields==1 && out.duration[0]==((i%2)?4804:3203));
 }
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST,SEQINFO_EXT_AVAILABLE,3203);
 assert(out.fields==1 && out.duration[0]==4003 && !hw.frame_prog);
 decoder.private=&hw;
 memset(&status,0,sizeof(status)); assert(status_fixture(&decoder,&status)==0);
 assert(status.frame_dur==out.duration[0] && status.frame_rate==24);
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST,SEQINFO_EXT_AVAILABLE,3203);
 assert(out.duration[0]==4804 && !hw.keep_prog.active);
 /* Genuine interlace clears cadence and never latches generated interlace on. */
 for(i=0;i<4;i++) out=picture(&hw,PICINFO_PROG|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3203);
 assert(hw.keep_prog.active);
 out=picture(&hw,PICINFO_TOP_FIRST,SEQINFO_EXT_AVAILABLE,3203);
 assert(out.fields==2 && out.duration[0]==1601 && !hw.keep_prog.active);
 assert(!(hw.dec_control&DEC_CONTROL_FLAG_FORCE_SEQ_INTERLACE));
 out=picture(&hw,PICINFO_PROG,SEQINFO_EXT_AVAILABLE,3203);
 assert(out.fields==1 && out.duration[0]==3203 && !hw.keep_prog.active);
 /* Sequence-native repeats and PAL never inherit an NTSC film verdict. */
 for(i=0;i<4;i++) out=picture(&hw,PICINFO_PROG|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3203);
 assert(hw.keep_prog.active);
 hw.seqinfo=progseq;
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST|PICINFO_TOP_FIRST,progseq,3203);
 assert(out.duration[0]==9609 && !hw.keep_prog.active);
 hw.seqinfo=SEQINFO_EXT_AVAILABLE; hw.frame_dur=3840;
 for(i=0;i<4;i++) {
  out=picture(&hw,PICINFO_PROG|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3840);
  assert(out.duration[0]==((i%2)?5760:3840) && !hw.keep_prog.active);
 }
 init(&hw,1,3200); hw.dec_control|=DEC_CONTROL_FLAG_FORCE_SEQ_INTERLACE|0x800;
 out=picture(&hw,PICINFO_PROG,SEQINFO_EXT_AVAILABLE,3200);
 assert(out.fields==1 && (hw.dec_control&0x800));
 /* Every existing explicit resolution/rate force flag still wins. */
 for(i=0;i<5;i++) {
  const u32 widths[5]={720,704,704,544,480};
  init(&hw,1,i==1?3200:3840);
  hw.frame_width=widths[i]; hw.frame_height=i==1?480:576;
  hw.dec_control|=2U<<i;
  out=picture(&hw,PICINFO_PROG,SEQINFO_EXT_AVAILABLE,hw.frame_dur);
  assert(out.fields==2 && !hw.keep_prog.active && (hw.dec_control&(2U<<i)));
 }
 init(&hw,1,3200);
 for(i=0;i<4;i++) out=picture(&hw,PICINFO_PROG|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3200);
 assert(hw.keep_prog.active);
 decoder.private=&hw; reset(&decoder);
 assert(!hw.keep_prog.active && !hw.keep_prog.output_duration && hw.keep_prog.last_rff==2);
 out=picture(&hw,PICINFO_PROG,SEQINFO_EXT_AVAILABLE,3200);
 assert(out.duration[0]==3200 && !hw.keep_prog.active);
 decoder.private=0; reset(&decoder); assert(status_fixture(&decoder,&status)==-1);

 init(&hw,1,3200); hw.reject_picture=1;
 for(i=0;i<6;i++) out=picture(&hw,PICINFO_PROG|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3200);
 assert(!hw.keep_prog.active && !hw.keep_prog.alternations);
 hw.reject_picture=0; hw.first_i_frame_ready=0;
 for(i=0;i<6;i++) out=picture(&hw,PICINFO_PROG|PICINFO_TYPE_P|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3200);
 assert(!hw.keep_prog.active && !hw.keep_prog.alternations);
 hw.first_i_frame_ready=1; hw.i_only=1;
 for(i=0;i<6;i++) out=picture(&hw,PICINFO_PROG|PICINFO_TYPE_P|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3200);
 assert(!hw.keep_prog.active && !hw.keep_prog.alternations);
 hw.i_only=0;
 for(i=0;i<4;i++) out=picture(&hw,PICINFO_PROG|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3200);
 assert(out.duration[0]==4000 && hw.keep_prog.active);
 hw.reject_picture=1;
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST,SEQINFO_EXT_AVAILABLE,3200);
 assert(hw.keep_prog.active && hw.keep_prog.output_duration==4000);
 hw.reject_picture=0;
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST,SEQINFO_EXT_AVAILABLE,3200);
 assert(!hw.keep_prog.active && out.duration[0]==4800);
 PICTURE_OWNERSHIP_TESTS
}
int main(void) { helper_cases(); integration_cases(); puts("production helper and driver policy seams passed"); return 0; }
'''

def fixture(source, header):
    prepare = function(source, 'prepare_display_buf')
    prefix = prepare[prepare.index('{')+1:prepare.index('\n\tfor (i = 0; i < field_num; i++)')]
    duration = prepare[prepare.index('\t\tset_frame_info(hw, vf);'):prepare.index('\n\t\tvf->signal_type')]
    force = function(source, 'force_interlace_picture')
    status = function(source, 'vmmpeg12_dec_status')
    status = status[:status.index('\n\tvstatus->frame_data')] + '\n return 0;\n}'
    status = replace_once(status,'vmmpeg12_dec_status','status_fixture')
    reset = function(source,'reset')
    initial=function(source,'vmpeg12_local_init')
    initial=re.search(r'\thw->frame_dur = hw->frame_prog = 0;\n\tmpeg12_keep_prog_reset\(&hw->keep_prog\);',initial).group(0)
    initial='static void local_init_fixture(struct vdec_mpeg12_hw_s *hw) {\n'+initial+'\n}\n'
    admission_start = prepare.index('\t\tif ((error_skip')
    admission_end = prepare.index(' {',admission_start)
    admission = prepare[admission_start:admission_end]
    commit_start=prepare.index('\t\t\tif (hw->dec_control & DEC_CONTROL_FLAG_KEEP_PROG_FRAME) {',admission_end)
    commit_end=prepare.index('\n\t\t\tdebug_print',commit_start)
    commit=prepare[commit_start:commit_end]
    table=re.search(r'static const u32 frame_rate_tab\[16\] = \{.*?\n\};',source,re.S).group(0)
    capture=re.search(r'\t\tnew_pic->buffer_info = info;\n\t\tnew_pic->seqinfo = seqinfo;\n\t\tnew_pic->frame_dur = [^;]+;',source).group(0)
    capture='static void capture_fixture(struct vdec_mpeg12_hw_s *hw, struct pic_info_t *new_pic, u32 info, u32 seqinfo) {\n sequence_register = seqinfo;\n (void)hw;\n'+capture+'\n}\n'
    native_duration=function(source,'set_frame_info')
    native_start=native_duration.index('\tif (hw->frame_dur > 0)')
    native_end=native_duration.index('\n/*',native_start)
    native_duration='static void set_frame_info(struct vdec_mpeg12_hw_s *hw, struct vframe_s *vf) {\n'+native_duration[native_start:native_end]+'\n}\n'

    defines='\n'.join(re.findall(r'^#define (?:PICINFO_|SEQINFO_|DEC_CONTROL_)[^\n]+',source,re.M))
    defines+='\n#define INTERLACE_SEQ_ALWAYS\n#define NV21\n'
    pic=re.search(r'struct pic_info_t \{.*?\n\};',source,re.S).group(0)
    prepare_fixture='''static struct output prepare_fixture(struct vdec_mpeg12_hw_s *hw, struct pic_info_t *pic) {\n'''+prefix+r'''
 struct output output;
 struct vframe_s frames[3];
 (void)index; (void)vdec; (void)pb_skip;
 memset(&output,0,sizeof(output)); memset(frames,0,sizeof(frames));
 output.fields=field_num;
 assert(field_num<=3);
 for(i=0;i<field_num;i++) {
  vf=&frames[i];
'''+duration+'\n'+admission+' { } else {\n'+commit+'\n}\n'+r'''
  output.duration[i]=vf->duration; output.type[i]=vf->type;
 }
 return output;
}
'''
    # Declarations must be at the start for strict GNU89, just like production.
    prepare_fixture=replace_once(prepare_fixture,' {\n',' {\n struct output output;\n struct vframe_s frames[3];\n')
    prepare_fixture=replace_once(prepare_fixture,'\n struct output output;\n struct vframe_s frames[3];\n (void)index;', '\n (void)index;')
    assignments=['/* metadata comes from actual ISR capture_fixture */']
    ownership=''
    if re.search(r'\bu32\s+seqinfo;',pic) and re.search(r'\bu32\s+frame_dur;',pic):
        ownership=r'''
 init(&hw,1,3203); hw.seqinfo=progseq; hw.frame_dur=3840;
 for(i=0;i<4;i++) out=picture(&hw,PICINFO_PROG|((i%2)?PICINFO_RPT_FIRST:0),SEQINFO_EXT_AVAILABLE,3203);
 assert(out.duration[0]==4003 && hw.keep_prog.active);
 assert(hw.seqinfo==progseq && hw.frame_dur==3203);
 out=picture(&hw,PICINFO_PROG|PICINFO_RPT_FIRST|PICINFO_TOP_FIRST,progseq,3200);
 assert(out.duration[0]==9600 && !hw.keep_prog.active);
 assert(hw.seqinfo==progseq && hw.frame_dur==3200);
'''
    tests=TESTS.replace('PICTURE_METADATA',' '.join(assignments)).replace('PICTURE_OWNERSHIP_TESTS',ownership)
    generated='#include "mpeg12_keep_prog.h"\n'+defines+STUBS+pic+'\n'+table+'\n'+native_duration+'\n'+capture+'\n'+force+'\n'+prepare_fixture+'\n'+status+'\n'+reset+'\n'+initial+'\n'+tests
    return generated,dict(production_functions=['prepare_display_buf policy prefix + duration/type block','force_interlace_picture full body', 'actual error/first-I admission expression + accepted cadence publication','vmmpeg12_dec_status through frame_dur publication','reset full body', 'local_init actual initial duration/progressive/cadence statements', 'ISR actual per-picture sequence/duration capture and full rate table', 'accepted picture duration publication and notify', 'set_frame_info actual native duration initialization before admission with distinct sequence register'],picture_metadata=bool(ownership))

KODI_STUBS = r'''
#include <cassert>
#include <atomic>
#include <fstream>
#include <string>
#include <cstdint>
#include <cmath>
#include <sys/stat.h>
#include <unistd.h>
constexpr unsigned int UNIT_FREQ=96000;
constexpr unsigned int MIN_DECODER_VIDEO_RATE=UNIT_FREQ/120;
constexpr unsigned int MAX_DECODER_VIDEO_RATE=UNIT_FREQ/5;
constexpr unsigned int DVD_TIME_BASE=1000000;
constexpr unsigned int DVP_FLAG_INTERLACED=1;
enum class MPEG2OutputMode {UNKNOWN, PROGRESSIVE, INTERLACED_FIELD};
struct Picture { unsigned int iFlags; double iDuration; MPEG2OutputMode mpeg2OutputMode; };
struct vdec_info { unsigned int frame_dur; };
static void seed(const std::string& path, const std::string& value) { std::ofstream output(path); output << value; assert(output.good()); }
static std::string read(const std::string& path) { std::ifstream input(path); return std::string(std::istreambuf_iterator<char>(input), {}); }
'''
KODI_TESTS = r'''
static void setter_cases(const std::string& root)
{
 const std::string path=root+"/control";
 assert(!SetMPEG2KeepProgressive((root+"/absent").c_str(),true));
 const char* invalid[]={"", "garbage", "4294967296", "18446744073709551616", "64xyz", "-1", "0x40", "64 32"};
 for(const char* value:invalid) { seed(path,value); assert(!SetMPEG2KeepProgressive(path.c_str(),true)); assert(read(path)==value); }
 seed(path,"2147483662\n"); assert(SetMPEG2KeepProgressive(path.c_str(),true)); assert(read(path)=="2147483726");
 assert(SetMPEG2KeepProgressive(path.c_str(),false)); assert(read(path)=="2147483662");
 seed(path,"4294967295"); assert(SetMPEG2KeepProgressive(path.c_str(),false)); assert(read(path)=="4294967231");
 assert(SetMPEG2KeepProgressive(path.c_str(),true)); assert(read(path)=="4294967295");
 seed(path,"0\n"); assert(SetMPEG2KeepProgressive(path.c_str(),false)); assert(read(path)=="0\n");
 assert(SetMPEG2KeepProgressive(path.c_str(),true)); assert(read(path)=="64");
 seed(path," \t14\n "); assert(SetMPEG2KeepProgressive(path.c_str(),true)); assert(read(path)=="78");
 seed(path,"14"); assert(chmod(path.c_str(),0400)==0);
 if(geteuid()!=0) { assert(!SetMPEG2KeepProgressive(path.c_str(),true)); assert(read(path)=="14"); }
 assert(chmod(path.c_str(),0600)==0);
}
static void output_cases(void)
{
 Picture picture;
 for(int enabled=0;enabled<2;enabled++) for(int interlace=0;interlace<2;interlace++) {
  picture=output_fixture(enabled,interlace,0,4004,12345.0);
  if(!enabled) { assert(picture.mpeg2OutputMode==MPEG2OutputMode::UNKNOWN && picture.iDuration==12345.0 && picture.iFlags==0); }
  else if(interlace) { assert(picture.mpeg2OutputMode==MPEG2OutputMode::INTERLACED_FIELD && picture.iDuration==12345.0 && picture.iFlags==DVP_FLAG_INTERLACED); }
  else { assert(picture.mpeg2OutputMode==MPEG2OutputMode::PROGRESSIVE && std::abs(picture.iDuration-41708.333333333333)<0.000001 && picture.iFlags==0); }
 }
 const unsigned int invalid[]={0,799,19201,0xffffffffU};
 for(unsigned int duration:invalid) {
  picture=output_fixture(true,false,0,duration,12345.0);
  assert(picture.mpeg2OutputMode==MPEG2OutputMode::UNKNOWN && picture.iDuration==12345.0);
 }
 picture=output_fixture(true,false,-1,4004,12345.0);
 assert(picture.mpeg2OutputMode==MPEG2OutputMode::UNKNOWN && picture.iDuration==12345.0);
 picture=output_fixture(true,false,0,800,12345.0); assert(picture.mpeg2OutputMode==MPEG2OutputMode::PROGRESSIVE);
 picture=output_fixture(true,false,0,19200,12345.0); assert(picture.mpeg2OutputMode==MPEG2OutputMode::PROGRESSIVE);
}
int main(int argc,char** argv) { assert(argc==2); setter_cases(argv[1]); output_cases(); }
'''

def kodi_fixture(source):
    setter=function(source,'SetMPEG2KeepProgressive')
    flags_start=source.index('    videoPicture.iFlags = 0;',source.index('CAMLCodec::GetPicture'))
    flags_end=source.index('\n\n    // Retain decoded-frame',flags_start)
    flags=source[flags_start:flags_end]
    status_start=source.index('    if (m_keepMPEG2Progressive && !interlaced)',flags_end)
    status_end=source.index('\n    if (vi.ratio_control)',status_start)
    status=source[status_start:status_end]
    output='''static Picture output_fixture(bool m_keepMPEG2Progressive,bool interlaced,int infoResult,unsigned int duration,double initial) {
 Picture videoPicture; vdec_info vi={duration}; videoPicture.iDuration=initial;
'''+flags+'\n'+status+'\n return videoPicture;\n}\n'
    return KODI_STUBS+setter+'\n'+output+KODI_TESTS

def run_kodi_cases(args,logs,report):
    source=(Path(args.kodi)/'xbmc/cores/VideoPlayer/DVDCodecs/Video/AMLCodec.cpp').read_text()
    code=kodi_fixture(source)
    variants=[('kodi-production',code,True),
      ('kodi-overwrite-unrelated-bits',replace_once(code,'enabled ? control | KEEP_PROG_FRAME : control & ~KEEP_PROG_FRAME','enabled ? KEEP_PROG_FRAME : 0'),False),
      ('kodi-accept-overflow',replace_once(code,' || control > 0xffffffffULL',''),False),
      ('kodi-accept-trailing-garbage',replace_once(code,'if (!input.eof())','if (false)'),False),
      ('kodi-disable-keeps-bit',replace_once(code,'control & ~KEEP_PROG_FRAME','control | KEEP_PROG_FRAME'),False),
      ('kodi-off-falsely-classified',replace_once(code,'videoPicture.mpeg2OutputMode = MPEG2OutputMode::UNKNOWN;','videoPicture.mpeg2OutputMode = MPEG2OutputMode::PROGRESSIVE;'),False),
      ('kodi-invalid-status-classified',replace_once(code,'else\n        videoPicture.mpeg2OutputMode = MPEG2OutputMode::UNKNOWN;','else\n        videoPicture.mpeg2OutputMode = MPEG2OutputMode::PROGRESSIVE;'),False),
      ('kodi-ignore-status-error',replace_once(code,'infoResult == 0 && vi.frame_dur >=','infoResult != 999 && vi.frame_dur >='),False),
      ('kodi-use-film-duration-for-fields',replace_once(code,'if (m_keepMPEG2Progressive && !interlaced)','if (m_keepMPEG2Progressive)'),False),
    ]
    report['kodi_source_sha256']=hashlib.sha256(source.encode()).hexdigest()
    env=os.environ.copy();env['ASAN_OPTIONS']='detect_leaks=0';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    with tempfile.TemporaryDirectory(prefix='mpeg2-kodi-policy-') as temporary:
        base=Path(temporary)
        for name,variant,expected in variants:
            case=base/name;case.mkdir();fixture_path=case/'fixture.cpp';fixture_path.write_text(variant)
            command=['g++','-std=c++17','-Wall','-Wextra','-Werror','-O1','-g','-fno-omit-frame-pointer','-fno-pie','-no-pie','-fsanitize=address,undefined',str(fixture_path),'-o',str(case/'fixture')]
            compiled=subprocess.run(command,text=True,capture_output=True)
            (logs/(name+'-compile.log')).write_text(compiled.stdout+compiled.stderr)
            (logs/(name+'.cpp')).write_text(variant)
            if compiled.returncode: raise AssertionError(name+' must compile: '+str(logs/(name+'-compile.log')))
            ran=subprocess.run([str(case/'fixture'),str(case)],text=True,capture_output=True,env=env)
            (logs/(name+'-run.log')).write_text(ran.stdout+ran.stderr)
            report['controls'].append(dict(name=name,compile_returncode=compiled.returncode,run_returncode=ran.returncode,expected_pass=expected,compile_argv=command))
            if expected!=(ran.returncode==0): raise AssertionError(name+' unexpected result: '+str(logs/(name+'-run.log')))
            if not expected and (ran.returncode != -signal.SIGABRT or not re.search(r'\bAssertion .+ failed\.', ran.stderr)):
                raise AssertionError(name+' did not fail a runtime assertion: '+str(logs/(name+'-run.log')))
            print(name+(': PASS' if expected else ': rejected'))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--module',default=DEFAULT_MODULE)
    parser.add_argument('--kodi', required=True, help='Kodi source checkout for the matching control/output tests')
    parser.add_argument('--logs', help='Directory for verification logs (defaults to a temporary directory)')
    parser.add_argument('--ce-syntax',action='store_true')
    parser.add_argument('--ce-flags-json', help='Saved actual CE compiler argv/kernel cwd JSON for syntax-only verification')
    args=parser.parse_args()
    if args.ce_syntax and not args.ce_flags_json:
        parser.error('--ce-syntax requires --ce-flags-json from the configured CE build')
    if not args.logs:
        args.logs = tempfile.mkdtemp(prefix='mpeg2-driver-regression-')
    module=Path(args.module); directory=module/'drivers/frame_provider/decoder/mpeg12'
    source=(directory/'vmpeg12_multi.c').read_text(); header=(directory/'mpeg12_keep_prog.h').read_text()
    code,coverage=fixture(source,header)
    logs=Path(args.logs);logs.mkdir(parents=True,exist_ok=True)
    report={'coverage':coverage,'limitations':'Policy seam test; queue/firmware/native device execution not modeled; actual admission predicate and cadence publication are executed.', 'source_sha256':hashlib.sha256(source.encode()).hexdigest(),'helper_sha256':hashlib.sha256(header.encode()).hexdigest(),'controls':[]}
    cases=[('production',code,header,True)]
    cases += [
      ('premature-film',code,replace_once(header,'state->alternations >= 3','state->alternations >= 2'),False),
      ('stale-current-rff',code,replace_once(header,'state->alternations = 0;\n\t\tstate->remainder = 0;','state->alternations = 3;\n\t\tstate->remainder = 0;'),False),
      ('missing-progressive-boundary-reset',code,replace_once(header,'!enabled || !progressive || progressive_sequence ||','!enabled || !progressive ||').replace('unsigned int numerator;', 'unsigned int numerator;\n\t(void)progressive_sequence;'),False),
      ('lost-quarter-remainder',code,replace_once(header,'5 * base_duration + state->remainder','5 * base_duration'),False),
      ('wrong-duration-scale',code,replace_once(header,'numerator / 4','numerator / 2'),False),
      ('ignored-explicit-force',replace_once(code,'frame_prog = force_interlace_picture(hw->dec_control, pic->width,\n\t\t\tpic->height, pic->frame_dur, info & PICINFO_PROG);','frame_prog = force_interlace_picture(0, pic->width,\n\t\t\tpic->height, pic->frame_dur, info & PICINFO_PROG);'),header,False),
      ('startup-captures-zero-cache',replace_once(code,'new_pic->frame_dur = frame_rate_tab[(seqinfo >> 4) & 0xf];','new_pic->frame_dur = hw->frame_dur;'),header,False),
      ('missing-accepted-rate-publication',replace_once(code,'hw->frame_dur = pic->frame_dur;','hw->frame_dur = hw->frame_dur;'),header,False),
      ('latest-sequence-used',replace_once(code,'(pic->seqinfo & SEQINFO_EXT_AVAILABLE) &&\n\t\t\t(pic->seqinfo & SEQINFO_PROG)','(hw->seqinfo & SEQINFO_EXT_AVAILABLE) &&\n\t\t\t(hw->seqinfo & SEQINFO_PROG)'),header,False),
      ('latest-duration-used',replace_once(code,'vf->duration = pic->frame_dur;','vf->duration = hw->frame_dur;'),header,False),
      ('dropped-picture-published',replace_once(code,'if ((error_skip(hw, pic->buffer_info, vf)) ||','if ((error_skip(hw, pic->buffer_info, vf) && 0) ||'),header,False),
      ('raw-film-status',replace_once(code,'hw->keep_prog.active ? hw->keep_prog.output_duration : hw->frame_dur','hw->frame_dur'),header,False),
      ('missing-initial-cadence-reset',replace_once(code,'hw->frame_dur = hw->frame_prog = 0;\n\tmpeg12_keep_prog_reset(&hw->keep_prog);','hw->frame_dur = hw->frame_prog = 0;'),header,False),
      ('missing-seek-reset',replace_once(code,'if (hw)\n\t\tmpeg12_keep_prog_reset(&hw->keep_prog);','if (hw)\n\t\t(void)hw;'),header,False),
      ('double-film-rff',replace_once(code,'vf->duration = film_duration;\n\t\t\t\tvf->duration_pulldown = 0;','vf->duration = film_duration;\n\t\t\t\tvf->duration_pulldown = (info & PICINFO_RPT_FIRST) ? (film_duration >> 1) : 0;'),header,False),
    ]
    env=os.environ.copy();env['ASAN_OPTIONS']='detect_leaks=0';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    flags=['gcc','-std=gnu89','-Wall','-Wextra','-Werror','-Wdeclaration-after-statement','-Wstrict-prototypes','-Werror=implicit-function-declaration','-O1','-g','-fno-omit-frame-pointer','-fno-pie','-no-pie','-fsanitize=address,undefined']
    with tempfile.TemporaryDirectory(prefix='mpeg2-driver-') as temporary:
        base=Path(temporary)
        for name,variant,variant_header,expected in cases:
            case=base/name;case.mkdir();(case/'mpeg12_keep_prog.h').write_text(variant_header);(case/'fixture.c').write_text(variant)
            command=flags+['-I',str(case),str(case/'fixture.c'),'-o',str(case/'fixture')]
            compiled=subprocess.run(command,text=True,capture_output=True)
            (logs/(name+'-compile.log')).write_text(compiled.stdout+compiled.stderr)
            if compiled.returncode:
                raise AssertionError(name+' must compile, see '+str(logs/(name+'-compile.log')))
            ran=subprocess.run([str(case/'fixture')],text=True,capture_output=True,env=env)
            (logs/(name+'-run.log')).write_text(ran.stdout+ran.stderr)
            (logs/(name+'.c')).write_text(variant)
            report['controls'].append({'name':name,'compile_returncode':compiled.returncode,'run_returncode':ran.returncode,'expected_pass':expected,'compile_argv':command})
            if expected != (ran.returncode==0):
                raise AssertionError(name+' unexpected result, see '+str(logs/(name+'-run.log')))
            if not expected and (ran.returncode != -signal.SIGABRT or not re.search(r'\bAssertion .+ failed\.', ran.stderr)):
                raise AssertionError(name+' did not fail a runtime assertion: '+str(logs/(name+'-run.log')))
            print(name+(': PASS' if expected else ': rejected'))
    run_kodi_cases(args,logs,report)
    if args.ce_syntax:
        ce=json.loads(Path(args.ce_flags_json).read_text());command=ce['argv']
        command=[str(directory/'vmpeg12_multi.c') if arg.endswith('/vmpeg12_multi.c') else arg for arg in command]
        ce_env=os.environ.copy();ce_env['CCACHE_DISABLE']='1';ce_env['CCACHE_DIR']=str(logs/'ccache');ce_env['CCACHE_TEMPDIR']=str(logs/'ccache-tmp');(logs/'ccache-tmp').mkdir(exist_ok=True)
        ran=subprocess.run(command,cwd=ce['kernel_cwd'],text=True,capture_output=True,env=ce_env)
        (logs/'ce-syntax.log').write_text(ran.stdout+ran.stderr)
        report['ce_syntax']={'cwd':ce['kernel_cwd'],'argv':command,'returncode':ran.returncode,'environment':{'CCACHE_DISABLE':'1','CCACHE_DIR':ce_env['CCACHE_DIR'],'CCACHE_TEMPDIR':ce_env['CCACHE_TEMPDIR']}}
        if ran.returncode:
            (logs/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            raise AssertionError('Actual CE syntax check failed, see '+str(logs/'ce-syntax.log'))
    (logs/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Verified production seams; '+str(sum(not item['expected_pass'] for item in report['controls']))+' compiled behavioural controls rejected; '+str(logs/'report.json'))

if __name__=='__main__':
    main()
