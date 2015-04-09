/*****************************************************************************
 * x264: h264 encoder/decoder testing program.
 *****************************************************************************
 * Copyright (C) 2003 Laurent Aimar
 * $Id: x264.c,v 1.1 2004/06/03 19:24:12 fenrir Exp $
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111, USA.
 *****************************************************************************/

#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <signal.h>

#if !(defined(_MSC_VER) || defined(__MINGW32__))
#include <sys/time.h>
#else
#include <sys/types.h>
#include <sys/timeb.h>
#endif
//#define _GNU_SOURCE
#include "getopt.h"

#ifdef _MSC_VER
#include <io.h>     /* _setmode() */
#include <fcntl.h>  /* _O_BINARY */
#endif

//#ifndef _MSC_VER
//#include "config.h"
//#endif

#include "common.h"
#include "x264.h"
//#include "muxers.h"

#define DATA_MAX 3000000
UINT8 data[DATA_MAX];

/* Ctrl-C handler */
static int     b_ctrl_c = 0;
static int     b_exit_on_ctrl_c = 0;
static void    SigIntHandler( int a )
{
    if( b_exit_on_ctrl_c )
        exit(0);
    b_ctrl_c = 1;
}

typedef struct {
    int b_decompress;
    int b_progress;
    int i_seek;
    hnd_t hin;
    hnd_t hout;
} cli_opt_t;

/* input file operation function pointers */
static int (*p_open_infile)( char *psz_filename, hnd_t *p_handle, x264_param_t *p_param );
static int (*p_get_frame_total)( hnd_t handle, int i_width, int i_height );
static int (*p_read_frame)( x264_picture_t *p_pic, hnd_t handle, int i_frame, int i_width, int i_height );
static int (*p_close_infile)( hnd_t handle );

/* output file operation function pointers */
static int (*p_open_outfile)( char *psz_filename, hnd_t *p_handle );
static int (*p_set_outfile_param)( hnd_t handle, x264_param_t *p_param );
static int (*p_write_nalu)( hnd_t handle, UINT8 *p_nal, int i_size );
static int (*p_set_eop)( hnd_t handle, x264_picture_t *p_picture );
static int (*p_close_outfile)( hnd_t handle );

static void Help( x264_param_t *defaults );
static int  Parse( int argc, char **argv, x264_param_t *param, cli_opt_t *opt );
static int  Encode( x264_param_t *param, cli_opt_t *opt );


/****************************************************************************
 * main:
 ****************************************************************************/
int main( int argc, char **argv )
{
    x264_param_t param;
    cli_opt_t opt;

#ifdef _MSC_VER
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    x264_param_default( &param );

    /* Parse command line */
    if( Parse( argc, argv, &param, &opt ) < 0 )
        return -1;

    /* Control-C handler */
    signal( SIGINT, SigIntHandler );

    return Encode( &param, &opt );
}

#if 1
static const char * const overscan_str[] = { "undef", "show", "crop", NULL };
static const char * const vidformat_str[] = { "component", "pal", "ntsc", "secam", "mac", "undef", NULL };
static const char * const fullrange_str[] = { "off", "on", NULL };
static const char * const colorprim_str[] = { "", "bt709", "undef", "", "bt470m", "bt470bg", "smpte170m", "smpte240m", "film", NULL };
static const char * const transfer_str[] = { "", "bt709", "undef", "", "bt470m", "bt470bg", "smpte170m", "smpte240m", "linear", "log100", "log316", NULL };
static const char * const colmatrix_str[] = { "GBR", "bt709", "undef", "", "fcc", "bt470bg", "smpte170m", "smpte240m", "YCgCo", NULL };
#endif
static char const *strtable_lookup( const char * const table[], int index )
{
    int i = 0; while( table[i] ) i++;
    return ( ( index >= 0 && index < i ) ? table[ index ] : "???" );
}

/*****************************************************************************
 * Help:
 *****************************************************************************/
static void Help( x264_param_t *defaults )
{
    fprintf(stdout, 
             "x264 core:%d%s\n"
             "Syntax: x264 [options] -o outfile infile [widthxheight]\n"
             "\n"
             "Infile can be raw YUV 4:2:0 (in which case resolution is required),\n"
             "  or AVI or Avisynth if compiled with AVIS support (%s).\n"
             "Outfile type is selected by filename:\n"
             " .264 -> Raw bytestream\n"
             " .mkv -> Matroska\n"
             " .mp4 -> MP4 if compiled with GPAC support (%s)\n"
             "\n"
             "Options:\n"
             "\n"
             "  -h, --help                  Print this help\n"
             "\n"
             "Frame-type options:\n"
             "\n"
             "  -I, --keyint <integer>      Maximum GOP size [%d]\n"
             "  -i, --min-keyint <integer>  Minimum GOP size [%d]\n"
             "      --scenecut <integer>    How aggressively to insert extra I-frames [%d]\n"
             "  -b, --bframes <integer>     Number of B-frames between I and P [%d]\n"
             "      --no-b-adapt            Disable adaptive B-frame decision\n"
             "      --b-bias <integer>      Influences how often B-frames are used [%d]\n"
             "      --b-pyramid             Keep some B-frames as references\n"
             "\n"
             "      --no-cabac              Disable CABAC\n"
             "  -r, --ref <integer>         Number of reference frames [%d]\n"
             "      --nf                    Disable loop filter\n"
             "  -f, --filter <alpha:beta>   Loop filter AlphaC0 and Beta parameters [%d:%d]\n"
             "\n"
             "Ratecontrol:\n"
             "\n"
             "  -q, --qp <integer>          Set QP (0=lossless) [%d]\n"
             "  -B, --bitrate <integer>     Set bitrate\n"
             "      --crf <integer>         Quality-based VBR (nominal QP)\n"
             "      --qpmin <integer>       Set min QP [%d]\n"
             "      --qpmax <integer>       Set max QP [%d]\n"
             "      --qpstep <integer>      Set max QP step [%d]\n"
             "      --ratetol <float>       Allowed variance of average bitrate [%.1f]\n"
             "      --vbv-maxrate <integer> Max local bitrate [%d]\n"
             "      --vbv-bufsize <integer> Size of VBV buffer [%d]\n"
             "      --vbv-init <float>      Initial VBV buffer occupancy [%.1f]\n"
             "\n"
             "      --ipratio <float>       QP factor between I and P [%.2f]\n"
             "      --pbratio <float>       QP factor between P and B [%.2f]\n"
             "      --chroma-qp-offset <integer>  QP difference between chroma and luma [%d]\n"
             "\n"
             "  -p, --pass <1|2|3>          Enable multipass ratecontrol:\n"
             "                                  - 1: First pass, creates stats file\n"
             "                                  - 2: Last pass, does not overwrite stats file\n"
             "                                  - 3: Nth pass, overwrites stats file\n"
             "      --stats <string>        Filename for 2 pass stats [\"%s\"]\n"
             "      --rceq <string>         Ratecontrol equation [\"%s\"]\n"
             "      --qcomp <float>         QP curve compression: 0.0 => CBR, 1.0 => CQP [%.2f]\n"
             "      --cplxblur <float>      Reduce fluctuations in QP (before curve compression) [%.1f]\n"
             "      --qblur <float>         Reduce fluctuations in QP (after curve compression) [%.1f]\n"
             "\n"
             "      --zones <zone0>/<zone1>/...\n"
             "                              Tweak the bitrate of some regions of the video\n"
             "                              Each zone is of the form\n"
             "                                  <start frame>,<end frame>,<option>\n"
             "                                  where <option> is either\n"
             "                                      q=<integer> (force QP)\n"
             "                                  or  b=<float> (bitrate multiplier)\n"
             "\n"
             "Analysis:\n"
             "\n"
             "  -A, --analyse <string>      Partitions to consider [\"p8x8,b8x8,i8x8,i4x4\"]\n"
             "                                  - p8x8, p4x4, b8x8, i8x8, i4x4\n"
             "                                  - none, all\n"
             "                                  (p4x4 requires p8x8. i8x8 requires --8x8dct.)\n"
             "      --direct <string>       Direct MV prediction mode [\"%s\"]\n"
             "                                  - none, spatial, temporal, auto\n"
             "  -w, --weightb               Weighted prediction for B-frames\n"
             "      --me <string>           Integer pixel motion estimation method [\"%s\"]\n"
             "                                  - dia: diamond search, radius 1 (fast)\n"
             "                                  - hex: hexagonal search, radius 2\n"
             "                                  - umh: uneven multi-hexagon search\n"
             "                                  - esa: exhaustive search (slow)\n"
             "      --merange <integer>     Maximum motion vector search range [%d]\n"
             "  -m, --subme <integer>       Subpixel motion estimation and partition\n"
             "                                  decision quality: 1=fast, 6=best. [%d]\n"
             "      --b-rdo                 RD based mode decision for B-frames. Requires subme 6.\n"
             "      --mixed-refs            Decide references on a per partition basis\n"
             "      --no-chroma-me          Ignore chroma in motion estimation\n"
             "      --bime                  Jointly optimize both MVs in B-frames\n"
             "  -8, --8x8dct                Adaptive spatial transform size\n"
             "  -t, --trellis <integer>     Trellis RD quantization. Requires CABAC. [%d]\n"
             "                                  - 0: disabled\n"
             "                                  - 1: enabled only on the final encode of a MB\n"
             "                                  - 2: enabled on all mode decisions\n"
             "      --no-fast-pskip         Disables early SKIP detection on P-frames\n"
             "      --nr <integer>          Noise reduction [%d]\n"
             "\n"
             "      --cqm <string>          Preset quant matrices [\"flat\"]\n"
             "                                  - jvt, flat\n"
             "      --cqmfile <string>      Read quant matrices from a JM-compatible file\n"
             "                                  Overrides any other --cqm* options.\n"
             "      --cqm4 <list>           Set all 4x4 quant matrices\n"
             "                                  Takes a comma-separated list of 16 integers.\n"
             "      --cqm8 <list>           Set all 8x8 quant matrices\n"
             "                                  Takes a comma-separated list of 64 integers.\n"
             "      --cqm4i, --cqm4p, --cqm8i, --cqm8p\n"
             "                              Set both luma and chroma quant matrices\n"
             "      --cqm4iy, --cqm4ic, --cqm4py, --cqm4pc\n"
             "                              Set individual quant matrices\n"
             "\n"
             "Video Usability Info (Annex E):\n"
             "The VUI settings are not used by the encoder but are merely suggestions to\n"
             "the playback equipment. See doc/vui.txt for details. Use at your own risk.\n"
             "\n"
             "      --sar width:height      Specify Sample Aspect Ratio\n"
             "      --overscan <string>     Specify crop overscan setting [\"%s\"]\n"
             "                                  - undef, show, crop\n"
             "      --videoformat <string>  Specify video format [\"%s\"]\n"
             "                                  - component, pal, ntsc, secam, mac, undef\n"
             "      --fullrange <string>    Specify full range samples setting [\"%s\"]\n"
             "                                  - off, on\n"
             "      --colorprim <string>    Specify color primaries [\"%s\"]\n"
             "                                  - undef, bt709, bt470m, bt470bg\n"
             "                                    smpte170m, smpte240m, film\n"
             "      --transfer <string>     Specify transfer characteristics [\"%s\"]\n"
             "                                  - undef, bt709, bt470m, bt470bg, linear,\n"
             "                                    log100, log316, smpte170m, smpte240m\n"
             "      --colormatrix <string>  Specify color matrix setting [\"%s\"]\n"
             "                                  - undef, bt709, fcc, bt470bg\n"
             "                                    smpte170m, smpte240m, GBR, YCgCo\n"
             "      --chromaloc <integer>   Specify chroma sample location (0 to 5) [%d]\n"
             "\n"
             "Input/Output:\n"
             "\n"
             "      --level <string>        Specify level (as defined by Annex A)\n"
             "      --fps <float|rational>  Specify framerate\n"
             "      --seek <integer>        First frame to encode\n"
             "      --frames <integer>      Maximum number of frames to encode\n"
             "  -o, --output                Specify output file\n"
             "\n"
             "      --threads <integer>     Parallel encoding (uses slices)\n"
             "      --no-asm                Disable all CPU optimizations\n"
             "      --no-psnr               Disable PSNR computation\n"
             "      --quiet                 Quiet Mode\n"
             "  -v, --verbose               Print stats for each frame\n"
             "      --progress              Show a progress indicator while encoding\n"
             "      --visualize             Show MB types overlayed on the encoded video\n"
             "      --aud                   Use access unit delimiters\n"
             "\n",
            X264_BUILD, X264_VERSION,
#ifdef AVIS_INPUT
            "yes",
#else
            "no",
#endif
#ifdef MP4_OUTPUT
            "yes",
#else
            "no",
#endif
            defaults->i_keyint_max,
            defaults->i_keyint_min,
            defaults->i_scenecut_threshold,
            defaults->i_bframe,
            defaults->i_bframe_bias,
            defaults->i_frame_reference,
            defaults->i_deblocking_filter_alphac0,
            defaults->i_deblocking_filter_beta,
            defaults->rc.i_qp_constant,
            defaults->rc.i_qp_min,
            defaults->rc.i_qp_max,
            defaults->rc.i_qp_step,
            defaults->rc.f_rate_tolerance,
            defaults->rc.i_vbv_max_bitrate,
            defaults->rc.i_vbv_buffer_size,
            defaults->rc.f_vbv_buffer_init,
            defaults->rc.f_ip_factor,
            defaults->rc.f_pb_factor,
            defaults->analyse.i_chroma_qp_offset,
            defaults->rc.psz_stat_out,
            defaults->rc.psz_rc_eq,
            defaults->rc.f_qcompress,
            defaults->rc.f_complexity_blur,
            defaults->rc.f_qblur,
            strtable_lookup( x264_direct_pred_names, defaults->analyse.i_direct_mv_pred ),
            strtable_lookup( x264_motion_est_names, defaults->analyse.i_me_method ),
            defaults->analyse.i_me_range,
            defaults->analyse.i_subpel_refine,
            defaults->analyse.i_trellis,
            defaults->analyse.i_noise_reduction,
            strtable_lookup( overscan_str, defaults->vui.i_overscan ),
            strtable_lookup( vidformat_str, defaults->vui.i_vidformat ),
            strtable_lookup( fullrange_str, defaults->vui.b_fullrange ),
            strtable_lookup( colorprim_str, defaults->vui.i_colorprim ),
            strtable_lookup( transfer_str, defaults->vui.i_transfer ),
            strtable_lookup( colmatrix_str, defaults->vui.i_colmatrix ),
            defaults->vui.i_chroma_loc
           );
}
#if 0
static int parse_enum( const char *arg, const char * const *names, int *dst )
{
    int i;
    for( i = 0; names[i]; i++ )
        if( !strcmp( arg, names[i] ) )
        {
            *dst = i;
            return 0;
        }
    return -1;
}

static int parse_cqm( const char *str, UINT8 *cqm, int length )
{
    int i = 0;
    do {
        int coef;
        if( !sscanf( str, "%d", &coef ) || coef < 1 || coef > 255 )
            return -1;
        cqm[i++] = coef;
    } while( i < length && (str = strchr( str, ',' )) && str++ );
    return (i == length) ? 0 : -1;
}
#endif
typedef enum{
	OPT_QPMIN=256, OPT_QPMAX,
	OPT_QPSTEP,
	OPT_IPRATIO, OPT_PBRATIO, OPT_RATETOL,
	OPT_RCSTATS, OPT_RCEQ, OPT_QCOMP,
	OPT_PSNR, OPT_SSIM,OPT_BRATE,
	OPT_QUIET,
	OPT_SCENECUT,OPT_NO_SCENECUT,
	OPT_QBLUR,
	OPT_CPLXBLUR,
	OPT_FRAMES, OPT_FPS,
	OPT_DIRECT,
	OPT_LEVEL,
	OPT_BADAPT, OPT_NOBADAPT, OPT_BBIAS, OPT_BPYRAMID,
	OPT_CHROMA_QP,
	OPT_NO_CHROMA_ME,
	OPT_NO_CABAC,
	OPT_AUD,
	OPT_PROGRESS,
	OPT_ME,
	OPT_MERANGE,
	OPT_VBVMAXRATE, OPT_VBVBUFSIZE, OPT_VBVINIT,
	OPT_SEEK,
	OPT_ZONES,
	OPT_THREADS,
	OPT_CQM,
	OPT_CQM4, OPT_CQM4I, OPT_CQM4IY, OPT_CQM4IC, OPT_CQM4P, OPT_CQM4PY, OPT_CQM4PC,
	OPT_CQM8, OPT_CQM8I, OPT_CQM8P,
	OPT_CQMFILE,
	OPT_SAR,
	OPT_OVERSCAN,
	OPT_VIDFORMAT,
	OPT_FULLRANGE,
	OPT_COLOURPRIM,
	OPT_TRANSFER,
	OPT_COLOURMATRIX,
	OPT_CHROMALOC,
	OPT_MIXED_REFS,
	OPT_CRF,
	OPT_B_RDO,
	OPT_NO_FAST_PSKIP,
	OPT_BIME,
	OPT_NR,
	OPT_DUMP_YUV,
	OPT_INPUT_RES,
	OPT_AQ_MODE,
	OPT_AQ_STRENGTH,
	OPT_NO_MBTREE,
	OPT_RC_LOOKAHEAD,
};
static char short_options[]="hi:I:b:r:cxB:q:f:o:A:m:p:t:vw8";
static struct option long_options[] =
{
	{ "help",    no_argument,       NULL, 'h' },
	{ "bitrate", required_argument, NULL, 'B' },
	{ "bframes", required_argument, NULL, 'b' },
	{ "b-adapt", required_argument,    NULL, OPT_BADAPT },
	{ "no-b-adapt", no_argument,    NULL, OPT_NOBADAPT },
	{ "b-bias",  required_argument, NULL, OPT_BBIAS },
	{ "b-pyramid", no_argument,     NULL, OPT_BPYRAMID },
	{ "min-keyint",required_argument,NULL,'i' },
	{ "keyint",  required_argument, NULL, 'I' },
	{ "scenecut",required_argument, NULL, OPT_SCENECUT },
	{ "no-scenecut",no_argument, NULL, OPT_NO_SCENECUT },
	{ "nf",      no_argument,       NULL, 'n' },
	{ "filter",  required_argument, NULL, 'f' },
	{ "no-cabac",no_argument,       NULL, OPT_NO_CABAC },
	{ "qp",      required_argument, NULL, 'q' },
	{ "qpmin",   required_argument, NULL, OPT_QPMIN },
	{ "qpmax",   required_argument, NULL, OPT_QPMAX },
	{ "qpstep",  required_argument, NULL, OPT_QPSTEP },
	{ "crf",     required_argument, NULL, OPT_CRF },
	{ "ref",     required_argument, NULL, 'r' },
	{ "no-asm",  no_argument,       NULL, 'C' },
	{ "sar",     required_argument, NULL, OPT_SAR },
	{ "fps",     required_argument, NULL, OPT_FPS },
	{ "frames",  required_argument, NULL, OPT_FRAMES },
	{ "seek",    required_argument, NULL, OPT_SEEK },
	{ "output",  required_argument, NULL, 'o' },
	{ "analyse", required_argument, NULL, 'A' },
	{ "direct",  required_argument, NULL, OPT_DIRECT },
	{ "weightb", no_argument,       NULL, 'w' },
	{ "me",      required_argument, NULL, OPT_ME },
	{ "merange", required_argument, NULL, OPT_MERANGE },
	{ "subme",   required_argument, NULL, 'm' },
	{ "b-rdo",   no_argument,       NULL, OPT_B_RDO },
	{ "mixed-refs", no_argument,    NULL, OPT_MIXED_REFS },
	{ "no-chroma-me", no_argument,  NULL, OPT_NO_CHROMA_ME },
	{ "bime",    no_argument,       NULL, OPT_BIME },
	{ "8x8dct",  no_argument,       NULL, '8' },
	{ "trellis", required_argument, NULL, 't' },
	{ "no-fast-pskip", no_argument, NULL, OPT_NO_FAST_PSKIP },
	{ "level",   required_argument, NULL, OPT_LEVEL },
	{ "ratetol", required_argument, NULL, OPT_RATETOL },
	{ "vbv-maxrate", required_argument, NULL, OPT_VBVMAXRATE },
	{ "vbv-bufsize", required_argument, NULL, OPT_VBVBUFSIZE },
	{ "vbv-init", required_argument,NULL,  OPT_VBVINIT },
	{ "ipratio", required_argument, NULL, OPT_IPRATIO },
	{ "pbratio", required_argument, NULL, OPT_PBRATIO },
	{ "chroma-qp-offset", required_argument, NULL, OPT_CHROMA_QP },
	{ "pass",    required_argument, NULL, 'p' },
	{ "stats",   required_argument, NULL, OPT_RCSTATS },
	{ "rceq",    required_argument, NULL, OPT_RCEQ },
	{ "qcomp",   required_argument, NULL, OPT_QCOMP },
	{ "qblur",   required_argument, NULL, OPT_QBLUR },
	{ "cplxblur",required_argument, NULL, OPT_CPLXBLUR },
	{ "zones",   required_argument, NULL, OPT_ZONES },
	{ "threads", required_argument, NULL, OPT_THREADS },
	{ "psnr", no_argument,       NULL, OPT_PSNR },
	{ "ssim", no_argument,       NULL, OPT_SSIM },
	{ "brate", no_argument,       NULL, OPT_BRATE },
	{ "quiet",   no_argument,       NULL, OPT_QUIET },
	{ "verbose", no_argument,       NULL, 'v' },
	{ "progress",no_argument,       NULL, OPT_PROGRESS },
	{ "aud",     no_argument,       NULL, OPT_AUD },
	{ "nr",      required_argument, NULL, OPT_NR },
#if 0
	{ "cqm",     required_argument, NULL, OPT_CQM },
	{ "cqmfile", required_argument, NULL, OPT_CQMFILE },
	{ "cqm4",    required_argument, NULL, OPT_CQM4 },
	{ "cqm4i",   required_argument, NULL, OPT_CQM4I },
	{ "cqm4iy",  required_argument, NULL, OPT_CQM4IY },
	{ "cqm4ic",  required_argument, NULL, OPT_CQM4IC },
	{ "cqm4p",   required_argument, NULL, OPT_CQM4P },
	{ "cqm4py",  required_argument, NULL, OPT_CQM4PY },
	{ "cqm4pc",  required_argument, NULL, OPT_CQM4PC },
	{ "cqm8",    required_argument, NULL, OPT_CQM8 },
	{ "cqm8i",   required_argument, NULL, OPT_CQM8I },
	{ "cqm8p",   required_argument, NULL, OPT_CQM8P },
	{ "overscan", required_argument, NULL, OPT_OVERSCAN },
	{ "videoformat", required_argument, NULL, OPT_VIDFORMAT },
	{ "fullrange", required_argument, NULL, OPT_FULLRANGE },
	{ "colorprim", required_argument, NULL, OPT_COLOURPRIM },
	{ "transfer", required_argument, NULL, OPT_TRANSFER },
	{ "colormatrix", required_argument, NULL, OPT_COLOURMATRIX },
	{ "chromaloc", required_argument, NULL, OPT_CHROMALOC },
#endif
	{ "dump-yuv", required_argument, NULL, OPT_DUMP_YUV},
	{ "aq-mode", required_argument, NULL, OPT_AQ_MODE},
	{ "aq-strength", required_argument, NULL, OPT_AQ_STRENGTH},
	{ "no-mbtree",no_argument,       NULL, OPT_NO_MBTREE },
	{ "rc-lookahead", required_argument, NULL, OPT_RC_LOOKAHEAD},
	{ "input-res", required_argument, NULL, OPT_INPUT_RES},
	{0, 0, 0, 0}
};
/*****************************************************************************
 * Parse:
 *****************************************************************************/
static int  Parse( int argc, char **argv,
                   x264_param_t *param, cli_opt_t *opt )
{
    char *psz_filename = NULL;
    x264_param_t defaults = *param;
    char *psz;
    char b_avis = 0;
	char *args=NULL;

    memset( opt, 0, sizeof(cli_opt_t) );

    /* Default input file driver */
    p_open_infile = open_file_yuv;
    p_get_frame_total = get_frame_total_yuv;
    p_read_frame = read_frame_yuv;
    p_close_infile = close_file_yuv;

    /* Default output file driver */
    p_open_outfile = open_file_bsf;
    p_set_outfile_param = set_param_bsf;
    p_write_nalu = write_nalu_bsf;
    p_set_eop = set_eop_bsf;
    p_close_outfile = close_file_bsf;

    /* Parse command line options */
    opterr = 0; // no error message
    for( ;; )
    {
        int b_error = 0;
        int long_options_index; 
        int c;

        c = getopt_long( argc, argv,short_options , long_options, &long_options_index);

        if( c == -1 )
        {
            break;
        }

        switch( c )
        {
            case 'h':
                Help( &defaults );
                return -1;

            case 0:
                break;
            case 'B':
                param->rc.i_bitrate = atol( optarg );
                param->rc.b_cbr = 1;
                break;
            case OPT_CRF:
                param->rc.i_rf_constant = atol( optarg );
                break;
            case 'b':
                param->i_bframe = atol( optarg );
                break;
            case OPT_NOBADAPT:
                param->i_bframe_adaptive = 0;
                break;
            case OPT_BADAPT:
                param->i_bframe_adaptive = atoi(optarg);
                break;
            case OPT_BBIAS:
                param->i_bframe_bias = atol( optarg );
                break;
            case OPT_BPYRAMID:
                param->b_bframe_pyramid = 1;
                break;
            case 'i':
                param->i_keyint_min = atol( optarg );
                if( param->i_keyint_max < param->i_keyint_min )
                    param->i_keyint_max = param->i_keyint_min;
                break;
            case 'I':
                param->i_keyint_max = atol( optarg );
                if( param->i_keyint_min > param->i_keyint_max )
                    param->i_keyint_min = param->i_keyint_max;
                break;
            case OPT_SCENECUT:
                param->i_scenecut_threshold = atol( optarg );
                break;
            case OPT_NO_SCENECUT:
                param->i_scenecut_threshold = 0;//atol( optarg );
                break;
            case 'n':
                param->b_deblocking_filter = 0;
                break;
            case 'f':
            {
                char *p = strchr( optarg, ':' );
                if( !p ) p = strchr( optarg, ',' );
                param->i_deblocking_filter_alphac0 = atoi( optarg );
                param->i_deblocking_filter_beta = p ? atoi( p+1 ) : param->i_deblocking_filter_alphac0;
                break;
            }
            case 'q':
                param->rc.i_qp_constant = atoi( optarg );
                break;
            case OPT_QPMIN:
                param->rc.i_qp_min = atoi( optarg );
                break;
            case OPT_QPMAX:
                param->rc.i_qp_max = atoi( optarg );
                break;
            case OPT_QPSTEP:
                param->rc.i_qp_step = atoi( optarg );
                break;
            case 'r':
                param->i_frame_reference = atoi( optarg );
                break;
            case OPT_NO_CABAC:
                param->b_cabac = 0;
                break;
				/*
            case 'x':
                opt->b_decompress = 1;
                break;
				*/
            case 'C':
                param->cpu = 0;
                break;
            case OPT_FRAMES:
                param->i_frame_total = atoi( optarg );
                break;
			case OPT_INPUT_RES:
				sscanf( optarg, "%dx%d", &param->i_width, &param->i_height );
				break;
            case OPT_SEEK:
                opt->i_seek = atoi( optarg );
                break;
            case 'o':
                if( !strcmp(optarg, "-") )
                    opt->hout = stdout;
                else if( p_open_outfile( optarg, &opt->hout ) )
                {
                    fprintf( stderr, "cannot open output file `%s'\n", optarg );
                    return -1;
                }
                break;
			case OPT_DUMP_YUV:
				if(strlen(optarg)>0){
					strncpy(param->psz_dump_yuv,optarg,100);
				}
				else
					strncpy(param->psz_dump_yuv,"rec.yuv",100);

				break;
            case OPT_SAR:
            {
                char *p = strchr( optarg, ':' );
                if( !p ) p = strchr( optarg, '/' );
                if( p )
                {
                    param->vui.i_sar_width = atoi( optarg );
                    param->vui.i_sar_height = atoi( p + 1 );
                }
                break;
            }
            case OPT_FPS:
            {
                float fps;
                if( sscanf( optarg, "%d/%d", &param->i_fps_num, &param->i_fps_den ) == 2 )
                    ;
                else if( sscanf( optarg, "%f", &fps ) )
                {
                    param->i_fps_num = (int)(fps * 1000 + .5);
                    param->i_fps_den = 1000;
                }
                else
                {
                    fprintf( stderr, "bad fps `%s'\n", optarg );
                    return -1;
                }
                break;
            }
            case 'A':
                param->analyse.inter = 0;
                if( strstr( optarg, "none" ) )  param->analyse.inter =  0;
                if( strstr( optarg, "all" ) )   param->analyse.inter = ~0;

                if( strstr( optarg, "i4x4" ) )  param->analyse.inter |= X264_ANALYSE_I4x4;
                if( strstr( optarg, "i8x8" ) )  param->analyse.inter |= X264_ANALYSE_I8x8;
                if( strstr( optarg, "p8x8" ) )  param->analyse.inter |= X264_ANALYSE_PSUB16x16;
                if( strstr( optarg, "p4x4" ) )  param->analyse.inter |= X264_ANALYSE_PSUB8x8;
                if( strstr( optarg, "b8x8" ) )  param->analyse.inter |= X264_ANALYSE_BSUB16x16;
                break;
            case OPT_DIRECT:
               // b_error |= parse_enum( optarg, x264_direct_pred_names, &param->analyse.i_direct_mv_pred );
				param->analyse.i_direct_mv_pred=atoi(optarg);
                break;
            case 'w':
                param->analyse.b_weighted_bipred = 1;
                break;
            case OPT_ME:
               // b_error |= parse_enum( optarg, x264_motion_est_names, &param->analyse.i_me_method );
				param->analyse.i_me_method=atoi(optarg);
                break;
            case OPT_MERANGE:
                param->analyse.i_me_range = atoi(optarg);
                break;
            case 'm':
                param->analyse.i_subpel_refine = atoi(optarg);
                break;
            case OPT_B_RDO:
                param->analyse.b_bframe_rdo = 1;
                break;
            case OPT_MIXED_REFS:
                param->analyse.b_mixed_references = 1;
                break;
            case OPT_NO_CHROMA_ME:
                param->analyse.b_chroma_me = 0;
                break;
            case OPT_BIME:
                param->analyse.b_bidir_me = 1;
                break;
            case '8':
                param->analyse.b_transform_8x8 = 1;
                break;
            case 't':
                param->analyse.i_trellis = atoi(optarg);
                break;
            case OPT_NO_FAST_PSKIP:
                param->analyse.b_fast_pskip = 0;
                break;
            case OPT_LEVEL:
                if( atof(optarg) < 6 )
                    param->i_level_idc = (int)(10*atof(optarg)+.5);
                else
                    param->i_level_idc = atoi(optarg);
                break;
            case OPT_RATETOL:
                param->rc.f_rate_tolerance = !strncmp("inf", optarg, 3) ? 1e9 : atof(optarg);
                break;
            case OPT_VBVMAXRATE:
                param->rc.i_vbv_max_bitrate = atoi( optarg );
                break;
            case OPT_VBVBUFSIZE:
                param->rc.i_vbv_buffer_size = atoi( optarg );
                break;
            case OPT_VBVINIT:
                param->rc.f_vbv_buffer_init = atof(optarg);
                break;
            case OPT_IPRATIO:
                param->rc.f_ip_factor = atof(optarg);
                break;
            case OPT_PBRATIO:
                param->rc.f_pb_factor = atof(optarg);
                break;
			case OPT_AQ_MODE:
				param->rc.i_aq_mode=atoi(optarg);
				break;
			case OPT_AQ_STRENGTH:
				param->rc.f_aq_strength=atof(optarg);
				break;
			case OPT_NO_MBTREE:
				param->rc.b_mb_tree=0;
				break;
			case OPT_RC_LOOKAHEAD:
				param->rc.i_lookahead=atoi(optarg);
				break;
            case OPT_CHROMA_QP:
                param->analyse.i_chroma_qp_offset = atoi(optarg);
                break;
            case 'p':
            {
                int i_pass = atoi(optarg);
                if( i_pass == 1 )
                    param->rc.b_stat_write = 1;
                else if( i_pass == 2 )
                    param->rc.b_stat_read = 1;
                else if( i_pass > 2 )
                    param->rc.b_stat_read =
                    param->rc.b_stat_write = 1;
                break;
            }
            case OPT_RCSTATS:
                param->rc.psz_stat_in = optarg;
                param->rc.psz_stat_out = optarg;
                break;
            case OPT_RCEQ:
                param->rc.psz_rc_eq = optarg;
               break;
            case OPT_QCOMP:
                param->rc.f_qcompress = atof(optarg);
                break;
            case OPT_QBLUR:
                param->rc.f_qblur = atof(optarg);
                break;
            case OPT_CPLXBLUR:
                param->rc.f_complexity_blur = atof(optarg);
                break;
            case OPT_ZONES:
                param->rc.psz_zones = optarg;
                break;
            case OPT_THREADS:
                param->i_threads = atoi(optarg);
                break;
            case OPT_PSNR:
                param->analyse.b_psnr = 1;
                break;
            case OPT_BRATE:
                param->rc.b_brate = 1;
                break;
            case OPT_SSIM:
                param->analyse.b_ssim = 1;
                break;
            case OPT_QUIET:
                param->i_log_level = X264_LOG_NONE;
                break;
            case 'v':
                param->i_log_level = X264_LOG_DEBUG;
                break;
            case OPT_AUD:
                param->b_aud = 1;
                break;
            case OPT_PROGRESS:
                opt->b_progress = 1;
                break;
            case OPT_NR:
                param->analyse.i_noise_reduction = atoi(optarg);
                break;
#if 0
            case OPT_CQM:
                if( strstr( optarg, "flat" ) )
                    param->i_cqm_preset = X264_CQM_FLAT;
                else if( strstr( optarg, "jvt" ) )
                    param->i_cqm_preset = X264_CQM_JVT;
                else
                {
                    fprintf( stderr, "bad CQM preset `%s'\n", optarg );
                    return -1;
                }
                break;

            case OPT_CQM4:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_4iy, 16 );
                b_error |= parse_cqm( optarg, param->cqm_4ic, 16 );
                b_error |= parse_cqm( optarg, param->cqm_4py, 16 );
                b_error |= parse_cqm( optarg, param->cqm_4pc, 16 );
                break;
            case OPT_CQM8:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_8iy, 64 );
                b_error |= parse_cqm( optarg, param->cqm_8py, 64 );
                break;
            case OPT_CQM4I:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_4iy, 16 );
                b_error |= parse_cqm( optarg, param->cqm_4ic, 16 );
                break;
            case OPT_CQM4P:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_4py, 16 );
                b_error |= parse_cqm( optarg, param->cqm_4pc, 16 );
                break;
            case OPT_CQM4IY:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_4iy, 16 );
                break;
            case OPT_CQM4IC:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_4ic, 16 );
                break;
            case OPT_CQM4PY:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_4py, 16 );
                break;
            case OPT_CQM4PC:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_4pc, 16 );
                break;
            case OPT_CQM8I:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_8iy, 64 );
                break;
            case OPT_CQM8P:
                param->i_cqm_preset = X264_CQM_CUSTOM;
                b_error |= parse_cqm( optarg, param->cqm_8py, 64 );
                break;
#endif
#if 0
            case OPT_OVERSCAN:
                //b_error |= parse_enum( optarg, overscan_str, &param->vui.i_overscan );
				param->vui.i_overscan=atoi(optarg);
                break;
            case OPT_VIDFORMAT:
                //b_error |= parse_enum( optarg, vidformat_str, &param->vui.i_vidformat );
				param->vui.i_vidformat=atoi(optarg);
                break;
            case OPT_FULLRANGE:
                //b_error |= parse_enum( optarg, fullrange_str, &param->vui.b_fullrange );
				param->vui.b_fullrange=atoi(optarg);
                break;
            case OPT_COLOURPRIM:
                //b_error |= parse_enum( optarg, colorprim_str, &param->vui.i_colorprim );
				param->vui.i_colorprim=atoi(optarg);
                break;
            case OPT_TRANSFER:
                //b_error |= parse_enum( optarg, transfer_str, &param->vui.i_transfer );
				param->vui.i_transfer=atoi(optarg);
                break;
            case OPT_COLOURMATRIX:
                //b_error |= parse_enum( optarg, colmatrix_str, &param->vui.i_colmatrix );
				param->vui.i_colmatrix=atoi(optarg);
                break;
            case OPT_CHROMALOC:
                param->vui.i_chroma_loc = atoi( optarg );
                b_error = ( param->vui.i_chroma_loc < 0 || param->vui.i_chroma_loc > 5 );
                break;
#endif
            default:
				args= argv[optind-1];
                fprintf( stderr, "unknown option (%d:%s)\n", optopt,args );
               // return -1;
        }

        if( b_error )
        {
            fprintf( stderr, "bad argument: %s %s\n", argv[optind-2], optarg );
            return -1;
        }
    }

    /* Get the file name */
    if( optind > argc - 1 || !opt->hout )
    {
        Help( &defaults );
        return -1;
    }
    psz_filename = argv[optind++];

//    if( !opt->b_decompress )
	if(param->i_width==0||param->i_height==0)
    {
        if( optind > argc - 1 )
        {
            /* try to parse the file name */
            for( psz = psz_filename; *psz; psz++ )
            {
                if( *psz >= '0' && *psz <= '9'
                    && sscanf( psz, "%ux%u", &param->i_width, &param->i_height ) == 2 )
                {
                    if( param->i_log_level >= X264_LOG_INFO )
                        fprintf( stderr, "x264 [info]: file name gives %dx%d\n", param->i_width, param->i_height );
                    break;
                }
            }
        }
        else
        {
            sscanf( argv[optind++], "%ux%u", &param->i_width, &param->i_height );
        }
        
        /* check avis input */
		/*
        psz = psz_filename + strlen(psz_filename) - 1;
        while( psz > psz_filename && *psz != '.' )
            psz--;

        if( !strncasecmp( psz, ".avi", 4 ) || !strncasecmp( psz, ".avs", 4 ) )
            b_avis = 1;

        if( !b_avis && ( !param->i_width || !param->i_height ) )
        {
            Help( &defaults );
            return -1;
        }
		*/
    }


    /* open the input */
    if( !strcmp( psz_filename, "-" ) )
    {
        opt->hin = stdin;
        optind++;
    }
    else
    {
        if( p_open_infile( psz_filename, &opt->hin, param ) )
        {
            fprintf( stderr, "could not open input file '%s'\n", psz_filename );
            return -1;
        }
    }

    return 0;
}

/*****************************************************************************
 * Decode:
 *****************************************************************************/

static int  Encode_frame( x264_t *h, hnd_t hout, x264_picture_t *pic )
{
    x264_picture_t pic_out;
    x264_nal_t *nal;
    int i_nal, i;
    int i_file = 0;

    /* Do not force any parameters */
    if( pic )
    {
        pic->i_type = X264_TYPE_AUTO;
        pic->i_qpplus1 = 0;
    }
    if( x264_encoder_encode( h, &nal, &i_nal, pic, &pic_out ) < 0 )
    {
        fprintf( stderr, "x264_encoder_encode failed\n" );
    }

    for( i = 0; i < i_nal; i++ )
    {
        int i_size;
        int i_data;

        i_data = DATA_MAX;
        if( ( i_size = x264_nal_encode( data, &i_data, 1, &nal[i] ) ) > 0 )
        {
            i_file += p_write_nalu( hout, data, i_size );
        }
        else if( i_size < 0 )
        {
            fprintf( stderr, "need to increase buffer size (size=%d)\n", -i_size );
        }
    }
    if (i_nal)
        p_set_eop( hout, &pic_out );

    return i_file;
}

/*****************************************************************************
 * Encode:
 *****************************************************************************/
static int  Encode( x264_param_t *param, cli_opt_t *opt )
{
    x264_t *h;
    x264_picture_t pic;

    int     i_frame, i_frame_total;
    INT64 i_start, i_end;
    INT64 i_file;
    int     i_frame_size;
    int     i_progress;

    i_frame_total = p_get_frame_total( opt->hin, param->i_width, param->i_height );
    i_frame_total -= opt->i_seek;
    if( ( i_frame_total == 0 || param->i_frame_total < i_frame_total )
        && param->i_frame_total > 0 )
        i_frame_total = param->i_frame_total;
    param->i_frame_total = i_frame_total;

    if( ( h = x264_encoder_open( param ) ) == NULL )
    {
        fprintf( stderr, "x264_encoder_open failed\n" );
        p_close_infile( opt->hin );
        p_close_outfile( opt->hout );
        return -1;
    }

    if( p_set_outfile_param( opt->hout, param ) )
    {
        fprintf( stderr, "can't set outfile param\n" );
        p_close_infile( opt->hin );
        p_close_outfile( opt->hout );
        return -1;
    }

    /* Create a new pic */
    x264_picture_alloc( &pic, X264_CSP_I420, param->i_width, param->i_height );

    i_start = x264_mdate();

    /* Encode frames */
    for( i_frame = 0, i_file = 0, i_progress = 0;
         b_ctrl_c == 0 && (i_frame < i_frame_total || i_frame_total == 0); )
    {
        if( p_read_frame( &pic, opt->hin, i_frame + opt->i_seek, param->i_width, param->i_height ) )
            break;

        pic.i_pts = (INT64)i_frame * param->i_fps_den;

        i_file += Encode_frame( h, opt->hout, &pic );

#if DEBUG_HFZ
		printf("For-loop:i_frame=%d,poc=%d\n",i_frame,h->i_frame);
		fflush(stdout);
#endif
        i_frame++;

        /* update status line (up to 1000 times per input file) */
        if( opt->b_progress && param->i_log_level < X264_LOG_DEBUG && 
            ( i_frame_total ? i_frame * 1000 / i_frame_total > i_progress
                            : i_frame % 10 == 0 ) )
        {
            INT64 i_elapsed = x264_mdate() - i_start;
            double fps = i_elapsed > 0 ? i_frame * 1000000. / i_elapsed : 0;
            if( i_frame_total )
            {
                int eta = i_elapsed * (i_frame_total - i_frame) / ((INT64)i_frame * 1000000);
                i_progress = i_frame * 1000 / i_frame_total;
                fprintf( stderr, "encoded frames: %d/%d (%.1f%%), %.2f fps, eta %d:%02d:%02d  \r",
                         i_frame, i_frame_total, (float)i_progress / 10, fps,
                         eta/3600, (eta/60)%60, eta%60 );
            }
            else
                fprintf( stderr, "encoded frames: %d, %.2f fps   \r", i_frame, fps );
            fflush( stderr ); // needed in windows
        }
    }
    /* Flush delayed B-frames */
    do {
        i_file +=
        i_frame_size = Encode_frame( h, opt->hout, NULL );
#if DEBUG_HFZ
		printf("While-Loop:fenc->i_frame=%d\n",h->i_frame);
		fflush(stdout);

#endif
    } while( i_frame_size );

    i_end = x264_mdate();
    x264_picture_clean( &pic );
    x264_encoder_close( h );
    fprintf( stderr, "\n" );

    if( b_ctrl_c )
        fprintf( stderr, "aborted at input frame %d\n", opt->i_seek + i_frame );

    p_close_infile( opt->hin );
    p_close_outfile( opt->hout );

    if( i_frame > 0 )
    {
        double fps = (double)i_frame * (double)1000000 /
                     (double)( i_end - i_start );

        fprintf( stderr, "encoded %d frames, %.2f fps, %.2f kb/s\n", i_frame, fps,
                 (double) i_file * 8 * param->i_fps_num /
                 ( (double) param->i_fps_den * i_frame * 1000 ) );
    }

    return 0;
}
/* raw 420 yuv file operation */
int open_file_yuv( char *psz_filename, hnd_t *p_handle, x264_param_t *p_param )
{
    if ((*p_handle = fopen(psz_filename, "rb")) == NULL)
        return -1;
    return 0;
}

int get_frame_total_yuv( hnd_t handle, int i_width, int i_height )
{
    FILE *f = (FILE *)handle;
    int i_frame_total = 0;

    if( !fseek( f, 0, SEEK_END ) )
    {
        UINT64 i_size = ftell( f );
        fseek( f, 0, SEEK_SET );
        i_frame_total = (int)(i_size / ( i_width * i_height * 3 / 2 ));
    }

    return i_frame_total;
}

int read_frame_yuv( x264_picture_t *p_pic, hnd_t handle, int i_frame, int i_width, int i_height )
{
    static int prev_frame = -1;
    FILE *f = (FILE *)handle;

    if( i_frame != prev_frame+1 )
        if( fseek( f, (UINT64)i_frame * i_width * i_height * 3 / 2, SEEK_SET ) )
            return -1;

    if( fread( p_pic->img.plane[0], 1, i_width * i_height, f ) <= 0
            || fread( p_pic->img.plane[1], 1, i_width * i_height / 4, f ) <= 0
            || fread( p_pic->img.plane[2], 1, i_width * i_height / 4, f ) <= 0 )
        return -1;

    prev_frame = i_frame;

    return 0;
}

int close_file_yuv(hnd_t handle)
{
    if (handle == NULL)
        return 0;
    return fclose((FILE *)handle);
}
int open_file_bsf( char *psz_filename, hnd_t *p_handle )
{
    if ((*p_handle = fopen(psz_filename, "w+b")) == NULL)
        return -1;

    return 0;
}

int set_param_bsf( hnd_t handle, x264_param_t *p_param )
{
    return 0;
}

int write_nalu_bsf( hnd_t handle, UINT8 *p_nalu, int i_size )
{
    if (fwrite(p_nalu, i_size, 1, (FILE *)handle) > 0)
        return i_size;
    return -1;
}

int set_eop_bsf( hnd_t handle,  x264_picture_t *p_picture )
{
    return 0;
}

int close_file_bsf( hnd_t handle )
{
    if ((handle == NULL) || (handle == stdout))
        return 0;

    return fclose((FILE *)handle);
}


INT64 x264_mdate( void )
{
#if !(defined(_MSC_VER) || defined(__MINGW32__))
    struct timeval tv_date;

    gettimeofday( &tv_date, NULL );
    return( (INT64) tv_date.tv_sec * 1000000 + (INT64) tv_date.tv_usec );
#else
    struct _timeb tb;
    _ftime(&tb);
    return ((INT64)tb.time * (1000) + (INT64)tb.millitm) * (1000);
#endif
}