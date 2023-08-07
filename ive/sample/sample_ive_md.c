/*
  Copyright (c), 2001-2022, Shenshu Tech. Co., Ltd.
 */
#include "sample_common_ive.h"
#include "ot_ivs_md.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "SDL.h"
#include "SDL_ttf.h"
#include <time.h>
#include "sample_comm.h"
#include <stdatomic.h>
#include "ot_common_dis.h"

#define FONT_PATH "./hisi_osd.ttf"

#define OT_SAMPLE_IVE_MD_IMAGE_NUM 2
#define OT_SAMPLE_IVE_MD_MILLIC_SEC 200
#define OT_SAMPLE_IVE_MD_ADD_X_VAL 32768
#define OT_SAMPLE_IVE_MD_ADD_Y_VAL 32768
#define OT_SAMPLE_IVE_MD_THREAD_NAME_LEN 16
#define OT_SAMPLE_IVE_MD_AREA_THR_STEP 8
#define OT_SAMPLE_IVE_MD_VPSS_CHN 2
#define OT_SAMPLE_IVE_MD_NUM_TWO 2
#define OT_SAMPLE_IVE_SAD_THRESHOLD 100
#define OVERLAYEX_MIN_HANDLE 20
#include "rtsp_demo.h"
typedef struct
{
    ot_svp_src_img img[OT_SAMPLE_IVE_MD_IMAGE_NUM];
    ot_svp_dst_mem_info blob;
    ot_md_attr md_attr;
    ot_sample_svp_rect_info region;
} ot_sample_ivs_md_info;

//typedef struct
//{
//    int arr[121];
//    int labelN[301];
//} Total_result;

typedef struct
{
    int arr[241];
    int labelN[301];
} Total_result;

typedef struct
{
    ot_md_chn md_chn;
    ot_vo_layer vo_layer;
    ot_vo_chn vo_chn;
    td_s32 vpss_grp;
} ot_sample_md_vo_vpss_hld;

static td_bool g_stop_signal = TD_FALSE;
static pthread_t g_md_thread;
static ot_sample_ivs_md_info g_md_info;
// static ot_sample_svp_switch g_md_switch = {TD_FALSE, TD_TRUE};
static ot_sample_svp_switch g_md_switch = {TD_TRUE, TD_FALSE};
static sample_vi_cfg g_vi_config;
static ot_sample_src_dst_size g_src_dst;

typedef struct
{
    rtsp_demo_handle g_rtsplive;
    rtsp_session_handle session;
    int channel_num;
} rtsp_handle_struct;

static pthread_t VencPid1;
static pthread_t VencPid2;
static int EXIT_MODE_X = 1;
static int End_Rtsp = 1;
rtsp_handle_struct rtsp_handle[2];
atomic_uint random_int;
// char  platename[]=" 京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危险品黄白黑绿未单双0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
//char  platename[]=" 京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危险品黑蓝未绿白黄单双0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
char  platename[]=" 京沪津渝冀晋蒙辽吉黑苏浙皖闽赣鲁豫鄂湘粤桂琼川贵云藏陕甘青宁新学警港澳挂使领民航危险品黑蓝未绿白黄单双0123456789ABCDEFGHJKLMNPQRSTUVWXYZ";
static td_void sample_ivs_md_uninit(ot_sample_ivs_md_info *md_info_ptr)
{
    td_s32 i;
    td_s32 ret;

    sample_svp_check_exps_return_void(md_info_ptr == TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    for (i = 0; i < OT_SAMPLE_IVE_MD_IMAGE_NUM; i++)
    {
        sample_svp_mmz_free(md_info_ptr->img[i].phys_addr[0], md_info_ptr->img[i].virt_addr[0]);
    }

    sample_svp_mmz_free(md_info_ptr->blob.phys_addr, md_info_ptr->blob.virt_addr);

    ret = ot_ivs_md_exit();
    if (ret != TD_SUCCESS)
    {
        sample_svp_trace_err("ot_ivs_md_exit fail,Error(%#x)\n", ret);
        return;
    }
}

static td_s32 sample_ivs_md_init(ot_sample_ivs_md_info *md_inf_ptr, td_u32 width, td_u32 height)
{
    td_s32 ret = OT_ERR_IVE_NULL_PTR;
    td_s32 i;
    td_u32 size, sad_mode;
    td_u8 wnd_size;

    sample_svp_check_exps_return(md_inf_ptr == TD_NULL, ret, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    for (i = 0; i < OT_SAMPLE_IVE_MD_IMAGE_NUM; i++)
    {
        ret = sample_common_ive_create_image(&md_inf_ptr->img[i], OT_SVP_IMG_TYPE_U8C1, width, height);
        sample_svp_check_exps_goto(ret != TD_SUCCESS, md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                   "Error(%#x),Create img[%d] image failed!\n", ret, i);
    }
    size = sizeof(ot_ive_ccblob);
    ret = sample_common_ive_create_mem_info(&md_inf_ptr->blob, size);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
                               "Error(%#x),Create blob mem info failed!\n", ret);

    /* Set attr info */
    md_inf_ptr->md_attr.alg_mode = OT_MD_ALG_MODE_BG;
    md_inf_ptr->md_attr.sad_mode = OT_IVE_SAD_MODE_MB_4X4;
    md_inf_ptr->md_attr.sad_out_ctrl = OT_IVE_SAD_OUT_CTRL_THRESHOLD;
    md_inf_ptr->md_attr.sad_threshold = OT_SAMPLE_IVE_SAD_THRESHOLD * (1 << 1);
    md_inf_ptr->md_attr.width = width;
    md_inf_ptr->md_attr.height = height;
    md_inf_ptr->md_attr.add_ctrl.x = OT_SAMPLE_IVE_MD_ADD_X_VAL;
    md_inf_ptr->md_attr.add_ctrl.y = OT_SAMPLE_IVE_MD_ADD_Y_VAL;
    md_inf_ptr->md_attr.ccl_ctrl.mode = OT_IVE_CCL_MODE_4C;
    sad_mode = (td_u32)md_inf_ptr->md_attr.sad_mode;
    wnd_size = (1 << (OT_SAMPLE_IVE_MD_NUM_TWO + sad_mode));
    md_inf_ptr->md_attr.ccl_ctrl.init_area_threshold = wnd_size * wnd_size;
    md_inf_ptr->md_attr.ccl_ctrl.step = wnd_size;

    ret = ot_ivs_md_init();
    sample_svp_check_exps_goto(ret != TD_SUCCESS, md_init_fail, SAMPLE_SVP_ERR_LEVEL_ERROR,
                               "Error(%#x),ot_ivs_md_init failed!\n", ret);

md_init_fail:
    if (ret != TD_SUCCESS)
    {
        sample_ivs_md_uninit(md_inf_ptr);
    }
    return ret;
}

static td_void sample_ivs_set_src_dst_size(ot_sample_src_dst_size *src_dst, td_u32 src_width,
                                           td_u32 src_height, td_u32 dst_width, td_u32 dst_height)
{
    src_dst->src.width = src_width;
    src_dst->src.height = src_height;
    src_dst->dst.width = dst_width;
    src_dst->dst.height = dst_height;
}

/* first frame just init reference frame, if not, change the frame idx */
static td_s32 sample_ivs_md_dma_data(td_u32 cur_idx, ot_video_frame_info *frm,
                                     ot_sample_ivs_md_info *md_ptr, td_bool *is_first_frm)
{
    td_s32 ret;
    td_bool is_instant = TD_TRUE;
    if (*is_first_frm != TD_TRUE)
    {
        ret = sample_common_ive_dma_image(frm, &md_ptr->img[cur_idx], is_instant);
        sample_svp_check_exps_return(ret != TD_SUCCESS, ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                     "sample_ive_dma_image fail,Err(%#x)\n", ret);
    }
    else
    {
        ret = sample_common_ive_dma_image(frm, &md_ptr->img[1 - cur_idx], is_instant);
        sample_svp_check_exps_return(ret != TD_SUCCESS, ret, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                     "sample_ive_dma_image fail,Err(%#x)\n", ret);

        *is_first_frm = TD_FALSE;
    }
    return TD_SUCCESS;
}

int yuv2rgb_nv12(unsigned char *pYuvBuf, unsigned char *pRgbBuf, int height, int width)
{
    if (width < 1 || height < 1 || pYuvBuf == NULL || pRgbBuf == NULL)
    {
        return 0;
    }

    const long len = height * width;

    // Y与UV数据地址
    unsigned char *yData = pYuvBuf;
    unsigned char *uvData = yData + len;

    // 	R、G、B数据地址
    unsigned char *rData = pRgbBuf;
    unsigned char *gData = rData + len;
    unsigned char *bData = gData + len;

    int R[4], G[4], B[4];
    int Y[4], U, V;
    int y0_Idx, y1_Idx, uIdx, vIdx;

    for (int i = 0; i < height; i = i + 2)
    {
        for (int j = 0; j < width; j = j + 2)
        {
            y0_Idx = i * width + j;
            y1_Idx = (i + 1) * width + j;

            // Y[0]、Y[1]、Y[2]、Y[3]分别代表 Y00、Y01、Y10、Y11
            Y[0] = yData[y0_Idx];
            Y[1] = yData[y0_Idx + 1];
            Y[2] = yData[y1_Idx];
            Y[3] = yData[y1_Idx + 1];

            uIdx = (i / 2) * width + j;
            vIdx = uIdx + 1;

            U = uvData[uIdx];
            V = uvData[vIdx];

            R[0] = Y[0] + 1.402 * (V - 128);
            G[0] = Y[0] - 0.34414 * (U - 128) + 0.71414 * (V - 128);
            B[0] = Y[0] + 1.772 * (U - 128);

            R[1] = Y[1] + 1.402 * (V - 128);
            G[1] = Y[1] - 0.34414 * (U - 128) + 0.71414 * (V - 128);
            B[1] = Y[1] + 1.772 * (U - 128);

            R[2] = Y[2] + 1.402 * (V - 128);
            G[2] = Y[2] - 0.34414 * (U - 128) + 0.71414 * (V - 128);
            B[2] = Y[2] + 1.772 * (U - 128);

            R[3] = Y[3] + 1.402 * (V - 128);
            G[3] = Y[3] - 0.34414 * (U - 128) + 0.71414 * (V - 128);
            B[3] = Y[3] + 1.772 * (U - 128);

            // 像素值限定在 0-255
            for (int k = 0; k < 4; ++k)
            {
                if (R[k] >= 0 && R[k] <= 255)
                {
                    R[k] = R[k];
                }
                else
                {
                    R[k] = (R[k] < 0) ? 0 : 255;
                }

                if (G[k] >= 0 && G[k] <= 255)
                {
                    G[k] = G[k];
                }
                else
                {
                    G[k] = (G[k] < 0) ? 0 : 255;
                }

                if (B[k] >= 0 && B[k] <= 255)
                {
                    B[k] = B[k];
                }
                else
                {
                    B[k] = (B[k] < 0) ? 0 : 255;
                }
            }

            *(rData + y0_Idx) = R[0];
            *(gData + y0_Idx) = G[0];
            *(bData + y0_Idx) = B[0];

            *(rData + y0_Idx + 1) = R[1];
            *(gData + y0_Idx + 1) = G[1];
            *(bData + y0_Idx + 1) = B[1];

            *(rData + y1_Idx) = R[2];
            *(gData + y1_Idx) = G[2];
            *(bData + y1_Idx) = B[2];

            *(rData + y1_Idx + 1) = R[3];
            *(gData + y1_Idx + 1) = G[3];
            *(bData + y1_Idx + 1) = B[3];
        }
    }
    return 1;
}

int shmid_tx;
void *ptr_tx;
// int shmid_rx;
// void* ptr_rx;
unsigned int size;
Total_result *p;

// p=malloc(sizeof(Total_result));
// memset(p, 0, sizeof(Total_result));
ot_bmp stBitmap;
// stBitmap.data = malloc(2 * 3840 * 60);
ot_sample_svp_rect_info region_tmp;
ot_sample_svp_rect_info region_tmp_old;
int bmp_w, bmp_h;


static int get_stream_from_one_channl(int s_LivevencChn, rtsp_demo_handle g_rtsplive,
                                      rtsp_session_handle session)
{
    static int s_LivevencFd = 0;
    static int s_maxFd = 0;
    td_s32 ret = 0;
    fd_set read_fds;
    int nSize;
    int i;
    ot_venc_stream stVStream;

    ot_venc_chn_status stStat;
    struct timeval TimeoutVal;
    TimeoutVal.tv_sec = 2;
    TimeoutVal.tv_usec = 0;

    s_LivevencFd = ss_mpi_venc_get_fd(s_LivevencChn);
    s_maxFd = s_maxFd > s_LivevencFd ? s_maxFd : s_LivevencFd;
    s_maxFd = s_maxFd + 1;

    pthread_detach(pthread_self());

    FD_ZERO(&read_fds);
    FD_SET(s_LivevencFd, &read_fds);

    ret = select(s_maxFd, &read_fds, NULL, NULL, &TimeoutVal);
    if (ret <= 0)
    {
        printf("%s select failed!\n", __FUNCTION__);
        // sleep(1);
        // continue;
        return -1;
    }

    // Live stream
    if (FD_ISSET(s_LivevencFd, &read_fds))
    {
        ret = ss_mpi_venc_query_status(s_LivevencChn, &stStat);
        if (TD_SUCCESS != ret)
        {
            printf("ss_mpi_venc_query_status chn[%d] failed with %#x!\n", s_LivevencChn, ret);
            // continue;
            return -1;
        }
        stVStream.pack = (ot_venc_pack *)malloc(sizeof(ot_venc_pack) * stStat.cur_packs);
        stVStream.pack_cnt = stStat.cur_packs;
        ret = ss_mpi_venc_get_stream(s_LivevencChn, &stVStream, TD_TRUE);
        if (TD_SUCCESS != ret)
        {
            printf("ss_mpi_venc_get_stream .. failed with %#x!\n", ret);
            // continue;
            return -1;
        }
        unsigned char *pStremData;
        for (i = 0; i < stVStream.pack_cnt; i++)
        {
            pStremData = (unsigned char *)stVStream.pack[i].addr + stVStream.pack[i].offset;
            nSize = stVStream.pack[i].len - stVStream.pack[i].offset;

            if (g_rtsplive)
            {
                rtsp_sever_tx_video(g_rtsplive, session, pStremData, nSize, stVStream.pack[i].pts);
                // usleep(50 *1000);
            }
        }

        ret = ss_mpi_venc_release_stream(s_LivevencChn, &stVStream);
        if (TD_SUCCESS != ret)
        {
            sample_print("ss_mpi_venc_release_stream chn[%d] .. failed with %#x!\n", s_LivevencChn, ret);
            free(stVStream.pack);
            stVStream.pack = NULL;
            // continue;
            return -1;
        }

        free(stVStream.pack);
        stVStream.pack = NULL;
    }

    // printf("=================venc end\n===============");
}

/******************************************************************************
 * funciton : get stream from each channels and save them
 ******************************************************************************/
td_void *VENC_GetVencStreamProc(td_void *p)
{
    td_s32 ret = 0;
    int i;

    printf("=========chn = %d\n", rtsp_handle[0].channel_num);
    // printf("=========chn = %d\n", rtsp_handle[1].channel_num);
    // s_LivevencChn = rtsp_p.channel_num;
    struct timeval TimeoutVal;
    TimeoutVal.tv_sec = 2;
    TimeoutVal.tv_usec = 0;
    while (End_Rtsp)
    {
        // for (i = 0; i < CHN_NUM_MAX; i++)
        // if(random_int == 1) {
        //     usleep(20);
        //     continue;
        // }

        for (i = 0; i < 1; i++)
        {
            ret = get_stream_from_one_channl(rtsp_handle[i].channel_num, rtsp_handle[i].g_rtsplive,
                                             rtsp_handle[i].session);
            // if (ret < 0)
            //     End_Rtsp = 0;
        }
        random_int = 1;
    }
    free(p);
  
    return NULL;
}


int string_to_bmp(char *pu8Str)
{
    SDL_PixelFormat *fmt;
    TTF_Font *font;
    SDL_Surface *text, *temp;
    if (TTF_Init() < 0)
    {
        fprintf(stderr, "Couldn't initialize TTF: %s\n", SDL_GetError());
        SDL_Quit();
    }

    font = TTF_OpenFont(FONT_PATH, 40); // change size
    if (font == NULL)
    {
        fprintf(stderr, "Couldn't load %d pt font from %s: %s\n", 18, "ptsize", SDL_GetError());
    }

    SDL_Color forecol = {0xff, 0x00, 0x00, 0xff};
    text = TTF_RenderUTF8_Solid(font, pu8Str, forecol);

    fmt = (SDL_PixelFormat *)malloc(sizeof(SDL_PixelFormat));
    memset(fmt, 0, sizeof(SDL_PixelFormat));
    fmt->BitsPerPixel = 16;
    fmt->BytesPerPixel = 2;
    //    fmt->colorkey = 0xffffffff;
    //    fmt->alpha = 0xff;

    temp = SDL_ConvertSurface(text, fmt, 0);
    // stBitmap.data = malloc(2 * (temp->w) * (temp->h));
    // if (stBitmap.data == NULL)
    // {
    //     printf("stBitmap.data faided\r\n");
    // }
    // 奇数会导致内容变成斜体
    if(temp->w%2 != 0)
	    bmp_w = temp->w + 1;
    else
	    bmp_w = temp->w;
    if(temp->h % 2 != 0)
	    bmp_h = temp->h+1;
    else
	    bmp_h = temp->h;

   // bmp_w = temp->w;
   // bmp_h = temp->h;
   // memset(stBitmap.data, 0, (2 * (temp->w) * (temp->h)));
   // memcpy(stBitmap.data, temp->pixels, (2 * (temp->w) * (temp->h)));

   // stBitmap.width = temp->w;
   // stBitmap.height = temp->h;
      memset(stBitmap.data, 0, (2 * bmp_w * bmp_h));
      memcpy(stBitmap.data, temp->pixels, (2 * temp->w * temp->h));
      stBitmap.width = bmp_w;
      stBitmap.height = bmp_h;

    //char savename[20] = {0};

    //snprintf(savename, 20, "./osd/now_time.bmp");
    // printf("savename = %s\n",savename);
    //SDL_SaveBMP(temp, savename);
   // printf("fmt addr is %p, text addr is %p, temp addr is %p, font addr is %p\n", fmt, text, temp, font);
    free(fmt);
    SDL_FreeSurface(text);
    SDL_FreeSurface(temp);
    TTF_CloseFont(font);
    TTF_Quit();

    return 0;
}

/* 
 *描述  ：用于osd 字体bmp图像生成
 *参数  ：NULL
 *返回值：无
 *注意  ：需要加载字体ttf才能使用，否则会报段错误
 */
void *bitmap_update(void )
{
    ot_rgn_handle OverlayHandle = 0;
    td_s32 s32Ret;
    int z = 0;
    // time_t now;
    // struct tm *ptm;
    // char timestr[OSD_LENGTH] = {0};
    while(1)
    {
        sleep(1);
        z++;
        if(z == 10)
        {
            z = 0;
            
        ss_mpi_rgn_update_canvas(OVERLAYEX_MIN_HANDLE);
        s32Ret = ss_mpi_rgn_set_bmp(OVERLAYEX_MIN_HANDLE,&stBitmap);//s32Ret 为RGN_HANDLE OverlayHandle
        // memset(stBitmap.data, 0, (2 * (bmp_w) * (bmp_h)));
        }

        if(p->labelN[0]==0 )
        {
            continue;
        }
            
        ss_mpi_rgn_update_canvas(OVERLAYEX_MIN_HANDLE);
        s32Ret = ss_mpi_rgn_set_bmp(OVERLAYEX_MIN_HANDLE,&stBitmap);//s32Ret 为RGN_HANDLE OverlayHandle
        if(s32Ret != TD_SUCCESS)
        {
            printf("HI_MPI_RGN_SetBitMap update failed with %#x!\n", s32Ret);
            return -1;
        }
       
       //memset(stBitmap.data, 0, (2 * (bmp_w) * (bmp_h)));
       memset(stBitmap.data, 0, (2 * 3840 * 56));
       
       
    }
    return 0;
}


void *osd_ttf_task(void)
{
    ot_rgn_handle OverlayHandle = 0;
    int s32Ret;
    time_t now;
    struct tm *ptm;
    char timestr[720] = {0};
    int i,j;
    char b[3];
    stBitmap.data = malloc(2 * 3840 * 56);
    if (stBitmap.data == NULL)
    {
        printf("stBitmap.data faided\r\n");
    }
    while (1)
    {
         usleep(100000);
        // time(&now);
        // ptm = localtime(&now);
        // snprintf(timestr, 100, "时间:%d-%02d-%02d %02d:%02d:%02d", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
        // printf("timstr = %s\n", timestr);
        // if(p->labelN[0]==0)
        // {
        //     ;
        // }
        if(p->labelN[0] == 0)
        {
		continue;
            timestr[0] = ' ';
            // printf("=====no license=======\n");
        }
        else
        {
		//printf("num is %d\n", p->labelN[0]);
            for(i = 0;i < p->labelN[0];i++)
            {
                for(j=1;j<=15;j++)
                {
                    if(p->labelN[(i*15)+j] < 154 && p->labelN[(i*15)+j] != 0)
                    {
                        b[0] = platename[p->labelN[(i*15)+j]];
                        b[1] = platename[p->labelN[(i*15)+j]+1];
                        b[2] = platename[p->labelN[(i*15)+j]+2];
			strcat(timestr,b);
                    }
                    else if(p->labelN[(i*15)+j] > 153)
                    {
			//b[0] = 'c';
			//strcat(timestr,b);
                        b[0] = platename[p->labelN[(i*15)+j]];
			strcat(timestr,b);
		      //sprintf(timestr,"%s%c",timestr,platename[p->labelN[(i*15)+j]]);
                    }
                    //strcat(timestr,b);
                    memset(b,0,3);
                }    
		strcat(timestr,"   ");
            }
        }
//        strcat(timestr," ");
        string_to_bmp(timestr);
        memset(timestr, 0, 720);
    }
    return 0;
}

/*
 *描述  ：用于将视频文件添加时间水印
 *参数  ：无
 *返回值：OverlayHandle
 *注意  ：参数在HI_MPI_RGN_Create并不做检查，只有在HI_MPI_RGN_AttachToChn的时候才会报出相应的错
 */
td_s32 RGN_AddOsdToVenc(void)
{
    td_s32 s32Ret;
    ot_rgn_attr stRgnAttr;
    ot_rgn_chn_attr stChnAttr;
    ot_mpp_chn stChn;
    ot_rgn_handle OverlayHandle;
    int handle_num = 1;
    int i = 0;
    int ret;
    // RGN_CANVAS_INFO_S stCanvasInfo;
    OverlayHandle = 0;
    stChn.mod_id = OT_ID_VPSS; /**模块号**/ // HI_ID_VPSS  HI_ID_VENC
    stChn.dev_id = 0;                       /**设备号**/
    stChn.chn_id = 0;                       /**通道号**/
    /**创建区域**/
    sleep(2); // 等待位图生成
    stRgnAttr.attr.overlay.canvas_num = 2;
    stRgnAttr.type = OT_RGN_OVERLAYEX;                                                /**区域类型:叠加**/
    stRgnAttr.attr.overlay.pixel_format = OT_PIXEL_FORMAT_ARGB_1555; /**像素格式**/ // PIXEL_FORMAT_BGR_565 PIXEL_FORMAT_ARGB_1555
    
    stBitmap.width = 2;
    stBitmap.height = 2;
    if (stBitmap.width % 2 != 0)
    {
        stBitmap.width += 1;
    }

    if (stBitmap.height % 2 != 0)
    {
        stBitmap.height += 1;
    }
    printf("stBitmap.width is %d ,stBitmap.height is %d\n", stBitmap.width, stBitmap.height);
    stRgnAttr.attr.overlay.size.width = 1920;   // 240;        /**区域宽**/
    stRgnAttr.attr.overlay.size.height = 56; // 192;        /**区域高**/
    stRgnAttr.attr.overlay.bg_color = 0xffff;         // 0x00007c00; /**区域背景颜色**/

    for (i = OVERLAYEX_MIN_HANDLE; i < OVERLAYEX_MIN_HANDLE + handle_num; i++) {
        ret = ss_mpi_rgn_create(i, &stRgnAttr);
        if (ret != TD_SUCCESS) {
            sample_print("ss_mpi_rgn_create failed with %#x!\n", ret);
            return TD_FAILURE;
        }
    }
    

    // s32Ret = ss_mpi_rgn_create(OverlayHandle, &stRgnAttr);
    // if (s32Ret != TD_SUCCESS)
    // {
    //     printf("RGN create failed: %#x\n", s32Ret);
	// return -1;
    // }
    /**将区域叠加到通道**/
    /**设置叠加区域的通道显示属性**/
    stChnAttr.is_show = TD_TRUE;
    stChnAttr.type = OT_RGN_OVERLAYEX;
    // stChnAttr.attr.overlay_chn.point.x = 640; // 240;
    // stChnAttr.attr.overlay_chn.point.y = 320; // 192;
    stChnAttr.attr.overlay_chn.point.x = 0; // 240;
    stChnAttr.attr.overlay_chn.point.y = 60; // 192;
    stChnAttr.attr.overlay_chn.bg_alpha = 0;
    stChnAttr.attr.overlay_chn.fg_alpha = 128;
    stChnAttr.attr.overlay_chn.layer = OverlayHandle;

    /**设置QP属性**/
    stChnAttr.attr.overlay_chn.qp_info.is_abs_qp = TD_TRUE;
    stChnAttr.attr.overlay_chn.qp_info.qp_val = 0;
    stChnAttr.attr.overlay_chn.qp_info.enable = TD_TRUE;

    /**定义 OSD 反色相关属性**/
    /**单元反色区域，反色处理的基本单元,[16, 64]，需 16 对齐**/
#if 0
        stChnAttr.attr.overlay_chn.stInvertColor.stInvColArea.height = 16;
        stChnAttr.attr.overlay_chn.stInvertColor.stInvColArea.width  = 16;

        /**亮度阈值,取值范围：[0, 255]**/
        stChnAttr.attr.overlay_chn.stInvertColor.u32LumThresh = 128;//128

        /**OSD 反色触发模式**/
        stChnAttr.attr.overlay_chn.stInvertColor.enChgMod     = LESSTHAN_LUM_THRESH;

        /**OSD 反色开关。overlay不支持反色**/
        stChnAttr.attr.overlay_chn.stInvertColor.bInvColEn    = TD_FALSE;
#endif
    stChnAttr.attr.overlay_chn.dst = OT_RGN_ATTACH_JPEG_MAIN;
    // OverlayHandle = 0;
    for (i = OVERLAYEX_MIN_HANDLE; i < OVERLAYEX_MIN_HANDLE + handle_num; i++) {
        // sample_region_get_overlayex_chn_attr(i, &chn_attr->attr.overlayex_chn);
        ret = sample_region_attach_to_chn(i, &stChn, &stChnAttr);
        if (ret != TD_SUCCESS) {
            sample_print("sample_region_attach_to_chn failed!\n");
            sample_comm_region_detach_frm_chn(i - OVERLAYEX_MIN_HANDLE + 1, OT_RGN_OVERLAYEX, &stChn);
            return ret;
        }
    }

    // s32Ret = ss_mpi_rgn_attach_to_chn(OverlayHandle, &stChn, &stChnAttr);
    // if (s32Ret != TD_SUCCESS)
    // {
    //     printf("HI_MPI_RGN_AttachToChn: %#x\n", s32Ret);
	// return -1;
    // }
    stBitmap.pixel_format = OT_PIXEL_FORMAT_ARGB_1555;
    // stBitmap.height = OVERLAY_H;
    // stBitmap.width = OVERLAY_W;

for (i = OVERLAYEX_MIN_HANDLE; i < OVERLAYEX_MIN_HANDLE + handle_num; i++) {
    s32Ret = ss_mpi_rgn_set_bmp(i, &stBitmap);
}
    if (s32Ret != TD_SUCCESS)
    {
        printf("HI_MPI_RGN_SetBitMap failed with %#x!\n", s32Ret);
	return -1;
    }
    // memset(stBitmap.data, 0, (2 * (622) * (56)));
    // s32Ret = HI_MPI_RGN_GetCanvasInfo(OverlayHandle,&stCanvasInfo);

    // s32Ret = sample_region_detach_from_chn(OverlayHandle, &stChn);//最后用户可以将该区域从通道中撤出（非必须操作），再销毁区域。
    // if(s32Ret != TD_SUCCESS)
    // {
    //      printf("sample_region_detach_from_chn: %#x\n", s32Ret);
    // }
    // s32Ret = ss_mpi_rgn_destroy(OverlayHandle);
    // if(s32Ret != TD_SUCCESS)
    // {
    //     printf("RGN destroy failed: %#x\n", s32Ret);
    // }
    return 0;
}


unsigned char *user_addr;

int sockfd;
struct sockaddr_in serverAddr;
char *buf;
#define HOST_IP "192.168.1.66"
const int PORT = 1777;
int boxcolor[10] = {0};
void *udp_recv_thread()
{

    //	void* ptr_recv;
    // ptr_recv = malloc(256);
    // int ptr_recv[512] = {0};
    //	printf("ptr size if %d\n",sizeof(ptr_recv));
    struct sockaddr_in caddr;
    socklen_t clen = sizeof(caddr);
    char ptr_recv[2480] = {0};
    int recv_num = 0;
    int i, j;
    
    while (1)
    {
        //  socklen_t len = sizeof(serverAddr);
        // memset(ptr_recv,0,256);
        //		printf("recv begin\n");
//	usleep(50 * 1000);
        recv_num = recvfrom(sockfd, ptr_recv, sizeof(ptr_recv), 0, (struct sockaddr *)&caddr, &clen);
        //	recv(sockfd,ptr_recv,256,0);
        	
        // memcpy(p, ptr_recv, 256);
        //		printf("recv num = %d\n", recv_num);
        //		printf("recv = %d, %d, %d\n", ptr_recv[0], ptr_recv[3], ptr_recv[4]);
        //		printf("p[0] = %d\n", p[0]);
        p = (Total_result *)ptr_recv;
        
        // printf("===========================================\n");
        // printf("====arr 0 = %d\n", p->arr[0]);
        // printf("===========================================\n");
        region_tmp_old.num = p->arr[0] > 8 ? 8 : p->arr[0];
        
        if (region_tmp_old.num != 0)
        {
            	//printf("test num is %d\n",region_tmp.num);
            for (i = 0; i < region_tmp_old.num; i++)
            {
                //	printf("p[6*i+6] = %d\n",p[6*i+6]);
                boxcolor[i] = (td_s32)p->arr[6 * i + 1];
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_ZERO].x = (td_s32)p->arr[6 * i + 3] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_ZERO].y = (td_s32)p->arr[6 * i + 4] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_ONE].x = (td_s32)p->arr[6 * i + 3] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_ONE].y = (td_s32)p->arr[6 * i + 6] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_TWO].x = (td_s32)p->arr[6 * i + 5] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_TWO].y = (td_s32)p->arr[6 * i + 6] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_THREE].x = (td_s32)p->arr[6 * i + 5] / 2 * 2;
                region_tmp_old.rect[i].point[OT_SAMPLE_POINT_IDX_THREE].y = (td_s32)p->arr[6 * i + 4] / 2 * 2;
            }
        }
    }
    
}

void start_udp_server()
{
    socklen_t addr_size;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    printf("start connect udp server...\n");
    memset(&serverAddr, '\0', sizeof(serverAddr));

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    // serverAddr.sin_addr.s_addr = inet_addr(HOST_IP);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    pthread_t recv_thread = 0;

    pthread_create(&recv_thread, NULL, udp_recv_thread, NULL);
    pthread_detach(recv_thread);
}

static td_void *sample_ivs_md_proc(td_void *args)
{
    td_s32 ret;
    ot_sample_ivs_md_info *md_ptr = (ot_sample_ivs_md_info *)(args);
    ot_video_frame_info frm[OT_SAMPLE_IVE_MD_VPSS_CHN]; /* 0:base_frm, 1:ext_frm */
    ot_sample_md_vo_vpss_hld hld = {0};
    td_s32 vpss_chn[] = {OT_VPSS_CHN0, OT_VPSS_CHN1};
    td_s32 cur_idx = 0;
    td_bool is_first_frm = TD_TRUE;
    atomic_init(&random_int, 1);
    

    sample_svp_check_exps_return(md_ptr == TD_NULL, TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "md_inf_ptr can't be null\n");

    /* Create chn */
    ret = ot_ivs_md_create_chn(hld.md_chn, &(md_ptr->md_attr));
    sample_svp_check_exps_return(ret != TD_SUCCESS, TD_NULL, SAMPLE_SVP_ERR_LEVEL_ERROR, "ot_ivs_md_create_chn fail\n");
    // udp server

    start_udp_server();

    // udpserver end
    //
    //
    pthread_t osd_task ;
    pthread_create(&osd_task, NULL, osd_ttf_task, NULL);
    pthread_detach(osd_task);
    RGN_AddOsdToVenc();
    pthread_t bitmap_update_t ;
    pthread_create(&bitmap_update_t, NULL, bitmap_update, NULL);
    pthread_detach(bitmap_update_t);    
    int count;
    size = 12441600;
    ptr_tx = malloc(size);
    user_addr = malloc(size);
    // ptr_rx = malloc(512);
    shmid_tx = shmget(100, size, IPC_CREAT | 0664);
    ptr_tx = shmat(shmid_tx, NULL, 0);
    // shmid_rx = shmget(111, 256, IPC_CREAT|0664);
    // ptr_rx = shmat(shmid_rx, NULL, 0);

    struct timeval tv;
    struct timezone tz;
    struct tm *t;

    while (g_stop_signal == TD_FALSE)
    {
        ret = ss_mpi_vpss_get_chn_frame(hld.vpss_grp, vpss_chn[1], &frm[1], OT_SAMPLE_IVE_MD_MILLIC_SEC);
//	printf("==============get frame\n");
//	gettimeofday(&tv, &tz);
//	t = localtime(&tv.tv_sec);
//	printf("%d-%d-%d %d:%d:%d.%ld\n", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);

        sample_svp_check_exps_continue(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                       "Err(%#x),vpss_get_chn_frame failed, vpss_grp(%d), vpss_chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[1]);

        ret = ss_mpi_vpss_get_chn_frame(hld.vpss_grp, vpss_chn[0], &frm[0], OT_SAMPLE_IVE_MD_MILLIC_SEC);
        sample_svp_check_failed_goto(ret, ext_free, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                     "Error(%#x),vpss_get_chn_frame failed, VPSS_GRP(%d), VPSS_CHN(%d)!\n", ret, hld.vpss_grp, vpss_chn[0]);

        ret = sample_ivs_md_dma_data(cur_idx, &frm[1], md_ptr, &is_first_frm);
        sample_svp_check_failed_goto(ret, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "dma data failed, Err(%#x)\n", ret);

        /* change idx */
        if (is_first_frm == TD_TRUE)
        {
            goto change_idx;
        }

        ret = ot_ivs_md_proc(hld.md_chn, &md_ptr->img[cur_idx], &md_ptr->img[1 - cur_idx], TD_NULL, &md_ptr->blob);
        sample_svp_check_failed_goto(ret, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "ivs_md_proc fail,Err(%#x)\n", ret);

        sample_ivs_set_src_dst_size(&g_src_dst, md_ptr->md_attr.width, md_ptr->md_attr.height,
                                    frm[0].video_frame.width, frm[0].video_frame.height);
        ret = sample_common_ive_blob_to_rect(sample_svp_convert_addr_to_ptr(ot_ive_ccblob, md_ptr->blob.virt_addr),
                                             &(md_ptr->region), OT_SVP_RECT_NUM, OT_SAMPLE_IVE_MD_AREA_THR_STEP, g_src_dst);
        sample_svp_check_exps_goto(ret != TD_SUCCESS, base_free, SAMPLE_SVP_ERR_LEVEL_ERROR, "blob to rect failed!\n");
        count++;
#if 1
        // if(strlen(ptr_tx) == 0) {

        if (count % 5 == 0)
        {
            user_addr = (unsigned char *)ss_mpi_sys_mmap(frm[0].video_frame.phys_addr[0], size);
            memcpy(ptr_tx, user_addr, size);
            ss_mpi_sys_munmap(user_addr, size);
        }

        // if(strlen(ptr_rx) != 0) {
        //	if(1) {
        //	    region_tmp = function_draw_region();
        //	}

        //		memcpy(p, ptr_rx, 244);
        //		memset(ptr_rx, 0, 244);
        //		memset(ptr_tx, 0, 12441600);
        //
        //	for (int i = 0; i < 6; i++)
        //	{
        //	      printf("p[%d] is %d,",i, p[i]);
        //	}
        //	printf("\n");

#endif

        /* Draw rect */
        // ret = sample_common_svp_vgs_f
        // if (region_tmp_old.num != region_tmp.num)
        // {
            region_tmp = region_tmp_old;
        // }

        // if(random_int == 1) {
        if(region_tmp.num != 0)
        {
        ret = sample_common_svp_vgs_fill_rect_changecolor(&frm[0], &region_tmp, boxcolor);
        //sample_svp_check_failed_err_level_goto(ret, base_free, "sample_svp_vgs_fill_rect fail,Err(%#x)\n", ret); 
        }
        ret = ss_mpi_venc_send_frame(hld.vo_chn, &frm[0], OT_SAMPLE_IVE_MD_MILLIC_SEC);
//	printf("=================send frame\n");
//	gettimeofday(&tv, &tz);
//    t = localtime(&tv.tv_sec);
//    printf("%d-%d-%d %d:%d:%d.%ld\n", 1900+t->tm_year, 1+t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tv.tv_usec);

        sample_svp_check_failed_err_level_goto(ret, base_free, "ss_mpi_venc_send_frame fail,Error(%#x)\n", ret);
        //random_int = 0;
        // }
        // usleep(500);
        // free(stBitmap.data);
    change_idx:
        /* Change reference and current frame index */
        cur_idx = 1 - cur_idx;
    base_free:
        ret = ss_mpi_vpss_release_chn_frame(hld.vpss_grp, vpss_chn[0], &frm[0]);
        sample_svp_check_exps_trace(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                    "Err(%#x),release_frame failed,grp(%d) chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[0]);

    ext_free:
        ret = ss_mpi_vpss_release_chn_frame(hld.vpss_grp, vpss_chn[1], &frm[1]);
        sample_svp_check_exps_trace(ret != TD_SUCCESS, SAMPLE_SVP_ERR_LEVEL_ERROR,
                                    "Err(%#x),release_frame failed,grp(%d) chn(%d)!\n", ret, hld.vpss_grp, vpss_chn[1]);
    }

    /* destroy */
    ret = ot_ivs_md_destroy_chn(hld.md_chn);
    sample_svp_check_failed_trace(ret, SAMPLE_SVP_ERR_LEVEL_ERROR, "ot_ivs_md_destroy_chn fail,Err(%#x)\n", ret);
    free(p);
    // free(stBitmap.data);
    return TD_NULL;
}

static td_s32 sample_ive_md_pause(td_void)
{
    printf("---------------press Enter key to exit!---------------\n");
    if (g_stop_signal == TD_TRUE)
    {
        if (g_md_thread != 0)
        {
            pthread_join(g_md_thread, TD_NULL);
            g_md_thread = 0;
        }
        sample_ivs_md_uninit(&(g_md_info));
        (td_void) memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));

        sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
        return TD_TRUE;
    }

    (void)getchar();

    if (g_stop_signal == TD_TRUE)
    {
        if (g_md_thread != 0)
        {
            pthread_join(g_md_thread, TD_NULL);
            g_md_thread = 0;
        }
        sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
        printf("\033[0;31mprogram termination abnormally!\033[0;39m\n");
        return TD_TRUE;
    }
    return TD_FALSE;
}

td_void sample_ive_md(td_void)
{
    ot_size pic_size;
    ot_pic_size pic_type = PIC_1080P;
    td_s32 ret;
    p= malloc(sizeof(Total_result));
    (td_void) memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));
    /*
     * step 1: start vi vpss venc vo
     */
    ret = sample_common_svp_start_vi_vpss_venc_vo(&g_vi_config, &g_md_switch, &pic_type);
    ot_dis_cfg dis_cfg;
    ss_mpi_vi_get_chn_dis_cfg(0, 0, &dis_cfg);
    dis_cfg.motion_level = 1;
    dis_cfg.mode = 1;

    ss_mpi_vi_set_chn_dis_cfg(0, 0, &dis_cfg);
    ot_dis_attr dis_attr;
    ss_mpi_vi_get_chn_dis_attr(0, 0, &dis_attr);
    dis_attr.enable = TD_TRUE;
    dis_attr.gdc_bypass = TD_FALSE;
    ss_mpi_vi_set_chn_dis_attr(0, 0, &dis_attr);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
                               "Error(%#x),sample_common_svp_start_vi_vpss_venc_vo failed!\n", ret);

    pic_type = PIC_1080P;
    ret = sample_comm_sys_get_pic_size(pic_type, &pic_size);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
                               "Error(%#x),sample_comm_sys_get_pic_size failed!\n", ret);
    /*
     * step 2: Init Md
     */
    ret = sample_ivs_md_init(&g_md_info, pic_size.width, pic_size.height);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR,
                               " Error(%#x),sample_ivs_md_init failed!\n", ret);
    g_stop_signal = TD_FALSE;
    /*
     * step 3: Create work thread
     */
    ret = prctl(PR_SET_NAME, "ive_md_proc", 0, 0, 0);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR, "set thread name failed!\n");
    ret = pthread_create(&g_md_thread, 0, sample_ivs_md_proc, (td_void *)&g_md_info);
    sample_svp_check_exps_goto(ret != TD_SUCCESS, end_md_0, SAMPLE_SVP_ERR_LEVEL_ERROR, "pthread_create failed!\n");
    // system("cd /root/YOLOV3_rtsp_opencv_ffmpeg/out && ./main");

    //    ret = sample_ive_md_pause();
    rtsp_handle[0].g_rtsplive = create_rtsp_demo(554);
    rtsp_handle[0].session = create_rtsp_session(rtsp_handle[0].g_rtsplive, "/live.264", 0);
    rtsp_handle[0].channel_num = 0;
    // usleep(333 *1000);
    pthread_create(&VencPid1, 0, VENC_GetVencStreamProc, NULL);
    while (1)
    {
    	usleep(500 *1000);
    }
    sample_svp_check_exps_return_void(ret == TD_TRUE, SAMPLE_SVP_ERR_LEVEL_ERROR, "md exist!\n");
    g_stop_signal = TD_TRUE;
    pthread_join(g_md_thread, TD_NULL);
    g_md_thread = 0;

    sample_ivs_md_uninit(&(g_md_info));
    (td_void) memset_s(&g_md_info, sizeof(g_md_info), 0, sizeof(g_md_info));

end_md_0:
    g_md_thread = 0;
    g_stop_signal = TD_TRUE;
    sample_common_svp_stop_vi_vpss_venc_vo(&g_vi_config, &g_md_switch);
    return;
}

/*
 * function : Md sample signal handle
 */
td_void sample_ive_md_handle_sig(td_void)
{
    g_stop_signal = TD_TRUE;
}
