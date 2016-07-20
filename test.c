/* ----------------------------------------------------------------------------
 * test.c : to test decode functions
 * 
 * author : Guangli Dong
 *
 * history: 2016/07/06 new
 *
 * refer  :
 *
 * ---------------------------------------------------------------------------*/

#include "decode.h"
#include "socket_lib.h"


/* internal function forward declaration -------------------------------------*/
static void decode_stream(raw_t *raw, unsigned char buff[], int len);

int main(int argc, char *argv[])
{
    /* local variables */
    raw_t   *raw;
    socket_t sock;
    unsigned char buff[4096];
    int recvnum, i;


    /* initialise raw */
    raw = (raw_t *)malloc(sizeof(raw_t));
    if (0 == init_raw(raw))
    {
        trace(0, "%s\n\n", "ERROR: memory allocation error!");
        return 0;
    }
    strcpy(raw->opt, "-LE");    /* the file type is set to LITTLE_ENDIAN */
    raw->outtype    = 1;        /* set to output message type id */

    /* initialize socket */
    sock = creat_client_socket("192.168.3.212", 40003);
    if(sock < 0) {
        printf("sock error\n");
        exit(0);
    }

    /* main loop */
    while(1) {
        memset(buff, 0x00, sizeof(buff));
        recvnum = recv(sock, (char*)buff, 4000, 0);
#ifdef WIN32_
        Sleep(1000);
#endif
        decode_stream(raw, buff, recvnum);
    }

    /* clear */
    free_raw(raw);
    close_client_socket(sock);
}

/* internal decode stream function */
static void decode_stream(raw_t *raw, unsigned char buff[], int len)
{
    /* local variable */
    int i, j, status;
    int sys, prn;
    for (i=0; i<len; i++) {
        status = decode_unicore(raw, buff[i]);
        /* get gsof_sat data */
        if (status == 48 ) {
            printf("%20s ==== azi & ele ====> %02d\n", time_str(raw->time, 3),
                raw->gsof.sat.num);
            for(j=0; j<raw->gsof.sat.num; j++) {
                printf("    %c%02d  %8.3f %8.3f\n", 
                    (raw->gsof.sat.data[j].sys == SYS_GPS)? 'G': 'C',
                    raw->gsof.sat.data[j].prn,
                    raw->gsof.sat.data[j].azi,
                    raw->gsof.sat.data[j].ele);
            }
            printf("-----------------------------------------------------\n");
        }
        /* get raw_sna data */
        if (status == 1 ) {
            printf("%20s ==== sna ====> %02d\n", time_str(raw->time, 3),
                raw->obs.n);
            for(j=0; j<raw->obs.n; j++) {
                sys = satsys(raw->obs.data[j].sat, &prn);
                printf("    %c%02d  %8.3f %8.3f\n", 
                    (sys == SYS_GPS)? 'G': 'C',
                    prn,
                    raw->obs.data[j].SNR[0]/4.0,
                    raw->obs.data[j].SNR[1]/4.0);
            }
            printf("-----------------------------------------------------\n");
        }
    }
}
