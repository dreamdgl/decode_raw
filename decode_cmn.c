/*-----------------------------------------------------------------------------
* decode_cmn.c : contains decode common functions
* 
* author       : Guangli Dong
*
* history      : 2016/04/14 created
*
* ----------------------------------------------------------------------------*/

#include "decode.h"

/* constants -----------------------------------------------------------------*/
#define POLYCRC32   0xEDB88320u /* CRC32 polynomial */

const static double gpst0[]={1980,1, 6,0,0,0}; /* gps time reference */
const static double gst0 []={1999,8,22,0,0,0}; /* galileo system time reference */
const static double bdt0 []={2006,1, 1,0,0,0}; /* beidou time reference */

/* debug trace functions -----------------------------------------------------*/
#ifdef TRACE
extern void trace(int level, const char *format, ...)
{
    va_list ap;

    /* print error message to stderr */
    if (level<=1) {
        va_start(ap,format); vfprintf(stderr,format,ap); va_end(ap);
    }
}
#else
extern void trace   (int level, const char *format, ...){}
#endif /* TRACE */

/* initialize receiver raw data control ----------------------------------------
* initialize receiver raw data control struct and reallocate obsevation and
* epheris buffer
* args   : raw_t  *raw   IO     receiver raw data control struct
* return : status (1:ok,0:memory allocation error)
*-----------------------------------------------------------------------------*/
extern int init_raw(raw_t *raw)
{
    /* Generate values for initializing */
    gtime_t time0={0};
    obsd_t data0={{0}};
    eph_t  eph0 ={0,-1,-1};
    geph_t geph0 = {0,-1};
    alm_t  alm0 ={0,-1};
    gsof_pos_t pos0 = {0};
    gsof_vel_t vel0 = {0};
    gsof_att_t att0 = {0};
    gsof_satd_t satd0 = {0};


    /* Loop values */
    int i,j,sys;

    trace(3,"init_raw:\n");

    /* Init time */
    raw->time   = raw->tobs = time0;
    raw->week=0;
    raw->seconds=0;

    /* Init ephemeris update flag */
    raw->ephsat = 0;

    /* Init antenna indicator for multi-antennas receiver */
    raw->antno  = 0;

    /* Init raw data control option */
    raw->opt[0]='\0';

    /* Init message tpye value and control option */
    raw->msgtype[0]='\0';
    raw->tbase=raw->outtype=0;

    /* Init message buffer */
    raw->nbyte=raw->len=0;
    memset(raw->buff, 0x00, MAXRAWLEN);

    /* Init packet buffer for RT17 */    
    raw->receive_time=0.0;
    raw->plen=raw->pbyte=raw->page=raw->reply=0;
    memset(raw->pbuff, 0x00, 255+4+2);

    /* Clear data pointers */
    raw->obs.data =NULL;
    raw->obuf.data = NULL;
    raw->nav.eph =NULL;
    raw->nav.geph = NULL;
    raw->nav.alm =NULL;

    /* allocate data to obs and nav pointers */
    if (!(raw->obs.data = (obsd_t   *)malloc(sizeof(obsd_t)*MAXOBS))||
        !(raw->obuf.data= (obsd_t   *)malloc(sizeof(obsd_t)*MAXOBS))||
        !(raw->nav.eph  = (eph_t    *)malloc(sizeof(eph_t )*MAXSAT))||
        !(raw->nav.geph = (geph_t   *)malloc(sizeof(geph_t)*NSATGLO))||
        !(raw->nav.alm  = (alm_t    *)malloc(sizeof(alm_t )*MAXSAT))) {
            free_raw(raw);
            return 0;
    }

    /* Reset data numbers */
    raw->obs.n  = 0;
    raw->obuf.n = 0;
    raw->nav.n=raw->nav.na = MAXSAT;
    raw->nav.ng = NSATGLO;
//    raw->nav.nb=raw->nav.nba = MAXSAT;

    /* Init data */
    for (i=0; i<MAXOBS; i++) raw->obs.data [i] = data0;
    for (i=0; i<MAXOBS; i++) raw->obuf.data[i] = data0;
    for (i=0; i<MAXSAT;i++) {
        raw->nav.eph[i] = eph0;
        raw->nav.alm[i] = alm0;
    }
    for (i=0; i<NSATGLO; i++) {
        raw->nav.geph[i] = geph0;
    }

    /* Init gsof data */
    raw->gsof.pos = pos0;
    raw->gsof.vel = vel0;
    raw->gsof.att = att0;
    raw->gsof.sat.num = 0;
    for (i=0; i<MAXOBS; i++) raw->gsof.sat.data[i] = satd0;

    return 1;
}
/* free receiver raw data control ----------------------------------------------
* free observation and ephemeris buffer in receiver raw data control struct
* args   : raw_t  *raw   IO     receiver raw data control struct
* return : none
*-----------------------------------------------------------------------------*/
extern void free_raw(raw_t *raw)
{
    trace(3,"free_raw:\n");

    free(raw->obs.data  );  raw->obs.data   = NULL; raw->obs.n  = 0;
    free(raw->obuf.data );  raw->obuf.data  = NULL; raw->obuf.n = 0;
    free(raw->nav.eph   );  raw->nav.eph    = NULL; raw->nav.n  = 0;
    free(raw->nav.geph  );  raw->nav.geph   = NULL; raw->nav.ng = 0;
    free(raw->nav.alm   );  raw->nav.alm    = NULL; raw->nav.na = 0;
}

/* satellite number to satellite system + prn----------------------------------
* convert satellite number to satellite system + prn
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
extern int satsys(int sat, int *prn)
{
    int sys=SYS_NONE;

    if (sat<=0||MAXSAT<sat) sat=0;
    else if (sat<=NSATGPS) {
        sys=SYS_GPS; sat+=MINPRNGPS-1;
    }
    else if ((sat-=NSATGPS)<=NSATGLO) {
        sys=SYS_GLO; sat+=MINPRNGLO-1;
    }
    else if ((sat-=NSATGLO)<=NSATGAL) {
        sys=SYS_GAL; sat+=MINPRNGAL-1;
    }
    else if ((sat-=NSATGAL)<=NSATQZS) {
        sys=SYS_QZS; sat+=MINPRNQZS-1; 
    }
    else if ((sat-=NSATQZS)<=NSATBDS) {
        sys=SYS_BDS; sat+=MINPRNBDS-1; 
    }
    else if ((sat-=NSATBDS)<=NSATLEO) {
        sys=SYS_LEO; sat+=MINPRNLEO-1; 
    }
    else if ((sat-=NSATLEO)<=NSATSBS) {
        sys=SYS_SBS; sat+=MINPRNSBS-1; 
    }
    else sat=0;
    if (prn) *prn=sat;
    return sys;
}

/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
extern int satno(int sys, int prn)
{
    if (prn<=0) return 0;
    switch (sys) {
    case SYS_GPS:
        if (prn<MINPRNGPS||MAXPRNGPS<prn) return 0;
        return prn-MINPRNGPS+1;
    case SYS_GLO:
        if (prn<MINPRNGLO||MAXPRNGLO<prn) return 0;
        return NSATGPS+prn-MINPRNGLO+1;
    case SYS_GAL:
        if (prn<MINPRNGAL||MAXPRNGAL<prn) return 0;
        return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
    case SYS_QZS:
        if (prn<MINPRNQZS||MAXPRNQZS<prn) return 0;
        return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
    case SYS_BDS:
        if (prn<MINPRNBDS||MAXPRNBDS<prn) return 0;
        return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNBDS+1;
    case SYS_LEO:
        if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
        return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATBDS+prn-MINPRNLEO+1;
    case SYS_SBS:
        if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
        return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATBDS+NSATLEO+prn-MINPRNSBS+1;
    }
    return 0;
}

/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec)
{
    gtime_t t=epoch2time(gpst0);

    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

/* crc-32 parity ---------------------------------------------------------------
* compute crc-32 parity for novatel raw
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-32 parity
* notes  : see NovAtel OEMV firmware manual 1.7 32-bit CRC
*-----------------------------------------------------------------------------*/
extern unsigned int crc32(const unsigned char *buff, int len)
{
    unsigned int crc=0;
    int i,j;

    trace(4,"crc32: len=%d\n",len);

    for (i=0;i<len;i++) {
        crc ^= buff[i];
        for (j=0; j<8; j++) {
            if (crc&1) crc = (crc>>1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern gtime_t epoch2time(const double *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    gtime_t time={0};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];

    if (year<1970||2099<year||mon<1||12<mon) return time;

    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(gpst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));

    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}

/* get time string -------------------------------------------------------------
* get time string
* args   : gtime_t t        I   gtime_t struct
*          int    n         I   number of decimals
* return : time string
* notes  : not reentrant, do not use multiple in a function
*-----------------------------------------------------------------------------*/
extern char *time_str(gtime_t t, int n)
{
    static char buff[64];
    time2str(t,buff,n);
    return buff;
}

/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
extern void time2str(gtime_t t, char *s, int n)
{
    double ep[6];

    if (n<0) n=0; else if (n>12) n=12;
    if (1.0-t.sec<0.5/pow(10.0,n)) {t.time++; t.sec=0.0;};
    time2epoch(t,ep);
    sprintf(s,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f",ep[0],ep[1],ep[2],
        ep[3],ep[4],n<=0?2:n+3,n<=0?0:n,ep[5]);
}

/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern void time2epoch(gtime_t t, double *ep)
{
    const int mday[]={ /* # of days in a month */
        31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
        31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
    };
    int days,sec,mon,day;

    /* leap year if year%4==0 in 1901-2099 */
    days=(int)(t.time/86400);
    sec=(int)(t.time-(time_t)days*86400);
    for (day=days%1461,mon=0;mon<48;mon++) {
        if (day>=mday[mon]) day-=mday[mon]; else break;
    }
    ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
    ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}

/* beidou time (bdt) to time ---------------------------------------------------
* convert week and tow in beidou time (bdt) to gtime_t struct
* args   : int    week      I   week number in bdt
*          double sec       I   time of week in bdt (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t bdt2time(int week, double sec)
{
    gtime_t t=epoch2time(bdt0);

    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

/* time to beidouo time (bdt) --------------------------------------------------
* convert gtime_t struct to week and tow in beidou time (bdt)
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in bdt (NULL: no output)
* return : time of week in bdt (s)
*-----------------------------------------------------------------------------*/
extern double time2bdt(gtime_t t, int *week)
{
    gtime_t t0=epoch2time(bdt0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));

    if (week) *week=w;
    return (double)(sec-w*86400*7)+t.sec;
}
