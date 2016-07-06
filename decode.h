/*------------------------------------------------------------------------------
* decode.h: contains decode functions
*
* author  : Guangli Dong
*
* history : 2016/04/14  created
*           2016/04/27  added gsof data type
*           2016/06/13  add sigma values for gsof_att_t data type
*
*-----------------------------------------------------------------------------*/

#ifndef DECODE_H
#define DECODE_H

#ifdef __cplusplus
extern "C" {
#endif


#define ENABDS
#define TRACE

/* option for convert rangeh data to range -----------------------------------
#define RANGEH_RANGE */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#ifdef WIN32
#include <winsock2.h>
#include <windows.h>
#else
#include <pthread.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif

/* constants -----------------------------------------------------------------*/

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_BDS     0x20                /* navigation system: BeiDou */
#define SYS_ALL     0xFF                /* navigation system: all */

#define TSYS_GPS    0                   /* time system: GPS time */
#define TSYS_UTC    1                   /* time system: UTC */
#define TSYS_GLO    2                   /* time system: GLONASS time */
#define TSYS_BDS    5                   /* time system: BeiDou time */

#ifndef NFREQ
#define NFREQ       3                   /* number of carrier frequencies */
#endif

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#ifdef ENAGLO
#define MINPRNGLO   38                  /* min satellite slot number of GLONASS */
#define MAXPRNGLO   62                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif

#ifdef ENABDS
#define MINPRNBDS   161                 /* min satellite sat number of BeiDou */
#define MAXPRNBDS   197                 /* max satellite sat number of BeiDou */
#define NSATBDS     (MAXPRNBDS-MINPRNBDS+1) /* number of BeiDou satellites */
#define NSYSBDS     1
#else
#define MINPRNBDS   0
#define MAXPRNBDS   0
#define NSATBDS     0
#define NSYSBDS     0
#endif

#define NSYS        (NSYSGPS+NSYSGLO+NSYSBDS) /* number of systems */

#define max(a,b)    (a>b?a:b)

#define MAXSAT      max( max(MAXPRNGPS,MAXPRNBDS), MAXPRNGLO)

#define TOTALSAT    (NSATGPS+NSATGLO+NSATBDS) /* total satellite number */

#define MAXRAWLEN   4096                /* max raw message buffer length */

#ifndef MAXOBS
#define MAXOBS      64                  /* max number of obs in an epoch */
#endif

#define CODE_NONE   0                   /* obs code: none or unknown */
#define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P    2                   /* obs code: L1P,G1P    (GPS,GLO) */
#define CODE_L1W    3                   /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y    4                   /* obs code: L1Y        (GPS) */
#define CODE_L1M    5                   /* obs code: L1M        (GPS) */
#define CODE_L1N    6                   /* obs code: L1codeless (GPS) */
#define CODE_L1S    7                   /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L    8                   /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E    9                   /* obs code: L1-SAIF    (QZS) */
#define CODE_L1A    10                  /* obs code: E1A        (GAL) */
#define CODE_L1B    11                  /* obs code: E1B        (GAL) */
#define CODE_L1X    12                  /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
#define CODE_L1Z    13                  /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
#define CODE_L2C    14                  /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D    15                  /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S    16                  /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L    17                  /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X    18                  /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
#define CODE_L2P    19                  /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W    20                  /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y    21                  /* obs code: L2Y        (GPS) */
#define CODE_L2M    22                  /* obs code: L2M        (GPS) */
#define CODE_L2N    23                  /* obs code: L2codeless (GPS) */
#define CODE_L5I    24                  /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
#define CODE_L5Q    25                  /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
#define CODE_L5X    26                  /* obs code: L5/E5aI+Q  (GPS,GAL,QZS,SBS) */
#define CODE_L7I    27                  /* obs code: E5bI,B2I   (GAL,CMP) */
#define CODE_L7Q    28                  /* obs code: E5bQ,B2Q   (GAL,CMP) */
#define CODE_L7X    29                  /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
#define CODE_L6A    30                  /* obs code: E6A        (GAL) */
#define CODE_L6B    31                  /* obs code: E6B        (GAL) */
#define CODE_L6C    32                  /* obs code: E6C        (GAL) */
#define CODE_L6X    33                  /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
#define CODE_L6Z    34                  /* obs code: E6A+B+C    (GAL) */
#define CODE_L6S    35                  /* obs code: LEXS       (QZS) */
#define CODE_L6L    36                  /* obs code: LEXL       (QZS) */
#define CODE_L8I    37                  /* obs code: E5(a+b)I   (GAL) */
#define CODE_L8Q    38                  /* obs code: E5(a+b)Q   (GAL) */
#define CODE_L8X    39                  /* obs code: E5(a+b)I+Q (GAL) */
#define CODE_L2I    40                  /* obs code: B1I        (CMP) */
#define CODE_L2Q    41                  /* obs code: B1Q        (CMP) */
#define CODE_L6I    42                  /* obs code: B3I        (CMP) */
#define CODE_L6Q    43                  /* obs code: B3Q        (CMP) */
#define CODE_L3I    44                  /* obs code: G3I        (GLO) */
#define CODE_L3Q    45                  /* obs code: G3Q        (GLO) */
#define CODE_L3X    46                  /* obs code: G3I+Q      (GLO) */
#define CODE_L1I    47                  /* obs code: B1I        (BDS) */
#define CODE_L1Q    48                  /* obs code: B1Q        (BDS) */
#define MAXCODE     48                  /* max number of obs code */

/* type definitions ----------------------------------------------------------*/

typedef struct {        /* time struct */
    time_t time;        /* time (s) expressed by standard time_t */
    double sec;         /* fraction of second under 1 s */
} gtime_t;

typedef struct {        /* observation data record */
    gtime_t time;       /* receiver sampling time (GPST) */
    unsigned char sys;  /* satellite system */
    unsigned char sat,rcv; /* satellite/receiver number */
    unsigned char SNR [NFREQ]; /* signal strength (0.25 dBHz) */
    unsigned char LLI [NFREQ]; /* loss of lock indicator */
    unsigned char code[NFREQ]; /* code indicator (CODE_???) */
    double L[NFREQ]; /* observation data carrier-phase (cycle) */
    double P[NFREQ]; /* observation data pseudorange (m) */
    float  D[NFREQ]; /* observation data doppler frequency (Hz) */
} obsd_t;

typedef struct {        /* observation data */
    int n,nmax;         /* number of obervation data/allocated */
    obsd_t *data;       /* observation data records */
} obs_t;

typedef struct {        /* almanac type */
    int sat;            /* satellite number */
    int svh;            /* sv health (0:ok) */
    int svconf;         /* as and sv config */
    int week;           /* GPS/QZS: gps week, GAL: galileo week */
    gtime_t toa;        /* Toa */
    /* SV orbit parameters */
    double A,e,i0,OMG0,omg,M0,OMGd;
    double toas;        /* Toa (s) in week */
    double f0,f1;       /* SV clock parameters (af0,af1) */
} alm_t;

typedef struct {        /* GPS/QZS/GAL broadcast ephemeris type */
    int prn;            /* satellite prn number */
    int iode,iodc,aode,aodc; /* IODE,IODC,AODE */
    int sva;            /* SV accuracy (URA index) */
    double ura;         /* SV accuracy (m^2) */
    int svh;            /* SV health (0:ok) */
    int week;           /* GPS/QZS: gps week, GAL: galileo week */
    int code;           /* GPS/QZS: code on L2, GAL/CMP: data sources */
    int flag;           /* GPS/QZS: L2 P data flag, CMP: nav type */
    gtime_t toe,toc,ttr; /* Toe,Toc,T_trans */
    /* SV orbit parameters */
    double A,e,i0,OMG0,omg,M0,deln,n,OMGd,idot;
    double crc,crs,cuc,cus,cic,cis;
    double toes;        /* Toe (s) in week */
    double tocs;        /* Toc (s) in week */
    double tow;         /* time (s) in week */
    double fit;         /* fit interval (h) */
    double f0,f1,f2;    /* SV clock parameters (af0,af1,af2) */
    double tgd[4];      /* group delay parameters */
} eph_t;

typedef struct {        /* navigation data type */
    int ng,ngmax;         /* number of gps broadcast ephemeris */
    int nb,nbmax;         /* number of bds broadcast ephemeris */
    int nga,ngamax;       /* number of gps almanac data */
    int nba,nbamax;       /* number of bds almanac data */
    eph_t *geph;         /* GPS ephemeris */
    eph_t *beph;        /* BDS ephemeris */
    alm_t *galm;         /* gps almanac data */
    alm_t *balm;         /* bds almanac data */
    double utc_gps[4];  /* GPS delta-UTC parameters {A0,A1,T,W} */
    double utc_bds[4];  /* BeiDou UTC parameters */
    double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_bds[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    int leaps;          /* leap seconds (s) */
    double cbias[TOTALSAT][3];   /* code bias (0:p1-p2,1:p1-c1,2:p2-c2) (m) */
} nav_t;

typedef struct {        /* gsof position data type in WGS84 */
    double  lat;        /* latitude (deg) */
    double  lon;        /* longitude (deg) */
    double  hgt;        /* height above sea level (m) */
    double  undulation; /* geiod undulation = geiod sea level - ellipsoid surface (m) */
} gsof_pos_t;

typedef struct {        /* gsof velocity data type */
    double  hspd;       /* horizontal speed relative to ground (m/s) */
    double  vspd;       /* vertical speed , positive for up derection (m/s) */
    double  heading;    /* moving direction relative to True North (deg) */
} gsof_vel_t;

typedef struct {        /* gsof attitude data type */
    double  length;     /* length of baseline between base and rover antenna */
    double  heading;    /* direction of the ship heading(0-360 deg), ie. from
                           the tail to the front of the ship(yaw) */
    float   heading_sig;/* heading stdev */
    double  pitch;      /* up and down angle (+/-90 deg) */
    float   pitch_sig;  /* pitch stdev */
    double  roll;       /* tuen left and turn right angle (+/-180 deg) */
} gsof_att_t;

typedef struct {        /* gsof data type */
    gsof_pos_t  pos;    /* position */
    gsof_vel_t  vel;    /* velocity */
    gsof_att_t  att;    /* attitude */
} gsof_t;

typedef struct {        /* receiver raw data control type */
    gtime_t time;       /* message time */
    obs_t obs;          /* observation data */
    nav_t nav;          /* satellite ephemerides */
    unsigned char ephsys; /* satellite system of update ephemeris */
    int ephprn;         /* sat prn number of update ephemeris (0:no satellite) */
    gsof_t  gsof;       /* gsof data */
    char msgtype[256];  /* last message type */
    int nbyte;          /* number of bytes in message buffer */ 
    int len;            /* message length (bytes) */
    int tbase;          /* time base (0:gpst,1:utc(usno),2:glonass,3:utc(su),4:bdst */
    int outtype;        /* output message type flag */
    unsigned char buff[MAXRAWLEN]; /* message buffer */
    char opt[256];      /* receiver dependent options */
    double receive_time;/* RT17: Reiceve time of week for week rollover detection */
    unsigned int plen;  /* RT17: Total size of packet to be read */
    unsigned int pbyte; /* RT17: How many packet bytes have been read so far */
    unsigned int page;  /* RT17: Last page number */
    unsigned int reply; /* RT17: Current reply number */
    int week;           /* RT17 & unicoreHeader: week number */
    double seconds;     /* unicoreHeader: seconds in gps week */
    unsigned char antno;/* antenna number for multi-antenna receiver */
    unsigned char pbuff[255+4+2]; /* RT17: Packet buffer */
} raw_t;

/* external call functions ---------------------------------------------------*/

extern int init_raw   (raw_t *raw);
extern void free_raw  (raw_t *raw);

extern int decode_unicore (raw_t *raw, unsigned char data);
extern int decode_unicoref (raw_t *raw, FILE *fp);


/* public functions for decoding ---------------------------------------------*/

/* debug trace functions */
extern void trace    (int level, const char *format, ...);
/* receiver raw data functions */
extern unsigned int crc32  (const unsigned char *buff, int len);
/* satellites, systems, codes functions */
extern int  satno   (int sys, int prn);
extern int  satsys  (int sat, int *prn);
/* time and string functions */
extern gtime_t gpst2time(int week, double sec);
extern double  time2gpst(gtime_t t, int *week);
extern gtime_t bdt2time(int week, double sec);
extern double  time2bdt(gtime_t t, int *week);
extern gtime_t epoch2time(const double *ep);
extern void    time2epoch(gtime_t t, double *ep);
extern char    *time_str(gtime_t t, int n);
extern void    time2str(gtime_t t, char *str, int n);

#ifdef __cplusplus
}
#endif

#endif // DECODE_H