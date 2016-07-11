/*------------------------------------------------------------------------------
* unicore.c : decode unicorecomm binary data
*
* author    : Guangli Dong
* 
* history : 2016/03/21  created
*           2016/04/27  added gsof decoding code
*           2016/06/12  added RANGEH to RANGE conversion function
*           2016/07/11  add decode_satvis function
*           2016/07/11  modify SNR[] decoding function
*-----------------------------------------------------------------------------*/

#include "decode.h"

/* constants -----------------------------------------------------------------*/
#define PI          3.1415926535897932  /* pi */

#define SYNC1           0xAA    /* synchronization charater 1 of packet head */
#define SYNC2           0x44    /* synchronization charater 2 of packet head */
#define SYNC3           0x12    /* synchronization charater 3 of packet head */
#define BIG_ENDIAN      1       /* Big-endian platform or data stream */
#define LITTLE_ENDIAN   2       /* Little-endian platform or data stream */

#define BD2EPHEM        1047    /* MSG ID: Beidou ephemeris */
#define BD2IONUTC       2010    /* MSG ID: Beidou ion and utc data */
#define GPSEPHEM        7       /* MSG ID: GPS ephemeris */
#define IONUTC          8       /* MSG ID: GPS ion and utc data */
#define RANGE           43      /* MSG ID: raw observables */
#define RANGEH          6005    /* MSG ID: raw observables of heading antenna */
#define HEADING         971     /* MSG ID: gsof attitude message */
#define PSRVEL          100     /* MSG ID: gsof velocity messgae */
#define PSRPOS          47      /* MSG ID: gsof position message */
#define SATVIS          48      /* MSG ID: gsof satellite information message */

/* Data conversion macros: ---------------------------------------------------*/
#define I1(p) (*((char*)(p)))          /* One byte signed integer */
#define U1(p) (*((unsigned char*)(p))) /* One byte unsigned integer */
#define I2(p,e) read_i2(p,e)           /* Two byte signed integer */
#define U2(p,e) read_u2(p,e)           /* Two byte unsigned integer */
#define I4(p,e) read_i4(p,e)           /* Four byte signed integer */
#define U4(p,e) read_u4(p,e)           /* Four byte unsigned integer */
#define R4(p,e) read_r4(p,e)           /* IEEE S_FLOAT floating point number */
#define R8(p,e) read_r8(p,e)           /* IEEE T_FLOAT floating point number */

/* Internal structure definitions. -------------------------------------------*/
typedef union {unsigned short u2; unsigned char c[2];} ENDIAN_TEST;

/* Internal private function forward declarations (in alphabetical order):----*/
static int sync_packet(raw_t *raw, unsigned char data);
static void clear_message_buffer(raw_t *raw);
static short read_i2(unsigned char *p, int endian);
static int read_i4(unsigned char *p, int endian);
static float read_r4(unsigned char *p, int endian);
static double read_r8(unsigned char *p, int endian);
static unsigned short read_u2(unsigned char *p, int endian);
static unsigned int read_u4(unsigned char *p, int endian);
static int decode_bd2ephem(raw_t *raw, int endian);
static int decode_gpsephem(raw_t *raw, int endian);
static int decode_bd2ionutc(raw_t *raw, int endian);
static int decode_gpsionutc(raw_t *raw, int endian);
static int decode_range(raw_t *raw, int endian);
static int decode_rangeh(raw_t *raw, int endian);
static int decode_attitude(raw_t *raw, int endian);
static int decode_position(raw_t *raw, int endian);
static int decode_velocity(raw_t *raw, int endian);
static int decode_satvis(raw_t *raw, int endian);
static unsigned char get_snr(float snr);
static int rangeh2range(raw_t *raw, FILE *fp);

/* RANGEH to RANGE conversion variants ---------------------------------------*/

/*
| Function: decode_unicore
| Purpose:  Decode an UnicoreComm mesasge from a raw data stream byte by byte
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw  = Receiver raw data control structure [Input]
|   data = stream data byte                    [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|   raw->len
|   raw->nbyte
|
| Implicit outputs:
|
|   raw->buff[]
|   raw->len
|   raw->nbyte
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|   11: input observation data (heading antenna)
|   21: input gsof position data
|   22: input gsof velocity data
|   23: input gsof attitude data
|
| Design Issues:
|
*/
extern int decode_unicore(raw_t *raw, unsigned char data)
{
    int status = 0;
    unsigned short msg_id = 0;

    /* If no current packet */
    if (raw->nbyte == 0)
    {
        /* Find something that looks like a packet */
        if(sync_packet(raw, data))
        {
            raw->len   = raw->buff[3] + 
                U2(raw->buff+8, strstr(raw->opt, "-LE")?LITTLE_ENDIAN:BIG_ENDIAN) +
                4;              /* header + message + CRC32 */
            raw->nbyte = 10;    /* we now have 10 bytes in message buffer */  
        }
        /* Continue reading the rest of the packet from the stream. */
        return 0;
    }

    /* Store the next byte of the packet */
    raw->buff[raw->nbyte++] = data;

    /* Keep storing bytes into the current packet 
     * until we have what we think are all of them. */    
    if (raw->nbyte < raw->len)
        return (0);

    /* At this point we think we have an entire packet.
     * Check the packet checksum CRC32 */
    if (crc32(raw->buff, raw->len-4) != 
        U4(raw->buff+raw->len-4, strstr(raw->opt, "-LE")?LITTLE_ENDIAN:BIG_ENDIAN) )
    {
        clear_message_buffer(raw);
        return 0;
    }

    /* Get time tag(gpst) from record header */
    raw->week = U2(raw->buff+14, strstr(raw->opt, "-LE")?LITTLE_ENDIAN:BIG_ENDIAN);
    raw->seconds = (double)U4(raw->buff+16, 
        strstr(raw->opt, "-LE")?LITTLE_ENDIAN:BIG_ENDIAN)/1000.0;
    raw->time = gpst2time(raw->week, raw->seconds);
    raw->tbase= 0;

    /* Get message id */
    msg_id = U2(raw->buff+4, strstr(raw->opt, "-LE")?LITTLE_ENDIAN:BIG_ENDIAN);

    /* Add to output message type id */
    if (raw->outtype) {
        sprintf(raw->msgtype,"unicore %6d (%4d)", msg_id, raw->len);
    }

    /* If this is a Beidou ephemeris packet, then process it immediately. */
    if (msg_id == BD2EPHEM)
    {
        status = decode_bd2ephem( raw, strstr(raw->opt,"-LE") ?
                                        LITTLE_ENDIAN : BIG_ENDIAN );
        clear_message_buffer(raw);
        return (status);
    }

    /* If this is a GPS ephemeris packet */
    if (msg_id == GPSEPHEM)
    {
        status = decode_gpsephem(raw, strstr(raw->opt, "-LE") ? 
                                        LITTLE_ENDIAN : BIG_ENDIAN );
        clear_message_buffer(raw);
        return (status);
    }

    /* If this is a Beidou ion/utc packet */
    if (msg_id == BD2IONUTC)
    {
        status = decode_bd2ionutc(raw, strstr(raw->opt, "-LE") ? 
                                        LITTLE_ENDIAN : BIG_ENDIAN );
        clear_message_buffer(raw);
        return (status);
    }

    /* If this is a GPS ion/utc packet */
    if (msg_id == IONUTC)
    {
        status = decode_gpsionutc(raw, strstr(raw->opt, "-LE") ? 
                                        LITTLE_ENDIAN : BIG_ENDIAN );
        clear_message_buffer(raw);
        return (status);
    }

    /* If this is a raw observation packet */
    if (msg_id == RANGE)
    {
        status = decode_range(raw, strstr(raw->opt, "-LE") ? 
                                        LITTLE_ENDIAN : BIG_ENDIAN );
        clear_message_buffer(raw);
        return (status);
    }

    /* If this is a raw observation packet of heading antenna */
    if (msg_id == RANGEH)
    {
#ifdef RANGEH_RANGE
        extern FILE *rangehb;
        rangeh2range(raw, rangehb);
#endif
        status = decode_rangeh(raw, strstr(raw->opt, "-LE") ? 
                                        LITTLE_ENDIAN : BIG_ENDIAN );

        clear_message_buffer(raw);
        return (status);
    }

    /* If this is a attitude packet */
    if(msg_id == HEADING)
    {
        status = decode_attitude(raw, strstr(raw->opt, "-LE") ?
                                        LITTLE_ENDIAN : BIG_ENDIAN);
        clear_message_buffer(raw);
        return (status);
    }

    /* If this is a position packet */
    if(msg_id == PSRPOS)
    {
        status = decode_position(raw, strstr(raw->opt, "-LE") ?
                                        LITTLE_ENDIAN : BIG_ENDIAN);
        clear_message_buffer(raw);
        return (status);
    }

    /* If this is a velocity packet */
    if(msg_id == PSRVEL)
    {
        status = decode_velocity(raw, strstr(raw->opt, "-LE") ?
                                        LITTLE_ENDIAN : BIG_ENDIAN);
        clear_message_buffer(raw);
        return (status);
    }

    /* Other packets, ignored */
    clear_message_buffer(raw);
    return (0);
}

/*
| Function: decode_unicoref
| Purpose:  Decode an UnicoreComm mesasge from a file byte by byte
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   Raw  = Receiver raw data control structure [Input]
|   Data = stream data byte                    [Input]
|
| Implicit Inputs:
|
|   Raw->buff[]
|   Raw->pbuff[]
|   Raw->len
|   Raw->plen
|   Raw->nbyte
|   Raw->pbyte
|   Raw->reply
|
| Implicit outputs:
|
|   Raw->buff[]
|   Raw->pbuff[]
|   Raw->len
|   Raw->plen
|   Raw->nbyte
|   Raw->pbyte
|   Raw->reply
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|   11: input observation data (heading antenna)
|   21: input gsof position data
|   22: input gsof velocity data
|   23: input gsof attitude data
|
| Design Issues:
|
*/
extern int decode_unicoref(raw_t *raw, FILE *fp)
{
    int data, status; 

    while (1)
    {
        if ((data = fgetc(fp)) == EOF) return (-2);
        if ((status = decode_unicore(raw, (unsigned char) data))) return (status);
                        /* if there is no message resolved, then continue the loop */
    }
}

/*
| Function: sync_packet
| Purpose:  Synchronize the raw data stream to the start of a series of unicore packets 
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   Raw  = Receiver raw data control structure [Input]
|   Data = Next character in raw data stream   [Input]
|
| Implicit Inputs:
|
|   Raw->buff[0-9]
|
| Implicit Outputs:
|
|   Raw=>buff[0-9]
|
| Return Value:
|
|   TRUE  = Start of packet sequence tentatively found
|   FALSE = Not found, keep reading data bytes from the stream
|
| Design Issues:
|
*/
static int sync_packet(raw_t *raw, unsigned char data)
{
    unsigned char type;
    unsigned char head_len;     /* header length */
    unsigned short msg_id;      /* message id */
    unsigned short msg_len;     /* message data length */

    raw->buff[0] = raw->buff[1];	/* delete pbuff[0] and move forward the next 10 char */
    raw->buff[1] = raw->buff[2];
    raw->buff[2] = raw->buff[3];
    raw->buff[3] = raw->buff[4];
    raw->buff[4] = raw->buff[5];
    raw->buff[5] = raw->buff[6];
    raw->buff[6] = raw->buff[7];
    raw->buff[7] = raw->buff[8];
    raw->buff[8] = raw->buff[9];
    raw->buff[9] = data;			/* add the new entered char 'data' to the end of message head */

    msg_id = U2(raw->buff+4, strstr(raw->opt, "-LE")?LITTLE_ENDIAN:BIG_ENDIAN);

    msg_len= U2(raw->buff+8, strstr(raw->opt, "-LE")?LITTLE_ENDIAN:BIG_ENDIAN);

    /*
    | Byte 0-2 = synchronize character: 0xAA 0x44 0x12
    | Byte 4-5 = message id which must be the one we're intrested in.
    | Byte 8-9 = message length which must be non-zero for any message we're intrested in.
    */
    return ( (raw->buff[0] == SYNC1 && raw->buff[1] == SYNC2 && raw->buff[2] == SYNC3)
//              && (msg_id == BD2EPHEM || msg_id == BD2IONUTC || msg_id == GPSEPHEM || 
//                  msg_id == IONUTC || msg_id == RANGE || msg_id == RANGEH) 
              &&  msg_len != 0);
}

/*
| Function: read_i2
| Purpose:  Fetch & convert a signed two byte integer (short)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted signed two byte integer (short)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that
|   is not naturally aligned. It is then converted from the input
|   endianness to our execution platform endianness.
*/
static short read_i2(unsigned char *p, int endian) 
{
    union I2 {short i2; unsigned char c[2];} u;
    ENDIAN_TEST et;

    memcpy(&u.i2, p, sizeof(u.i2));

    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[1]; u.c[1] = t;
    }
    return (u.i2);
}

/*
| Function: read_i4
| Purpose:  Fetch & convert a four byte signed integer (int)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted four byte signed integer (int)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that is not
|   naturally aligned. It is then converted from the input endianness to our
|   execution platform endianness.
*/
static int read_i4(unsigned char *p, int endian)
{
    union i4 {int i4; unsigned char c[4];} u;
    ENDIAN_TEST et;

    memcpy(&u.i4, p, sizeof(u.i4));

    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[3]; u.c[3] = t;
        t = u.c[1]; u.c[1] = u.c[2]; u.c[2] = t;
    }   
    return (u.i4);
}

/*
| Function: read_r4
| Purpose:  Fetch & convert an IEEE S_FLOAT (float)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted IEEE S_FLOAT (float)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that is not
|   naturally aligned. It is then converted from the input endianness to our
|   execution platform endianness.
*/
static float read_r4(unsigned char *p, int endian)
{
    union R4 {float f; unsigned int u4;} u; 
    u.u4 = U4(p, endian);
    return (u.f);
}

/*
| Function: read_r8
| Purpose:  Fetch & convert an IEEE T_FLOAT (double)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted IEEE T_FLOAT (double)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that is not
|   naturally aligned. It is then converted from the input endianness to our
|   execution platform endianness.
*/
static double read_r8(unsigned char *p, int endian)
{
    ENDIAN_TEST et;
    union R8 {double d; unsigned char c[8];} u;

    memcpy(&u.d, p, sizeof(u.d));
 
    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[7]; u.c[7] = t;
        t = u.c[1]; u.c[1] = u.c[6]; u.c[6] = t;
        t = u.c[2]; u.c[2] = u.c[5]; u.c[5] = t;
        t = u.c[3]; u.c[3] = u.c[4]; u.c[4] = t;  
    }
    return (u.d);
}

/*
| Function: read_u2
| Purpose:  Fetch & convert an unsigned twe byte integer (unsigned short)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted two byte unsigned integer (unsigned short)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that
|   is not naturally aligned. It is then converted from the input
|   endianness to our execution platform endianness.
*/
static unsigned short read_u2(unsigned char *p, int endian)
{
    ENDIAN_TEST et;
    union U2 {unsigned short u2; unsigned char c[2];} u;

    memcpy(&u.u2, p, sizeof(u.u2)); 
 
    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[1]; u.c[1] = t;
    }
    return (u.u2);
}

/*
| Function: read_u4
| Purpose:  Fetch & convert a four byte unsigned integer (unsigned int)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted four byte unsigned integer (unsigned int)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that is not
|   naturally aligned. It is then converted from the input endianness to our
|   execution platform endianness.
*/
static unsigned int read_u4(unsigned char *p, int endian)
{
    ENDIAN_TEST et;
    union U4 {unsigned int u4; unsigned char c[4];} u;

    memcpy(&u.u4, p, sizeof(u.u4));
 
    et.u2 = 0; et.c[0] = 1;  
	
	/* et.u2 == 1 means host platform is LITTLE_ENDIAN, if stream platform is 
	 * BIG_ENDIAN, then we need to convert it our host platform.
	 * This ignores the occasion that host is BIG_ENDIAN while stream is LITTLE_ENDIAN
	 * by gldong 2016/03/18
	 */
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN)) 
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[3]; u.c[3] = t;
        t = u.c[1]; u.c[1] = u.c[2]; u.c[2] = t;
    }   
    return (u.u4);
}

/*
| Function: clear_message_buffer
| Purpose:  Clear the packet buffer
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|
| Implicit Outputs:
|
|   Raw->pbuff[0-9]
|   Raw->pbyte;
|   Raw->plen;
|
| Return Value:
|
|   <none>
|
| Design issues:
|
*/
static void clear_message_buffer(raw_t *raw)
{
    int i;

    memset(raw->buff, 0x00, 10);

    raw->len = raw->nbyte = 0;
}

/*
| Function: decode_bd2ephem
| Purpose:  Decode a BDS Ephemeris record
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   Raw = Receiver raw data control structure [Input]
|   Endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|
| Implicit outputs:
|
|   raw->nav.beph[]
|   raw->ephsys
|   raw->ephprn
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|   11: input observation data (heading antenna)
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_bd2ephem(raw_t *raw, int e)
{
    unsigned char header_len = (raw->buff[3]);  /* length of record header */
    unsigned char *p = raw->buff + header_len;  /* set p point to the message data */
    int prn, sat, toc, tow;
    unsigned int flags, toe;
    double sqrtA;
    eph_t eph={0};
    struct tm *tm_tm;

    /* Get prn */
    sat = U4(p, e);                     /* 000-003: System PRN */
    if (satsys(sat, &prn) != SYS_BDS)
    {
        trace(0, "unicore: BDS ephemeris satellite number error, PRN=%d.\n", prn);
        return (-1);
    }

    eph.tow = R8(p+4, e);               /* 004-011: Time (s) of 1st subfram in week, based on GPS time */
    eph.svh = U4(p+12, e);              /* 012-015: SV health */
    eph.aode= U4(p+16, e);              /* 016-019: AODE Age Of Data, Ephemeris*/
                                        /* 020-023: Another AODE, ignored */
    eph.week= U4(p+24, e);              /* 024-027: GPS Week */
                                        /* 028-031: Z week based on GPS for week-roll, ignored */
    eph.toes= R8(p+32, e);              /* 032-039: Ephemeris reference time */
    eph.A   = R8(p+40, e);              /* 040-047: Semi-major axis (m) */
    eph.deln= R8(p+48, e);              /* 048-055: Correction of mean angle velocity (rad/s) */
    eph.M0  = R8(p+56, e);              /* 056-063: Mean anomaly at reference time (rad) */
    eph.e   = R8(p+64, e);              /* 064-071: Eccentricity */
    eph.omg = R8(p+72, e);              /* 072-079: Argument of perigee (rad) */
    eph.cuc = R8(p+80, e);              /* 080-087: Amplitude of consine harmonic correction term to argument of latitude (rad) */
    eph.cus = R8(p+88, e);              /* 088-095: Amplitude of sine harmonic correction term to argument of latitude (rad) */
    eph.crc = R8(p+96, e);              /* 096-103: Amplitude of consine harmonic correction term to orbit radius (m) */
    eph.crs = R8(p+104, e);             /* 103-111: Amplitude of sine harmonic correction term to orbit radius (m) */
    eph.cic = R8(p+112, e);             /* 112-119: Amplitude of consine harmonic correction term to angle of inclination (rad) */
    eph.cis = R8(p+120, e);             /* 120-127: Amplitude of sine harmonic correction term to angle of inclination (rad) */
    eph.i0  = R8(p+128, e);             /* 128-135: Inclination angle at reference time (rad) */
    eph.idot= R8(p+136, e);             /* 136-143: Rate of inclination anlge (rad/s) */
    eph.OMG0= R8(p+144, e);             /* 134-151: Longitude of ascending node (rad) */
    eph.OMGd= R8(p+152, e);             /* 152-159: Rate of Longitude of ascending node (rad/s) */
    eph.aodc= U4(p+160, e);             /* 160-163: Age of data, clock */
    eph.tocs= R8(p+164, e);             /* 164-171: Reference time of clock parameters (s) */
    eph.tgd[0]  = R8(p+172, e);         /* 172-179: Equipment group delay differential of B1 (s) */
    eph.tgd[1]  = R8(p+180, e);         /* 180-187: Equipment group delay differential of B2 (s) */
    eph.f0  = R8(p+188, e);             /* 188-195: Satellite clock bias (s) */
    eph.f1  = R8(p+196, e);             /* 196-203: Satellite clock rate (s/s) */
    eph.f2  = R8(p+204, e);             /* 204-211: Satellite clock acceleration (s/s^2) */
                                        /* 212-215: Anti-sproof, ignored */
    eph.n   = R8(p+216, e);             /* 216-223: Corrected mean angle rate (rad/s) */
    eph.ura = R8(p+224, e);             /* 224-231: SV accuracy (m^2) */

    /* Convert  week and tow in gps time to gtime_t struct */
    eph.toc   = gpst2time(eph.week, eph.tocs);
    eph.toe   = gpst2time(eph.week, eph.toes);
    eph.ttr   = gpst2time(eph.week, eph.tow);

    /* Update BDS nav data */
    eph.prn = prn;
    raw->nav.beph[prn-1] = eph;
    raw->ephsys = SYS_BDS;
    raw->ephprn = prn;

    return (2);
}

/*
| Function: decode_gpsephem
| Purpose:  Decode a GPS Ephemeris record
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|   rndian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|
| Implicit outputs:
|
|   raw->nav.geph[]
|   raw->ephsys
|   raw->ephprn
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|   11: input observation data (heading antenna)
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_gpsephem(raw_t *raw, int e)
{
    unsigned char header_len = (raw->buff[3]);  /* length of record header */
    unsigned char *p = raw->buff + header_len;  /* set p point to the message data */
    int prn, sat, toc, tow;
    unsigned int flags, toe;
    double sqrtA;
    eph_t eph={0};
    struct tm *tm_tm;

    /* Get prn */
    sat = U4(p, e);                     /* 000-003: System PRN */
    if ( satsys(sat, &prn) != SYS_GPS )
    {
        trace(0, "unicore: GPS ephemeris satellite number error, PRN=%d.\n", prn);
        return (-1);
    }

    eph.tow = R8(p+4, e);               /* 004-011: Time (s) of 1st subfram in week, based on GPS time */
    eph.svh = U4(p+12, e);              /* 012-015: SV health */
                                        /* 016-019: ephemeris #1 age */
    eph.iode= U4(p+20, e);              /* 020-023: ephemeris #2 age = GPS IODE1 */
    eph.week= U4(p+24, e);              /* 024-027: GPS Week */
                                        /* 028-031: Z week based on GPS for week-roll, ignored */
    eph.toes= R8(p+32, e);              /* 032-039: Ephemeris reference time */
    eph.A   = R8(p+40, e);              /* 040-047: Semi-major axis (m) */
    eph.deln= R8(p+48, e);              /* 048-055: Correction of mean angle velocity (rad/s) */
    eph.M0  = R8(p+56, e);              /* 056-063: Mean anomaly at reference time (rad) */
    eph.e   = R8(p+64, e);              /* 064-071: Eccentricity */
    eph.omg = R8(p+72, e);              /* 072-079: Argument of perigee (rad) */
    eph.cuc = R8(p+80, e);              /* 080-087: Amplitude of consine harmonic correction term to argument of latitude (rad) */
    eph.cus = R8(p+88, e);              /* 088-095: Amplitude of sine harmonic correction term to argument of latitude (rad) */
    eph.crc = R8(p+96, e);              /* 096-103: Amplitude of consine harmonic correction term to orbit radius (m) */
    eph.crs = R8(p+104, e);             /* 103-111: Amplitude of sine harmonic correction term to orbit radius (m) */
    eph.cic = R8(p+112, e);             /* 112-119: Amplitude of consine harmonic correction term to angle of inclination (rad) */
    eph.cis = R8(p+120, e);             /* 120-127: Amplitude of sine harmonic correction term to angle of inclination (rad) */
    eph.i0  = R8(p+128, e);             /* 128-135: Inclination angle at reference time (rad) */
    eph.idot= R8(p+136, e);             /* 136-143: Rate of inclination anlge (rad/s) */
    eph.OMG0= R8(p+144, e);             /* 134-151: Longitude of ascending node (rad) */
    eph.OMGd= R8(p+152, e);             /* 152-159: Rate of Longitude of ascending node (rad/s) */
    eph.aodc= U4(p+160, e);             /* 160-163: Age of data, clock */
    eph.tocs= R8(p+164, e);             /* 164-171: Reference time of clock parameters (s) */
    eph.tgd[0]  = R8(p+172, e);         /* 172-179: Equipment group delay differential of B1 (s) */
    eph.f0  = R8(p+180, e);             /* 180-187: Satellite clock bias (s) */
    eph.f1  = R8(p+188, e);             /* 188-195: Satellite clock rate (s/s) */
    eph.f2  = R8(p+196, e);             /* 196-203: Satellite clock acceleration (s/s^2) */
                                        /* 204-207: Anti-sproof, ignored */
    eph.n   = R8(p+208, e);             /* 208-215: Corrected mean angle rate (rad/s) */
    eph.ura = R8(p+216, e);             /* 216-223: SV accuracy (m^2) */

    /* Convert  week and tow in gps time to gtime_t struct */
    eph.toc   = gpst2time(eph.week, eph.tocs);
    eph.toe   = gpst2time(eph.week, eph.toes);
    eph.ttr   = gpst2time(eph.week, eph.tow);

    /* Update GPS nav data */
    eph.prn = prn;
    raw->nav.beph[prn-1] = eph;
    raw->ephsys = SYS_GPS;
    raw->ephprn = prn;

    return (2);
}

/*
| Function: decode_bd2ionutc
| Purpose:  Decode a BDS ion/utc record
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|   endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|
| Implicit outputs:
|
|   raw->nav.ion_bds[]
|   raw->nav.utc_bds[]
|   raw->nav.leaps
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|   11: input observation data (heading antenna)
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_bd2ionutc(raw_t *raw, int e)
{
    unsigned char header_len = (raw->buff[3]);  /* length of record header */
    unsigned char *p = raw->buff + header_len;  /* set p point to the message data */
    double a0, a1, a2, a3, b0, b1, b2, b3;      /* ion parameters */
    double A0, A1;                              /* utc parameter */
    unsigned long utc_wn, tot;                  /* reference time of utc paramters */ 
    unsigned long wn_lsf, dn;                   /* new leap second effect week and day of week
                                                   dn: 0-6, 0 for Sunday */
    long deltat_ls, deltat_lsf;                 /* bds leap seconds before / after new leap second
                                                   effective */
    unsigned long deltat_utc;                   /* difference between BDT and UTC */

    /* Decode ion parameters */
    a0  = R8(p,    e);
    a1  = R8(p+8,  e);
    a2  = R8(p+16, e);
    a3  = R8(p+24, e);    
    b0  = R8(p+32, e);
    b1  = R8(p+40, e);
    b2  = R8(p+48, e);
    b3  = R8(p+56, e);

    /* Decode utc parameters */
    utc_wn  = U4(p+64, e);
    tot     = U4(p+68, e);
    A0      = R8(p+72, e);
    A1      = R8(p+80, e);

    /* Decode other parameters */
    wn_lsf  = U4(p+88, e);
    dn      = U4(p+92, e);
    deltat_ls   = I4(p+96, e);
    deltat_lsf  = I4(p+100, e);
    deltat_utc  = U4(p+104, e);

    /* update ion and utc parameters in raw->nav */
    raw->nav.ion_bds[0] = a0;
    raw->nav.ion_bds[1] = a1;
    raw->nav.ion_bds[2] = a2;
    raw->nav.ion_bds[3] = a3;
    raw->nav.ion_bds[4] = b0;
    raw->nav.ion_bds[5] = b1;
    raw->nav.ion_bds[6] = b2;
    raw->nav.ion_bds[7] = b3;

    raw->nav.utc_bds[0] = A0;
    raw->nav.utc_bds[1] = A1;
    raw->nav.utc_bds[2] = tot;
    raw->nav.utc_bds[3] = utc_wn;

    /* update leaps */
    raw->nav.leaps = deltat_ls + 14;    /* to convert leaps from bdst to gpst */

    return (9);
}

/*
| Function: decode_gpsionutc
| Purpose:  Decode a GPS ion/utc record
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|   endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|
| Implicit outputs:
|
|   raw->nav.ion_gps[]
|   raw->nav.utc_gps[]
|   raw->nav.leaps
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|   11: input observation data (heading antenna)
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_gpsionutc(raw_t *raw, int e)
{
    unsigned char header_len = (raw->buff[3]);  /* length of record header */
    unsigned char *p = raw->buff + header_len;  /* set p point to the message data */
    double a0, a1, a2, a3, b0, b1, b2, b3;      /* ion parameters */
    double A0, A1;                              /* utc parameter */
    unsigned long utc_wn, tot;                  /* reference time of utc paramters */ 
    unsigned long wn_lsf, dn;                   /* new leap second effect week and day of week
                                                   dn: 0-6, 0 for Sunday */
    long deltat_ls, deltat_lsf;                 /* gps leap seconds before / after new leap second
                                                   effective */
    unsigned long deltat_utc;                   /* difference between GPST and UTC */

    /* Decode ion parameters */
    a0  = R8(p,    e);
    a1  = R8(p+8,  e);
    a2  = R8(p+16, e);
    a3  = R8(p+24, e);    
    b0  = R8(p+32, e);
    b1  = R8(p+40, e);
    b2  = R8(p+48, e);
    b3  = R8(p+56, e);

    /* Decode utc parameters */
    utc_wn  = U4(p+64, e);
    tot     = U4(p+68, e);
    A0      = R8(p+72, e);
    A1      = R8(p+80, e);

    /* Decode other parameters */
    wn_lsf  = U4(p+88, e);
    dn      = U4(p+92, e);
    deltat_ls   = I4(p+96, e);
    deltat_lsf  = I4(p+100, e);
    deltat_utc  = U4(p+104, e);

    /* update ion and utc parameters in raw->nav */
    raw->nav.ion_gps[0] = a0;
    raw->nav.ion_gps[1] = a1;
    raw->nav.ion_gps[2] = a2;
    raw->nav.ion_gps[3] = a3;
    raw->nav.ion_gps[4] = b0;
    raw->nav.ion_gps[5] = b1;
    raw->nav.ion_gps[6] = b2;
    raw->nav.ion_gps[7] = b3;

    raw->nav.utc_gps[0] = A0;
    raw->nav.utc_gps[1] = A1;
    raw->nav.utc_gps[2] = tot;
    raw->nav.utc_gps[3] = utc_wn;

    /* update leaps */
    raw->nav.leaps = deltat_ls;

    return (9);
}

/*
| Function: decode_range
| Purpose:  Decode a raw observation record
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|   endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|   raw->time
|
| Implicit outputs:
|
|   raw->obs
|   raw->antno
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|   11: input observation data (heading antenna)
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_range(raw_t *raw, int e)
{
    typedef union {                             /* for analyse tracking status */
        unsigned int u;
        unsigned char c[4];
    } track_t;

    unsigned char header_len = (raw->buff[3]);  /* length of record header */
    unsigned char *p = raw->buff + header_len;  /* set p point to the message data */
    int i, j, k;
    int nobs;                                   /* observation number*/
    int prn;                                    /* satellite prn */
    unsigned char sys;                          /* satellite system */
    double psr, adr;                            /* pseudo range (m)/ carrier phase obs (cycle) */
    float dopp, cno;                            /* instant doppler (Hz)/ carrier noise ratio (dB-Hz)*/
    unsigned char code;                         /* code indicator (CODE_??)*/
    track_t ch_tr_status;                       /* channel tracking status 32 bits */
    unsigned char tmp;                          /* for ch_tr_status parsing */
    unsigned char *pp = NULL;                   /* pointer of the start for each obs */
    int nfreq;                                  /* frequency number e.g. L[nfreq] */
    int prnFlag;                                /* to check if this prn already has a record */
    
    /* Get the number of observations in this epoch */
    nobs    = I4(p, e); 
    /* Reset the number of obs in this epoch */
    raw->obs.n = 0;
    /* Read obs one by one */
    for(i=0; i<nobs; i++)
    {
        /* reset the pointer for the start of current obs */
        pp = p + 4 + i*44;  /* H + nobs + #obs*44 */        
        /* Get prn */
        prn = U2(pp, e);
        if (161<=prn && prn<=197 ) prn = prn -160;  /* BDS */
        /* ignore glofreq for glonass */

        /* Get pseduo range */
        psr = R8(pp+8-4, e);
        /* Get carrier phase */
        adr = R8(pp+20-4, e) * (-1.0); /* multiply by -1.0 to get non-negative value */
        /* Get instant doppler obs */
        dopp = R4(pp+32-4, e);
        /* Get carrier noise ratio */
        cno = R4(pp+36-4, e);
        /* Get channel tracking status without endian conversion */
        for(j=0; j<4; j++)
            ch_tr_status.c[j] = *(pp+44-4+j);   /* c[0] is earlier than c[3] */

        /* Get satellite system */
        tmp = (unsigned char)(ch_tr_status.u >> 16) & 0x07; /* 0000 0111 */
        switch(tmp)
        {
        case 0:
            sys = SYS_GPS;
            break;
        case 1:
            sys = SYS_GLO;
            break;
        case 4:
            sys = SYS_BDS;
            break;
        default:
            trace(0, "Unkown Satellite System!\n");
            return (0);
        }

        /* Get frequency number for the current system */
        tmp = (unsigned char)(ch_tr_status.u >> 21) & 0x1F; /* 0001 1111 */
        if(sys == SYS_GPS) {
            if(tmp == 0) {
                nfreq = 0;
                code  = CODE_L1C;
            }
            else if(tmp == 5 || tmp == 9) {
                nfreq = 1;
                code  = CODE_L2P;
            }/* L2P || L2P codeness */
            else if(tmp == 17) {
                nfreq = 1;
                code  = CODE_L2C;
            }
            else if(tmp == 14) {
                nfreq = 2;
                code  = CODE_L5Q;
            }
            else { 
                trace(0,"GPS Frequency Recorgnise Error!\n");
                return (0); 
            }
        }
        else if(sys == SYS_GLO) {
            if(tmp == 0) {
                nfreq = 0;
                code  = CODE_L1C;
            }
            else if(tmp == 5) {
                nfreq = 1;
                code  = CODE_L2P;
            }
            else {
                trace(0, "GLO Frequency Recorgnise Error!\n");
                return (0);
            }
        }
        else if(sys == SYS_BDS) {
            if(tmp == 0)        /* B1 */
                nfreq = 0;
            else if(tmp == 17)  /* B2 */
                nfreq = 1;
            else if(tmp == 21)  /* B3 */
                nfreq = 2;
            else {
                trace(0, "BDS Frequency Recorgnise Error!\n");
                return (0); 
            }
        }

        /* Update obs in raw */
        prnFlag = 0;
        for(k=0; k<raw->obs.n; k++) /* To find if the satellite already has a record */
            if (raw->obs.data[k].sys == sys &&
                raw->obs.data[k].sat == prn) {
                    prnFlag = 1;    /* found one */
                    break;
            }
        if( !prnFlag )  /* not found, then add a new record */
            k = raw->obs.n++;
        raw->obs.data[k].sys = sys;
        raw->obs.data[k].sat = prn;
        raw->obs.data[k].time= raw->time;
        raw->obs.data[k].P[nfreq] = psr;
        raw->obs.data[k].L[nfreq] = adr;
        raw->obs.data[k].D[nfreq] = dopp;
        raw->obs.data[k].SNR[nfreq] = floor(cno*4.0); /* refs to definition */
        raw->obs.data[k].code[nfreq]= code;
    }

    /* Set antenna number for current obs */
    raw->antno = 0;

    return (1);
}

/*
| Function: decode_rangeh
| Purpose:  Decode a raw observation record of the heading antenna
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|   endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|   raw->time
|
| Implicit outputs:
|
|   raw->obs
|   raw->antno
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|   11: input observation data (heading antenna)
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_rangeh(raw_t *raw, int e)
{
    int status;

    if(status=decode_range(raw, e) == 1) {
        raw->antno = 1; /* set the antenna number for current obs */
        return (11);
    }
    
    return (status);
}

/*
| Function: decode_attitude
| Purpose:  Decode gsof attitude message
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|   endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|
| Implicit outputs:
|
|   raw->gsof.att
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|   23: input gsof attitude data
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_attitude(raw_t *raw, int e)
{
    unsigned char header_len = (raw->buff[3]);  /* length of record header */
    unsigned char *p = raw->buff + header_len;  /* set p point to the message data */
    double  len, heading, pitch;
    float   heading_sig, pitch_sig;

    /* Get length */
    len     = (double)R4(p+8, e);
    /* Get heading */
    heading = (double)R4(p+12, e);
    heading_sig = R4(p+24, e);
    /* Get picth */
    pitch   = (double)R4(p+16, e);
    pitch_sig   = R4(p+28, e);

    /* check the sigma value is acceptable 
    if(heading_sig >= 0.2)
        return (0); */

    /* Update gsof attitude data */
    raw->gsof.att.length = len;
    raw->gsof.att.heading= heading;
    raw->gsof.att.heading_sig = heading_sig;
    raw->gsof.att.pitch  = pitch;
    raw->gsof.att.pitch_sig = pitch_sig;

    return (23);
}

/*
| Function: decode_position
| Purpose:  Decode gsof position message
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|   endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|
| Implicit outputs:
|
|   raw->gsof.pos
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|   21: input gsof position data
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_position(raw_t *raw, int e)
{
    unsigned char header_len = (raw->buff[3]);  /* length of record header */
    unsigned char *p = raw->buff + header_len;  /* set p point to the message data */
    double  lat, lon, hgt, undulation;

    /* Get latitude */
    lat = R8(p+8, e);
    /* Get longitude */
    lon = R8(p+16, e);
    /* Get geoid height */
    hgt = R8(p+24, e);
    /* Get undulation */
    undulation = (double)R4(p+32, e);

    /* Update gsof position data */
    raw->gsof.pos.lat = lat;
    raw->gsof.pos.lon = lon;
    raw->gsof.pos.hgt = hgt;
    raw->gsof.pos.undulation = undulation;

    return (21);
}

/*
| Function: decode_velocity
| Purpose:  Decode gsof velocity message
| Authors:  Guangli Dong
|
| Formal Parameters: 
|
|   raw = Receiver raw data control structure [Input]
|   endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   raw->buff[]
|
| Implicit outputs:
|
|   raw->gsof.vel
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|   22: input gsof velocity data
|
| Design Issues:
|
|   See BeiDou ICD V2.0 for documentation of the BDS satellite ephemeris.
|   See UnicoreComm command information reference manual V7.7.
*/
static int decode_velocity(raw_t *raw, int e)
{
    unsigned char header_len = (raw->buff[3]);  /* length of record header */
    unsigned char *p = raw->buff + header_len;  /* set p point to the message data */
    double  hspd, vspd, heading;

    /* Get horizontal speed */
    hspd = R8(p+16, e);
    /* Get horizontal speed direction relative to True North */
    heading = R8(p+24, e);
    /* Get vertical speed */
    vspd = R8(p+32, e);

    /* Update gsof velocity data */
    raw->gsof.vel.hspd = hspd;
    raw->gsof.vel.vspd = vspd;
    raw->gsof.vel.heading = heading;

    return (22);
}

/*
| Function: get_snr
| Purpose : convert snr(float) to snr index
| Author  : Guangli Dong
| 
| Formal parameter:
|   snr = carrier noise ratio
|
| Return value:
|   snr index
|
| Reference:
|   GPS ICD
|
*/
static unsigned char get_snr(float snr)
{
    if(snr < 12.0)
        return 1;
    else if(snr < 17.0)
        return 2;
    else if(snr < 23.0)
        return 3;
    else if(snr < 29.0)
        return 4;
    else if(snr < 35.0)
        return 5;
    else if(snr < 41.0)
        return 6;
    else if(snr < 47.0)
        return 7;
    else if(snr < 53.0)
        return 8;
    else
        return 9;
}

/*
| Function: rangeh2range
| Purpose : convert binary data from rangeh to range
| Author  : Guangli Dong
| 
| Formal parameter:
|   raw = receiver raw data control [INPUT]
|   fp  = output binary file        [INPUT] [OUTPUT]
|
| Implict input:
|   raw->buff[]
|   raw->len
|
| Return value:
|   non zero as succeed
|
| Remarks:
|   As unicore data converter cannot directly handle rangeh data to get O files,
| we need to convert rangeh data block to range format, so as to cheat the cove-
| rtion software. What we need to do is just change the rangeh msg id from 6005 
| (0x1775) to 43(0x2B) and recalculate the CRC32 checksum.
|
*/ 
/* option for convert rangeh data to range -----------------------------------*/
#ifdef RANGEH_RANGE
static int rangeh2range(raw_t *raw, FILE *fp)
{
    unsigned int newcrc32;
    /* change the msg id from 6005 to 43 */
    raw->buff[4] = 0x2B;
    raw->buff[5] = 0x00; 

    /* TODO: need to apply the CRC32 check at the end of raw->buff, maybe  
       need to use a union type to get every char of unsigned int */

    /* re-calculate CRC32 checksum */
    newcrc32 = crc32(raw->buff, raw->len-4);

    /* write binary data */
    fwrite(raw->buff, sizeof(unsigned char), raw->len-4, fp);
    fwrite(&newcrc32, sizeof(unsigned int), 1, fp);

    return (1);
}
#endif