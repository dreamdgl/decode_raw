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


/* internal function forward declaration -------------------------------------*/
static void write_obs_head(FILE *fp);
static void write_obs(raw_t *raw, FILE *fp);
#ifdef RANGEH_RANGE
FILE * rangehb;
#endif

int main()
{
    /* print version information ---------------------------------------------*/
    FILE    *binary_in, *pos_out, *vel_out, *att_out;
    extern FILE *rangehb;
    raw_t   *raw = (raw_t *)malloc(sizeof(raw_t));
    int     status = 0;
    int     record_threshold = 0;
    int     obs_counter, obsh_counter, eph_counter, ion_counter;
    int     i, j;
    char    *timetag;

    /* init the counters ----------------------------------------------------*/
    obs_counter=obsh_counter=eph_counter=ion_counter = 0;

    /* open output file ------------------------------------------------------*/
    pos_out = fopen("pos.txt", "w");
    vel_out = fopen("vel.txt", "w");
    att_out = fopen("att.txt", "w");
    if (pos_out==NULL || vel_out==NULL || att_out==NULL) {
        trace(0, "ERROR: open output file error !\n");
        return 0;
    }
    /*write_obs_head(ascii_out);*/
    
    /* check file path -------------------------------------------------------*/
    binary_in = fopen( "./logdata/log20160613_byGNSStick_1.cdtlog", "rb");
    if (binary_in == NULL)
    {
        trace(0, "%s\n\n", "ERROR: open input file error!");
        return 0;
    }

    /* open rangeh out binary file */
#ifdef RANGEH_RANGE
    rangehb = fopen( "rangeh2.log", "wb");
    if (rangehb == NULL)
    {
        trace(0, "%s\n\n", "ERROR: open rangehb file error!");
        return 0;
    }
#endif

    /* initialise raw ---------------------------------------------------------*/
    if (0 == init_raw(raw))
    {
        trace(0, "%s\n\n", "ERROR: memory allocation error!");
        return 0;
    }

    strcpy(raw->opt, "-LE");    /* the file type is set to LITTLE_ENDIAN by default */
    raw->outtype    = 1;        /* set to output message type id */

    /* read file data in a loop -----------------------------------------------*/
    while(status != -2/* && record_threshold < 2000*/)
    {
        record_threshold++;
        status = decode_unicoref(raw, binary_in);
        /* avoid useless loop calculation consumptiom */
        if (status == 0)
            continue;
        /* get timetag */
        timetag = time_str(raw->time, 3);

        switch(status)
        {
        case 1:         /* input observation data */
            ++obs_counter;
            for (i=0; i<raw->obs.n; i++) {
                printf("%c%02d: %5.2f  %5.2f \n",
                    (raw->obs.data[i].sys==SYS_GPS)?'G':'B',
                    raw->obs.data[i].sat,
                    raw->obs.data[i].SNR[0]/4.0,
                    raw->obs.data[i].SNR[1]/4.0);
            }
#if 0
            /*write_obs(raw, ascii_out);*/
            if (raw->antno == 0) {
                for (i=0; i<raw->obs.n; i++) {
                    if (raw->obs.data[i].sys == SYS_GPS &&
                        raw->obs.data[i].sat == 31)
                        /* output observations 
                        fprintf(ascii_out, "%15.3f %15.3f\n", 
                        raw->obs.data[i].P[0],
                        raw->obs.data[i].P[1]); */
                        /* calc elevation */
                        fprintf(ascii_out, "%25s %15.6f %15.6f\n",
                        time_str(raw->time, 3),
                        180.0/3.1415926*(3.1415926/2.0 
                        - acos((26550*26550 + raw->obs.data[i].P[0]*raw->obs.data[i].P[0]/1000000
                        - 6368*6368)/(2*26550*raw->obs.data[i].P[0]/1000))
                        - acos((26550*26550 - raw->obs.data[i].P[0]*raw->obs.data[i].P[0]/1000000
                        + 6368*6368)/(2*26550*6368))) ,
                        180.0/3.1415926*(3.1415926/2.0 
                        - acos((26550*26550 + raw->obs.data[i].P[1]*raw->obs.data[i].P[1]/1000000
                        - 6368*6368)/(2*26550*raw->obs.data[i].P[1]/1000))
                        - acos((26550*26550 - raw->obs.data[i].P[1]*raw->obs.data[i].P[1]/1000000
                        + 6368*6368)/(2*26550*6368))) );
                }
            }
#endif
            break;
        case 2:         /* input ephemeris */
            ++eph_counter;
            break;
        case 3:         /* input sbas message */
            break;
        case 9:         /* input ion/utc parameter */
            ++ion_counter;
/*            printf("leaps: %4d\n", raw->nav.leaps); */
            break;
        case 11:        /* input observation data of heading antenna */
            ++obsh_counter;
/*            write_obs(raw, ascii_out);*/
            break;
        case 21:        /* input gsof position data */
            fprintf(pos_out, "%25s %15.8lf %15.8lf %15.3f %15.3f\n",
                timetag,
                raw->gsof.pos.lat * 111138.555 - 3463400,
                raw->gsof.pos.lon * 111138.555 - 13481850,
                raw->gsof.pos.hgt,
                raw->gsof.pos.undulation);
            break;
        case 22:        /* input gsof velocity data */
            fprintf(vel_out, "%25s %15.3lf %15.3lf %15.3lf\n",
                timetag,
                raw->gsof.vel.hspd,
                raw->gsof.vel.vspd,
                raw->gsof.vel.heading);
            break;
        case 23:        /* input gsof attitude data */
            fprintf(att_out, "%25s %15.3lf %15.3lf %15.3lf %15.3lf %15.3lf \
                             %15.3lf\n",
                timetag,
                raw->gsof.att.heading,
                raw->gsof.att.heading_sig,
                raw->gsof.att.pitch,
                raw->gsof.att.pitch_sig,
                raw->gsof.att.roll,
                raw->gsof.att.length);
            break;

        default:
            break;
        }
        
    }

    /* clear process ---------------------------------------------------------*/
    fclose(binary_in);
    fclose(pos_out);    fclose(vel_out);    fclose(att_out);

#ifdef RANGEH_RANGE
    fclose(rangehb);
#endif

    free_raw(raw);
    if(status == -2)
        trace(0, "%s\n", "WARNING: data end !");
    else
        trace(0, "%s\n", "WARNING: read assigned record!"); 

    /* show summary message */
    printf( "--------------------------SUMMARY-----------------------------\n" );
    printf( "obs :%6d\n", obs_counter );
    printf( "obsh:%6d\n", obsh_counter );
    printf( "eph :%6d\n", eph_counter);
    printf( "ion :%6d\n", ion_counter);
    printf( "--------------------------------------------------------------\n");

    return 0;
}

/*
| write obs for test 
*/
static void write_obs(raw_t *raw, FILE *fp)
{
    int i, j;
    fprintf(fp, "%25s \n", time_str(raw->time, 3));
    for (i=0; i<raw->obs.n; i++) {
        fprintf(fp, "%25s %4s %2d%2d ", 
            " ",
            raw->obs.data[i].sys == SYS_GPS?"GPS":"BDS",
            raw->obs.data[i].sat,
            raw->antno);
        for (j=0; j<3; j++) {
            fprintf(fp, "%15.3f %15.3f %5d ",
                raw->obs.data[i].P[j],
                raw->obs.data[i].L[j],
                raw->obs.data[i].SNR[j]);
        }
        fprintf(fp, "\n");
    }
}

/*
| write obs head line
*/
static void write_obs_head(FILE *fp)
{
    int i;
    fprintf(fp, "%25s %4s %4s %15s %15s %5s %15s %15s %5s %15s %15s %5s\n",
        "time", "sys", "prn",
        "C1", "L1", "SNR1",
        "C2", "L2", "SNR2",
        "C5", "L5", "SNR5");
    for(i=0; i<149; i++)
        fprintf(fp, "%c", '-');
    fprintf(fp, "\n");
}
