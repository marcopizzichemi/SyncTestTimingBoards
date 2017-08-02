#ifndef __SYNCTEST_H
#define __SYNCTEST_H

#define SyncTest_Release        "1.2"
#include <CAENDigitizer.h>
#include <math.h>

#ifdef WIN32

    #include <time.h>
    #include <sys/timeb.h>
    #include <conio.h>
    #include <process.h>


	#define	_PACKED_
	#define	_INLINE_		
    #define popen  _popen    /* redefine POSIX 'deprecated' popen as _popen */
    #define pclose  _pclose  /* redefine POSIX 'deprecated' pclose as _pclose */

    #define SLEEP(x) Sleep(x)

#else
    #include <sys/time.h> /* struct timeval, select() */
    #include <termios.h> /* tcgetattr(), tcsetattr() */
    #include <stdlib.h> /* atexit(), exit(), C99 compliant compilers: uint64_t */
    #include <unistd.h> /* read() */
    #include <stdio.h> /* printf() */
    #include <string.h> /* memcpy() */
    #include <ctype.h>    /* toupper() */

    #define UINT64_T uint64_t
    #define UINT32_T uint32_t

	#define		_PACKED_		__attribute__ ((packed, aligned(1)))
	#define		_INLINE_		__inline__ 

    #define SLEEP(x) usleep(x*1000)

static struct termios g_old_kbd_mode;

/*****************************************************************************/
static void cooked(void)
{
	tcsetattr(0, TCSANOW, &g_old_kbd_mode);
}

static void raw(void)
{
	static char init;
	struct termios new_kbd_mode;

	if(init) return;
/* put keyboard (stdin, actually) in raw, unbuffered mode */
	tcgetattr(0, &g_old_kbd_mode);
	memcpy(&new_kbd_mode, &g_old_kbd_mode, sizeof(struct termios));
	new_kbd_mode.c_lflag &= ~(ICANON | ECHO);
	new_kbd_mode.c_cc[VTIME] = 0;
	new_kbd_mode.c_cc[VMIN] = 1;
	tcsetattr(0, TCSANOW, &new_kbd_mode);
/* when we exit, go back to normal, "cooked" mode */
	atexit(cooked);
	init = 1;
}

/*****************************************************************************/
/*  GETCH  */
/*****************************************************************************/
int getch(void);

/*****************************************************************************/
/*  KBHIT  */
/*****************************************************************************/
int kbhit();

#endif

//****************************************************************************
// Some register addresses
//****************************************************************************
#define ADDR_GLOBAL_TRG_MASK     0x810C
#define ADDR_TRG_OUT_MASK        0x8110
#define ADDR_FRONT_PANEL_IO_SET  0x811C
#define ADDR_ACQUISITION_MODE    0x8100
#define ADDR_EXT_TRG_INHIBIT     0x817C
#define ADDR_RUN_DELAY           0x8170
#define ADDR_FORCE_SYNC			 0x813C
#define ADDR_RELOAD_PLL			 0xEF34


//****************************************************************************
// Run Modes
//****************************************************************************
// start on software command 
#define RUN_START_ON_SOFTWARE_COMMAND     0xC 
// start on S-IN level (logical high = run; logical low = stop)
#define RUN_START_ON_SIN_LEVEL            0xD
// start on first TRG-IN or Software Trigger 
#define RUN_START_ON_TRGIN_RISING_EDGE    0xE
// start on LVDS I/O level
#define RUN_START_ON_LVDS_IO              0xF


//****************************************************************************
// Sync Modes
//****************************************************************************
#define COMMONT_EXTERNAL_TRIGGER_TRGIN_TRGOUT   0
#define INDIVIDUAL_TRIGGER_SIN_TRGOUT			1
#define TRIGGER_ONE2ALL_EXTOR					2

//****************************************************************************
// Start Modes
//****************************************************************************
#define START_SW_CONTROLLED   0
#define START_HW_CONTROLLED   1

//****************************************************************************
// Max number of event to read in a Block Transfer
//****************************************************************************
#define MAX_EVENTS_XFER   255


//****************************************************************************
// Variables for the user parameters
//****************************************************************************
typedef struct {
    int ConnectionType[2];
    int LinkNum[2];
    int ConetNode[2];
    uint32_t BaseAddress[2];
    
    int RefChannel[2];
	uint16_t FastTriggerThreshold[2];
    uint16_t FastTriggerOffset[2];
    uint16_t DCoffset[2];
    uint16_t ChannelThreshold[2];
    uint16_t TRThreshold[2];
	CAEN_DGTZ_TriggerPolarity_t ChannelPulseEdge[2];
    uint32_t PostTrigger[2];
    
	int RecordLength;
	CAEN_DGTZ_TriggerPolarity_t TriggerEdge;
	int MatchingWindow;
	int TestPattern;
	int IOlevel;
//	int SyncMode;
	int StartMode;
	int EnableLog;
	CAEN_DGTZ_DRS4Frequency_t DRS4Frequency;

	int HistoNbins;
	double HistoOffset;
	double HistoBinSize;
} UserParams_t;

typedef struct
{
  float TTT[2];
  float *Wave[2];
  float *Trigger[2];
  float PulseEdgeTime[2];
  float TrEdgeTime[2];
} OutputData_t;


//****************************************************************************
// Functions
//****************************************************************************
void SetUserParams(UserParams_t *Params);
#endif
