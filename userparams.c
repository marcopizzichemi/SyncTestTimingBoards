#include "synctest.h"

void SetUserParams(UserParams_t *Params)
{
	// CONNECTION PARAMETERS:
	// ConnectionType: can be CAEN_DGTZ_USB, CAEN_DGTZ_OpticalLink (A2818 or A3818)
	// LinkNum: USB or PCI/PCIe enumeration (typ=0)
	// ConetNode: position in the optical daisy chain
	// BaseAddress: only for VME access (otherwise 0)

	Params->ConnectionType[0]	= CAEN_DGTZ_OpticalLink; //CAEN_DGTZ_USB;
	Params->LinkNum[0]		= 0;
	Params->ConetNode[0]		= 0;
	Params->BaseAddress[0]		= 0x33100000; //Master board

	Params->ConnectionType[1]	= CAEN_DGTZ_OpticalLink; //CAEN_DGTZ_USB;
	Params->LinkNum[1]		= 0;
	Params->ConetNode[1]		= 1;
	Params->BaseAddress[1]		= 0x32100000; //Slave board
	
	// TR and CHANNEL SETTINGS
    // NIM signal on TR:	DC_OFFSET 32768 	TRIGGER_THRESHOLD 20934 
    // NIM/2 signal on TR:	DC_OFFSET 32768 	TRIGGER_THRESHOLD 23574 
	// AC signal on TR:		DC_OFFSET 32768		TRIGGER_THRESHOLD 26214 
	// +2V signal on TR: 	DC_OFFSET 43520		TRIGGER_THRESHOLD 26214
	Params->RefChannel[0]			=16;		// Channel(or group?) of the Master used for the acquisition
	Params->FastTriggerThreshold[0]	= 20934;	// TR Trigger threshold (for fast triggering)
	Params->FastTriggerOffset[0]	= 32768;   // TR Offset adjust (DAC value)
	Params->DCoffset[0]				= 0x7FFF;   // input DC offset adjust (DAC value)
	Params->PostTrigger[0]			= 0;		// Post trigger in percent of the acquisition window
	// Edge Setting
	Params->ChannelThreshold[0]		= 1500;		// Threshold for Time Pulse Calculation
	Params->ChannelPulseEdge[0]		= CAEN_DGTZ_TriggerOnFallingEdge;
	Params->TRThreshold[0]			= 1500;		// Threshold for Time Pulse Calculation

	Params->RefChannel[1]			= 16;		// Group of the Slave used for the acquisition
	Params->FastTriggerThreshold[1]	= 20934;		// TR Trigger threshold (for fast triggering)
	Params->FastTriggerOffset[1]	= 32768;   // TR Offset adjust (DAC value)
	Params->DCoffset[1]				= 0x7FFF;   // input DC offset adjust (DAC value)
	Params->PostTrigger[1]			= 0;		// Post trigger in percent of the acquisition window
	// Edge Setting
	Params->ChannelThreshold[1]		= 1500;		// Threshold for Time Pulse Calculation
	Params->ChannelPulseEdge[1]		= CAEN_DGTZ_TriggerOnFallingEdge;
	Params->TRThreshold[1]			= 1500;		// Threshold for Time Pulse Calculation

	// Trigger edge (CAEN_DGTZ_TriggerOnRisingEdge, CAEN_DGTZ_TriggerOnFallingEdge)
	Params->TriggerEdge			= CAEN_DGTZ_TriggerOnFallingEdge;

	// Number of samples in the acquisition windows (Valid Options: 1024, 520, 256 and 136)
	Params->RecordLength		= 1024;

	// Max. distance between the trigger time tags in order to consider a valid coincidence
	Params->MatchingWindow		= 200;
	
	// Front Panel LEMO I/O level (NIM or TTL). Options: CAEN_DGTZ_IOLevel_NIM, CAEN_DGTZ_IOLevel_TTL
	Params->IOlevel				= CAEN_DGTZ_IOLevel_NIM;

	// Internal Test Pattern (triangular wave replacing ADC data): 0=disabled, 1=enabled
	Params->TestPattern			= 0;

	// DRS4SamplingFrequency. Options: CAEN_DGTZ_DRS4_5GHz, CAEN_DGTZ_DRS4_2_5GHz,CAEN_DGTZ_DRS4_1GHz
	Params->DRS4Frequency		= CAEN_DGTZ_DRS4_5GHz;

	// ***************************************************************************************************
	// Start Mode. Options: START_SW_CONTROLLED, START_HW_CONTROLLED 
	// ***************************************************************************************************
	Params->StartMode = START_SW_CONTROLLED;

	// ***************************************************************************************************
	// Sync Mode: INDIVIDUAL_TRIGGER_SIN_TRGOUT ()
	/*
	N boards with TR triggers 
	SETUP:
	daisy chain SIN-TRGOUT to propagate the start of run

	START OF RUN: 
	1.	All boards: set start mode = start on SIN high level; 
	2.	All boards: set Tr parameters and enable triggering and digitizing
	3.	All boards: set propagation of SIN to TRG-OUT
	4.	All boards armed to start
	5.	Send SW Start to 1st board
	6.	The RUN signal is propagate through the daisy chain SIN-TRGOUT and start all the boards. 
		Delay can be compensated by means of RUN_DELAY register
	7.	Once started, each board is triggered by the TR signal

	NOTE1: Start and Stop can also be controlled by an external signal going into SIN of the 1st board
	NOTE2: the stop is also synchronous:  when the 1st board is stopped (by SW command), the stop is propagated to the other boards 
		   through the daisy chain.
	*/
	
	// ***************************************************************************************************
	// Log Enabling
	// ***************************************************************************************************
	Params->EnableLog			= 1;

	// ***************************************************************************************************
	// Parameters for the time distribution histogram
	// ***************************************************************************************************
	Params->HistoNbins			= 2000;		// Number of bins of the histogram
	Params->HistoBinSize		= 0.005;		// Bin size in ns
	Params->HistoOffset			= -4.0;	// Lower value of the histogram in ns

}


