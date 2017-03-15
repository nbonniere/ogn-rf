/*
    OGN - Open Glider Network - http://glidernet.org/
    Copyright (c) 2015 The OGN Project

    A detailed list of copyright holders can be found in the file "AUTHORS".

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this software.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <time.h>
#include <sys/time.h>

#include <math.h>

// #include "alloc.h"
#include "asciitime.h"
#include "thread.h"

#include "buffer.h"

// =================================================================================

//#include <rtl-sdr.h>
#include <airspy.h>

//#ifndef NEW_RTLSDR_LIB
//#define NEW_RTLSDR_LIB
//#endif

   const char *Device_Name = "Airspy";
   char FWver[255 + 1];

   // gain tables in 0.1 dB steps
   const char *LNA_Gain_Name = "LNA";
#define LNA_GAIN_COUNT (16)
   static int LNA_Gain_Table[LNA_GAIN_COUNT] =   { 0,9,22,62,100,113,144,166,192,223,249,263,282,287,322,335 };
   const char *Mixer_Gain_Name = "Mixer";
#define MIXER_GAIN_COUNT (15)
   static int Mixer_Gain_Table[MIXER_GAIN_COUNT] = { 0,5,15,25,44,53,63,88,105,115,123,139,152,158,161 };
   const char *VGA_Gain_Name = "VGA";
#define VGA_GAIN_COUNT (16)
   static int VGA_Gain_Table[VGA_GAIN_COUNT] =   {-47,-21,5,35,77,112,136,149,163,195,231,265,300,337,372,408 };

//#define DEFAULT_GAIN_COUNT (29)
   // static const int Default_Gain_Table[DEFAULT_GAIN_COUNT] = { 0,9,14,27,37,77,87,125,144,157,166,197,207,229,254,280,297,328,338,364,372,386,402,421,434,439,445,480,496 };
#define GAIN_COUNT (22)
   static int Linearity_Gain_Table[GAIN_COUNT] =   { -77,-59,9,36,98,111,143,174,188,197,229,251,287,313,354,388,439,489,504,547,632,661 };
   static int Sensitivity_Gain_Table[GAIN_COUNT] = { -66,-23,0,41,82,117,158,209,240,292,319,324,375,399,425,439,471,507,541,576,624,661 };

class RTLSDR
{ public:
   MutEx         Lock;           // for multi-threading
   Condition     BufferWait;     // for multi-threading

   uint32_t      DeviceIndex;    // RTL dongle index
//n   rtlsdr_dev_t *Device;         // RTL dongle handle
   airspy_device *Device;        // RTL dongle handle

   int           GainMode;       // current gain mode
   int           GainIndex;      // current gain index
   int           Gains;          // number of possible gain settings
   int           Gain[64];       // [0.1 dB] list of possible gain settings

#define MAX_SAMPLE_RATES (16)
   uint32_t      SampleRates;             // number of possible sample rates
   uint32_t      Rates[MAX_SAMPLE_RATES]; // [MSps] list of possible sample rates

   uint32_t      Sample_Rate;       // current sample rate
   uint32_t      Centre_Frequency;  // current frequency

   int           Bandwidths;
   int           Bandwidth[16];

   int           Stages;
   int           StageGains[8];
   char          StageName[8][32];
   int           StageGain[8][32];

   uint64_t      BytesRead;      // Counts number of bytes read (1 sample = 2 bytes: I/Q)
   int         (*Callback)(uint8_t *Buffer, int Samples, double SampleTime, double SamplePeriod, void *Contex);
   void         *CallbackContext;

#ifndef __MACH__ // _POSIX_TIMERS
   clockid_t     RefClock;           // CLOCK_REALTIME, CLOCK_MONOTONIC or CLOCK_MONOTONIC_RAW
#endif

   double        SampleTime;         // [sec] time when a batch of samples starts
   double        StartTime;          // [sec] time when acquisition started
   double        AverPeriod;         // [sec] averaging period for TimeRef, TimeRef_DMS and SamplePeriod
   double        SamplePeriod;       // [sec] time per sample
   double        PrevTime;           // [sec]
   double        SampleTime_DMS;     // [sec^2] mean square variation of SampleTime

//   uint8_t* DestBufferPtr;  // destination buffer
   SDATA* DestBufferPtr;  // destination buffer
   int desiredSampleCount;
   int skipSampleCount;
   void* rx_ctx;

  public:
   RTLSDR()
   { DeviceIndex=0; Device=0; Callback=0; CallbackContext=0; // Gain=0;
     AverPeriod=100.0; GainIndex=0; GainMode=0;
#ifndef __MACH__ // _POSIX_TIMERS
     RefClock=CLOCK_REALTIME;
#endif
   }

  ~RTLSDR()
   { Close(); }

   bool isOpen(void) const {
     return Device != 0;
   }

   void Close(void) {
     if (Device) {
       // printf("RTLSDR::Close() => %3.1f MB read, %3.1f samples/sec\n", BytesRead/(1024*1024.0), 1.0/SamplePeriod);
//n       rtlsdr_cancel_async(Device);
       airspy_stop_rx(Device);
//n       rtlsdr_close(Device);
       airspy_close(Device);
     }
     // free(Gain); Gain=0;
     GainIndex=0; GainMode=0; Gains=0; Stages=0; Bandwidths=0;
     // free(SampleTimePipe); free(SampleIdxPipe); PipeSize=0; SampleTimePipe=0; SampleIdxPipe=0;
     Device=0; Callback=0;
   }

   int getNumberOfDevices(void) {
//n     return rtlsdr_get_device_count(); // number of connected devices (USB RTL dongles)
   /* TBD
   */
   // for now check for 1 device or none
     int result;
     result = airspy_open(&Device);
     if (result == AIRSPY_SUCCESS) {
       airspy_close(Device);
	   return 1;
     } else {
	   return 0;
	 }
   }
   static int  getDeviceUsbStrings(uint32_t DeviceIndex, char *Manufacturer, char *Product, char *Serial) {
//n     return rtlsdr_get_device_usb_strings(DeviceIndex, Manufacturer, Product, Serial);  // USB description strings
     return 0;
   }
   int getUsbStrings(char *Manufacturer, char *Product, char *Serial) {
   // { return rtlsdr_get_device_usb_strings(DeviceIndex, Manufacturer, Product, Serial);   // USB description strings
//n     return rtlsdr_get_usb_strings(Device, Manufacturer, Product, Serial);  // USB description strings
     uint8_t board_id = AIRSPY_BOARD_ID_INVALID;
     airspy_read_partid_serialno_t read_partid_serialno;
     int result;
     result = airspy_board_id_read(Device, &board_id);
     if (result == AIRSPY_SUCCESS) {
//       sprintf(Id,"Board ID Number: %d (%s)\n", board_id,
//         airspy_board_id_name(board_id));
     }

     result = airspy_version_string_read(Device, &FWver[0], 255);
     if (result == AIRSPY_SUCCESS) {
       sprintf(Product,"FW: %s", FWver);
     }

     result = airspy_board_partid_serialno_read(Device, &read_partid_serialno);
     if (result == AIRSPY_SUCCESS) {
       sprintf(Manufacturer,"Airspy 0x%08X 0x%08X",
         read_partid_serialno.part_id[0],
         read_partid_serialno.part_id[1]);
       sprintf(Serial, "0x%08X%08X",
         read_partid_serialno.serial_no[2],
         read_partid_serialno.serial_no[3]);
     }
     return 0;
   }
   static const char *getDeviceName(uint32_t DeviceIndex) {
//n     return rtlsdr_get_device_name(DeviceIndex);  // name of given device
     return Device_Name;
   }
   const char *getDeviceName(void) {
//n     return rtlsdr_get_device_name(DeviceIndex); // name of this device open by this object
     return Device_Name;
   }
   static const char TunerTypeCount = 8;
   int getTunerType(void) {
//n     return rtlsdr_get_tuner_type(Device);
     return TunerTypeCount-1;  // airspy
   }
   const char *getTunerTypeName(void) {
//n     const char *TunerType[7] = { "UNKNOWN", "E4000", "FC0012", "FC0013", "FC2580", "R820T", "R828D" } ;
     const char *TunerType[TunerTypeCount] = { "UNKNOWN", "E4000", "FC0012", "FC0013", "FC2580", "R820T", "R828D", "R820T2" } ;
     int Type = getTunerType();
//n	 if ((Type<0) && (Type>=7)) {
	 if ((Type < 0) && (Type >= TunerTypeCount-1)) {
	   Type = 0;
	 }
	 return TunerType[Type];
   }

   int getXtalFreq(uint32_t &RtlFreq, uint32_t &TunerFreq) {
//n     return rtlsdr_get_xtal_freq(Device, &RtlFreq, &TunerFreq);
     return -1;
   }
   int setXtalFreq(uint32_t  RtlFreq, uint32_t  TunerFreq) {
//n    return rtlsdr_set_xtal_freq(Device,  RtlFreq,  TunerFreq);
     return -1;
   }

   int ReadEEPROM (uint8_t *Data, uint8_t Offset, uint16_t Size) {
//n    return rtlsdr_read_eeprom (Device, Data, Offset, Size);  // read  the EEPROM
     return -1;
   }
   int WriteEEPROM(uint8_t *Data, uint8_t Offset, uint16_t Size) {
//n    return rtlsdr_write_eeprom(Device, Data, Offset, Size);  // write the EEPROM
     return -1;
   }

   int setOffsetTuning(int ON=1) {
//n    return rtlsdr_set_offset_tuning(Device, ON);
     return -1;
   }
   int getOffsetTuning(void) {
//n    return rtlsdr_get_offset_tuning(Device);
     return -1;
   }
   int setCenterFreq(uint32_t Frequency) {
//n     return rtlsdr_set_center_freq(Device, Frequency);  // [Hz]
     Centre_Frequency = Frequency;
	 return airspy_set_freq(Device, Centre_Frequency);
   }
   uint32_t getCenterFreq(void) {
//n     return rtlsdr_get_center_freq(Device);  // (fast call)
      return Centre_Frequency;
   }
   int setFreqCorrection(int PPM) {
//n     return rtlsdr_set_freq_correction(Device, PPM); // [PPM] (Part-Per-Million)
     return 0;
   }
   int getFreqCorrection(void) {
//n    return rtlsdr_get_freq_correction(Device); // (fast call)
     return 0;
   }

#ifdef NEW_RTLSDR_LIB
   int setTunerBandwidth(int Bandwidth) {
//n     return rtlsdr_set_tuner_bandwidth(Device, Bandwidth);    // [Hz] a new (advanced) function
     return 0;
   }
   // int getTunerBandwidth(void)     { int Bandwidth; return rtlsdr_get_tuner_bandwidth(Device, &Bandwidth); return Bandwidth; }
   int getTunerBandwidths(int *Bandwidth=0) {
//n     return rtlsdr_get_tuner_bandwidths(Device, Bandwidth);
     return 0;
   }
#endif

   int getTunerGains(int *Gain=0)           {
//n     return rtlsdr_get_tuner_gains(Device, Gain);
     switch (GainMode) {
       case 2: //linearity
	     if (Gain != 0) {  // return value only if not 0
           memcpy(Gain, &Linearity_Gain_Table, GAIN_COUNT * sizeof(int));
		 }
		 return GAIN_COUNT;
       break;

       case 0: //auto
       //break;

       case 1: //manual
       //break;

       case 3: //sensitivity
       //break;

       default:
	     if (Gain != 0) {  // return value only if not 0
           memcpy(Gain, &Sensitivity_Gain_Table, GAIN_COUNT * sizeof(int));
		 }
		 return GAIN_COUNT;
       break;
     }
   }

#ifdef NEW_RTLSDR_LIB
   int getTunerStageGains(int Stage, int32_t *Gain, char *Description=0) {
//n     return rtlsdr_get_tuner_stage_gains(Device, Stage, Gain, Description);
     switch (Stage) {
       case 0: //LNA
		 Description = LNA_Gain_Name;
         memcpy(Gain, &LNA_Gain_Table, LNA_GAIN_COUNT * sizeof(int));
		 return LNA_GAIN_COUNT;
       break;

       case 1: //Mixer
		 Description = Mixer_Gain_Name;
         memcpy(Gain, &MIXER_Gain_Table, MIXER_GAIN_COUNT * sizeof(int));
		 return MIXER_GAIN_COUNT;
       break;

       case 2: //VGA
		 Description = VGA_Gain_Name;
         memcpy(Gain, &VGA_Gain_Table, VGA_GAIN_COUNT * sizeof(int));
		 return VGA_GAIN_COUNT;
       break;

       default:
         return 0;
       break;
     }
   }
   int setTunerStageGain(int Stage, int Gain) {
//n     return rtlsdr_set_tuner_stage_gain(Device, Stage, Gain);
     int i;
     switch (Stage) {
       case 0: //LNA
	     //search table for gain to find index
		 for (i = 0; i < LNA_GAIN_COUNT; i++) {
		   if (Gain <= LNA_Gain_Table[i]) {
		     break;
		   }
		 }
         airspy_set_lna_gain(Device, i); // index: 0..15
       break;

       case 1: //Mixer
	     //search table for gain to find index
		 for (i = 0; i < MIXER_GAIN_COUNT; i++) {
		   if (Gain <= Mixer_Gain_Table[i]) {
		     break;
		   }
		 }
         airspy_set_mixer_gain(Device, i); // index: 0..14
       break;

       case 2: //VGA
	     //search table for gain to find index
		 for (i = 0; i < VGA_GAIN_COUNT; i++) {
		   if (Gain <= VGA_Gain_Table[i]) {
		     break;
		   }
		 }
         airspy_set_vga_gain(Device, i); // index: 0..15
       break;

       default:
         return -1;
       break;
   }
#endif
   int setTunerGain(int Gain) {
//n     return rtlsdr_set_tuner_gain(Device, Gain);    // [0.1 dB]    set tuner gain when in manual mode
     int i;
     switch (GainMode) {
       case 0: //auto
       break;

       case 2: //linearity
	     //search table for gain to find index
		 for (i = 0; i < GAIN_COUNT; i++) {
		   if (Gain <= Linearity_Gain_Table[i]) {
		     break;
		   }
		 }
		 GainIndex = i; // remember index
         airspy_set_linearity_gain(Device, GainIndex); // index: 0..21
       break;

       case 1: //manual
       //break;

       case 3: //sensitivity
	     //search table for gain to find index
		 for (i = 0; i < GAIN_COUNT; i++) {
		   if (Gain <= Sensitivity_Gain_Table[i]) {
		     break;
		   }
		 }
		 GainIndex = i; // remember index
         airspy_set_sensitivity_gain(Device, GainIndex); // index: 0..21
       break;

       default:
       break;
     }
// printf("Gain %d %d\n", Gain, GainIndex);
	 return 0;
   }
   int getTunerGain(void)     {
//n     return rtlsdr_get_tuner_gain(Device);
     switch (GainMode) {
       case 0: //auto
	     return 0;
       break;

       case 2: //linearity
         return Linearity_Gain_Table[GainIndex]; // index: 0..21
       break;

       case 1: //manual
       //break;

       case 3: //sensitivity
         return Sensitivity_Gain_Table[GainIndex]; // index: 0..21
       break;

       default:
         return 0;
       break;
     }
   }

   // note: new gain modes are possible with the more advanced drivers: 2=Linearity, 3=Sensitivity
   int setTunerGainMode(int gMode=1) {  // defaut to manual=1
//n     return rtlsdr_set_tuner_gain_mode(Device, Manual);  // set radio-tuner gain mode: manual or automatic
     GainMode = gMode;
     switch (GainMode) {
       case 0: //auto
         airspy_set_lna_agc(Device, 1);
         airspy_set_mixer_agc(Device, 1);
         airspy_set_vga_gain(Device, 11); // index: 11 -> 31.2 dB
       break;

       case 2: //linearity
       //break;

       case 1: //manual
       //break;

       case 3: //sensitivity
       //break;

       default:
         airspy_set_lna_agc(Device, 0);
         airspy_set_mixer_agc(Device, 0);
       break;
     }
// printf("GainMode %d\n", GainMode);
     return 0;
   }
   int setTunerGainManual(int Manual=1) {
     return setTunerGainMode(Manual);  // set manual mode
   }
   int setTunerGainAuto(void) {
     return setTunerGainManual(0);     // set automatic mode
   }
   int setTestMode(int Test=1) {
//n     return rtlsdr_set_testmode(Device, Test);  // Enable/Disable test mode - a counter is send, not real data
     return 0;
   }
   int ResetBuffer(void) {
//n     return rtlsdr_reset_buffer(Device); }      // obligatory, the docs say, before you start reading
     return 0;
   }
   int setBiasTee(int On=1) {
//n     return rtlsdr_set_bias_tee(Device, On); }  // turn on or off the T-bias circuit to power extenal LNA: never use with DC-shorted antennas !
     return airspy_set_rf_bias(Device, On);
   }

   int getTunerSampleRates(uint32_t *Rates=0) {
     airspy_get_samplerates(Device, &SampleRates, 0); // get number of sample rates available
//     Rates = (uint32_t *) malloc(SampleRates * sizeof(uint32_t));
     if (SampleRates > MAX_SAMPLE_RATES) {
	   SampleRates = MAX_SAMPLE_RATES;  // only get what will fit
	 }  
     airspy_get_samplerates(Device, Rates, SampleRates);
     return SampleRates;
   }

   double getTime(void) const                                                 // read the system time at this very moment
#ifndef __MACH__ // _POSIX_TIMERS
   { struct timespec now; clock_gettime(RefClock, &now); return now.tv_sec + 1e-9*now.tv_nsec; }
#else                                                                         // for OSX, there is no clock_gettime()
   { struct timeval now; gettimeofday(&now, 0); return now.tv_sec + 1e-6*now.tv_usec; }
#endif

   int setSampleRate(uint32_t SampleRate) {
     SamplePeriod = 1.0/SampleRate;
     SampleTime_DMS=0.0001*0.0001;
//n     return rtlsdr_set_sample_rate(Device, SampleRate);// [samples-per-second]
     Sample_Rate = SampleRate;
     return airspy_set_samplerate(Device, Sample_Rate);  // can be an index or actual sample rate
   }
   uint32_t getSampleRate(void) {
//n     return rtlsdr_get_sample_rate(Device);
     return Sample_Rate;
   }
   int setSampleType(enum airspy_sample_type sample_type) {
     return airspy_set_sample_type(Device, sample_type);
   }
   int setPacking(uint8_t value) {
     return airspy_set_packing(Device, value);
   }
   
   int getDeviceIndexBySerial(const char *Serial) {
//n     return rtlsdr_get_index_by_serial(Serial);
     /* TBD
	for (i = 0; i < AIRSPY_MAX_DEVICE; i++) {
		result = airspy_open_sn(&devices[i], serial_number_val);
		if (result != AIRSPY_SUCCESS) {
			if(i == 0) {
				printf("airspy_open() board %d failed: %s (%d)\n",
						i+1, airspy_error_name(result), result);
			}
			break;
		}
	}
    */
     return 0;
   }

   int Open(uint32_t DeviceIndex=0, uint32_t Frequency=868000000, uint32_t SampleRate=2048000) { // open given device (by the index)
     Close();

	 // show airspy library version
     airspy_lib_version_t lib_version;
     airspy_lib_version(&lib_version);
     printf("AirSpy lib version: %d.%d.%d\n",lib_version.major_version, lib_version.minor_version, lib_version.revision);

     this->DeviceIndex = DeviceIndex;
//n     if(rtlsdr_open(&Device, DeviceIndex)<0) {    // open the RTLSDR device
	 /* TBD
     if ((DeviceIndex < 0) |(DeviceIndex >= AIRSPY_MAX_DEVICE)) {
       printf("Device Index out of range #%d\n", DeviceIndex);
	   Device=0; 
	   return -1;
     }

 	 int result, i;
     for (i = 0; i <= DeviceIndex; i++) {
	   result = airspy_open(&devices[i]);
	   if (result != AIRSPY_SUCCESS) {
	     if (i == 0) {
           return -1; // no devices found
         }
         break;
       }
     }
	 //now close devices not used
	 for (i) {
			airspy_close(devices[i]);
	 }

     if (i != DeviceIndex) {
     */
     if (airspy_open(&Device) != 0) {
       printf("Cannot open device #%d\n", DeviceIndex);
	   Device=0;
	   return -1;
     }
     if(setCenterFreq(Frequency) < 0) {                                                           // set the desired frequency
       printf("Cannot set the frequency %d for device #%d\n", Frequency, DeviceIndex);
     }
//     if(setSampleRate(1000000)<0) {                                                          // set the desired sample rate
     if(setSampleRate(SampleRate) < 0) {                                                          // set the desired sample rate
       printf("Cannot set the sample rate %d for device #%d\n", SampleRate, DeviceIndex);
     }
     if(setSampleType(AIRSPY_SAMPLE_INT16_IQ) < 0) {                                              // set the desired sample type
       printf("Cannot set the sample type %d for device #%d\n", AIRSPY_SAMPLE_INT16_IQ, DeviceIndex);
     }
     if(setPacking(0) < 0) {    // 0 -> no 12 bit packing                                         // set the desired packing mode, 12 or 16 bit
       printf("Cannot set the packing mode %d for device #%d\n", 0, DeviceIndex);
     }

     printf("RTLSDR::Open(%d,%d,%d) => %s, %8.3f MHz, %5.3f Msps\n",
       DeviceIndex, Frequency, SampleRate, getDeviceName(), 1e-6*getCenterFreq(), 1e-6*getSampleRate());

     Gains = getTunerGains(Gain);                                                               // get list of possible tuner gains
#ifdef NEW_RTLSDR_LIB
     for(Stages=0; Stages<8; Stages++) {
       StageGains[Stages]=getTunerStageGains(Stages, StageGain[Stages], StageName[Stages]);
       if(StageGains[Stages]<=0) {
         break;
       }
     }
#endif
     PrintGains();

#ifdef NEW_RTLSDR_LIB
     Bandwidths = getTunerBandwidths(Bandwidth);
     PrintBandwidths();
#endif

     SampleRates = getTunerSampleRates(Rates);                                                   // get list of possible tuner sample rates
     PrintSampleRates();

     if(ResetBuffer()<0) {                                                                       // reset the buffers (after the manual...)
       printf("Cannot reset buffer for device #%d\n", DeviceIndex); 
     }
     return 1;
   }

   void PrintGains(void) const {

#ifdef NEW_RTLSDR_LIB
     for(int Stage = 0; Stage < Stages; Stage++) {
       printf("RTLSDR::%s[%d] =", StageName[Stage], StageGains[Stage]);
       for(int Idx = 0; Idx < StageGains[Stage]; Idx++) {
         printf(" %+5.1f", 0.1 * StageGain[Stage][Idx]);
       }
       printf(" [dB]\n");
     }
#endif
     printf("RTLSDR::Gain[%d] =", Gains);
     for(int Idx = 0; Idx < Gains; Idx++) {
       printf(" %+5.1f", 0.1 * Gain[Idx]); 
     }
     printf(" [dB]\n");
   }

   void PrintBandwidths(void) const {
     printf("RTLSDR::Bandwidth[%d] =", Bandwidths);
     for(int Idx = 0; Idx < Bandwidths; Idx++) {
       printf(" %5.3f", 1e-6 * Bandwidth[Idx]);
     }
     printf(" [MHz]\n");
   }

   void PrintSampleRates(void) const {
     printf("RTLSDR::Rate[%d] =", SampleRates);
     for(int Idx = 0; Idx < SampleRates; Idx++) {
       printf(" %5.3f", 1e-6 * Rates[Idx]);
     }
     printf(" [MSps]\n");
   }

   double SampleTimeJitter(void) {
     return sqrt(SampleTime_DMS);
   }

   static void StaticCallback(unsigned char *Buffer, uint32_t Len, void *Contex) {        // callback that receives the data
     RTLSDR *This = (RTLSDR *)Contex; return This->ClassCallback(Buffer, Len);            // "This" points now to this class instance
   }
   void ClassCallback(unsigned char *Buffer, uint32_t Len) {                              // callback but already in this class instance
     Lock.Lock();
     int Samples = Len / 2;                                                                 // number of samples is half the buffer size
     BytesRead += Len;                                                                      // count number of bytes read
     double ReadTime = getTime();                                                           // read the time at this moment
/*
     uint32_t PrevSampleIdx=SampleIdxPipe[PipeWrite];                                     // previous SampleIdx
     PipeWrite++; if(PipeWrite>=PipeSize) PipeWrite=0;                                    // advance pipe write pointer
     double FirstSampleTime = SampleTimePipe[PipeRead];
     uint32_t FirstSampleIdx  = SampleIdxPipe[PipeRead];
     if(PipeWrite==PipeRead) { PipeRead++; if(PipeRead>=PipeSize) PipeRead=0; }
     SampleTimePipe[PipeWrite]=ReadTime;                                                  // ReadTime -> Pipe
     SampleIdxPipe[PipeWrite]=PrevSampleIdx+Samples;                                      // next SampleIdx -> Pipe
     double   SampleTimeDiff = ReadTime-FirstSampleTime;
     uint32_t SampleIdxDiff  = (PrevSampleIdx+Samples)-FirstSampleIdx;;
*/
     double AcqTime = Samples * SamplePeriod;    // time it took to acquire these samples
     double AverWeight = AcqTime / AverPeriod;   // ratio: acquisition period : averaging period

     int Ret = 0;
     if (Callback)
     { Ret=(*(Callback))(Buffer, Samples,                                   // buffer, number of samples
                         SampleTime - Samples * SamplePeriod, SamplePeriod, // SampleTime = time of the first sample, SamplePeriod = time period of one sample
                         CallbackContext);
     }
     if (Ret) {
	   CancelAsync();                                 // call the user callback, if it returns non-zero, then stop data acquisition
     }
     SampleTime += Samples * SamplePeriod;            // increment predicted time for this batch
     double TimeDiff = ReadTime - SampleTime;         // difference: time read now versus predicted time
     SampleTime += 0.125 * TimeDiff;                  // follow the ReadTime with weight 1/8 (a bit arbitrary...)
     SampleTime_DMS += AverWeight * (TimeDiff*TimeDiff - SampleTime_DMS); // integrate the difference RMS

     double PeriodDiff = (ReadTime - PrevTime) - AcqTime; // difference: measured time period to acquire this batch versus predicted time period
     SamplePeriod += AverWeight*(PeriodDiff/Samples);
     PrevTime = ReadTime;

     // printf("%14.3f (%+7.3f:%+7.3f ms): RTLSDR::Callback( , %d, ) => %10.1f (%10.1f) samples/sec, %6.3f ms\r",
     //        ReadTime, 1e3*TimeDiff, 1e3*PeriodDiff, Len, 1.0/SamplePeriod, SampleIdxDiff/SampleTimeDiff, 1e3*SampleTimeJitter() );
/*
     char Time[24]; AsciiTime_DDDDDHHMMSSFFF(Time, ReadTime);
     printf("%s (%+7.3f:%+7.3f ms): RTLSDR::Callback( , %d, ) => %10.1f samples/sec, %6.3f ms\r",
            Time, 1e3*TimeDiff, 1e3*PeriodDiff, Len, 1.0/SamplePeriod, 1e3*SampleTimeJitter() );
     fflush(stdout);
*/
     Lock.Unlock();
   }

   // read in async. mode, call Callback() for the data being received, block, wait and return when Callback() returns non-zero
   int ReadAsync(int (*Callback)(uint8_t *Buffer, int Samples, double SampleTime, double SamplePeriod, void *Contex)=0, void *Contex=0,
                 int Buffers=0, int BlockSize=0) {
     this->Callback = Callback; 
     StartTime=SampleTime=PrevTime=getTime();
     this->CallbackContext = Contex;
     // SampleTimePipe[0]=SampleTime; SampleIdxPipe[0]=0; PipeWrite=0; PipeRead=0;
//     return rtlsdr_read_async(Device, StaticCallback, this, Buffers, BlockSize);
     return 0; // do nothing for airspy for now
   }

   int CancelAsync(void) {
//n     return rtlsdr_cancel_async(Device);
     return airspy_stop_rx(Device);
   }

/*   // read directly given number of samples (remember to ResetBuffer() !)
   int Read(uint8_t *Buffer, int Samples) {
     Samples &= 0xFFFFFF00;               // number of samples must be a multiply of 256
     int BufferSize = 2 * Samples;
     int ReadSize = 0;
     if(rtlsdr_read_sync(Device, Buffer, BufferSize, &ReadSize)<0) {
	   return -1;
	 }
     return ReadSize / 2;
   }
*/
//   int Read(SampleBuffer<uint8_t> &Buffer, int Samples) {
//   int Read(SampleBuffer<uint8_t> &Buffer, int Samples, void* ctx) {
   int Read(SBUFF &Buffer, int Samples, void* ctx) {
     if(Buffer.Allocate(2, Samples) <= 0) {
       return 0;
     }
//     int ReadSamples=Read(Buffer.Data, Samples);
     int ReadSamples=Read(Buffer.Data, Samples, ctx);
     double Time = getTime();
     if (ReadSamples > 0) {
       Buffer.Full = ReadSamples * 2;
       Buffer.Rate = getSampleRate();
       Buffer.Freq = getCenterFreq();
       Buffer.Time = Time - (double)ReadSamples / Buffer.Rate;
     }
     // printf("RTLSDR::Read( , %d) => %d, %7.3fMHz %14.3fsec\n", Samples, ReadSamples, 1e-6*getCenterFreq(), Buffer.Time );
// printf("Samples %d\n", ReadSamples);
     return ReadSamples;
   }

	 static int StaticAirspyCallback(airspy_transfer_t* transfer) {        // callback that receives the data
     RTLSDR *This = (RTLSDR *)transfer->ctx; return  This->ClassAirspyCallback(transfer);            // "This" points now to this class instance
   }
   // airspy callback with 48K or 64K samples depending on sample type
//   int rx_callback(airspy_transfer_t* transfer) {
   int ClassAirspyCallback(airspy_transfer_t* transfer) {                              // callback but already in this class instance
     if (desiredSampleCount == 0) { // do nothing until count > 0
	   return 0;
	 }  
     int SampleType = transfer->sample_type;
//     int SamplesReceived = transfer->sample_count;
     int SamplesReceived = transfer->sample_count;
     SDATA* SrcBufferPtr = (SDATA*)transfer->samples;
// printf("Callback %d %d\n", SamplesReceived, desiredSampleCount);
     if (skipSampleCount > 0) {
       if (SamplesReceived > skipSampleCount) {
	     SamplesReceived -= skipSampleCount;
	     SrcBufferPtr += skipSampleCount * 2;
  	   skipSampleCount = 0;
       } else {
         skipSampleCount -= SamplesReceived;
         return 0;
       }
     }
     if (SamplesReceived > desiredSampleCount) {
       SamplesReceived = desiredSampleCount;
     }
     desiredSampleCount -= SamplesReceived;

     switch(SampleType) {
       case AIRSPY_SAMPLE_FLOAT32_IQ:
         break;

       case AIRSPY_SAMPLE_FLOAT32_REAL:
         break;

       case AIRSPY_SAMPLE_INT16_IQ:
         memcpy(DestBufferPtr, SrcBufferPtr, SamplesReceived*2*2);    // 2 for I,Q, 2 for 16 bit integer
/*         // for 8 bits only for testing. re-insert bias(128) for compatibility
         for (int i = 0; i < SamplesReceived * 2; i++) {
       //    *((uint8_t*)(DestBufferPtr)) = (uint8_t)((*((uint16_t*)(SrcBufferPtr))) >> 4) + 128; 
       //    DestBufferPtr++;
       //    ((uint16_t*&)SrcBufferPtr)++;
		   ((uint8_t*)DestBufferPtr)[i] = (uint8_t) ((((uint16_t *)SrcBufferPtr)[i] >> 4) + 128);
//		   ((uint8_t*)DestBufferPtr)[i] = 128 - rand() % 30 + 15;
		 }
*/
// printf("IQ %d %d\n", ((uint8_t*)DestBufferPtr)[0], ((uint8_t*)DestBufferPtr)[1]);
		 DestBufferPtr = DestBufferPtr + SamplesReceived * 2;
         break;

       case AIRSPY_SAMPLE_INT16_REAL:
         break;

       case AIRSPY_SAMPLE_UINT16_REAL:
         break;

       case AIRSPY_SAMPLE_RAW:
/*         if (packing_val) {
         } else {
         }
*/         break;

       default:
         break;
     }

     if (desiredSampleCount == 0) {  // all done ?
//     if (0 == 0) {  // all done ?
//       airspy_stop_rx(Device);
	   BufferWait.Signal();  // buffer available
     }
	 return 0;
   }

   // read directly given number of samples
//   int Read(uint8_t *Buffer, int Samples, void* ctx) {
   int Read(SDATA *Buffer, int Samples, void* ctx) {
     Samples &= 0xFFFFFF00;               // number of samples must be a multiple of 256

	 // set up callback variables
     DestBufferPtr = Buffer;
     skipSampleCount = 24000;    // skip initial samples
     desiredSampleCount = Samples;

	 if (airspy_is_streaming(Device) != AIRSPY_TRUE) {
	 // start the airspy async read process
//     int result = airspy_start_rx(Device, rx_callback, NULL);
     int result = airspy_start_rx(Device, (airspy_sample_block_cb_fn)StaticAirspyCallback, ctx);
     if( result != AIRSPY_SUCCESS ) {
       // printf("airspy_start_rx() failed: %s (%d)\n", airspy_error_name(result), result);
//       return -1;
       return result;
     }
	 }
     BufferWait.Lock();
	 // wait until buffer filled
//	 BufferWait.Wait();
	 int WaitResult = BufferWait.TimedWait(1000000);
     BufferWait.Unlock();
//     airspy_stop_rx(Device);
// printf("Go\n");
// printf("Go %d\n", Samples);
	 
	 if (WaitResult == 0) {
       return Samples;    // I and Q, each 2 bytes
	 } else {
 printf("Read time-out\n");
	   return -1;         // time out, something went wrong
     }	 
   }

} ;

// =================================================================================
