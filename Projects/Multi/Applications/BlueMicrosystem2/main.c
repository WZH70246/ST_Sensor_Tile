

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "MetaDataManager.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "HWAdvanceFeatures.h"
#include "oled.h"
#include "bmp.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


#define BLUEMSYS_N_BUTTON_PRESS 3
#define BLUEMSYS_CHECK_CALIBRATION ((uint32_t)0x12345678)


/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern int connected;
extern int Oled_Init;
extern uint8_t Need_Reinit;

/* Code for MotionAR integration - Start Section */
extern osx_MAR_output_t ActivityCode;
/* Code for MotionAR integration - End Section */

/* Code for MotionCP integration - Start Section */
extern osx_MCP_output_t CarryPositionCode;
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
extern osx_MGR_output_t GestureRecognitionCode;
/* Code for MotionGR integration - End Section */

#ifdef STM32_SENSORTILE
  #ifdef OSX_BMS_ENABLE_PRINTF
    extern TIM_HandleTypeDef  TimHandle;
    extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
  #endif /* OSX_BMS_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */

/* Code for MotionFX integration - Start Section */
extern uint32_t osx_mfx_license[3][4];
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
extern uint32_t osx_mar_license[3][4];
/* Code for MotionAR integration - End Section */

/* Code for MotionCP integration - Start Section */
extern uint32_t osx_mcp_license[3][4];
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
extern uint32_t osx_mgr_license[3][4];
/* Code for MotionGR integration - End Section */

/* Code for BlueVoice integration - Start Section */
extern uint32_t osx_bv_license[3][4];
/* Code for BlueVoice integration - End Section */

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
extern uint32_t osx_asl_license[3][4];
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

/* Exported Variables -------------------------------------------------------------*/

float sensitivity;
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

osxMFX_calibFactor magOffset; 

uint32_t ConnectionBleStatus  =0;

uint32_t ForceReCalibration    =0;
uint32_t FirstConnectionConfig =0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef    TimCCHandle;
TIM_HandleTypeDef    TimEnvHandle;
TIM_HandleTypeDef    TimAudioDataHandle;

uint8_t bdaddr[6];

uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;

/* Table with All the known osx License */
MDM_knownOsxLicense_t known_OsxLic[]={
  
  /* Code for MotionFX integration - Start Section */
  {OSX_MOTION_FX,"Motion","osxMotionFX x9/x6 v1.0.7"},
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  {OSX_MOTION_AR,"Motion","osxMotionAR v1.2.0"},
  /* Code for MotionAR integration - End Section */
  
  /* Code for MotionCP integration - Start Section */
  {OSX_MOTION_CP,"Motion","osxMotionCP v1.0.0"},
  /* Code for MotionCP integration - End Section */
  
  /* Code for MotionGR integration - Start Section */
  {OSX_MOTION_GR,"Motion","osxMotionGR v1.0.0"},
  /* Code for MotionGR integration - End Section */
  
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
  {OSX_ACOUSTIC_SL,"Acoustic","osxAcousticSL v1.1.0"},
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */
  
  /* Code for BlueVoice integration - Start Section */
  {OSX_AUDIO_BV,"Audio","osxBlueVoice v2.0.0"},
  /* Code for BlueVoice integration - End Section */
  
  {OSX_END,"LAST",""}/* THIS MUST BE THE LAST ONE */
};

extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];


extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];



/* Private variables ---------------------------------------------------------*/
static volatile int ButtonPressed        =0;
static volatile int MEMSInterrupt        =0;
static volatile uint32_t HCI_ProcessEvent=0;
static volatile uint32_t SendEnv         =0;
static volatile uint32_t SendAudioLevel  =0;
static volatile uint32_t SendAccGyroMag  =0;

/* Code for MotionFX integration - Start Section */
static volatile uint32_t Quaternion      =0;
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
static volatile uint32_t UpdateMotionAR  =0;
/* Code for MotionAR integration - End Section */

/* Code for MotionCP integration - Start Section */
static volatile uint32_t UpdateMotionCP  =0;
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
static volatile uint32_t UpdateMotionGR  =0;
/* Code for MotionGR integration - End Section */

/* Code for BlueVoice integration - Start Section */
static uint16_t num_byte_sent = 0;
/* Code for BlueVoice integration - End Section */

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
static volatile uint32_t SendAudioSourceLocalization=0;

extern volatile int32_t SourceLocationToSend;
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

static unsigned char isCal = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);

static unsigned char SaveCalibrationToMemory(uint32_t *MagnetoCalibration);
static unsigned char ResetCalibrationInMemory(uint32_t *MagnetoCalibration);
static unsigned char ReCallCalibrationFromMemory(uint32_t *MagnetoCalibration);

static void InitTimers(void);
static void SendEnvironmentalData(void);
static void SendEnvironmentalData_Oled(void);
static void MEMSCallback(void);
static void ReCalibration(void);
static void ButtonCallback(void);
static void SendMotionData(void);
static void SendAudioLevelData(void);

void AudioProcess(void);

/* Code for MotionFX integration - Start Section */
static void ComputeQuaternions(void);
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
static void ComputeMotionAR(void);
/* Code for MotionAR integration - End Section */

/* Code for MotionCP integration - Start Section */
static void ComputeMotionCP(void);
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
static void ComputeMotionGR(void);
/* Code for MotionGR integration - End Section */

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
static void SendAudioSourceLocalizationData(void);
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

uint32_t t_coin=0;
uint32_t Read_High_Count=0;
float High_Average=0,High_Sum=0;
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{ 
  HAL_Init();

  /* Configure the System clock */
  SystemClock_Config();
#ifdef STM32_SENSORTILE 
    
  InitTargetPlatform(TARGET_SENSORTILE); 
   //  
#endif /* STM32_NUCLEO */
  
  /* Init the Meta Data Manager */
  InitMetaDataManager((void *)known_OsxLic,MDM_DATA_TYPE_LIC,NULL);
  /* Enable all the osx Motion License found on Meta Data Manager */
  {
    int32_t Index=0;
    while(known_OsxLic[Index].LicEnum!=OSX_END) {
      MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[known_OsxLic[Index].LicEnum].Address;
      if(PayLoad->osxLicenseInitialized) {
        switch(known_OsxLic[Index].LicEnum) {
          
          /* Code for MotionFX integration - Start Section */
          case OSX_MOTION_FX:
            MCR_OSX_COPY_LICENSE_FROM_MDM(osx_mfx_license,PayLoad->osxLicense);
            MotionFX_License_init(PayLoad);
          break;
          /* Code for MotionFX integration - End Section */
          
          /* Code for MotionAR integration - Start Section */
          case OSX_MOTION_AR:
            MCR_OSX_COPY_LICENSE_FROM_MDM(osx_mar_license,PayLoad->osxLicense);
            MotionAR_License_init(PayLoad);
          break;
          /* Code for MotionAR integration - End Section */
         
          /* Code for MotionCP integration - Start Section */
          case OSX_MOTION_CP:
            MCR_OSX_COPY_LICENSE_FROM_MDM(osx_mcp_license,PayLoad->osxLicense);
            MotionCP_License_init(PayLoad);
          break;
          /* Code for MotionCP integration - End Section */
          
          /* Code for MotionGR integration - Start Section */
          case OSX_MOTION_GR:
            MCR_OSX_COPY_LICENSE_FROM_MDM(osx_mgr_license,PayLoad->osxLicense);
            MotionGR_License_init(PayLoad);
          break;
          /* Code for MotionGR integration - End Section */
          
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
          case OSX_ACOUSTIC_SL:
            MCR_OSX_COPY_LICENSE_FROM_MDM(osx_asl_license,PayLoad->osxLicense);
            AcousticSL_License_init(PayLoad);
          break;
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */
          
          /* Code for BlueVoice integration - Start Section */
          case OSX_AUDIO_BV:
            MCR_OSX_COPY_LICENSE_FROM_MDM(osx_bv_license,PayLoad->osxLicense);
            AudioBV_License_init(PayLoad);
          break;
          /* Code for BlueVoice integration - End Section */

          default:
            /* Only for removing the GCC warning */
            OSX_BMS_PRINTF("Should never reach this point...\r\n");
            break;
        }
#ifdef OSX_BMS_LICENSE_H_FILE
      } else {
        switch(known_OsxLic[Index].LicEnum) {
          
          /* Code for MotionFX integration - Start Section */
          case OSX_MOTION_FX:
            MotionFX_License_init(PayLoad);
          break;
          /* Code for MotionFX integration - End Section */
          
          /* Code for MotionAR integration - Start Section */
          case OSX_MOTION_AR:
            MotionAR_License_init(PayLoad);
          break;
          /* Code for MotionAR integration - End Section */
         
          /* Code for MotionCP integration - Start Section */
          case OSX_MOTION_CP:
            MotionCP_License_init(PayLoad);
          break;
          /* Code for MotionCP integration - End Section */
          
          /* Code for MotionGR integration - Start Section */
          case OSX_MOTION_GR:
            MotionGR_License_init(PayLoad);
          break;
          /* Code for MotionGR integration - End Section */
          
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
          case OSX_ACOUSTIC_SL:
            AcousticSL_License_init(PayLoad);
          break;
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */
          
          /* Code for BlueVoice integration - Start Section */
          case OSX_AUDIO_BV:
            AudioBV_License_init(PayLoad);
          break;
          /* Code for BlueVoice integration - End Section */
        }
      }
#else /* OSX_BMS_LICENSE_H_FILE */
      }  
#endif /* OSX_BMS_LICENSE_H_FILE */
      Index++;
    }
  }
  
  OSX_BMS_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"

#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (openstm32)\r\n"
#endif
         "\tSend Every %4dmS %d Short precision Quaternions\r\n"
         "\tSend Every %4dmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4dmS Acc/Gyro/Magneto\r\n"
         "\tSend Every %4dmS dB noise\r\n\n",
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         QUAT_UPDATE_MUL_10MS*10,SEND_N_QUATERNIONS,
         ENV_UPDATE_MUL_100MS * 100,
         DEFAULT_uhCCR4_Val/10,
         MICS_DB_UPDATE_MUL_10MS * 10);

#ifdef OSX_BMS_DEBUG_CONNECTION
  OSX_BMS_PRINTF("Debug Connection         Enabled\r\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */

#ifdef OSX_BMS_DEBUG_NOTIFY_TRAMISSION
  OSX_BMS_PRINTF("Debug Notify Trasmission Enabled\r\n");
#endif /* OSX_BMS_DEBUG_NOTIFY_TRAMISSION */

  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();  

  if(TargetBoardFeatures.HWAdvanceFeatures) {
    InitHWFeatures();
  }

  /* Set Accelerometer Full Scale to 2G */
  Set2GAccelerometerFullScale();

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);

  /* initialize timers */
  InitTimers();

  /* Control if the calibration is already available in memory */
  if(MDM_LicTable[OSX_MOTION_FX].Address) {
    MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_FX].Address;
    if(PayLoad->osxLicenseInitialized) {
      ReCallCalibrationFromMemory(PayLoad->ExtraData);    
	}
  }

  /* Infinite loop */
  while (1){ 
    /* Led Blinking when there is not a client connected */
    if(!connected) {
//            OLED_Init();//初始化OLED
//                 OLED_Clear(); 
          
      if(!TargetBoardFeatures.LedStatus) {
        if(!(HAL_GetTick()&0x3FF)) {
          
          //OLED_SCLK_Clr();
        if(Oled_Init>=+0)//检测初始化是否完成
          {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
             OLED_Init();
             OLED_Clear();
             OLED_ShowCHinese(0,0,0);//充
        OLED_ShowCHinese(16,0,1);//电
        OLED_ShowString(32,0,":",16);
        
       OLED_ShowCHinese(60,0,1);//电
        OLED_ShowCHinese(76,0,2);//量
        //OLED_ShowString(92,0,":",16);
        OLED_ShowString(115,0,"%  ",16);
        
        OLED_ShowCHinese(0,2,3);//温
        OLED_ShowCHinese(16,2,5);//度
        OLED_ShowString(32,2,":",16);
        
         OLED_ShowCHinese(0,4,4);//湿
        OLED_ShowCHinese(16,4,5);//度
        OLED_ShowString(32,4,":",16);
        
        OLED_ShowCHinese(0,6,6);//海
        OLED_ShowCHinese(16,6,7);//拔
        OLED_ShowString(32,6,":",16);             
           }// 
        if(Need_Reinit==TRUE)
        {
          HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_SET);
          OLED_Init();
          OLED_Clear();
          OLED_ShowCHinese(0,0,0);//充
        OLED_ShowCHinese(16,0,1);//电
        OLED_ShowString(32,0,":",16);
        
       OLED_ShowCHinese(60,0,1);//电
        OLED_ShowCHinese(76,0,2);//量
        //OLED_ShowString(92,0,":",16);
        OLED_ShowString(115,0,"%  ",16);
        
        OLED_ShowCHinese(0,2,3);//温
        OLED_ShowCHinese(16,2,5);//度
        OLED_ShowString(32,2,":",16);
        
         OLED_ShowCHinese(0,4,4);//湿
        OLED_ShowCHinese(16,4,5);//度
        OLED_ShowString(32,4,":",16);
        
        OLED_ShowCHinese(0,6,6);//海
        OLED_ShowCHinese(16,6,7);//拔
        OLED_ShowString(32,6,":",16); 
        Need_Reinit=FALSE;
        }
            //OLED_Clear();	
	       // LedOnTargetPlatform();
           TargetBoardFeatures.LedStatus=1;

        SendEnvironmentalData_Oled(); 
        }
      }
      else {
        if(!(HAL_GetTick()&0x3F)) {
        //  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, GPIO_PIN_RESET);
         LedOffTargetPlatform();
  //       TargetBoardFeatures.LedStatus=0;
        }
      }
    }
    if(set_connectable){

	  MDM_PayLoadLic_t *PayLoad;
      /* Initializes the osx libraries if there is a valid license */
      
      /* Code for MotionFX integration - Start Section */    
      /* Initialize MotionFX library */
      PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_FX].Address;
      if(PayLoad) {
         if((PayLoad->osxLicenseInitialized) & (TargetBoardFeatures.osxMotionFXIsInitalized==0)){
          MotionFX_manager_init();
          MotionFX_manager_start_9X();
        }
      }
      /* Code for MotionFX integration - End Section */
      
      /* Code for MotionAR integration - Start Section */
      /* Initialize MotionAR Library */
      PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_AR].Address;
      if(PayLoad) {
        if((PayLoad->osxLicenseInitialized) & (TargetBoardFeatures.osxMotionARIsInitalized==0)){
          MotionAR_manager_init();
        }
      }
      /* Code for MotionAR integration - End Section */
      
      /* Code for MotionCP integration - Start Section */
      /* Initialize MotionCP Library */
      PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_CP].Address;
      if(PayLoad) {
        if((PayLoad->osxLicenseInitialized) & (TargetBoardFeatures.osxMotionCPIsInitalized==0)){
          MotionCP_manager_init();
        }
      }
      /* Code for MotionCP integration - End Section */

      /* Code for MotionGR integration - Start Section */
      /* Initialize MotionGR Library */
      PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_GR].Address;
      if(PayLoad) {
        if((PayLoad->osxLicenseInitialized) & (TargetBoardFeatures.osxMotionGRIsInitalized==0)){
          MotionGR_manager_init();
        }
      }
      /* Code for MotionGR integration - End Section */
      
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
      /* Initialize AcousticSL Library */
      PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_ACOUSTIC_SL].Address;
      if(PayLoad) {
        if((PayLoad->osxLicenseInitialized) & (TargetBoardFeatures.osxAcousticSLIsInitalized==0)){
          AcousticSL_Manager_init();
        }
      }
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

      /* Code for BlueVoice integration - Start Section */
      /* Initialize BlueVoice Library */
      PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_AUDIO_BV].Address;
      if(PayLoad) {
        if((PayLoad->osxLicenseInitialized) & (TargetBoardFeatures.osxAudioBVIsInitalized==0)){
          AudioBV_Manager_init();
        }
      }
      /* Code for BlueVoice integration - End Section */

      if(NecessityToSaveMetaDataManager) {
        uint32_t Success = EraseMetaDataManager();
        if(Success) {
          SaveMetaDataManager();
        }
      }

      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;   
    }//设置蓝牙
    
    /* Handle Interrupt from MEMS */
    if(MEMSInterrupt) {
      MEMSCallback();
      MEMSInterrupt=0;
    }

    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;       
    }

    /* Handle Re-Calibration */
    if(ForceReCalibration) {
      ForceReCalibration=0;
      ReCalibration();
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      HCI_Process();
    }

    /* Environmental Data */
    if(SendEnv) {
         OLED_Clear();
         Need_Reinit=TRUE;
      SendEnv=0;
       SendEnvironmentalData(); 
       LedOffTargetPlatform();
    }
    
    /* Mic Data */
    if (SendAudioLevel) {
      SendAudioLevel = 0;
      SendAudioLevelData();
    }

    /* Motion Data */
    if(SendAccGyroMag) {
      SendAccGyroMag=0;
      SendMotionData();
    }

    /* Code for MotionFX integration - Start Section */
    if(Quaternion) {
      Quaternion=0;
      ComputeQuaternions();
    }
    /* Code for MotionFX integration - End Section */

    /* Code for MotionAR integration - Start Section */
    if(UpdateMotionAR) {
      UpdateMotionAR=0;
      ComputeMotionAR();
    }
    /* Code for MotionAR integration - End Section */
    
    /* Code for MotionCP integration - Start Section */
    if(UpdateMotionCP) {
      UpdateMotionCP=0;
      ComputeMotionCP();
    }
    /* Code for MotionCP integration - End Section */

    /* Code for MotionGR integration - Start Section */
    if(UpdateMotionGR) {
      UpdateMotionGR=0;
      ComputeMotionGR();
    }
    /* Code for MotionGR integration - End Section */
    
    /* Code for BlueVoice integration - Start Section */
    /* BlueVoice Data */
    if(SendBlueVoice){
      osx_BlueVoice_SendData(&num_byte_sent);
      SendBlueVoice = 0;
    }     
    /* Code for BlueVoice integration - End Section */
    
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
    /* Audio Source Localization Data */
    if (SendAudioSourceLocalization)
    {
      SendAudioSourceLocalization = 0;
      SendAudioSourceLocalizationData();
    }
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */ 

    /* Wait for Event */
    __WFI();
      
  }
}

/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
void Set2GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-2g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,2.0f);
  
  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}
/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
void Set4GAccelerometerFullScale(void)
{
  
  /* Set Full Scale to +/-4g */
  BSP_ACCELERO_Set_FS_Value(TargetBoardFeatures.HandleAccSensor,4.0f);

  /* Read the Acc Sensitivity */
  BSP_ACCELERO_Get_Sensitivity(TargetBoardFeatures.HandleAccSensor,&sensitivity);
  sensitivity_Mul = sensitivity* ((float) FROM_MG_TO_G);
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  
  /* Code for MotionFX and MotionGR integration - Start Section */
  /* TIM1_CH1 toggling with frequency = 100Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));

    /* Code for MotionFX integration - Start Section */
    if ((W2ST_CHECK_CONNECTION(W2ST_CONNECT_QUAT)) | (W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC))) {
      Quaternion=1;
    }
    /* Code for MotionFX integration - End Section */

    /* Code for MotionGR integration - Start Section */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GR)) {
      UpdateMotionGR=1;
    }
    /* Code for MotionGR integration - End Section */
  }
  /* Code for MotionFX and MotionGR integration - End Section */

  /* Code for MotionCP integration - Start Section */
  /* TIM1_CH2 toggling with frequency = 50Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_CP)) {
      UpdateMotionCP=1;
    }
  }
  /* Code for MotionCP integration - End Section */

  /* Code for MotionAR integration - Start Section */
  /* TIM1_CH3 toggling with frequency = 16Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
    
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR)) {
      UpdateMotionAR=1;
    }
  }
  /* Code for MotionAR integration - End Section */

  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    SendAccGyroMag=1;
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimEnvHandle)) {
    /* Environmental */
    SendEnv=1;
#ifdef STM32_SENSORTILE
#ifdef OSX_BMS_ENABLE_PRINTF
    } else if(htim == (&TimHandle)) {
      CDC_TIM_PeriodElapsedCallback(htim);
#endif /* OSX_BMS_ENABLE_PRINTF */
#endif /* STM32_SENSORTILE */
  } else if(htim == (&TimAudioDataHandle)) {
    /* Mic Data */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
      SendAudioLevel=1;
    
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION    
    /* Audio Source Localization Data */
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SL))
      SendAudioSourceLocalization= 1;
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */
  }
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  /* Only if connected */
  if(connected) {
    static uint32_t HowManyButtonPress=0;
    static uint32_t tickstart=0;
    uint32_t tickstop;

    if(!tickstart)
      tickstart = HAL_GetTick();

    tickstop = HAL_GetTick();

    if((tickstop-tickstart)>2000) {
      HowManyButtonPress=0;
      tickstart=tickstop;
    }

    if(MDM_LicTable[OSX_MOTION_FX].Address) {
      MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_FX].Address;
      if(PayLoad->osxLicenseInitialized) {
        if((HowManyButtonPress+1)==BLUEMSYS_N_BUTTON_PRESS){
          ForceReCalibration=1;
          HowManyButtonPress=0;
        } else {
          HowManyButtonPress++;
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
             BytesToWrite = sprintf((char *)BufferToWrite, "%ld in %ldmS Reset Calib\r\n",3-HowManyButtonPress,2000-(tickstop-tickstart));
             Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("%ld in %ldmS Reset Calib\r\n",3-HowManyButtonPress,2000-(tickstop-tickstart));
          }
        }
      }
    } else {
      OSX_BMS_PRINTF("UserButton Pressed\r\n");
    }
  }
}

/**
  * @brief  Reset the magneto calibration 
  * @param  None
  * @retval None
  */
static void ReCalibration(void)
{
  /* Only if connected */
  if(connected) {
    /* Reset the Compass Calibration */
    isCal=0;

    /* Notifications of Compass Calibration */
    Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);

    /* Reset the Calibration */
    osx_MotionFX_compass_forceReCalibration();
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite, "Force ReCalibration\n\r");
       Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("Force ReCalibration\n\r");
    {
      MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_FX].Address;
      ResetCalibrationInMemory(PayLoad->ExtraData);
    }

    /* Reset Calibation offset */
    magOffset.magOffX = magOffset.magOffY= magOffset.magOffZ=0;

    /* Switch off the LED */    
    LedOffTargetPlatform();
  }
}

/**
  * @brief  Send Notification where there is a interrupt from MEMS
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  uint8_t stat = 0;

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
    /* Check if the interrupt is due to Free Fall */
    BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_FREE_FALL);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    /* Check if the interrupt is due to Double Tap */
    BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_DOUBLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    /* Check if the interrupt is due to Single Tap */
    BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_SINGLE_TAP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    BSP_ACCELERO_Get_Wake_Up_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_WAKE_UP);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    /* Check if the interrupt is due to Tilt */
    BSP_ACCELERO_Get_Tilt_Detection_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEvent_Notify(ACC_TILT);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    /* Check if the interrupt is due to 6D Orientation */
    BSP_ACCELERO_Get_6D_Orientation_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      AccEventType Orientation = GetHWOrientation6D();
      AccEvent_Notify(Orientation);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    /* Check if the interrupt is due to Pedometer */
    BSP_ACCELERO_Get_Pedometer_Status_Ext(TargetBoardFeatures.HandleAccSensor,&stat);
    if(stat) {
      uint16_t StepCount = GetStepHWPedometer();
      AccEvent_Notify(StepCount);
    }
  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  SensorAxes_t ACC_Value;
  SensorAxes_t GYR_Value;
  SensorAxes_t MAG_Value;

  /* Read the Acc values */
  BSP_ACCELERO_Get_Axes(TargetBoardFeatures.HandleAccSensor,&ACC_Value);

  /* Read the Magneto values */
  BSP_MAGNETO_Get_Axes(TargetBoardFeatures.HandleMagSensor,&MAG_Value);

  /* Read the Gyro values */
  BSP_GYRO_Get_Axes(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);

  AccGyroMag_Update(&ACC_Value,&GYR_Value,&MAG_Value);
}

/* Code for MotionFX integration - Star Section */
/* @brief  osxMotionFX Working function
 * @param  None
 * @retval None
 */
static void ComputeQuaternions(void)
{
  static SensorAxes_t quat_axes[SEND_N_QUATERNIONS];
  static int32_t calibIndex =0;
  static int32_t CounterFX  =0;
  static int32_t CounterEC  =0;
  SensorAxes_t ACC_Value;
  SensorAxesRaw_t ACC_Value_Raw;
  SensorAxes_t GYR_Value;
  SensorAxes_t MAG_Value;

  /* Increment the Counter */
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC)) {
    CounterEC++;
  } else {
    CounterFX++;
  }

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);

  /* Read the Magneto values */
  BSP_MAGNETO_Get_Axes(TargetBoardFeatures.HandleMagSensor,&MAG_Value);

  /* Read the Gyro values */
  BSP_GYRO_Get_Axes(TargetBoardFeatures.HandleGyroSensor,&GYR_Value);

  MotionFX_manager_run(ACC_Value_Raw,GYR_Value,MAG_Value);
      
  /* Check if is calibrated */
  if(isCal!=0x01){
    /* Run Compass Calibration @ 25Hz */
    calibIndex++;
    if (calibIndex == 4){
      calibIndex = 0;
      ACC_Value.AXIS_X = (int32_t)(ACC_Value_Raw.AXIS_X * sensitivity);
      ACC_Value.AXIS_Y = (int32_t)(ACC_Value_Raw.AXIS_Y * sensitivity);
      ACC_Value.AXIS_Z = (int32_t)(ACC_Value_Raw.AXIS_Z * sensitivity);
      osx_MotionFX_compass_saveAcc(ACC_Value.AXIS_X,ACC_Value.AXIS_Y,ACC_Value.AXIS_Z);	/* Accelerometer data ENU systems coordinate	*/
      osx_MotionFX_compass_saveMag(MAG_Value.AXIS_X,MAG_Value.AXIS_Y,MAG_Value.AXIS_Z);	/* Magnetometer  data ENU systems coordinate	*/            
      osx_MotionFX_compass_run();

      /* Control the calibration status */
      isCal = osx_MotionFX_compass_isCalibrated();
      if(isCal == 0x01){
        if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
          BytesToWrite = sprintf((char *)BufferToWrite, "Compass Calibrated\n\r");
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          OSX_BMS_PRINTF("Compass Calibrated\n\r");
        }

        /* Get new magnetometer offset */
        osx_MotionFX_getCalibrationData(&magOffset);

        /* Save the calibration in Memory */
        {
          MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OSX_MOTION_FX].Address;
          SaveCalibrationToMemory(PayLoad->ExtraData);
        }

        /* Switch on the Led */
        LedOnTargetPlatform();

        /* Notifications of Compass Calibration */
        Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
      }
    }
  }else {
    calibIndex=0;
  }

  /* Read the quaternions */
  osxMFX_output *MotionFX_Engine_Out = MotionFX_manager_getDataOUT();

  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC)) {
    /* E-Compass Updated every 0.1 Seconds*/
    if(CounterEC==10) {
      uint16_t Angle = (uint16_t)trunc(100*MotionFX_Engine_Out->heading_9X);
      CounterEC=0;
      ECompass_Update(Angle);
    }
  } else {
    int32_t QuaternionNumber = (CounterFX>SEND_N_QUATERNIONS) ? (SEND_N_QUATERNIONS-1) : (CounterFX-1);

    /* Scaling quaternions data by a factor of 10000
      (Scale factor to handle float during data transfer BT) */

    /* Save the quaternions values */
    if(MotionFX_Engine_Out->quaternion_9X[3] < 0){
      quat_axes[QuaternionNumber].AXIS_X = (int32_t)(MotionFX_Engine_Out->quaternion_9X[0] * (-10000));
      quat_axes[QuaternionNumber].AXIS_Y = (int32_t)(MotionFX_Engine_Out->quaternion_9X[1] * (-10000));
      quat_axes[QuaternionNumber].AXIS_Z = (int32_t)(MotionFX_Engine_Out->quaternion_9X[2] * (-10000));
    } else {
      quat_axes[QuaternionNumber].AXIS_X = (int32_t)(MotionFX_Engine_Out->quaternion_9X[0] * 10000);
      quat_axes[QuaternionNumber].AXIS_Y = (int32_t)(MotionFX_Engine_Out->quaternion_9X[1] * 10000);
      quat_axes[QuaternionNumber].AXIS_Z = (int32_t)(MotionFX_Engine_Out->quaternion_9X[2] * 10000);
    }
      
    /* Every QUAT_UPDATE_MUL_10MS*10 mSeconds Send Quaternions informations via bluetooth */
    if(CounterFX==QUAT_UPDATE_MUL_10MS){
      Quat_Update(quat_axes);
      CounterFX=0;
    }
  }
}
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
/**
  * @brief  osxMotionAR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionAR(void)
{
  static osx_MAR_output_t ActivityCodeStored = OSX_MAR_NOACTIVITY;
  SensorAxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);

  MotionAR_manager_run(ACC_Value_Raw);

  if(ActivityCodeStored!=ActivityCode){
    ActivityCodeStored = ActivityCode;

    ActivityRec_Update(ActivityCode);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: AR=%d\r\n",ActivityCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Sending: AR=%d\r\n",ActivityCode);
    }
  }
}
/* Code for MotionAR integration - End Section */

/* Code for MotionCP integration - Start Section */
/**
  * @brief  osxMotionCP Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionCP(void)
{  
  static osx_MCP_output_t CarryPositionCodeStored = OSX_MCP_UNKNOWN;
  SensorAxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);
  MotionCP_manager_run(ACC_Value_Raw);

  if(CarryPositionCodeStored!=CarryPositionCode){
    CarryPositionCodeStored = CarryPositionCode;
    CarryPosRec_Update(CarryPositionCode);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: CP=%d\r\n",CarryPositionCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Sending: CP=%d\r\n",CarryPositionCode);
    }
  }
}
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
/**
  * @brief  osxMotionGR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionGR(void)
{
  static osx_MGR_output_t GestureRecognitionCodeStored = OSX_MGR_NOGESTURE;
  SensorAxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  BSP_ACCELERO_Get_AxesRaw(TargetBoardFeatures.HandleAccSensor,&ACC_Value_Raw);
  MotionGR_manager_run(ACC_Value_Raw);

  if(GestureRecognitionCodeStored!=GestureRecognitionCode){
    GestureRecognitionCodeStored = GestureRecognitionCode;
    GestureRec_Update(GestureRecognitionCode);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: GR=%d\r\n",GestureRecognitionCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Sending: GR=%d\r\n",GestureRecognitionCode);
    }
  }
}
/* Code for MotionGR integration - End Section */

/**
* @brief  User function that is called when 1 ms of PDM data is available.
* @param  none
* @retval None
*/
void AudioProcess(void)
{
  int32_t i;
  int32_t NumberMic;
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)) {
    for(i = 0; i < 16; i++){
      for(NumberMic=0;NumberMic<AUDIO_CHANNELS;NumberMic++) {
        RMS_Ch[NumberMic] += (float)((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic] * ((int16_t)PCM_Buffer[i*AUDIO_CHANNELS+NumberMic]));
      }
    }
  }
}

/**
  * @brief  Send Audio Level Data (Ch1) to BLE
  * @param  None
  * @retval None
  */
static void SendAudioLevelData(void)
{
  int32_t NumberMic;
  uint16_t DBNOISE_Value_Ch[AUDIO_CHANNELS];
  
  for(NumberMic=0;NumberMic<AUDIO_CHANNELS;NumberMic++) {
    DBNOISE_Value_Ch[NumberMic] = 0;

    RMS_Ch[NumberMic] /= (16.0f*MICS_DB_UPDATE_MUL_10MS*10);

    DBNOISE_Value_Ch[NumberMic] = (uint16_t)((120.0f - 20 * log10f(32768 * (1 + 0.25f * (AUDIO_VOLUME_VALUE /*AudioInVolume*/ - 4))) + 10.0f * log10f(RMS_Ch[NumberMic])) * 0.3f + DBNOISE_Value_Old_Ch[NumberMic] * 0.7f);
    DBNOISE_Value_Old_Ch[NumberMic] = DBNOISE_Value_Ch[NumberMic];
    RMS_Ch[NumberMic] = 0.0f;
  }
  
  AudioLevel_Update(DBNOISE_Value_Ch);
}

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION

/**
  * @brief  Send Audio Source Localization Data to BLE
  * @param  None
  * @retval None
  */
void SendAudioSourceLocalizationData(void)
{
  AudioSourceLocalization_Update(SourceLocationToSend);
}

#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

/**
* @brief  Half Transfer user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  /*for L4 PDM to PCM conversion is performed in hardware by DFSDM peripheral*/
  
#ifdef USE_STM32F4XX_NUCLEO
    BSP_AUDIO_IN_PDMToPCM(PDM_Buffer, PCM_Buffer);
#endif /* USE_STM32F4XX_NUCLEO */
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
  {
      AudioProcess();
  }
  
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SL))
    AudioProcess_SL();
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */
  
  /* Code for BlueVoice integration - Start Section */
  if(((W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_AUDIO))!=0) & ((W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_SYNC))!=0))
  {
    AudioProcess_BV();
  }
  /* Code for BlueVoice integration - End Section */
}

/**
* @brief  Transfer Complete user callback, called by BSP functions.
* @param  None
* @retval None
*/
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  /*for L4 PDM to PCM conversion is performed in hardware by DFSDM peripheral*/
  
#ifdef USE_STM32F4XX_NUCLEO
    BSP_AUDIO_IN_PDMToPCM(PDM_Buffer, PCM_Buffer);
#endif /* USE_STM32F4XX_NUCLEO */
  
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL))
  {
      AudioProcess();
  }
  
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_SL))
    AudioProcess_SL();
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */
  
  /* Code for BlueVoice integration - Start Section */
  if(((W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_AUDIO))!=0) & ((W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_SYNC))!=0))
  {
    AudioProcess_BV();
  }
  /* Code for BlueVoice integration - End Section */
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  uint8_t Status;
  /* Notifications of Compass Calibration status*/
  if(FirstConnectionConfig) {
    Config_Notify(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
    FirstConnectionConfig=0;   
    /* Switch on/off the LED according to calibration */
    if(isCal){
      LedOnTargetPlatform();
    } else {
      LedOffTargetPlatform();
    }
  }
#ifdef STM32_SENSORTILE
  /* Battery Informations */
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT)) {
    GG_Update();
  }
#endif /* STM32_SENSORTILE */
  /* Pressure,Humidity, and Temperatures*/
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV)) {
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;

    if(TargetBoardFeatures.HandlePressSensor) {
      if(BSP_PRESSURE_IsInitialized(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
        BSP_PRESSURE_Get_Press(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
        PressToSend=intPart*100+decPart;
      }
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      if(BSP_HUMIDITY_IsInitialized(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
        BSP_HUMIDITY_Get_Hum(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        HumToSend = intPart*10+decPart;
      }
    }

    if(TargetBoardFeatures.NumTempSensors==2) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart; 
      }

      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp2ToSend = intPart*10+decPart;
      }
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
      }
    }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend-48,Temp1ToSend-33);//最终的环境数据上传步骤
  }
}


//用于未连接状态下的环境数据读取

static void SendEnvironmentalData_Oled(void)
{
  uint8_t Status;
  
  #ifdef STM32_SENSORTILE
  /* Battery Informations */
    GG_Update_Oled();
#endif /* STM32_SENSORTILE */

  /* Pressure,Humidity, and Temperatures*/
  if(1) {
    float    SensorValue;
    int32_t  PressToSend=0;
    uint16_t HumToSend=0;
    int16_t  Temp2ToSend=0,Temp1ToSend=0;
    int32_t  decPart, intPart;
    
    int16_t  humi_1,humi_2,temp1_1,temp1_2,mbar_1,mbar_2,High_1,High_2;
    
    float    High,Temp2,Press;
    
    if(TargetBoardFeatures.HandlePressSensor) {
      if(BSP_PRESSURE_IsInitialized(TargetBoardFeatures.HandlePressSensor,&Status)==COMPONENT_OK) {
        BSP_PRESSURE_Get_Press(TargetBoardFeatures.HandlePressSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
        PressToSend=intPart*100+decPart;
      }
    }

    if(TargetBoardFeatures.HandleHumSensor) {
      if(BSP_HUMIDITY_IsInitialized(TargetBoardFeatures.HandleHumSensor,&Status)==COMPONENT_OK){
        BSP_HUMIDITY_Get_Hum(TargetBoardFeatures.HandleHumSensor,(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        HumToSend = intPart*10+decPart;
      }
    }

    if(TargetBoardFeatures.NumTempSensors==2) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart; 
      }

      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[1],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[1],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp2ToSend = intPart*10+decPart;
      }
    } else if(TargetBoardFeatures.NumTempSensors==1) {
      if(BSP_TEMPERATURE_IsInitialized(TargetBoardFeatures.HandleTempSensors[0],&Status)==COMPONENT_OK){
        BSP_TEMPERATURE_Get_Temp(TargetBoardFeatures.HandleTempSensors[0],(float *)&SensorValue);
        MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
        Temp1ToSend = intPart*10+decPart;
      }
    }
    humi_1=HumToSend/10;
    humi_2=HumToSend%10;
    temp1_1=Temp1ToSend/10;
    temp1_2=Temp1ToSend%10;
    
    mbar_1=PressToSend/100;
    mbar_2=PressToSend%100;
    
    Temp2=(float)Temp2ToSend/10;
    Press=(float)PressToSend/1000;//单位转化为kPa
    
    
 //   High=8000*((1+(Temp2/273))/Press);//h≈8000（1+t/273）/P（m/hPa）
    High=((6357*1000*(1-pow((Press/101.3),1/5.256)))/(6357*0.0255-1+pow((Press/101.3),1/5.256)));//海拔转换公式
    High_Sum=High_Sum+High;
    Read_High_Count++;
    if(Read_High_Count==5)
    {
      High_Average=High_Sum/5;
      Read_High_Count=0;
      High_Sum=0;
      High_Average*=10;        //得到三位整数
    }//求10次数据的平均值
    
//    High=3.1415;
                    
    High_1=(int)High_Average/10;
    High_2=(int)High_Average%10;
    
     OLED_ShowNum(40,4,humi_1,3,16);   //湿度显示
     OLED_ShowString(64,4,".",16);
     OLED_ShowNum(69,4,humi_2,1,16);
     OLED_ShowString(77,4,"%     ",16);
     
     OLED_ShowNum(40,2,temp1_1,2,16);  //温度1显示
     OLED_ShowString(56,2,".",16);
     OLED_ShowNum(61,2,temp1_2,1,16);
     OLED_ShowChar(71,2,'。',12);
     OLED_ShowString(74,2,"C   ",16);
     
     OLED_ShowNum(40,6,High_1,2,16);   //mbar显示
     OLED_ShowString(56,6,".",16);
     OLED_ShowNum(61,6,High_2,1,16);   //mbar显示
//     OLED_ShowString(64,6,".",16);
//     OLED_ShowNum(69,6,mbar_2,2,16);
     OLED_ShowCHinese(70,6,10);//米 

   // Environmental_Update(PressToSend,HumToSend,Temp2ToSend-48,Temp1ToSend-33);//最终的环境数据上传步骤
  }
}

/**
* @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;   

  /* Compute the prescaler value to have TIM4 counter clock equal to 2 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2000) - 1); 
  
  /* Set TIM4 instance (Environmental)*/    
  TimEnvHandle.Instance = TIM4;
  /* Initialize TIM4 peripheral */
  TimEnvHandle.Init.Period = ENV_UPDATE_MUL_100MS*200 - 1;
  TimEnvHandle.Init.Prescaler = uwPrescalerValue;
  TimEnvHandle.Init.ClockDivision = 0;
  TimEnvHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimEnvHandle) != HAL_OK) {
    /* Initialization Error */
  }

    /* Compute the prescaler value to have TIM1 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  /* Set TIM1 instance (Motion)*/
  TimCCHandle.Instance = TIM1;  
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler( );
  }
  
 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Code for MotionFX and MotionGR integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionFX and MotionGR integration - End Section */
  
  /* Code for MotionCP integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel2 */
  sConfig.Pulse = DEFAULT_uhCCR2_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionCP integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel3 */
  sConfig.Pulse = DEFAULT_uhCCR3_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionAR integration - End Section */
  
  /* Output Compare Toggle Mode configuration: Channel4 */
  sConfig.Pulse = DEFAULT_uhCCR4_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /* Set TIM5 instance (Mic )*/
  TimAudioDataHandle.Instance = TIM5;
  /* Initialize TIM5 peripheral as follow:
     + Period = 100 - 1 (10ms)
     + Prescaler = ((SystemCoreClock/2)/10000) - 1
     + ClockDivision = 0
     + Counter direction = Up
  */
  TimAudioDataHandle.Init.Period = MICS_DB_UPDATE_MUL_10MS*100 - 1;
  TimAudioDataHandle.Init.Prescaler = uwPrescalerValue;
  TimAudioDataHandle.Init.ClockDivision = 0;
  TimAudioDataHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimAudioDataHandle) != HAL_OK) {
    /* Initialization Error */
    Error_Handler();
  }
}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void)
{
  const char BoardName[8] = {NAME_BLUEMS,0};
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;

#ifdef MAC_BLUEMS
  {
    uint8_t tmp_bdaddr[6]= {MAC_BLUEMS};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* MAC_BLUEMS */
  
#ifndef STM32_NUCLEO
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();
#endif /* STM32_NUCLEO */

  /* Initialize the BlueNRG HCI */
  HCI_Init();
    
  /* Reset BlueNRG hardware */
  BlueNRG_RST();

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  if (hwVersion > 0x30) {
    /* X-NUCLEO-IDB05A1 expansion board is used */
    TargetBoardFeatures.bnrg_expansion_board = IDB05A1;
  } else {
    /* X-NUCLEO-IDB0041 expansion board is used */
    TargetBoardFeatures.bnrg_expansion_board = IDB04A1;
  }
  
  /* 
   * Reset BlueNRG again otherwise it will fail.
   */
  BlueNRG_RST();

#ifndef MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
  /* Create a Unique BLE MAC Related to STM32 UID */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
#ifdef STM32_NUCLEO
    /* if IDB05A1 = Number between 100->199
     * if IDB04A1 = Number between 0->99
     * where Y == (OSX_BMS_VERSION_MAJOR + OSX_BMS_VERSION_MINOR)&0xF */
    bdaddr[4] = (hwVersion > 0x30) ?
         ((((OSX_BMS_VERSION_MAJOR-48)*10) + (OSX_BMS_VERSION_MINOR-48)+100)&0xFF) :
         ((((OSX_BMS_VERSION_MAJOR-48)*10) + (OSX_BMS_VERSION_MINOR-48)    )&0xFF) ;
#else /* STM32_NUCLEO */
    bdaddr[4] = (((OSX_BMS_VERSION_MAJOR-48)*10) + (OSX_BMS_VERSION_MINOR-48)+100)&0xFF;
#endif  /* STM32_NUCLEO */
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
  #else /* MAC_STM32UID_BLUEMS */
  {
    /* we will let the BLE chip to use its Random MAC address */
    uint8_t data_len_out;
    ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

    if(ret){
      OSX_BMS_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
      goto fail;
    }
  }
  #endif /* MAC_STM32UID_BLUEMS */
#else /* MAC_BLUEMS */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);

  if(ret){
     OSX_BMS_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
     goto fail;
  }
#endif /* MAC_BLUEMS */

  ret = aci_gatt_init();    
  if(ret){
     OSX_BMS_PRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  if (TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(ret != BLE_STATUS_SUCCESS){
     OSX_BMS_PRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

#ifndef  MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
    ret = hci_le_set_random_address(bdaddr);

    if(ret){
       OSX_BMS_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
       goto fail;
    }
  #endif /* MAC_STM32UID_BLUEMS */
#endif /* MAC_BLUEMS */

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     OSX_BMS_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     OSX_BMS_PRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  OSX_BMS_PRINTF("SERVER: BLE Stack Initialized \r\n"
         "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
         "\t\tBoardName= %s\r\n"
         "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
         (TargetBoardFeatures.bnrg_expansion_board==IDB05A1) ? "IDB05A1" : "IDB04A1",
         hwVersion,
         fwVersion>>8,
         (fwVersion>>4)&0xF,
         (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;
  
  ret = Add_HW_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     OSX_BMS_PRINTF("HW & SW Service W2ST added successfully\r\n");
  }
  else
  {
     OSX_BMS_PRINTF("\r\nError while adding HW & SW Service W2ST\r\n");
  }

  ret = Add_ConsoleW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     OSX_BMS_PRINTF("Console Service W2ST added successfully\r\n");
  }
  else
  {
     OSX_BMS_PRINTF("\r\nError while adding Console Service W2ST\r\n");
  }

  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     OSX_BMS_PRINTF("Config  Service W2ST added successfully\r\n");
  }
  else
  {
     OSX_BMS_PRINTF("\r\nError while adding Config Service W2ST\r\n");
  }
}




#ifdef STM32_SENSORTILE
/**
* @brief  System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  
  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();
  
  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }
  
  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();
  
  /* Select MSI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    while(1);
  }
}
#endif /* USE_STM32L4XX_NUCLEO */

/**
  * @brief This function provides accurate delay (in milliseconds) based 
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)//精确延时程序
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
#ifdef STM32_NUCLEO
    case SPI1_CMN_DEFAULT_IRQ_PIN:
#else
    case BNRG_SPI_EXTI_PIN:
#endif /* STM32_NUCLEO */   
      HCI_Isr();
      HCI_ProcessEvent=1;
    break;
#ifdef STM32_NUCLEO
  case KEY_BUTTON_PIN:
    ButtonPressed = 1;
    break;
#endif /* STM32_NUCLEO */

#ifdef STM32_NUCLEO
  #ifdef IKS01A1
    case M_INT1_PIN:
  #elif IKS01A2
    case LSM6DSL_INT1_O_PIN:
  #endif /* IKS01A1 */
#elif STM32_SENSORTILE
  case LSM6DSM_INT2_PIN:
#endif /* STM32_NUCLEO */
    MEMSInterrupt=1;
    break;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: OSX_BMS_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif

/**
 * @brief  Save the Magnetometer Calibration Values to Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
static unsigned char SaveCalibrationToMemory(uint32_t *MagnetoCalibration)
{
  unsigned char Success=1;

  /* Reset Before The data in Memory */
  Success = ResetCalibrationInMemory(MagnetoCalibration);

  if(Success) {
    void *temp;
    /* Store in RAM */
    MagnetoCalibration[0] = BLUEMSYS_CHECK_CALIBRATION;
    MagnetoCalibration[1] = (uint32_t) magOffset.magOffX;
    MagnetoCalibration[2] = (uint32_t) magOffset.magOffY;
    MagnetoCalibration[3] = (uint32_t) magOffset.magOffZ;

    temp = ((void *) &(magOffset.magGainX));
    MagnetoCalibration[4] = *((uint32_t *) temp);
    temp = ((void *) &(magOffset.magGainY));
    MagnetoCalibration[5] = *((uint32_t *) temp);
    temp = ((void *) &(magOffset.magGainZ));
    MagnetoCalibration[6] = *((uint32_t *) temp);
    temp = ((void *) &(magOffset.expMagVect));
    MagnetoCalibration[7] = *((uint32_t *) temp);

    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be saved in FLASH\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Magneto Calibration will be saved in FLASH\r\n");
    }
	NecessityToSaveMetaDataManager=1;
  }

  return Success;
}

/**
 * @brief  Reset the Magnetometer Calibration Values in Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
static unsigned char ResetCalibrationInMemory(uint32_t *MagnetoCalibration)
{
  /* Reset Calibration Values in RAM */
  unsigned char Success=1;
  int32_t Counter;

  for(Counter=0;Counter<8;Counter++)
    MagnetoCalibration[Counter]=0xFFFFFFFF;

  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
     BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be eresed in FLASH\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    OSX_BMS_PRINTF("Magneto Calibration will be eresed in FLASH\r\n");
  }
  NecessityToSaveMetaDataManager=1;
  
  return Success;
}

/**
 * @brief  Check if there are a valid Calibration Values in Memory and read them
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallCalibrationFromMemory(uint32_t *MagnetoCalibration)
{
  /* ReLoad the Calibration Values from RAM */
  unsigned char Success=1;

  if(MagnetoCalibration[0]== BLUEMSYS_CHECK_CALIBRATION) {
    void *temp;
    magOffset.magOffX    = (signed short) MagnetoCalibration[1];
    magOffset.magOffY    = (signed short) MagnetoCalibration[2];
    magOffset.magOffZ    = (signed short) MagnetoCalibration[3];
    temp= (void *)&(MagnetoCalibration[4]);
    magOffset.magGainX = *((float *)temp);
    temp= (void *)&(MagnetoCalibration[5]);
    magOffset.magGainY = *((float *)temp);
    temp= (void *)&(MagnetoCalibration[6]);
    magOffset.magGainZ = *((float *)temp);
    temp= (void *)&(MagnetoCalibration[7]);
    magOffset.expMagVect = *((float *)temp);

    /* Set the Calibration Structure */
    osx_MotionFX_setCalibrationData(&magOffset);
    OSX_BMS_PRINTF("Magneto Calibration Read\r\n");

    /* Control the calibration status */
    isCal = osx_MotionFX_compass_isCalibrated();
  } else {
    OSX_BMS_PRINTF("Magneto Calibration Not present\r\n");
    isCal=0;
  }

  return Success;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
