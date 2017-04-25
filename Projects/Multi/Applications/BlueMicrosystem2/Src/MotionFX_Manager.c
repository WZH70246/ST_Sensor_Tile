/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "TargetFeatures.h"
//#include "MotionFX_Manager.h"
#include "../../../../../Middlewares/ST/STM32_OSX_MotionFX_Library/osx_license.h"

/* Private defines -----------------------------------------------------------*/
#define FROM_MDPS_TO_DPS    0.001
#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)
#define SAMPLETODISCARD 15
#define GBIAS_ACC_TH_SC_6X (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_6X (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_6X (2.0f*0.001500f)
#define GBIAS_ACC_TH_SC_9X (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_9X (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_9X (2.0f*0.001500f)
/* Delta time mSec for Deltafusion */
#define DELTATIMESENSORFUSION 0.01

/* Exported Variables -------------------------------------------------------------*/
osxMFX_output iDataOUT;
osxMFX_input iDataIN;

/* Imported Variables -------------------------------------------------------------*/
extern float sensitivity_Mul;

extern osxMFX_calibFactor magOffset;

/* Private Variables -------------------------------------------------------------*/
static int discardedCount = 0;

static osxMFX_knobs iKnobs;
static osxMFX_knobs* ipKnobs;

/**
  * @brief  Run sensor fusion algorithm
  * @param SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
  * @param SensorAxes_t GYR_Value Gyroscope value (x/y/z)
  * @param SensorAxes_t MAG_Value Magneto value (x/y/z)
  * @retval None
  */
void MotionFX_manager_run( SensorAxesRaw_t ACC_Value_Raw,SensorAxes_t GYR_Value,SensorAxes_t MAG_Value)
{  
  iDataIN.gyro[0] = GYR_Value.AXIS_X  * FROM_MDPS_TO_DPS;
  iDataIN.gyro[1] = GYR_Value.AXIS_Y  * FROM_MDPS_TO_DPS;
  iDataIN.gyro[2] = GYR_Value.AXIS_Z  * FROM_MDPS_TO_DPS; 

  iDataIN.acc[0] = ACC_Value_Raw.AXIS_X * sensitivity_Mul;
  iDataIN.acc[1] = ACC_Value_Raw.AXIS_Y * sensitivity_Mul;
  iDataIN.acc[2] = ACC_Value_Raw.AXIS_Z * sensitivity_Mul;

  iDataIN.mag[0] = (MAG_Value.AXIS_X - magOffset.magOffX) * FROM_MGAUSS_TO_UT50;
  iDataIN.mag[1] = (MAG_Value.AXIS_Y - magOffset.magOffY) * FROM_MGAUSS_TO_UT50;
  iDataIN.mag[2] = (MAG_Value.AXIS_Z - magOffset.magOffZ) * FROM_MGAUSS_TO_UT50;

  if(discardedCount == SAMPLETODISCARD){
    osx_MotionFX_propagate(&iDataOUT, &iDataIN, DELTATIMESENSORFUSION);
    osx_MotionFX_update(&iDataOUT, &iDataIN, DELTATIMESENSORFUSION, NULL);
  } else {
    discardedCount++;
  }  
}

/**
  * @brief  Initialize MotionFX License
  * @param  MDM_PayLoadLic_t *PayLoad Pointer to the osx License MetaData
  * @retval None
  */
void MotionFX_License_init(MDM_PayLoadLic_t *PayLoad)
{
  MCR_OSX_COPY_LICENSE_TO_MDM(osx_mfx_license,PayLoad->osxLicense);
  if(!osx_MotionFX_initialize()) {
    OSX_BMS_PRINTF("Error MotionFX License authentication\n\r");
    while(1) {
      ;
    }
  } else {
    osx_MotionFX_getLibVersion(PayLoad->osxLibVersion);
    OSX_BMS_PRINTF("Enabled %s\n\r",PayLoad->osxLibVersion);
    if(PayLoad->osxLicenseInitialized==0) {
      NecessityToSaveMetaDataManager=1;
      PayLoad->osxLicenseInitialized=1;
    }
  }
}

/**
  * @brief  Initialize MotionFX engine
  * @retval None
  */
void MotionFX_manager_init(void)
{
  //  ST MotionFX Engine Initializations
  ipKnobs = &iKnobs;

  osx_MotionFX_compass_init();

  osx_MotionFX_getKnobs(ipKnobs);

  ipKnobs->gbias_acc_th_sc_6X = GBIAS_ACC_TH_SC_6X;
  ipKnobs->gbias_gyro_th_sc_6X = GBIAS_GYRO_TH_SC_6X;
  ipKnobs->gbias_mag_th_sc_6X = GBIAS_MAG_TH_SC_6X;

  ipKnobs->gbias_acc_th_sc_9X = GBIAS_ACC_TH_SC_9X;
  ipKnobs->gbias_gyro_th_sc_9X = GBIAS_GYRO_TH_SC_9X;
  ipKnobs->gbias_mag_th_sc_9X = GBIAS_MAG_TH_SC_9X;
  

#ifdef STM32_SENSORTILE
  ipKnobs->acc_orientation[0] ='w';
  ipKnobs->acc_orientation[1] ='s';
  ipKnobs->acc_orientation[2] ='u';

  ipKnobs->gyro_orientation[0] = 'w';
  ipKnobs->gyro_orientation[1] = 's';
  ipKnobs->gyro_orientation[2] = 'u';
  
  ipKnobs->mag_orientation[0] = 's';
  ipKnobs->mag_orientation[1] = 'w';
  ipKnobs->mag_orientation[2] = 'u';
#endif /* STM32_NUCLEO */

  ipKnobs->output_type = OSXMFX_ENGINE_OUTPUT_ENU;

  ipKnobs->LMode = 1;
  ipKnobs->modx  = 1;

  osx_MotionFX_setKnobs(ipKnobs);

  osx_MotionFX_enable_6X(OSXMFX_ENGINE_DISABLE);

  osx_MotionFX_enable_9X(OSXMFX_ENGINE_DISABLE);

  discardedCount = 0;

  /* Reset MagnetoOffset */
  magOffset.magOffX = magOffset.magOffY= magOffset.magOffZ=0;

  TargetBoardFeatures.osxMotionFXIsInitalized=1;
  OSX_BMS_PRINTF("Initialized osxMotionFX\n\r");
}

/**
 * @brief  Start 6 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_start_6X(void)
{
  osx_MotionFX_enable_6X(OSXMFX_ENGINE_ENABLE);
}

/**
 * @brief  Stop 6 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_stop_6X(void)
{
  osx_MotionFX_enable_6X(OSXMFX_ENGINE_DISABLE);
}

/**
 * @brief  Start 9 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_start_9X(void)
{
  osx_MotionFX_enable_9X(OSXMFX_ENGINE_ENABLE);
}

/**
 * @brief  Stop 9 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_stop_9X(void)
{
  osx_MotionFX_enable_9X(OSXMFX_ENGINE_DISABLE);
}

/**
* @brief  Get MotionFX Engine data Out
* @param  None
* @retval osxMFX_output *iDataOUT MotionFX Engine data Out
*/
osxMFX_output* MotionFX_manager_getDataOUT(void)
{
  return &iDataOUT;
}

/**
* @brief  Get MotionFX Engine data IN
* @param  None
* @retval osxMFX_input *iDataIN MotionFX Engine data IN
*/
osxMFX_input* MotionFX_manager_getDataIN(void)
{
  return &iDataIN;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
