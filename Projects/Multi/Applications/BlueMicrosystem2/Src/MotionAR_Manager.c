/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

/* Code for MotionAR integration - Start Section */
#include "../../../../../Middlewares/ST/STM32_OSX_MotionAR_Library/osx_license.h"

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
osx_MAR_output_t ActivityCode = OSX_MAR_NOACTIVITY;


/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionAR    Drv_MotionAR
  * @{
  */   

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run activity recognition algorithm. This function collects and scale data 
* from accelerometer and calls the Activity Recognition Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
* @retval None
*/
void MotionAR_manager_run(SensorAxesRaw_t ACC_Value_Raw)
{
  osx_MAR_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.AXIS_X * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.AXIS_Y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.AXIS_Z * sensitivity_Mul;

  ActivityCode = osx_MotionAR_Update(&iDataIN);
}

/**
  * @brief  Initialize MotionAR License
  * @param  MDM_PayLoadLic_t *PayLoad Pointer to the osx License MetaData
  * @retval None
  */
void MotionAR_License_init(MDM_PayLoadLic_t *PayLoad)
{
  MCR_OSX_COPY_LICENSE_TO_MDM(osx_mar_license,PayLoad->osxLicense);

  if(osx_MotionAR_Initialize()==0) {
    OSX_BMS_PRINTF("Error MotionAR License authentication \n\r");
    while(1) {
      ;
    }
  } else {
    osx_MotionAR_GetLibVersion(PayLoad->osxLibVersion);
    OSX_BMS_PRINTF("Enabled %s\n\r",PayLoad->osxLibVersion);
    if(PayLoad->osxLicenseInitialized==0) {
      NecessityToSaveMetaDataManager=1;
      PayLoad->osxLicenseInitialized=1;
    }
  }
}

/**
* @brief  Initialises MotionAR algorithm
* @param  None
* @retval None
*/
void MotionAR_manager_init(void)
{
  char acc_orientation[3];


#ifdef STM32_SENSORTILE
  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';
#endif /* STM32_NUCLEO */
  osx_MotionAR_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.osxMotionARIsInitalized=1;
  OSX_BMS_PRINTF("Initialized osxMotionAR\n\r");
}

/**
 * @}
 */ /* end of group  Drv_MotionAR        Drv_MotionAR*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

/* Code for MotionAR integration - End Section */
/************************ (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
