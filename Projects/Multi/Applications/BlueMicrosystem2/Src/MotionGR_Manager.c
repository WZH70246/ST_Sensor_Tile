

/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"
/* Code for MotionGR integration - Start Section */
#include "../../../../../Middlewares/ST/STM32_OSX_MotionGR_Library/osx_license.h"

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
osx_MGR_output_t GestureRecognitionCode = OSX_MGR_NOGESTURE;


/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionGR    Drv_MotionGR
  * @{
  */   

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run gesture recognition algorithm. This function collects and scale data 
* from accelerometer and calls the Gesture Recognition Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
* @retval None
*/
void MotionGR_manager_run(SensorAxesRaw_t ACC_Value_Raw)
{
  osx_MGR_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.AXIS_X * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.AXIS_Y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.AXIS_Z * sensitivity_Mul;
    
  GestureRecognitionCode = osx_MotionGR_Update(&iDataIN);
  
}

/**
  * @brief  Initialize MotionGR License
  * @param  MDM_PayLoadLic_t *PayLoad Pointer to the osx License MetaData
  * @retval None
  */
void MotionGR_License_init(MDM_PayLoadLic_t *PayLoad)
{
  MCR_OSX_COPY_LICENSE_TO_MDM(osx_mgr_license,PayLoad->osxLicense);

  if(!osx_MotionGR_Initialize()) {
    OSX_BMS_PRINTF("Error MotionGR License authentication \n\r");
    while(1) {
      ;
    }
  } else {
    osx_MotionGR_GetLibVersion(PayLoad->osxLibVersion);
    OSX_BMS_PRINTF("Enabled %s\n\r",PayLoad->osxLibVersion);
    if(PayLoad->osxLicenseInitialized==0) {
      NecessityToSaveMetaDataManager=1;
      PayLoad->osxLicenseInitialized=1;
    }
  }
}

/**
* @brief  Initialises MotionGR algorithm
* @param  None
* @retval None
*/
void MotionGR_manager_init(void)
{
  char acc_orientation[3];


#ifdef STM32_SENSORTILE
  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';
#endif /* STM32_NUCLEO */

  osx_MotionGR_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.osxMotionGRIsInitalized=1;
  OSX_BMS_PRINTF("Initialized osxMotionGR\n\r");
}

/**
 * @}
 */ /* end of group  Drv_MotionGR        Drv_MotionGR*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

/* Code for MotionGR integration - End Section */
/************************ (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
