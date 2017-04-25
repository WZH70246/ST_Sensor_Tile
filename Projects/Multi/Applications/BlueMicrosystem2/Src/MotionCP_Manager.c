/* Includes ------------------------------------------------------------------*/
#include "TargetFeatures.h"

/* Code for MotionCP integration - Start Section */
#include "../../../../../Middlewares/ST/STM32_OSX_MotionCP_Library/osx_license.h"

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
osx_MCP_output_t CarryPositionCode = OSX_MCP_UNKNOWN;


/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionCP    Drv_MotionCP
  * @{
  */   

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run carry position algorithm. This function collects and scale data 
* from accelerometer and calls the Carry Position Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration values (x,y,z)
* @retval None
*/
void MotionCP_manager_run(SensorAxesRaw_t ACC_Value_Raw)
{
  osx_MCP_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.AXIS_X * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.AXIS_Y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.AXIS_Z * sensitivity_Mul;

  CarryPositionCode = osx_MotionCP_Update(&iDataIN);
}

/**
  * @brief  Initialize MotionCP License
  * @param  MDM_PayLoadLic_t *PayLoad Pointer to the osx License MetaData
  * @retval None
  */
void MotionCP_License_init(MDM_PayLoadLic_t *PayLoad)
{
  MCR_OSX_COPY_LICENSE_TO_MDM(osx_mcp_license,PayLoad->osxLicense);

  if(!osx_MotionCP_Initialize()) {
    OSX_BMS_PRINTF("Error MotionCP License authentication \n\r");
    while(1) {
      ;
    }
  } else {
    osx_MotionCP_GetLibVersion(PayLoad->osxLibVersion);
    OSX_BMS_PRINTF("Enabled %s\n\r",PayLoad->osxLibVersion);
    if(PayLoad->osxLicenseInitialized==0) {
      NecessityToSaveMetaDataManager=1;
      PayLoad->osxLicenseInitialized=1;
    }
  }
}

/**
* @brief  Initialises MotionCP algorithm
* @param  None
* @retval None
*/
void MotionCP_manager_init(void)
{
  char acc_orientation[3];


#ifdef STM32_SENSORTILE
  acc_orientation[0] ='w';
  acc_orientation[1] ='s';
  acc_orientation[2] ='u';
#endif /* STM32_NUCLEO */ 

  osx_MotionCP_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.osxMotionCPIsInitalized=1;
  OSX_BMS_PRINTF("Initialized osxMotionCP\n\r");
}

/**
 * @}
 */ /* end of group  Drv_MotionCP        Drv_MotionCP*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

/* Code for MotionCP integration - End Section */
/************************ (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
