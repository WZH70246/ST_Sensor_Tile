#include <stdio.h>
#include "HWAdvanceFeatures.h"
#include "TargetFeatures.h"

/* Exported variables ---------------------------------------------------------*/
uint32_t HWAdvanceFeaturesStatus=0;

/* Imported Variables -------------------------------------------------------------*/
static float DefaultAccODR;

/**
  * @brief  This function Reads the default Acceleration Output Data Rate
  * @param  None
  * @retval None
  */
void InitHWFeatures(void){
   /* Read the Default Output Data Rate for Accelerometer */
  BSP_ACCELERO_Get_ODR(TargetBoardFeatures.HandleAccSensor,&DefaultAccODR);
}

/**
  * @brief  This function disables all the HW's Features
  * @param  None
  * @retval None
  */
void DisableHWFeatures(void)
{
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    DisableHWPedometer();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
    DisableHWFreeFall();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    DisableHWDoubleTap();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    DisableHWSingleTap();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    DisableHWWakeUp();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    DisableHWTilt();
  }
  
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    DisableHWOrientation6D();
  }
}



/**
  * @brief  This function enables the HW's 6D Orientation
  * @param  None
  * @retval None
  */
void EnableHWOrientation6D(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable all the HW features before */
    DisableHWFeatures();

    /* Enable Free Fall detection */
    if(BSP_ACCELERO_Enable_6D_Orientation_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Enabling 6D Orientation\r\n");
    } else {
      OSX_BMS_PRINTF("Enabled 6D Orientation\r\n");
      W2ST_ON_HW_FEATURE(W2ST_HWF_6DORIENTATION);
    }
  }
}

/**
  * @brief  This function disables the HW's 6D Orientation
  * @param  None
  * @retval None
  */
void DisableHWOrientation6D(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable Free Fall detection */
    if(BSP_ACCELERO_Disable_6D_Orientation_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Disabling 6D Orientation\r\n");
    } else {
      OSX_BMS_PRINTF("Disabled 6D Orientation\r\n");
      W2ST_OFF_HW_FEATURE(W2ST_HWF_6DORIENTATION);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
  }
}

/**
  * @brief  This function eturns the HW's 6D Orientation result
  * @param  None
  * @retval AccEventType 6D Orientation Found
  */
AccEventType GetHWOrientation6D(void)
{  
  AccEventType OrientationResult = ACC_NOT_USED;
  
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    uint8_t xl = 0;
    uint8_t xh = 0;
    uint8_t yl = 0;
    uint8_t yh = 0;
    uint8_t zl = 0;
    uint8_t zh = 0;
    
    if ( BSP_ACCELERO_Get_6D_Orientation_XL_Ext( TargetBoardFeatures.HandleAccSensor, &xl ) == COMPONENT_ERROR ){
      OSX_BMS_PRINTF("Error getting 6D orientation XL axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_XH_Ext( TargetBoardFeatures.HandleAccSensor, &xh ) == COMPONENT_ERROR ){
      OSX_BMS_PRINTF("Error getting 6D orientation XH axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_YL_Ext( TargetBoardFeatures.HandleAccSensor, &yl ) == COMPONENT_ERROR ){
      OSX_BMS_PRINTF("Error getting 6D orientation YL axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_YH_Ext( TargetBoardFeatures.HandleAccSensor, &yh ) == COMPONENT_ERROR ){
      OSX_BMS_PRINTF("Error getting 6D orientation YH axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_ZL_Ext( TargetBoardFeatures.HandleAccSensor, &zl ) == COMPONENT_ERROR ){
      OSX_BMS_PRINTF("Error getting 6D orientation ZL axis from LSM6DS3\r\n");
    }

    if ( BSP_ACCELERO_Get_6D_Orientation_ZH_Ext( TargetBoardFeatures.HandleAccSensor, &zh ) == COMPONENT_ERROR ){
      OSX_BMS_PRINTF("Error getting 6D orientation ZH axis from LSM6DS3\r\n");
    }
    
    if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
      OrientationResult = ACC_6D_OR_RIGTH;
    } else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
      OrientationResult = ACC_6D_OR_TOP;
    } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
      OrientationResult = ACC_6D_OR_BOTTOM;
    } else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
      OrientationResult = ACC_6D_OR_LEFT;
    } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
      OrientationResult = ACC_6D_OR_UP;
    } else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ){
      OrientationResult = ACC_6D_OR_DOWN;
    } else {
      OSX_BMS_PRINTF("None of the 6D orientation axes is set in LSM6DS3\r\n");
    }
  }
  return OrientationResult;
}
/**
  * @brief  This function enables the HW's Tilt Detection
  * @param  None
  * @retval None
  */
void EnableHWTilt(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable all the HW features before */
    DisableHWFeatures();

    /* Enable Tilt detection */
    if(BSP_ACCELERO_Enable_Tilt_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Enabling Tilt Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Enabled Tilt\r\n");
      W2ST_ON_HW_FEATURE(W2ST_HWF_TILT);
    }
  }
}

/**
  * @brief  This function disables the HW's Tilt Detection
  * @param  None
  * @retval None
  */
void DisableHWTilt(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable Tilt detection */
    if(BSP_ACCELERO_Disable_Tilt_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Disabling Tilt Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Disabled Tilt\r\n");
      W2ST_OFF_HW_FEATURE(W2ST_HWF_TILT);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
  }
}


/**
  * @brief  This function enables the HW's Wake Up Detection
  * @param  None
  * @retval None
  */
void EnableHWWakeUp(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable all the HW features before */
    DisableHWFeatures();

    /* Enable Wake up detection */
    if(BSP_ACCELERO_Enable_Wake_Up_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Enabling Wake Up Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Enabled Wake Up\r\n");
      W2ST_ON_HW_FEATURE(W2ST_HWF_WAKE_UP);
    }
  }
}

/**
  * @brief  This function disables the HW's Wake Up Detection
  * @param  None
  * @retval None
  */
void DisableHWWakeUp(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable Wake up detection */
    if(BSP_ACCELERO_Disable_Wake_Up_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Disabling Wake Up Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Disabled Wake Up\r\n");
      W2ST_OFF_HW_FEATURE(W2ST_HWF_WAKE_UP);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
  }
}

/**
  * @brief  This function enables the HW's Free Fall Detection
  * @param  None
  * @retval None
  */
void EnableHWFreeFall(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable all the HW features before */
    DisableHWFeatures();

    /* Enable Free Fall detection */
    if(BSP_ACCELERO_Enable_Free_Fall_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Enabling Free Fall Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Enabled Free Fall\r\n");
      W2ST_ON_HW_FEATURE(W2ST_HWF_FREE_FALL);
    }
    

#ifdef STM32_SENSORTILE
    if(BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(TargetBoardFeatures.HandleAccSensor,LSM6DSM_ACC_GYRO_FF_THS_219mg)==COMPONENT_ERROR) {   
#endif /* STM32_NUCLEO */
      OSX_BMS_PRINTF("Error setting Free Fall Treshold\r\n");
    }
  }
}

/**
  * @brief  This function disables the HW's Free Fall Detection
  * @param  None
  * @retval None
  */
void DisableHWFreeFall(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable Free Fall detection */
    if(BSP_ACCELERO_Disable_Free_Fall_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Disabling Free Fall Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Disabled Free Fall\r\n");
      W2ST_OFF_HW_FEATURE(W2ST_HWF_FREE_FALL);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
  }
}

/**
  * @brief  This function enables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void EnableHWDoubleTap(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable all the HW features before */
    DisableHWFeatures();

    /* Enable Double Tap detection */
    if(BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Enabling Double Tap Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Enabled Double Tap\r\n");
      W2ST_ON_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
    }
    

#ifdef STM32_SENSORTILE
    if(BSP_ACCELERO_Set_Tap_Threshold_Ext(TargetBoardFeatures.HandleAccSensor,LSM6DSM_TAP_THRESHOLD_MID)==COMPONENT_ERROR) {    
#endif /* STM32_NUCLEO */
      OSX_BMS_PRINTF("Error setting Double Tap Treshold\r\n");
    }
  }
}

/**
  * @brief  This function disables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void DisableHWDoubleTap(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable Double Tap detection */
    if(BSP_ACCELERO_Disable_Double_Tap_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Disabling Double Tap Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Disabled Double Tap\r\n");
      W2ST_OFF_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
  }
}

/**
  * @brief  This function enables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void EnableHWSingleTap(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable all the HW features before */
    DisableHWFeatures();

    /* Enable Single Tap detection */
    if(BSP_ACCELERO_Enable_Single_Tap_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Enabling Single Tap Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Enabled Sigle Tap\r\n");
      W2ST_ON_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
    }
  }
}

/**
  * @brief  This function disables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void DisableHWSingleTap(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable Single Tap detection */
    if(BSP_ACCELERO_Disable_Single_Tap_Detection_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Disabling Single Tap Detection\r\n");
    } else {
      OSX_BMS_PRINTF("Disabled Sigle Tap\r\n");
      W2ST_OFF_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
  }
}

/**
  * @brief  This function enables the HW's pedometer
  * @param  None
  * @retval None
  */
void EnableHWPedometer(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    /* Disable all the HW features before */
    DisableHWFeatures();

    if(BSP_ACCELERO_Enable_Pedometer_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Enabling Pedometer\r\n");
    } else {
      OSX_BMS_PRINTF("Enabled Pedometer\r\n");
      W2ST_ON_HW_FEATURE(W2ST_HWF_PEDOMETER);
    }
  }
}

/**
  * @brief  This function disables the HW's pedometer
  * @param  None
  * @retval None
  */
void DisableHWPedometer(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    if(BSP_ACCELERO_Disable_Pedometer_Ext(TargetBoardFeatures.HandleAccSensor)==COMPONENT_ERROR) {
      OSX_BMS_PRINTF("Error Disabling Pedometer\r\n");
    } else {
      OSX_BMS_PRINTF("Disabled Pedometer\r\n");
      W2ST_OFF_HW_FEATURE(W2ST_HWF_PEDOMETER);
    }

    /* Set the Output Data Rate to Default value */
    BSP_ACCELERO_Set_ODR_Value(TargetBoardFeatures.HandleAccSensor,DefaultAccODR);
  }
}

/**
  * @brief  This function resets the HW's pedometer steps counter
  * @param  None
  * @retval None
  */
void ResetHWPedometer(void)
{
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    if ( BSP_ACCELERO_Reset_Step_Counter_Ext( TargetBoardFeatures.HandleAccSensor ) == COMPONENT_ERROR ){
      OSX_BMS_PRINTF("Error Resetting Pedometer's Counter\r\n");
    } else {
      OSX_BMS_PRINTF("Reset Pedometer's Counter\r\n");
    }
  }
}

/**
  * @brief  This function retunrs the HW's pedometer steps counter value
  * @param  None
  * @retval uint16_t Steps Counter
  */
uint16_t GetStepHWPedometer(void)
{
  uint16_t step_count=0;
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    if(BSP_ACCELERO_Get_Step_Count_Ext( TargetBoardFeatures.HandleAccSensor, &step_count ) == COMPONENT_ERROR ){
      OSX_BMS_PRINTF("Error Reading Pedometer's Counter\r\n");
    } else {
      OSX_BMS_PRINTF("Pedometer's Counter=%u\r\n",step_count);
    }
  }
  return step_count;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
