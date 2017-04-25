
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "MetaDataManager.h"
#include "sensor_service.h"
#include "console.h"
#include "HWAdvanceFeatures.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"
#include "OTA.h"

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE,Need_Reinit=FALSE;

/* Imported Variables -------------------------------------------------------------*/
extern uint32_t ConnectionBleStatus;

/* Code for MotionFX integration - Start Section */
extern osxMFX_calibFactor magOffset;
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
extern osx_MAR_output_t ActivityCode;
/* Code for MotionAR integration - End Section */

/* Code for MotionCP integration - Start Section */
extern osx_MCP_output_t CarryPositionCode;
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
extern osx_MGR_output_t GestureRecognitionCode;
/* Code for MotionGR integration - End Section */

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
extern osx_AcousticSL_Handler_t libSoundSourceLoc_Handler_Instance;
extern osx_AcousticSL_Config_t  libSoundSourceLoc_Config_Instance;
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

extern uint32_t ForceReCalibration;
extern uint32_t FirstConnectionConfig;

extern TIM_HandleTypeDef    TimCCHandle;
extern TIM_HandleTypeDef    TimEnvHandle;
extern TIM_HandleTypeDef    TimAudioDataHandle;

extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];



extern uint8_t bdaddr[6];

extern uint32_t uhCCR4_Val;

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

/* Private variables ------------------------------------------------------------*/
static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t AccGyroMagCharHandle;
static uint16_t AccEventCharHandle;
static uint16_t AudioLevelCharHandle;

#ifdef STM32_SENSORTILE
static uint16_t GGCharHandle;
#endif /* STM32_SENSORTILE */

/* Code for MotionFX integration - Start Section */
static uint16_t QuaternionsCharHandle;
static uint16_t ECompassCharHandle;
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
static uint16_t ActivityRecCharHandle;
/* Code for MotionAR integration - End Section */

/* Code for MotionCP integration - Start Section */
static uint16_t CarryPosRecCharHandle;
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
static uint16_t GestureRecCharHandle;
/* Code for MotionGR integration - End Section */

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
uint16_t AudioSourceLocalizationCharHandle;
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

/* Code for BlueVoice integration - Start Section */
OSX_BLUEVOICE_ProfileHandle_t BLUEVOICE_tx_handle;
OSX_BLUEVOICE_uuid_t BLUEVOICE_Char_uuids;
/* Code for BlueVoice integration - End Section */

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

static uint8_t  EnvironmentalCharSize=2; /* Size for Environmental BLE characteristic */

static uint32_t SizeOfUpdateBlueFW=0;

static uint16_t connection_handle = 0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
static uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}//上传保护

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */


/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //OSX_BMS_PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR;
}


/**
 * @brief  Add the Console service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConsoleW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONSOLE_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*2,&ConsoleW2STHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_TERM_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &TermCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_STDERR_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &StdErrCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
     goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //OSX_BMS_PRINTF("Error while adding Console service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(10);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
        OSX_BMS_PRINTF("Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(20);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, LastStderrLen , LastStderrBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Term_Update_AfterRead(void)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, LastTermLen , LastTermBuffer);
  if (ret != BLE_STATUS_SUCCESS) {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Stdout Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Stdout Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }

  return BLE_STATUS_SUCCESS;
}

/* Code for MotionFX integration - Start Section */
/**
 * @brief  Update quaternions characteristic value
 * @param  SensorAxes_t *data Structure containing the quaterions
 * @retval tBleStatus      Status
 */
tBleStatus Quat_Update(SensorAxes_t *data)
{
  tBleStatus ret;    

  uint8_t buff[2+ 6*SEND_N_QUATERNIONS];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));

#if SEND_N_QUATERNIONS == 1
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);
#elif SEND_N_QUATERNIONS == 2
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);

  STORE_LE_16(buff+8 ,data[1].AXIS_X);
  STORE_LE_16(buff+10,data[1].AXIS_Y);
  STORE_LE_16(buff+12,data[1].AXIS_Z);
#elif SEND_N_QUATERNIONS == 3
  STORE_LE_16(buff+2,data[0].AXIS_X);
  STORE_LE_16(buff+4,data[0].AXIS_Y);
  STORE_LE_16(buff+6,data[0].AXIS_Z);

  STORE_LE_16(buff+8 ,data[1].AXIS_X);
  STORE_LE_16(buff+10,data[1].AXIS_Y);
  STORE_LE_16(buff+12,data[1].AXIS_Z);

  STORE_LE_16(buff+14,data[2].AXIS_X);
  STORE_LE_16(buff+16,data[2].AXIS_Y);
  STORE_LE_16(buff+18,data[2].AXIS_Z);
#else
#error SEND_N_QUATERNIONS could be only 1,2,3
#endif
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, QuaternionsCharHandle, 0, 2+6*SEND_N_QUATERNIONS, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Quat Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Quat Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update E-Compass characteristic value
 * @param  uint16_t Angle To Magnetic North in cents of degree [0.00 -> 259,99]
 * @retval tBleStatus      Status
 */
tBleStatus ECompass_Update(uint16_t Angle)
{
  tBleStatus ret;

  uint8_t buff[2+ 2];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,Angle);

  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, ECompassCharHandle, 0, 2+2,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating E-Compass Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating E-Compass Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionFX integration - End Section */

/* Code for MotionAR integration - Start Section */
/**
 * @brief  Update Activity Recognition value
 * @param  osx_MAR_output_t ActivityCode Activity Recognized
 * @retval tBleStatus      Status
 */
tBleStatus ActivityRec_Update(osx_MAR_output_t ActivityCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = ActivityCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, ActivityRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating ActivityRec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating ActivityRec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionAR integration - End Section */

/* Code for MotionCP integration - Start Section */
/**
 * @brief  Update Carry Postion Recognition value
 * @param  osx_MCP_output_t CarryPositionCode Carry Position Recognized
 * @retval tBleStatus      Status
 */
tBleStatus CarryPosRec_Update(osx_MCP_output_t CarryPositionCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = CarryPositionCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, CarryPosRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating CarryPosRec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating CarryPosRec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionCP integration - End Section */

/* Code for MotionGR integration - Start Section */
/**
 * @brief  Update Gesture Recognition value
 * @param  osx_MGR_output_t GestureCode Gesture Recognized
 * @retval tBleStatus      Status
 */
tBleStatus GestureRec_Update(osx_MGR_output_t GestureCode)
{
  tBleStatus ret;

  uint8_t buff[2+ 1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = GestureCode;

  ret = aci_gatt_update_char_value(HWServW2STHandle, GestureRecCharHandle, 0, 2+1, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Gesture Rec Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Gesture Rec Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}
/* Code for MotionGR integration - End Section */

/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notify(uint16_t Command)
{
  tBleStatus ret;
  uint8_t buff[2+2];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,Command);

  ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+2,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating AccEvent_Notify Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating AccEvent_Notify Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add HW and SW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HW_SW_ServW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  /* Environmental */
  uint8_t max_attr_records= 4;

#ifdef STM32_SENSORTILE
  if(TargetBoardFeatures.HandleGGComponent){
    /* Battery Present */
    max_attr_records++;
  }
#endif /* STM32_SENSORTILE */
  
  /* Code for MotionFX integration - Start Section */
  /* Sensor Fusion Short */
  max_attr_records++;
  
  /* Only for IDB05A1 */
  if(TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
    /* ECompass */
    max_attr_records++;
  }
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  max_attr_records++;
  /* Code for MotionAR integration - End Section */

  /* Code for MotionCP integration - Start Section */
  max_attr_records++;
  /* Code for MotionCP integration - End Section */

  /* Code for MotionGR integration - Start Section */
  max_attr_records++;
  /* Code for MotionGR integration - End Section */
  
  /* Code for BlueVoice integration - Start Section */
  max_attr_records+=2;
  /* Code for BlueVoice integration - End Section */
  
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
  max_attr_records++;
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */
  
  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE,
                          1+3*max_attr_records,
                          &HWServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  /* Fill the Environmental BLE Characteristc */
  COPY_ENVIRONMENTAL_W2ST_CHAR_UUID(uuid);
  if(TargetBoardFeatures.NumTempSensors==2) {
    uuid[14] |= 0x05; /* Two Temperature values*/
    EnvironmentalCharSize+=2*2;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    uuid[14] |= 0x04; /* One Temperature value*/
    EnvironmentalCharSize+=2;
  }

  if(TargetBoardFeatures.HandleHumSensor) {
   uuid[14] |= 0x08; /* Humidity */
   EnvironmentalCharSize+=2;
  }

  if(TargetBoardFeatures.HandlePressSensor) {
    uuid[14] |= 0x10; /* Pressure value*/
    EnvironmentalCharSize+=4;
  }

  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, EnvironmentalCharSize,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &EnvironmentalCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_ACC_GYRO_MAG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+3*3*2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccGyroMagCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_ACC_EVENT_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AccEventCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  COPY_MIC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid,2+AUDIO_CHANNELS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AudioLevelCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
#ifdef STM32_SENSORTILE
  if(TargetBoardFeatures.HandleGGComponent){
    COPY_GG_W2ST_CHAR_UUID(uuid);
    ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2+2+2+1,
                             CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                             ATTR_PERMISSION_NONE,
                             GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                             16, 0, &GGCharHandle);

    if (ret != BLE_STATUS_SUCCESS) {
      goto fail;
    }
  }
#endif /* STM32_SENSORTILE */

  /* Code for MotionFX integration - Start Section */
  COPY_QUATERNIONS_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+6*SEND_N_QUATERNIONS,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &QuaternionsCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  /* Only for IDB05A1 */
  if(TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
    /* ECompass */
    COPY_ECOMPASS_W2ST_CHAR_UUID(uuid);
    ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2,
                             CHAR_PROP_NOTIFY,
                             ATTR_PERMISSION_NONE,
                             GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                             16, 0, &ECompassCharHandle);
    
    if (ret != BLE_STATUS_SUCCESS) {
      goto fail;
    }
  }
  /* Code for MotionFX integration - End Section */

  /* Code for MotionAR integration - Start Section */
  COPY_ACTIVITY_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ActivityRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionAR integration - End Section */

  /* Code for MotionCP integration - Start Section */
  COPY_CARRY_POSITION_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &CarryPosRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionCP integration - End Section */
  
  /* Code for MotionGR integration - Start Section */
  COPY_GESTURE_REC_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &GestureRecCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  /* Code for MotionGR integration - End Section */
  
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
  COPY_MIC_ANGLE_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+2,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &AudioSourceLocalizationCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

  /* Code for BlueVoice integration - Start Section */
  memcpy(&BLUEVOICE_tx_handle.ServiceHandle, &HWServW2STHandle, sizeof(uint16_t));
  
  COPY_AUDIO_ADPCM_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 20,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           0, 16, 1, &BLUEVOICE_tx_handle.CharAudioHandle);
  
  COPY_AUDIO_ADPCM_SYNC_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 6,
                           CHAR_PROP_NOTIFY,
                           ATTR_PERMISSION_NONE,
                           0, 16, 1, &BLUEVOICE_tx_handle.CharAudioSyncHandle);
  /* Code for BlueVoice integration - End Section */

  return BLE_STATUS_SUCCESS;

fail:
  //OSX_BMS_PRINTF("Error while adding HW's Characteristcs service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  SensorAxes_t Acc Structure containing acceleration value in mg
 * @param  SensorAxes_t Gyro Structure containing Gyroscope value
 * @param  SensorAxes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(SensorAxes_t *Acc,SensorAxes_t *Gyro,SensorAxes_t *Mag)
{  
  tBleStatus ret;
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;

  uint8_t buff[2+3*3*2];

  STORE_LE_16(buff   ,(HAL_GetTick()>>3));
  
  STORE_LE_16(buff+2 ,Acc->AXIS_X);
  STORE_LE_16(buff+4 ,Acc->AXIS_Y);
  STORE_LE_16(buff+6 ,Acc->AXIS_Z);
  
  Gyro->AXIS_X/=100;
  Gyro->AXIS_Y/=100;
  Gyro->AXIS_Z/=100;

  STORE_LE_16(buff+8 ,Gyro->AXIS_X);
  STORE_LE_16(buff+10,Gyro->AXIS_Y);
  STORE_LE_16(buff+12,Gyro->AXIS_Z);
    
  /* Apply Magneto calibration */
  AXIS_X = Mag->AXIS_X - magOffset.magOffX;
  AXIS_Y = Mag->AXIS_Y - magOffset.magOffY;
  AXIS_Z = Mag->AXIS_Z - magOffset.magOffZ;

  STORE_LE_16(buff+14,AXIS_X);
  STORE_LE_16(buff+16,AXIS_Y);
  STORE_LE_16(buff+18,AXIS_Z);
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);
	
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Acc/Gyro/Mag Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 */
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1)//环境数据上传
{
  tBleStatus ret;
  uint8_t BuffPos;
  
  uint8_t buff[2+4/*Press*/+2/*Hum*/+2/*Temp2*/+2/*Temp1*/];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos=2;

  if(TargetBoardFeatures.HandlePressSensor) {
    STORE_LE_32(buff+BuffPos,Press);
    BuffPos+=4;
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    STORE_LE_16(buff+BuffPos,Hum);
    BuffPos+=2;
  }

  if(TargetBoardFeatures.NumTempSensors==2) {
    STORE_LE_16(buff+BuffPos,Temp2);
    BuffPos+=2;
    
    STORE_LE_16(buff+BuffPos,Temp1);
    BuffPos+=2;
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    STORE_LE_16(buff+BuffPos,Temp1);
    BuffPos+=2;
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, EnvironmentalCharSize,buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Environmental Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating Environmental Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Microphones characteristic values
 * @param  uint16_t *Mic SNR dB Microphones array
 * @retval tBleStatus   Status
 */
tBleStatus AudioLevel_Update(uint16_t *Mic)//麦克风数据上传
{  
  tBleStatus ret;
  uint16_t Counter;
  
  uint8_t buff[2+1*AUDIO_CHANNELS]; /* BlueCoin has 4 Mics */

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  for(Counter=0;Counter<AUDIO_CHANNELS;Counter++) {
    buff[2+Counter]= Mic[Counter]&0xFF;
  }
    
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AudioLevelCharHandle, 0, 2+AUDIO_CHANNELS,buff);
  
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Mic Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;	
}

#ifdef STM32_SENSORTILE
/**
 * @brief  Update Gas Gouge characteristic value
 * @param  None
 * @retval tBleStatus   Status
 */
tBleStatus GG_Update(void)//电池数据上传
{
  tBleStatus ret;
  uint32_t voltage, soc;
  int32_t current;
  uint8_t v_mode;

  uint8_t buff[2+2+2+2+1];

  /* Update Gas Gouge Status */
  BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

  /* Read the Gas Gouge Status */
  BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
  BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);
  BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);

  STORE_LE_16(buff,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,soc*10);
  STORE_LE_16(buff+4,voltage);
  STORE_LE_16(buff+6,current);

  if(soc<15) {
    /* if it's < 15% Low Battery*/
	buff[8] = 0x00; /* Low Battery */
  } else {
    static uint32_t PreVoltage = 0;
    static uint32_t Status     = 0x04; /* Unknown */
    if(PreVoltage!=0) {
      if(PreVoltage>voltage) {
        Status = 0x01; /* Discharging */
      } else if(PreVoltage<voltage){
        Status = 0x03; /* Charging */
      }
    }
    buff[8] = Status;
    PreVoltage = voltage;
  }//电池信息及数据整合

  ret = aci_gatt_update_char_value(HWServW2STHandle, GGCharHandle, 0, 2+2+2+2+1,buff);//上传整合完成的buff数据

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating GG Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Error Updating GG Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

//电池数据oled显示
tBleStatus GG_Update_Oled(void)//电池数据显示
{
  tBleStatus ret;
  uint32_t voltage, soc;
  int32_t current;
  uint8_t v_mode;

  uint8_t buff[2+2+2+2+1];

  /* Update Gas Gouge Status */
  BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

  /* Read the Gas Gouge Status */
  BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
  BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);
  BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);

  STORE_LE_16(buff,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,soc*10);
  STORE_LE_16(buff+4,voltage);
  STORE_LE_16(buff+6,current);

  if(soc<15) {
    /* if it's < 15% Low Battery*/
	buff[8] = 0x00; /* Low Battery */
  } else {
    static uint32_t PreVoltage = 0;
    static uint32_t Status     = 0x04; /* Unknown */
    if(PreVoltage!=0) {
      if(PreVoltage>voltage) {
        Status = 0x01; /* Discharging */
      } else if(PreVoltage<voltage){
        Status = 0x03; /* Charging */
      }
    }
    buff[8] = Status;
    PreVoltage = voltage;
  }//电池信息及数据整合
  
  
           
          if(buff[8]==0x03) OLED_ShowCHinese(40,0,8);//是
          else OLED_ShowCHinese(40,0,9);//否        
          OLED_ShowNum(91,0,soc,3,16); 
}


#endif /* STM32_SENSORTILE */

#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION

/**
 * @brief  Update Microphones characteristic values
 * @param  uint16_t Angle
 * @retval tBleStatus   Status
 */
tBleStatus AudioSourceLocalization_Update(uint16_t Angle)
{  
  tBleStatus ret;
  
  uint8_t buff[2+2];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2  ,Angle);
    
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AudioSourceLocalizationCharHandle, 0, 2+2, buff);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite = sprintf((char *)BufferToWrite, "Error Updating Mic Audio Source Localization Data Char\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;	
}

#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)//建立连接表用来放置器件连接模式
{  
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_BLUEMS};
  uint8_t manuf_data[26] = {
    2,0x0A,0x00 /* 0 dBm */, // Trasmission Power
    8,0x09,NAME_BLUEMS, // Complete Name
    13,0xFF,0x01/*SKD version */,
#ifdef  STM32_NUCLEO
    0x80,
#elif STM32_SENSORTILE
    0x02,
#endif /* STM32_NUCLEO */
    0x00 /* AudioSync+AudioData */,
    0xE0 /* ACC+Gyro+Mag*/,
    0x00 /*  */,
    0x00, /*  */
    0x00, /* BLE MAC start */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
  };
  
  /* BLE MAC */
  manuf_data[20] = bdaddr[5];
  manuf_data[21] = bdaddr[4];
  manuf_data[22] = bdaddr[3];
  manuf_data[23] = bdaddr[2];
  manuf_data[24] = bdaddr[1];
  manuf_data[25] = bdaddr[0];
  
#ifdef STM32_SENSORTILE
  if(TargetBoardFeatures.HandleGGComponent){
    manuf_data[17] |= 0x02; /* Battery Present */
  }
#endif /* STM32_SENSORTILE */

   manuf_data[16] |= 0x04; /* Mic */

  if(TargetBoardFeatures.NumTempSensors==2) {
    manuf_data[17] |= 0x05; /* Two Temperature values*/
  } else if(TargetBoardFeatures.NumTempSensors==1) {
    manuf_data[17] |= 0x04; /* One Temperature value*/
  }//温度传感器

  if(TargetBoardFeatures.HandleHumSensor) {
    manuf_data[17] |= 0x08; /* Humidity */
  }//湿度传感器

  if(TargetBoardFeatures.HandlePressSensor) {
    manuf_data[17] |= 0x10; /* Pressure value*/
  }//气压传感器/压力
  
  /* DS3 DIL24  present*/
  if(TargetBoardFeatures.HWAdvanceFeatures) {    
    /* Accelerometer Events */
    manuf_data[18] |=0x04;    
  }//加速度
  
  /* Code for MotionFX integration - Start Section */
  if(TargetBoardFeatures.osxMotionFXIsInitalized) {
    manuf_data[18] |= 0x01;
    /* Only for IDB05A1 */
    if(TargetBoardFeatures.bnrg_expansion_board == IDB05A1) {
      /* ECompass */
      manuf_data[19] |= 0x40;
    }
  }
  /* Code for MotionFX integration - End Section */

  /* Code for MotionAR integration - Start Section */
  if(TargetBoardFeatures.osxMotionARIsInitalized) {
    manuf_data[19] |= 0x10;
  }//动作识别
  /* Code for MotionAR integration - End Section */

  /* Code for MotionCP integration - Start Section */
  if(TargetBoardFeatures.osxMotionCPIsInitalized) {
    manuf_data[19] |= 0x08;
  }//佩戴识别
  /* Code for MotionCP integration - End Section */

  /* Code for MotionGR integration - Start Section */
  if(TargetBoardFeatures.osxMotionGRIsInitalized) {
    manuf_data[19] |= 0x02;
  }//姿态识别
  /* Code for MotionGR integration - End Section */

  /* Code for BlueVoice integration - Start Section */
  if(TargetBoardFeatures.osxAudioBVIsInitalized) {
    manuf_data[16] |= 0x48; /* AudioSync+AudioData */
  }
  /* Code for BlueVoice integration - End Section */
  
  /* Audio Source Localization */
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION
  if(TargetBoardFeatures.osxAcousticSLIsInitalized) {
    manuf_data[16] |= 0x10;
  }
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */

  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0, 0,
#ifndef MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
                           STATIC_RANDOM_ADDR,
  #else /* MAC_STM32UID_BLUEMS */
                           RANDOM_ADDR,
  #endif /* MAC_STM32UID_BLUEMS */
#else /* MAC_BLUEMS */  
                           PUBLIC_ADDR,  
#endif /* MAC_BLUEMS */
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(26, manuf_data);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

#ifdef OSX_BMS_DEBUG_CONNECTION
  OSX_BMS_PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",addr[5],addr[4],addr[3],addr[2],addr[1],addr[0]);
#endif /* OSX_BMS_DEBUG_CONNECTION */

  ConnectionBleStatus=0;
  
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    DisableHWFeatures();
  }

  ForceReCalibration     =0;
  FirstConnectionConfig  =0;

  /* Code for BlueVoice integration - Start Section */
  if(TargetBoardFeatures.osxAudioBVIsInitalized){
    osx_BlueVoice_ConnectionComplete_CB(handle); 
    {
      int ret;
      ret = aci_l2cap_connection_parameter_update_request(handle,
                                                  8 /* interval_min*/,
                                                  17 /* interval_max */,
                                                  0   /* slave_latency */,
                                                  400 /*timeout_multiplier*/);
      if (ret != BLE_STATUS_SUCCESS) {
        while (1) {
          ;
        }
      }
    }
  }
  /* Code for BlueVoice integration - End Section */  
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;

#ifdef OSX_BMS_DEBUG_CONNECTION  
  OSX_BMS_PRINTF("<<<<<<DISCONNECTED\r\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */

  /* Code for BlueVoice integration - Start Section */
  if(TargetBoardFeatures.osxAudioBVIsInitalized){
    osx_BlueVoice_DisconnectionComplete_CB();
  }
  /* Code for BlueVoice integration - End Section */

  /* Make the device connectable again. */
  set_connectable = TRUE;

  ConnectionBleStatus=0;
  
  if(TargetBoardFeatures.HWAdvanceFeatures) {
    DisableHWFeatures();
  }

  ForceReCalibration     =0;
  FirstConnectionConfig  =0;
  
  /* Reset for any problem during FOTA update */
  SizeOfUpdateBlueFW = 0;
  
  /************************/
  /* Stops all the Timers */
  /************************/
  
  /* Code for MotionFX and MotionGR integration - Start Section */
  /* Stop Timer For MotionFX and MotionGR */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  /* Code for MotionFX and MotionGR integration - End Section */

  /* Code for MotionCP integration - Start Section */
  /* Stop Timer For MotionCP */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  /* Code for MotionCP integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  /* Stop Timer For MotionAR */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  /* Code for MotionAR integration - End Section */
  
  /* Stop Timer For Acc/Gyro/Mag */
  if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  
  /* Stop Timer For Environmental */
  if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
  
  /* Stop Timer For Audio Level and Source Localisation */
  if(HAL_TIM_Base_Stop_IT(&TimAudioDataHandle) != HAL_OK){
    /* Stopping Error */
    Error_Handler();
  }
}

/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  uint8_t Status;  
  if(handle == EnvironmentalCharHandle + 1){
    /* Read Request for Pressure,Humidity, and Temperatures*/
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
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  } else if(handle == AccEventCharHandle +1) {
    /* Read Request for Acc Pedometer DS3 */
    if(TargetBoardFeatures.HWAdvanceFeatures){
      uint16_t StepCount;
      if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
        StepCount = GetStepHWPedometer();
      } else {
        StepCount = 0;
      }
      AccEvent_Notify(StepCount);
    }
  } else if (handle == StdErrCharHandle + 1) {
    /* Send again the last packet for StdError */
    Stderr_Update_AfterRead();
  } else if (handle == TermCharHandle + 1) {
    /* Send again the last packet for Terminal */
    Term_Update_AfterRead();
  /* Code for MotionAR integration - Start Section */
  } else if(handle == ActivityRecCharHandle + 1){
     ActivityRec_Update(ActivityCode);
  /* Code for MotionAR integration - End Section */
  /* Code for MotionCP integration - Start Section */
  } else if(handle == CarryPosRecCharHandle + 1){
    CarryPosRec_Update(CarryPositionCode);
  /* Code for MotionCP integration - End Section */
    
  /* Code for MotionGR integration - Start Section */
  } else if(handle == GestureRecCharHandle + 1){
    GestureRec_Update(GestureRecognitionCode);
  /* Code for MotionGR integration - End Section */
    
#ifdef STM32_SENSORTILE
  } else if(handle == GGCharHandle + 1){
    GG_Update();
#endif /* STM32_SENSORTILE */
  }

  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length) {
  
  /* Code for BlueVoice integration - Start Section */
  if(TargetBoardFeatures.osxAudioBVIsInitalized){
   osx_BlueVoice_AttributeModified_CB(attr_handle, data_length, att_data);
  }
  /* Code for BlueVoice integration - End Section */

   if(attr_handle == QuaternionsCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_QUAT);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
      }
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_QUAT);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Quater=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_QUAT) ? " ON\r\n" : " OFF\r\n\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Quater=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_QUAT) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
  else if(attr_handle == ECompassCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_EC);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
      }
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_EC);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->ECompass=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC) ? " ON\r\n" : " OFF\r\n\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->ECompass=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_EC) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
  /* Code for BlueVoice integration - Start Section */
  else if(attr_handle == BLUEVOICE_tx_handle.CharAudioHandle + 2){
    if(att_data[0] == 01){
      
      InitMics(BV_AUDIO_SAMPLING_FREQUENCY, BV_AUDIO_VOLUME_VALUE);
      
      W2ST_ON_CONNECTION(W2ST_CONNECT_BV_AUDIO);
    } else if (att_data[0] == 0){
      LedOffTargetPlatform();
      
      DeInitMics();
      
      W2ST_OFF_CONNECTION(W2ST_CONNECT_BV_AUDIO);
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->BV_ADPCM=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_AUDIO) ? " ON\r\n" : " OFF\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->BV_ADPCM=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_AUDIO) ? " ON\r\n" : " OFF\r\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }else if(attr_handle == BLUEVOICE_tx_handle.CharAudioSyncHandle + 2){
    if(att_data[0] == 01){      
      W2ST_ON_CONNECTION(W2ST_CONNECT_BV_SYNC);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_BV_SYNC);
    }
#ifdef OSX_BMS_DEBUG_CONNECTION   
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->BV_ADPCM_Sync=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_SYNC) ? " ON\r\n" : " OFF\r\n\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->BV_ADPCM_Sync=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_BV_SYNC) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
  /* Code for BlueVoice integration - End Section */ 
  /* Code for MotionAR integration - Start Section */
   else if(attr_handle == ActivityRecCharHandle + 2){
     if (att_data[0] == 01) {

       Set4GAccelerometerFullScale();

       W2ST_ON_CONNECTION(W2ST_CONNECT_AR);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));
      }
     } else if (att_data[0] == 0){

       Set2GAccelerometerFullScale();

       W2ST_OFF_CONNECTION(W2ST_CONNECT_AR);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
     }
#ifdef OSX_BMS_DEBUG_CONNECTION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite =sprintf((char *)BufferToWrite,"--->ActRec=%s",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR) ? " ON\r\n" : " OFF\r\n\n");
       Term_Update(BufferToWrite,BytesToWrite);
     } else
       OSX_BMS_PRINTF("--->ActRec=%s",  W2ST_CHECK_CONNECTION(W2ST_CONNECT_AR) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
   }
  /* Code for MotionAR integration - End Section */
  /* Code for MotionCP integration - Start Section */
   else if(attr_handle == CarryPosRecCharHandle + 2){
     if (att_data[0] == 01) {

      Set4GAccelerometerFullScale();

       W2ST_ON_CONNECTION(W2ST_CONNECT_CP);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));
      }
     } else if (att_data[0] == 0){

       Set2GAccelerometerFullScale();

       W2ST_OFF_CONNECTION(W2ST_CONNECT_CP);

       /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
     }
#ifdef OSX_BMS_DEBUG_CONNECTION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite =sprintf((char *)BufferToWrite,"--->CarryPosRec=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_CP) ? " ON\r\n" : " OFF\r\n\n");
       Term_Update(BufferToWrite,BytesToWrite);
     } else
       OSX_BMS_PRINTF("--->CarryPosRec=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_CP) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
  /* Code for MotionCP integration - End Section */
  /* Code for MotionGR integration - Start Section */
   else if(attr_handle == GestureRecCharHandle + 2){
     if (att_data[0] == 01) {

       Set4GAccelerometerFullScale();

       W2ST_ON_CONNECTION(W2ST_CONNECT_GR);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + DEFAULT_uhCCR1_Val));
      }
     } else if (att_data[0] == 0){

       Set2GAccelerometerFullScale();

       W2ST_OFF_CONNECTION(W2ST_CONNECT_GR);

       /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
     }
#ifdef OSX_BMS_DEBUG_CONNECTION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite =sprintf((char *)BufferToWrite,"--->GestureRec=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GR) ? " ON\r\n" : " OFF\r\n\n");
       Term_Update(BufferToWrite,BytesToWrite);
     } else
       OSX_BMS_PRINTF("--->GestureRec=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GR) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
  /* Code for MotionGR integration - End Section */
#ifdef OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION  
  else if ( attr_handle == AudioSourceLocalizationCharHandle + 2) {
    //uint8_t ret;
    if (att_data[0] == 01)
    {
      W2ST_ON_CONNECTION(W2ST_CONNECT_SL);
      
      InitMics(AUDIO_SAMPLING_FREQUENCY, AUDIO_VOLUME_VALUE);
      
      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Start_IT(&TimAudioDataHandle) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
    }
    else if (att_data[0] == 0)
    {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_SL);

      DeInitMics();
      
      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Stop_IT(&TimAudioDataHandle) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->AudioSourceLocalization=%s", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_SL)   ? " ON\r\n" : " OFF\r\n\n"));
     Term_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("--->AudioSourceLocalization=%s", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_SL)   ? " ON\r\n" : " OFF\r\n\n"));
    }
#endif /* OSX_BMS_DEBUG_CONNECTION */ 
  }
#endif /* OSX_BMS_ACOUSTIC_SOURCE_LOCALIZATION */
#ifdef STM32_SENSORTILE
   else if(attr_handle == GGCharHandle + 2){
     if (att_data[0] == 01) {
       W2ST_ON_CONNECTION(W2ST_CONNECT_GG_EVENT);
       /* Start the TIM Base generation in interrupt mode */
       if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
         /* Starting Error */
         Error_Handler();
       }
     } else if (att_data[0] == 0){
       W2ST_OFF_CONNECTION(W2ST_CONNECT_GG_EVENT);
       /* Stop the TIM Base generation in interrupt mode */
       if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
         /* Stopping Error */
         Error_Handler();
       }
     }
#ifdef OSX_BMS_DEBUG_CONNECTION
     if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
       BytesToWrite =sprintf((char *)BufferToWrite,"---GG=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT) ? " ON\r\n" : " OFF\r\n\n");
       Term_Update(BufferToWrite,BytesToWrite);
     } else
       OSX_BMS_PRINTF("--->GG=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_GG_EVENT) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  }
#endif /* STM32_SENSORTILE */
  else if(attr_handle == ConfigCharHandle + 2){
    if (att_data[0] == 01) {
      FirstConnectionConfig=1;
    } else if (att_data[0] == 0){
      FirstConnectionConfig=0;
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Calib=%s\r\n\n", FirstConnectionConfig ? "ON" : "OFF");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Calib=%s\r\n\n", FirstConnectionConfig ? "ON" : "OFF");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if(attr_handle == AccGyroMagCharHandle + 2) {
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }

      /* Set the new Capture compare value */
      {
        uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
        /* Set the Capture Compare Register value */
        __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
      }
    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }      
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Acc/Gyro/Mag=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? " ON\r\n" : " OFF\r\n\n");
      Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Acc/Gyro/Mag=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_GYRO_MAG) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if(attr_handle == AccEventCharHandle + 2) {
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ACC_EVENT);
    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ACC_EVENT);
      DisableHWFeatures();
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->AccEvent=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_EVENT) ? " ON\r\n" : " OFF\r\n\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->AccEvent=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ACC_EVENT) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if(attr_handle == EnvironmentalCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_ENV);

      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_ENV);

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"--->Env=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? " ON\r\n" : " OFF\r\n\n");
     Term_Update(BufferToWrite,BytesToWrite);
    } else
      OSX_BMS_PRINTF("--->Env=%s", W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV) ? " ON\r\n" : " OFF\r\n\n");
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if(attr_handle == StdErrCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_ERR);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_ERR);
    }
  } else if(attr_handle == TermCharHandle + 2){
    if (att_data[0] == 01) {
      W2ST_ON_CONNECTION(W2ST_CONNECT_STD_TERM);
    } else if (att_data[0] == 0){
      W2ST_OFF_CONNECTION(W2ST_CONNECT_STD_TERM);
    }
  } else if (attr_handle == TermCharHandle + 1){
    uint32_t SendBackData =1; /* By default Answer with the same message received */

    if(SizeOfUpdateBlueFW!=0) {
      /* BLUEMICROSYSTEM2 firwmare update */
      int8_t RetValue = UpdateFWBlueMS(&SizeOfUpdateBlueFW,att_data, data_length,1);
      if(RetValue!=0) {
        MCR_FAST_TERM_UPDATE_FOR_OTA(((uint8_t *)&RetValue));
        if(RetValue==1) {
          /* if OTA checked */
          BytesToWrite =sprintf((char *)BufferToWrite,"The Board will restart in 5 seconds\r\n");
          Term_Update(BufferToWrite,BytesToWrite);
          OSX_BMS_PRINTF("%s will restart in 5 seconds\r\n",OSX_BMS_PACKAGENAME);
          HAL_Delay(5000);
          HAL_NVIC_SystemReset();
        }
      }
      SendBackData=0;
    } else {
      /* Received one write from Client on Terminal characteristc */
      SendBackData = DebugConsoleCommandParsing(att_data,data_length);
    }

    /* Send it back for testing */
    if(SendBackData) {
      Term_Update(att_data,data_length);
    }
  } else if (attr_handle == AudioLevelCharHandle + 2) {
    //uint8_t ret;
    if (att_data[0] == 01) {
      int32_t Count;
      //uint8_t ret;
      
      W2ST_ON_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL);
      
      InitMics(AUDIO_SAMPLING_FREQUENCY, AUDIO_VOLUME_VALUE);
      
      for(Count=0;Count<TargetBoardFeatures.NumMicSensors;Count++) {
        RMS_Ch[Count]=0;
        DBNOISE_Value_Old_Ch[Count] =0;
      }
      
      /* Start the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Start_IT(&TimAudioDataHandle) != HAL_OK){
        /* Starting Error */
        Error_Handler();
      }
    } else if (att_data[0] == 0) {
      W2ST_OFF_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL);

      DeInitMics();

      /* Stop the TIM Base generation in interrupt mode */
      if(HAL_TIM_Base_Stop_IT(&TimAudioDataHandle) != HAL_OK){
        /* Stopping Error */
        Error_Handler();
      }      
    }
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite = sprintf((char *)BufferToWrite,"--->dB Noise AudioLevel=%s", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)   ? " ON\r\n" : " OFF\r\n\n") );
     Term_Update(BufferToWrite,BytesToWrite);
    }else {
      OSX_BMS_PRINTF("--->dB Noise AudioLevel=%s", (W2ST_CHECK_CONNECTION(W2ST_CONNECT_AUDIO_LEVEL)   ? " ON\r\n" : " OFF\r\n\n"));
    }
#endif /* OSX_BMS_DEBUG_CONNECTION */
  } else if (attr_handle == ConfigCharHandle + 1) {
    /* Received one write command from Client on Configuration characteristc */
    ConfigCommandParsing(att_data, data_length);    
  } else {
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      BytesToWrite =sprintf((char *)BufferToWrite, "Notification UNKNOW handle\r\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Notification UNKNOW handle\r\n");
    }
  }
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;
  static int32_t osxLicenseStart=0;  

  if(osxLicenseStart==0){
    /* No osx License activation phase */
    if((att_data[0]=='?') & (att_data[1]=='?')) {
      /* Print Legend */
      SendBackData=0;

      BytesToWrite =sprintf((char *)BufferToWrite,"Command:\r\n"
        "pr->HW pedometer reset\r\n"
         "info-> System Info\r\n"
#ifdef STM32_SENSORTILE
         "powerstatus-> Battery Status [%% mV]\r\n"
#endif /* STM32_SENSORTILE */
         "versionFw-> FW Version\r\n"
         "versionBle-> Ble Version\r\n");
      Term_Update(BufferToWrite,BytesToWrite);

    } else if((att_data[0]=='p') & (att_data[1]=='r')) {
    /* Reset the pedometer DS3 HW counter */
      ResetHWPedometer();
      SendBackData=0;
    }
    else if(!strncmp("versionFw",(char *)(att_data),9)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"%s_%s_%c.%c.%c\r\n",
#ifdef STM32F401xE
                            "F401"
#elif STM32F446xx
                            "F446"
#elif STM32L476xx
                            "L476"
#else
#error "Undefined STM32 processor type"
#endif
                            ,OSX_BMS_PACKAGENAME,
                            OSX_BMS_VERSION_MAJOR,
                            OSX_BMS_VERSION_MINOR,
                            OSX_BMS_VERSION_PATCH);
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
#ifdef STM32_SENSORTILE
    } else if(!strncmp("powerstatus",(char *)(att_data),11)) {
      SendBackData=0;
      if(TargetBoardFeatures.HandleGGComponent) {
        uint32_t voltage, soc;
        uint8_t v_mode;
        int32_t current;
        /* Update Gas Gouge Status */
        BSP_GG_Task(TargetBoardFeatures.HandleGGComponent,&v_mode);

        /* Read the Gas Gouge Status */
        BSP_GG_GetVoltage(TargetBoardFeatures.HandleGGComponent, &voltage);
        BSP_GG_GetSOC(TargetBoardFeatures.HandleGGComponent, &soc);
        BSP_GG_GetCurrent(TargetBoardFeatures.HandleGGComponent, &current);

        BytesToWrite =sprintf((char *)BufferToWrite,"Battery %ld%% %ld mV Current=%d mA\r\n",soc,voltage,current);
      } else {
        BytesToWrite =sprintf((char *)BufferToWrite,"Battery not present\r\n");
      }
      Term_Update(BufferToWrite,BytesToWrite);
#endif /* STM32_SENSORTILE */
    } else if(!strncmp("info",(char *)(att_data),4)) {
      SendBackData=0;
      
      BytesToWrite =sprintf((char *)BufferToWrite,"\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
#ifdef USE_STM32F4XX_NUCLEO
  #ifdef STM32_NUCLEO
        "\tSTM32F401RE-Nucleo board"
  #endif /* STM32_NUCLEO */
#elif USE_STM32L4XX_NUCLEO
  #ifdef STM32_SENSORTILE
        "\tSTM32476RG-SensorTile board"
  #elif STM32_NUCLEO
        "\tSTM32L476RG-Nucleo board"
  #endif /* STM32_SENSORTILE */
#endif /* USE_STM32L4XX_NUCLEO */
          "\r\n",
          OSX_BMS_PACKAGENAME,
          OSX_BMS_VERSION_MAJOR,OSX_BMS_VERSION_MINOR,OSX_BMS_VERSION_PATCH);
      Term_Update(BufferToWrite,BytesToWrite);

      BytesToWrite =sprintf((char *)BufferToWrite,"\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n",
#elif defined (__CC_ARM)
        " (KEIL)\r\n",
#elif defined (__GNUC__)
        " (openstm32)\r\n",
#endif
          HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
           __DATE__,__TIME__);
      Term_Update(BufferToWrite,BytesToWrite);
      
#ifdef STM32_NUCLEO
  #ifdef IKS01A1
      BytesToWrite =sprintf((char *)BufferToWrite,"Code compiled for X-NUCLEO-IKS01A1\r\n");
  #elif IKS01A2
      BytesToWrite =sprintf((char *)BufferToWrite,"Code compiled for X-NUCLEO-IKS01A2\r\n");
  #endif /* IKS01A1 */
      Term_Update(BufferToWrite,BytesToWrite);
#endif /* STM32_NUCLEO */

    }  if(!strncmp("upgradeFw",(char *)(att_data),9)) {
      uint32_t uwCRCValue;
      uint8_t *PointerByte = (uint8_t*) &SizeOfUpdateBlueFW;

      SizeOfUpdateBlueFW=atoi((char *)(att_data+9));
      PointerByte[0]=att_data[ 9];
      PointerByte[1]=att_data[10];
      PointerByte[2]=att_data[11];
      PointerByte[3]=att_data[12];

      /* Check the Maximum Possible OTA size */
      if(SizeOfUpdateBlueFW>OTA_MAX_PROG_SIZE) {
        OSX_BMS_PRINTF("OTA %s SIZE=%ld > %d Max Allowed\r\n",OSX_BMS_PACKAGENAME,SizeOfUpdateBlueFW, OTA_MAX_PROG_SIZE);
        /* Answer with a wrong CRC value for signaling the problem to BlueMS application */
        PointerByte[0]= att_data[13];
        PointerByte[1]=(att_data[14]!=0) ? 0 : 1;/* In order to be sure to have a wrong CRC */
        PointerByte[2]= att_data[15];
        PointerByte[3]= att_data[16];
        BytesToWrite = 4;
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        PointerByte = (uint8_t*) &uwCRCValue;
        PointerByte[0]=att_data[13];
        PointerByte[1]=att_data[14];
        PointerByte[2]=att_data[15];
        PointerByte[3]=att_data[16];

        OSX_BMS_PRINTF("OTA %s SIZE=%ld uwCRCValue=%lx\r\n",OSX_BMS_PACKAGENAME,SizeOfUpdateBlueFW,uwCRCValue);
	  
        /* Reset the Flash */
        StartUpdateFWBlueMS(SizeOfUpdateBlueFW,uwCRCValue);
      
#ifdef USE_STM32F4XX_NUCLEO
        /* Save the Meta Data Manager.
         * We had always a Meta Data Manager*/
        SaveMetaDataManager();
        NecessityToSaveMetaDataManager =0;
#endif /* USE_STM32F4XX_NUCLEO */

        /* Reduce the connection interval */
        {
          int ret = aci_l2cap_connection_parameter_update_request(connection_handle,
                                                        10 /* interval_min*/,
                                                        10 /* interval_max */,
                                                        0   /* slave_latency */,
                                                        400 /*timeout_multiplier*/);
          /* Go to infinite loop if there is one error */
          if (ret != BLE_STATUS_SUCCESS) {
            while (1) {
              OSX_BMS_PRINTF("Problem Changing the connection interval\r\n");
            }
          }
        }
        
        /* Signal that we are ready sending back the CRV value*/
        BufferToWrite[0] = PointerByte[0];
        BufferToWrite[1] = PointerByte[1];
        BufferToWrite[2] = PointerByte[2];
        BufferToWrite[3] = PointerByte[3];
        BytesToWrite = 4;
        Term_Update(BufferToWrite,BytesToWrite);
      }
      
      SendBackData=0;
      
    } else if(!strncmp("versionBle",(char *)(att_data),10)) { //else if(!strcmp("versionBle\n",(char *)(att_data))) {
      uint8_t  hwVersion;
      uint16_t fwVersion;
      /* get the BlueNRG HW and FW versions */
      getBlueNRGVersion(&hwVersion, &fwVersion);
      BytesToWrite =sprintf((char *)BufferToWrite,"%s_%d.%d.%c\r\n",
                            (hwVersion > 0x30) ? "BleMS" : "Ble",
                            fwVersion>>8, 
                            (fwVersion>>4)&0xF,
                            (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a');
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    } else if((att_data[0]=='u') & (att_data[1]=='i') & (att_data[2]=='d')) {
      /* Write back the STM32 UID */
      uint8_t *uid = (uint8_t *)STM32_UUID;
      uint32_t MCU_ID = STM32_MCU_ID[0]&0xFFF;
      BytesToWrite =sprintf((char *)BufferToWrite,"%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\r\n",
                            uid[ 3],uid[ 2],uid[ 1],uid[ 0],
                            uid[ 7],uid[ 6],uid[ 5],uid[ 4],
                            uid[11],uid[ 10],uid[9],uid[8],
                            MCU_ID);
      Term_Update(BufferToWrite,BytesToWrite);
      SendBackData=0;
    } else if((att_data[0]=='l') & (att_data[1]=='i') & (att_data[2]=='c')) {
      /* Write back the osx License status */
      int32_t Index=0;
      MDM_OsxLicenseType_t osxLic = known_OsxLic[Index].LicEnum;
      while(osxLic!=OSX_END) {
        //BytesToWrite =sprintf((char *)BufferToWrite,"osxMotion%s %s\r\n", MDM_LicTable[osxLic].LicName,(MetaDataLicenseStatus(osxLic) ? "OK" : "--"));
        BytesToWrite =sprintf((char *)BufferToWrite,"osx%s%s %s\r\n", MDM_LicTable[osxLic].LicType,MDM_LicTable[osxLic].LicName,(MetaDataLicenseStatus(osxLic) ? "OK" : "--"));
        Term_Update(BufferToWrite,BytesToWrite);
        osxLic = known_OsxLic[++Index].LicEnum;
      }
      SendBackData=0;
    } else if((att_data[0]=='X') & (att_data[1]=='X') & (att_data[2]=='0')){
      BytesToWrite =sprintf((char *)BufferToWrite,"Reset of All the osx licenses\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite =sprintf((char *)BufferToWrite,"The Board will restart in 5 seconds\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
	  EraseMetaDataManager();
      HAL_Delay(5000);
      HAL_NVIC_SystemReset();
    } else {
      /* Try to understand if we want to Initializes one osx License */
      osxLicenseStart = MDM_LicenseParsing(att_data, data_length);
      if(osxLicenseStart) {
        SendBackData=0;
      }
    }
  } else {
    /* We need to Finalize one osx License */
    osxLicenseStart = MDM_LicenseParsing(att_data, data_length);
    if(osxLicenseStart<0) {
      MDM_OsxLicenseType_t OsxLicense = (MDM_OsxLicenseType_t) (-osxLicenseStart);
      MDM_PayLoadLic_t *PayLoad = (MDM_PayLoadLic_t *) MDM_LicTable[OsxLicense].Address;

      osxLicenseStart = 0;
      switch(OsxLicense) {
        
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
          while(1) {
                ; /* Should never reach this point... error */
        }
      }
      BytesToWrite =sprintf((char *)BufferToWrite,"License Write Successful\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
    }    
	SendBackData=0;
  }
  
#if 1

  /* If it's something not yet recognized... only for testing.. This must be removed*/
   if(SendBackData) {
    if(att_data[0]=='@') {

      if(att_data[1]=='T') {
        uint8_t loc_att_data[6];
        uint8_t loc_data_length=6;

        loc_att_data[0] = (FEATURE_MASK_TEMP1>>24)&0xFF;
        loc_att_data[1] = (FEATURE_MASK_TEMP1>>16)&0xFF;
        loc_att_data[2] = (FEATURE_MASK_TEMP1>>8 )&0xFF;
        loc_att_data[3] = (FEATURE_MASK_TEMP1    )&0xFF;
        loc_att_data[4] = 255;
        
        switch(att_data[2]) {
          case 'L':
            loc_att_data[5] = 50; /* @5S */
          break;
          case 'M':
            loc_att_data[5] = 10; /* @1S */
          break;
          case 'H':
            loc_att_data[5] = 1; /* @100mS */
          break;
          case 'D':
            loc_att_data[5] = 0; /* Default */
          break;
        }
        SendBackData = ConfigCommandParsing(loc_att_data,loc_data_length);
      } else if(att_data[1]=='A') {
        uint8_t loc_att_data[6];
        uint8_t loc_data_length=6;

        loc_att_data[0] = (FEATURE_MASK_ACC>>24)&0xFF;
        loc_att_data[1] = (FEATURE_MASK_ACC>>16)&0xFF;
        loc_att_data[2] = (FEATURE_MASK_ACC>>8 )&0xFF;
        loc_att_data[3] = (FEATURE_MASK_ACC    )&0xFF;
        loc_att_data[4] = 255;

        switch(att_data[2]) {
          case 'L':
            loc_att_data[5] = 50; /* @5S */
          break;
          case 'M':
            loc_att_data[5] = 10; /* @1S */
          break;
          case 'H':
            loc_att_data[5] = 1; /* @100mS */
          break;
          case 'D':
            loc_att_data[5] = 0; /* Default */
          break;
        }
        SendBackData = ConfigCommandParsing(loc_att_data,loc_data_length);
      } else if(att_data[1]=='M') {
        uint8_t loc_att_data[6];
        uint8_t loc_data_length=6;

        loc_att_data[0] = (FEATURE_MASK_MIC>>24)&0xFF;
        loc_att_data[1] = (FEATURE_MASK_MIC>>16)&0xFF;
        loc_att_data[2] = (FEATURE_MASK_MIC>>8 )&0xFF;
        loc_att_data[3] = (FEATURE_MASK_MIC    )&0xFF;
        loc_att_data[4] = 255;

        switch(att_data[2]) {
          case 'L':
            loc_att_data[5] = 50; /* @5S */
          break;
          case 'M':
            loc_att_data[5] = 10; /* @1S */
          break;
          case 'H':
            loc_att_data[5] = 1; /* @100mS */
          break;
          case 'D':
            loc_att_data[5] = 0; /* Default */
          break;
        }
        SendBackData = ConfigCommandParsing(loc_att_data,loc_data_length);
      }    
    }
  }
#endif
  
  return SendBackData;
}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval uint32_t SendItBack true/false
 */
static uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
  uint8_t Command = att_data[4];
  uint8_t Data    = att_data[5];
  uint32_t SendItBack = 1;

  switch (FeatureMask) {
    case FEATURE_MASK_SENSORFUSION_SHORT:
      /* Sensor Fusion */
      switch (Command) {
        case W2ST_COMMAND_CAL_STATUS:
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Calibration STATUS Signal For Features=%lx\n\r",FeatureMask);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          /* Replay with the calibration status for the feature */
          /* Control the calibration status */
          {
            unsigned char isCal = osx_MotionFX_compass_isCalibrated();
            Config_Notify(FeatureMask,Command,isCal ? 100: 0);
          }
        break;
        case W2ST_COMMAND_CAL_RESET:
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Calibration RESET Signal For Feature=%lx\n\r",FeatureMask);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          /* Reset the calibration */
          ForceReCalibration=1;
        break;
        case W2ST_COMMAND_CAL_STOP:
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite =sprintf((char *)BufferToWrite,"Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Calibration STOP Signal For Feature=%lx\n\r",FeatureMask);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          /* Do nothing in this case */
        break;
        default:
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
            BytesToWrite =sprintf((char *)BufferToWrite, "Calibration UNKNOW Signal For Feature=%lx\n\r",FeatureMask);
            Stderr_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Calibration UNKNOW Signal For Feature=%lx\n\r",FeatureMask);
          }
      }      
    break;
  case FEATURE_MASK_ACC_EVENTS:
    /* Acc events */
#ifdef OSX_BMS_DEBUG_CONNECTION
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%c D=%x\n\r",FeatureMask,Command,Data);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      OSX_BMS_PRINTF("Conf Sig F=%lx C=%c D=%x\n\r",FeatureMask,Command,Data);
    }
#endif /* OSX_BMS_DEBUG_CONNECTION */      
    switch(Command) {
      case 'f':
        /* FreeFall */
        switch(Data) {
          case 1:
            EnableHWFreeFall();
             Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWFreeFall();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
      break;
      case 'd':
        /* Double Tap */
        switch(Data) {
          case 1:
            EnableHWDoubleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWDoubleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 's':
        /* Single Tap */
        switch(Data) {
          case 1:
            EnableHWSingleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWSingleTap();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 'p':
        /* Pedometer */
        switch(Data) {
          case 1:
            EnableHWPedometer();
            ResetHWPedometer();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWPedometer();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
       break;
      case 'w':
        /* Wake UP */
        switch(Data) {
          case 1:
            EnableHWWakeUp();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWWakeUp();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
       break;
       case 't':
         /* Tilt */
        switch(Data) {
          case 1:
            EnableHWTilt();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWTilt();
            Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 'o' :
        /* Tilt */
        switch(Data) {
        case 1:
          EnableHWOrientation6D();
          Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
          break;
        case 0:
          DisableHWOrientation6D();
          Config_Notify(FEATURE_MASK_ACC_EVENTS,Command,Data);
          break;
        }
      break;
    }
    break;
    /* Environmental features */
    case FEATURE_MASK_TEMP1:
    case FEATURE_MASK_TEMP2:
    case FEATURE_MASK_PRESS:
    case FEATURE_MASK_HUM:
      switch(Command) {
        case 255:
          /* Change the Sending interval */
          if(Data!=0) {
            /* Multiple of 100mS */
            __HAL_TIM_SET_AUTORELOAD(&TimEnvHandle,(Data*200 - 1));
            __HAL_TIM_SET_COUNTER(&TimEnvHandle,0);
            TimEnvHandle.Instance->EGR = TIM_EGR_UG;
          } else {
            /* Default Values */
            __HAL_TIM_SET_AUTORELOAD(&TimEnvHandle,(ENV_UPDATE_MUL_100MS*200 - 1));
            __HAL_TIM_SET_COUNTER(&TimEnvHandle,0);
          }
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          SendItBack = 0;
        break;
      }
    break;
    /* Inertial features */
    case FEATURE_MASK_ACC:
    case FEATURE_MASK_GRYO:
    case FEATURE_MASK_MAG:
      switch(Command) {
        case 255:
          /* Change the Sending interval */
          if(Data!=0) {
            /* Multiple of 100mS */
            uhCCR4_Val  = 1000*Data;
          } else {
            /* Default Value */
            uhCCR4_Val  = DEFAULT_uhCCR4_Val;
          }
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          SendItBack = 0;
        break;
      }
    break;
    /* Environmental features */
    case FEATURE_MASK_MIC:
      switch(Command) {
        case 255:
          /* Change the Sending interval */
          if(Data!=0) {
            /* Multiple of 100mS */
            __HAL_TIM_SET_AUTORELOAD(&TimAudioDataHandle,(Data*1000 - 1));
            __HAL_TIM_SET_COUNTER(&TimAudioDataHandle,0);
            TimAudioDataHandle.Instance->EGR = TIM_EGR_UG;
          } else {
            /* Default Values */
            __HAL_TIM_SET_AUTORELOAD(&TimAudioDataHandle,(MICS_DB_UPDATE_MUL_10MS*100 - 1));
            __HAL_TIM_SET_COUNTER(&TimAudioDataHandle,0);
          }
#ifdef OSX_BMS_DEBUG_CONNECTION
          if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            OSX_BMS_PRINTF("Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
          }
#endif /* OSX_BMS_DEBUG_CONNECTION */
          SendItBack = 0;
        break;
      }
    break;
  }
  return SendItBack;
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        if(TargetBoardFeatures.bnrg_expansion_board==IDB05A1) {
              evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            } else {
              evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            }
        break;
      }
    }
    break;
  }
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
