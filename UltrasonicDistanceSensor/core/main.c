#include "stm8s.h"

#define TIM1_PRESCALER_1                    (uint8_t)0x00
#define TIM1_PERIOD                         (uint16_t)399
#define HalfDutyCycle                       (uint16_t)200
#define TIM1_REPETITION_COUNT_0             (uint8_t)0
#define I2C_CLK_FREQ_HZ_100000              100000
#define I2C_CLK_FREQ_HZ_400000              400000
#define TIM2_PRESCALER_15                   16
#define TIM2_REPETITION_COUNT_0             0
#define FW_MAJOR                            7
#define FW_MINOR                            5
#define read_digital                        (uint8_t)0x15
#define sensor_change_ADDR                  (uint8_t)0x16
#define sensor_FW_Version                   (uint8_t)0x17
#define FLASH_ADDR                          (uint32_t)0x00004000
#define EEPROM_FLAG                         (uint32_t)0x00004001
#define AF_FLAG_LOCATION                    2
#define EIGHT_CLK_CYCLE                     8
#define MAX_TIM_VALUE                       65535
#define I2C_OVR_FLAG                        3
#define OPTIONAL_IN_LOOP
#define OPTION_BYTE_AFR                     0x4803   
#define AFR0                                0x01
#define ADDR_Change                         1
#define FirstDeviceADDR                     (uint8_t)0x32

#define I2C_INPUT_CLC_FREQ(ClockFreq)       (ClockFreq/1000000)
#define UNUSED_VARIABLES(x)                 (void)x
#define logicOneAsSowtware()                (I2C->OARH |= 1<<6)

static void ClockConfig(void);
static void GpioConfig(void);
static void Timer_Config(void);
static void I2C_Config(void);
static void Delay_us(void);
static void SYS_Init(void);
static void NoAffectAsHardware(void);
static void EnableOutputMode(void);
static void DisableOutputMode(void);
static void DisableInterruptForTrigPin(void);
static void EnableTrigInterrupt(void);
static void WriteADDRToEEPROM(uint8_t Address);
static void Flash_Init(void);
static void InterruptDisableForReceiver(void);
static void WriteFLAGToEEPROM(void);
static void FlagFunctionForData(void);
static void i2c_disable_interrupts(void);
static void i2c_enable_interrupts(void);
static void TransmitOperation(void);
static void ResetGPIO(void);
static void WaitForEndingOfEcho(void);

void TransmitSoundWave(void);
void InterruptEnableForReceiver(void);
void PreparingSensorValue(void);
void ChangeSlaveADDR(void);
void TransmitFW_Version(void);
void getSlaveADDR(void);
void BlockForNoise(void);
void InterruptForTrigPin(void);
void ClearAF_Flag(void);
void ClearRelatedI2C_Flag(void);
void ClearADDR_Flag(void);
void ClearStopFlag(void);
void HardwareControl(void);
void DisableInterrupt(void);
void TimSpesificIrqHandler(void);
void TrigIrqHandlr(void);
void IrqHandlerForReceiving(void);
void I2C_HandlerForReceiving(void);
void ClearOverrun_Flag(void);
void I2C_Addr_Detection(void);
void verify_option( uint16_t address, uint8_t data );

uint32_t ReadFLASH(uint32_t* Address);
OPTIONAL_IN_LOOP static void StartSleepMode(void);

uint8_t NewSlaveADDR;
uint8_t DummyForReading;
uint8_t DummyForWriting = (uint8_t)(0x00);
uint8_t I2C_OWN_ADDR;
uint8_t ValueInRelatedFlashSector;
uint8_t COMMAND_CODE;

volatile uint8_t IdentifierFlag = 0;
volatile uint8_t ADDR_Flag = 0;
volatile uint8_t Flag = 0;
volatile uint8_t FlagForReceiving = 1;
volatile uint8_t FW_Flag;
volatile uint8_t FlagForDistance;
volatile uint8_t FlagForData;
volatile uint8_t TimerFinishFlag;
volatile uint8_t FlagForTransmittingSoundWave = 1;
volatile uint8_t ReceivedEcho = 1;
  
uint16_t Distance;

int main(void)
{
      
  SYS_Init();
  while (1)
  { 
    if(FlagForData)
    {
      PreparingSensorValue();
      FlagForData = RESET ;
    }
    //StartSleepMode();
  }
   
}

OPTIONAL_IN_LOOP static void StartSleepMode(void)
{
    wfi();
}

static void SYS_Init(void)
{
  ClockConfig();
  GpioConfig();
  Flash_Init();
  Timer_Config();
  I2C_Config();  
  Delay_us();
  enableInterrupts();
}

static void ClockConfig(void)
{
  
  CLK_DeInit();
  
  CLK_HSICmd(ENABLE);
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

  CLK_AdjustHSICalibrationValue(CLK_HSITRIMVALUE_0);
  
  while(CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == RESET);
  

  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
  CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1);
  

  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER1,ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C,ENABLE);
  
}

static void GpioConfig(void)
{
  
  ResetGPIO();

  GPIO_Init(GPIOA,GPIO_PIN_3,GPIO_MODE_IN_FL_NO_IT);

  GPIO_Init(GPIOB,GPIO_PIN_4,GPIO_MODE_OUT_OD_HIZ_FAST); //SCL
  GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_MODE_OUT_OD_HIZ_FAST); //SDA
  
  GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_IN_FL_IT);  
  GPIO_Init(GPIOC,GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);

}

static void Timer_Config(void)
{
  
  TIM1_DeInit();

  TIM1_TimeBaseInit(TIM1_PRESCALER_1, 
                       TIM1_COUNTERMODE_UP,
                       TIM1_PERIOD,TIM1_REPETITION_COUNT_0);
  

  TIM1_CounterModeConfig(TIM1_COUNTERMODE_UP);
  
  
  TIM1_OC1Init(TIM1_OCMODE_PWM1,TIM1_OUTPUTSTATE_ENABLE,TIM1_OUTPUTNSTATE_DISABLE,
               HalfDutyCycle,TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_HIGH,
               TIM1_OCIDLESTATE_RESET,
               TIM1_OCNIDLESTATE_RESET);
  
  TIM1_OC1PreloadConfig(ENABLE);        

  TIM1_OC2Init(TIM1_OCMODE_PWM1,TIM1_OUTPUTSTATE_ENABLE,TIM1_OUTPUTNSTATE_DISABLE,
             HalfDutyCycle,TIM1_OCPOLARITY_LOW,TIM1_OCNPOLARITY_HIGH,
             TIM1_OCIDLESTATE_RESET,
             TIM1_OCNIDLESTATE_RESET);
  
  
  TIM1_OC2PreloadConfig(ENABLE);
  
  TIM1_ARRPreloadConfig(ENABLE);
  
  //DEAD TIME   
  TIM1_BDTRConfig(TIM1_OSSISTATE_ENABLE, TIM1_LOCKLEVEL_1, 117, TIM1_BREAK_ENABLE, 
                  TIM1_BREAKPOLARITY_HIGH, TIM1_AUTOMATICOUTPUT_ENABLE);
  
  TIM1_Cmd(DISABLE);
  TIM1_CtrlPWMOutputs(DISABLE);
  
}
 
void InterruptEnableForReceiver(void)
{
  
   GPIO_Init(GPIOD,GPIO_PIN_2,GPIO_MODE_IN_FL_IT);
   
   EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);
     
   enableInterrupts();
   
}

static void I2C_Config(void)
{
  
  I2C_Addr_Detection();
  
  I2C_DeInit();
  
  I2C_Init(I2C_CLK_FREQ_HZ_100000, I2C_OWN_ADDR, 
              I2C_DUTYCYCLE_2, I2C_ACK_CURR, 
              I2C_ADDMODE_7BIT, I2C_INPUT_CLC_FREQ(CLK_GetClockFreq()) );
 

  logicOneAsSowtware();
  
  I2C_StretchClockCmd(ENABLE);
  
  I2C_FastModeDutyCycleConfig(I2C_DUTYCYCLE_2);
  
  I2C_AcknowledgeConfig(I2C_ACK_CURR);
  
  i2c_enable_interrupts();

  WriteADDRToEEPROM(I2C_OWN_ADDR);
  
  I2C_Cmd(ENABLE);
  
}

void PreparingSensorValue(void)
{ 

   ReceivedEcho = SET;
   
   TransmitSoundWave();
     
   InterruptDisableForReceiver();
   
      
}

static void NoAffectAsHardware(void)
{
  
  GPIO_Init(GPIOA,GPIO_PIN_3,GPIO_MODE_IN_FL_NO_IT);
  
}

static void TransmitSoundWave(void)
{
  
  InterruptEnableForReceiver();
  
  TIM2_SetCounter(0); 
  
  TIM1_Cmd(ENABLE);  
  TIM1_CtrlPWMOutputs(ENABLE);

  while(FlagForTransmittingSoundWave);
  
  WaitForEndingOfEcho();
  
  
}

static void Delay_us(void)
{

  TIM2_DeInit();
  
  TIM2_TimeBaseInit(TIM2_PRESCALER_16,MAX_TIM_VALUE);
 
  TIM2_Cmd(ENABLE);
  
}

void ChangeSlaveADDR(void)
{

  I2C->OARL = (uint8_t)(NewSlaveADDR<<1);
  
  WriteADDRToEEPROM(NewSlaveADDR);

  WriteFLAGToEEPROM();
  
}

void TransmitFW_Version(void)
{  
    FW_Flag = SET;
}

void getSlaveADDR(void)
{
  ADDR_Flag = SET; 
}

void BlockForNoise(void)
{
  
  GPIO_Init(GPIOA,GPIO_PIN_3,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_WriteLow(GPIOA,GPIO_PIN_3); ///

}

void InterruptForTrigPin(void)
{
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_RISE_ONLY);
  enableInterrupts();
}

static void EnableOutputMode(void)
{
  GPIO_Init(GPIOC,GPIO_PIN_6,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(GPIOC,GPIO_PIN_7,GPIO_MODE_OUT_PP_HIGH_FAST);
}

static void DisableOutputMode(void)
{
  GPIO_Init(GPIOC,GPIO_PIN_6,GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(GPIOC,GPIO_PIN_7,GPIO_MODE_IN_FL_NO_IT);
}

void ClearRelatedI2C_Flag(void)
{
  
  if(I2C_GetITStatus(I2C_ITPENDINGBIT_OVERRUNUNDERRUN))
  {
       ClearOverrun_Flag();  
       I2C_ClearITPendingBit(I2C_ITPENDINGBIT_OVERRUNUNDERRUN);
  }
  
  if(I2C_CheckEvent(I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED))
  {
       ClearADDR_Flag();  
       
  }
  if(I2C_CheckEvent(I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED))
  {
       ClearADDR_Flag();  
  }
  
  if(I2C_CheckEvent(I2C_EVENT_SLAVE_STOP_DETECTED))
  {
       ClearStopFlag();
  }
   
  
  if(I2C_GetITStatus(I2C_ITPENDINGBIT_ACKNOWLEDGEFAILURE))
  {
    if(I2C_CheckEvent(I2C_EVENT_SLAVE_ACK_FAILURE))
    {
      ClearAF_Flag();
      I2C_ClearITPendingBit(I2C_ITPENDINGBIT_ACKNOWLEDGEFAILURE);
    }
  }
  
}

void ClearAF_Flag(void)
{
  I2C->SR2 &= ~(1<<AF_FLAG_LOCATION);
}


void ClearOverrun_Flag(void)
{
  I2C->SR2 &= ~(1<<I2C_OVR_FLAG);
}

void HardwareControl(void)
{
  GPIO_Init(GPIOA,GPIO_PIN_3,GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_WriteLow(GPIOA,GPIO_PIN_3); ///
}

void ClearADDR_Flag(void)
{
  DummyForReading = I2C->SR1;
  DummyForReading = I2C->SR3;
  UNUSED_VARIABLES(DummyForReading); 
}

void ClearStopFlag(void)
{
  DummyForReading = I2C->SR1;
  I2C->CR2 |= DummyForWriting; 
  UNUSED_VARIABLES(DummyForWriting);
}

void DisableInterrupt(void)
{
  
  GPIO_Init(GPIOD,GPIO_PIN_2,GPIO_MODE_IN_FL_NO_IT);
  
}

void TimSpesificIrqHandler(void)
{

       ++Flag;

       if(Flag == EIGHT_CLK_CYCLE)
       {

           TIM1_Cmd(DISABLE);
           TIM1_ITConfig(TIM1_IT_UPDATE,DISABLE);
           TIM1_CtrlPWMOutputs(DISABLE);
         
       }

}

void TrigIrqHandlr(void)
{

     if(GPIO_ReadInputPin(GPIOC,GPIO_PIN_4))
     {
       GPIO_WriteLow(GPIOC,GPIO_PIN_5); 
       TransmitSoundWave();
       InterruptEnableForReceiver();
       IdentifierFlag = SET;
     }
     DisableInterruptForTrigPin();
}

void IrqHandlerForReceiving(void)
{

  if(ReceivedEcho == SET)
  {
       FlagForTransmittingSoundWave = RESET;
       
       TIM1_Cmd(DISABLE);
       TIM1_CtrlPWMOutputs(DISABLE);
       
       Distance = TIM2_GetCounter()/58;
    
       FlagForDistance = SET;
               
       i2c_enable_interrupts();
       
       ReceivedEcho = RESET;
  }

}

void I2C_HandlerForReceiving(void)
{

  ClearRelatedI2C_Flag();
  
        if(I2C_CheckEvent(I2C_EVENT_SLAVE_BYTE_RECEIVED))
        {
          if(ADDR_Flag == SET)
          {
            
            NewSlaveADDR = I2C_ReceiveData();
            ChangeSlaveADDR();
            ADDR_Flag = RESET;
      
          }
          else
          {

             COMMAND_CODE = I2C_ReceiveData();
          
             switch(COMMAND_CODE)
             {
                  case read_digital        :  FlagFunctionForData() ;   break;
                  case sensor_change_ADDR  :  getSlaveADDR()        ;   break;
                  case sensor_FW_Version   :  TransmitFW_Version()  ;   break;
                  default                  :  /*NOTHING TO DO*/     ;
             }
          }
            
        }
        
        TransmitOperation();
}

static void DisableInterruptForTrigPin(void)
{
  GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_IN_FL_NO_IT);
}

static void EnableTrigInterrupt(void)
{
   GPIO_Init(GPIOC,GPIO_PIN_4,GPIO_MODE_IN_FL_IT);
   EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC, EXTI_SENSITIVITY_RISE_ONLY);
   enableInterrupts();
}

static void WriteADDRToEEPROM(uint8_t Address)
{

  FLASH_Unlock(FLASH_MEMTYPE_DATA);

  while(!FLASH_GetFlagStatus(FLASH_FLAG_DUL));
  
  FLASH_EraseByte(FLASH_ADDR);

  FLASH_ProgramByte(FLASH_ADDR, Address);

  while(FLASH_WaitForLastOperation(FLASH_MEMTYPE_DATA) != FLASH_STATUS_SUCCESSFUL_OPERATION);  
    
  FLASH_Lock(FLASH_MEMTYPE_DATA);

}

static void WriteFLAGToEEPROM(void)
{

  FLASH_Unlock(FLASH_MEMTYPE_DATA);

  while(!FLASH_GetFlagStatus(FLASH_FLAG_DUL));
  
  FLASH_EraseByte(EEPROM_FLAG);

  FLASH_ProgramByte(EEPROM_FLAG, ADDR_Change);

  while(FLASH_WaitForLastOperation(FLASH_MEMTYPE_DATA) != FLASH_STATUS_SUCCESSFUL_OPERATION);

  FLASH_Lock(FLASH_MEMTYPE_DATA);

}

uint32_t ReadFLASH(uint32_t* Address)
{
  return *Address;
}

static void Flash_Init(void)
{
  
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  
}

static void InterruptDisableForReceiver(void)
{
  GPIO_Init(GPIOD,GPIO_PIN_2,GPIO_MODE_IN_FL_NO_IT);
}

void I2C_Addr_Detection(void)
{
  if(FLASH_ReadByte(EEPROM_FLAG) != ADDR_Change)
  { 
     I2C_OWN_ADDR = FirstDeviceADDR;
     //0x1A  -> write addr
     //0x19 -> read addr
  }
  else
  {
     ValueInRelatedFlashSector = FLASH_ReadByte(FLASH_ADDR);
     I2C_OWN_ADDR = ValueInRelatedFlashSector;
  }
}

static void FlagFunctionForData(void)
{
  i2c_disable_interrupts();
  FlagForData = SET;
}

static void i2c_disable_interrupts(void)
{
   I2C_ITConfig(I2C_IT_ERR,DISABLE);
   I2C_ITConfig(I2C_IT_EVT,DISABLE);
   I2C_ITConfig(I2C_IT_BUF,DISABLE);
}

static void i2c_enable_interrupts(void)
{
    I2C_ITConfig(I2C_IT_ERR,ENABLE);
    I2C_ITConfig(I2C_IT_EVT,ENABLE);
    I2C_ITConfig(I2C_IT_BUF,ENABLE);
}

static void TransmitOperation(void)
{
        if(I2C_GetFlagStatus(I2C_FLAG_TXEMPTY))
        {
          if(FW_Flag)
          {
            I2C_SendData(FW_MAJOR);
            I2C_SendData(FW_MINOR);
            FW_Flag = RESET;
          }
          if(FlagForDistance)
          {
            I2C_SendData(Distance);
            FlagForDistance = RESET;
          }
        }
}

static void ResetGPIO(void)
{
  GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOC);
  GPIO_DeInit(GPIOD);
}

void verify_option( uint16_t address, uint8_t data )
{
  uint16_t stored_data = FLASH_ReadOptionByte( address );
  
  if( ( FLASH_OPTIONBYTE_ERROR == stored_data ) || ( data != (uint8_t)( stored_data>>8 ) ) )
  {
    FLASH_Unlock( FLASH_MEMTYPE_DATA );
    FLASH_EraseOptionByte( address );
    FLASH_ProgramOptionByte( address, data );
    FLASH_Lock( FLASH_MEMTYPE_DATA );

    IWDG->KR = IWDG_KEY_ENABLE;
  }  

}

static void WaitForEndingOfEcho(void)
{
  while(TIM2_GetCounter() < 100);
}

#ifdef USE_FULL_ASSERT
void assert_failed(u8* file,u32 line)
{
  while(1);
}
#endif
