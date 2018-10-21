/* Includes ------------------------------------------------------------------*/
#include "proj_conf.h"
#include "stm8_tsl_api.h"


/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  LOFF = 0,
  LON = 1
} LedState_T;

typedef enum
{
  OFF = 0,
  AUTO = 1,
  MAN100=2,
  MAN50=3,
  MAN25=4
} OutMode_T;

  typedef enum{ // PWM mode
  PWM0=0,
  PWM25=1,
  PWM50=2,
  PWM100=3
} PwmMode_T;

/* Private define ------------------------------------------------------------*/
#define MAXTIMER (1800)//60*30sec



/* Private macro -------------------------------------------------------------*/
/* LD1 (PB1) */
#define LED1_PIN_MASK  ((u8)0x02)
#define LED1_PORT_ODR  GPIOB->ODR
#define LED1_PORT_DDR  GPIOB->DDR
#define LED1_PORT_CR1  GPIOB->CR1

/* LD2  (PB2) */
#define LED2_PIN_MASK  ((u8)0x04)
#define LED2_PORT_ODR  GPIOB->ODR
#define LED2_PORT_DDR  GPIOB->DDR
#define LED2_PORT_CR1  GPIOB->CR1
/* LD3  (PB3) */
#define LED3_PIN_MASK  ((u8)0x08)
#define LED3_PORT_ODR  GPIOB->ODR
#define LED3_PORT_DDR  GPIOB->DDR
#define LED3_PORT_CR1  GPIOB->CR1

#define LED1_ON()  {LED1_PORT_ODR |= LED1_PIN_MASK; Led1State = LON;}
#define LED1_OFF() {LED1_PORT_ODR &= (u8)(~LED1_PIN_MASK); Led1State = LOFF;}
#define LED1_TOG() {if (Led1State == LOFF) LED1_ON() else LED1_OFF(); }

#define LED2_ON()  {LED2_PORT_ODR |= LED2_PIN_MASK; Led2State = LON;}
#define LED2_OFF() {LED2_PORT_ODR &= (u8)(~LED2_PIN_MASK); Led2State = LOFF;}
#define LED2_TOG() {if (Led2State == LOFF) LED2_ON() else LED2_OFF(); }

#define LED3_ON()  {LED3_PORT_ODR |= LED3_PIN_MASK; Led3State = LON;}
#define LED3_OFF() {LED3_PORT_ODR &= (u8)(~LED3_PIN_MASK); Led3State = LOFF;}
#define LED3_TOG() {if (Led3State == LOFF) LED3_ON() else LED3_OFF(); }

#if NUMBER_OF_SINGLE_CHANNEL_KEYS > 0
#define KEY01_DETECTED (sSCKeyInfo[0].Setting.b.DETECTED)
#define KEY02_DETECTED (sSCKeyInfo[1].Setting.b.DETECTED)
#define KEY03_DETECTED (sSCKeyInfo[2].Setting.b.DETECTED)
#define KEY04_DETECTED (sSCKeyInfo[3].Setting.b.DETECTED)
#define KEY05_DETECTED (sSCKeyInfo[4].Setting.b.DETECTED)
#else
#define KEY01_DETECTED (0)
#define KEY02_DETECTED (0)
#define KEY03_DETECTED (0)
#define KEY04_DETECTED (0)
#define KEY05_DETECTED (0)
#endif

#define IS_ADC_ON (CLK->PCKENR2 & 0x08)
#define IS_TIM5_ON (CLK->PCKENR1 & 0x20)

/* Private variables ---------------------------------------------------------*/

LedState_T Led1State;
LedState_T Led2State;
LedState_T Led3State;
OutMode_T OutState=AUTO;
PwmMode_T PWM=PWM0;

volatile OutTemp_T OutTemperature=TEMP0;
volatile bool_T bChange=true;

u32 Timer30=0;

u8 Key01Touched = 0;
u8 Key02Touched = 0;

u8 ToggleLED1 = 0;
u8 ToggleLED2 = 0;
u8 ToggleLED3 = 0;
u8  cnt=0;


/* Private function prototypes -----------------------------------------------*/
void ExtraCode_Init(void);
void ExtraCode_StateMachine(void);
void SystemRecovery_Action(void);
void setPwmByTemp(void);
void runPWM(void );
void setLeds(void);
void assert_failed(uint8_t* file, uint32_t line){};
void runADC(bool_T start);
void runTIM5(bool_T start);

/* Private functions ---------------------------------------------------------*/

/**
  ******************************************************************************
  * @brief Main function.
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */
void main(void)
{

      CLK_DeInit();
      //CLK->CKDIVR = 0x00; // Main freq divided by 1 = Full Freq
      CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
      CLK_HSICmd(ENABLE);


      ExtraCode_Init();
      TSL_Init();
      
 //  /* Init Timer used to measure VIH/VIL */ 
//  TIMACQ->PSCRL = 0;
// original  TIMACQ->PSCR = 0;
// file stm8_tsl_rc_acquisition.c        //
      
      for (;;) {
        TSL_Action();
        ExtraCode_StateMachine();
      }
  
}


/**
  ******************************************************************************
  * @brief Initialize all the keys, Tim, ADC I/Os for LED
  * @par Parameters:
  * None
  * @retval void None
  * @par Required preconditions:
  * None
  ******************************************************************************
  */
void ExtraCode_Init(void)
{

  u8 i;
  GPIO_DeInit(GPIOC);
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_SLOW);

  CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, DISABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_SPI, DISABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, DISABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, DISABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER5, DISABLE);


  /* All keys are implemented and enabled */

#if NUMBER_OF_SINGLE_CHANNEL_KEYS > 0
  for (i = 0; i < NUMBER_OF_SINGLE_CHANNEL_KEYS; i++)
  {
    sSCKeyInfo[i].Setting.b.IMPLEMENTED = 1;
    sSCKeyInfo[i].Setting.b.ENABLED = 1;
    sSCKeyInfo[i].DxSGroup = 0x00; /* 0x00 = DxS disabled, other values = DxS enabled */
  }
#endif

#if NUMBER_OF_MULTI_CHANNEL_KEYS > 0
  for (i = 0; i < NUMBER_OF_MULTI_CHANNEL_KEYS; i++)
  {
    sMCKeyInfo[i].Setting.b.IMPLEMENTED = 1;
    sMCKeyInfo[i].Setting.b.ENABLED = 1;
    sMCKeyInfo[i].DxSGroup = 0x00; /* 0x00 = DxS disabled, other values = DxS enabled */
  }
#endif

  /* Init I/O in Output Push-Pull to drive the LED */
  LED1_PORT_ODR |= LED1_PIN_MASK; /* LED1 is ON by default */
  LED1_PORT_DDR |= LED1_PIN_MASK;
  LED1_PORT_CR1 |= LED1_PIN_MASK;

  /* Init I/O in Output Push-Pull to drive the LED */
  LED2_PORT_ODR |= LED2_PIN_MASK; /* LED2 is ON by default */
  LED2_PORT_DDR |= LED2_PIN_MASK;
  LED2_PORT_CR1 |= LED2_PIN_MASK;
  
    /* Init I/O in Output Push-Pull to drive the LED */
  LED3_PORT_ODR |= LED3_PIN_MASK; /* LED3 is ON by default */
  LED3_PORT_DDR |= LED3_PIN_MASK;
  LED3_PORT_CR1 |= LED3_PIN_MASK;
  
  LED1_OFF();
  LED2_OFF();
  LED3_OFF();
  
  OutState=AUTO;
  
  //ADC init
  runADC(true);

  
//remap TIM5-CH1
  if (FLASH_ReadByte(0x4803)!=0x01){
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
    FLASH_ProgramOptionByte(0x4803, 0x01); 
    FLASH_Lock(FLASH_MEMTYPE_DATA);
  }
  
//pwm 500HZ 
  runTIM5(true);

  enableInterrupts();

}


void ExtraCode_StateMachine(void)	{
  //start ADC every 1s, switch off device after MAXTIMER sec
  if(TSL_Tick_Flags.b.DTO_1sec && OutState!=OFF){
    Timer30++;
    TSL_Tick_Flags.b.DTO_1sec=0;
    if (Timer30>MAXTIMER){
      OutState=OFF;
      runTIM5(false);
      runADC(false);
      bChange=true;
      Timer30=0;
    }else if(OutState==AUTO){ //start temperature mesurement
      ADC1_StartConversion();
    }
  }
  //delay 300ms before next key  click
  if (TSL_Tick_Flags.b.User1_Flag_100ms && cnt){
    cnt--;
    TSL_Tick_Flags.b.User1_Start_100ms=1;
    if(cnt==0){
      Key01Touched=0;
      Key02Touched=0;
    }
  }
  // check click
  if ((TSL_GlobalSetting.b.CHANGED) && (TSLState == TSL_IDLE_STATE))
  {
   
    TSL_GlobalSetting.b.CHANGED = 0;
    // manual  key
    if (KEY01_DETECTED && !Key01Touched)
    {
      Key01Touched=1;
      cnt=3;
      switch(OutState){
        case AUTO:
          OutState=MAN100;
          runADC(false);
          break;
        case MAN100:
          OutState=MAN50;
          break;
        case MAN50:
          OutState=MAN25;
          break;
        case MAN25:
          OutState=OFF;
          break;
        case OFF:
          OutState=MAN100;
          break;
      }
      bChange=true;
    }
    // key2 - auto
    if (KEY02_DETECTED && !Key02Touched)
    {
      Key02Touched = 1;
      cnt=3;
      if(OutState!=AUTO){
        OutState=AUTO;
        runADC(true);
      }
      bChange=true;
    }

    if(KEY02_DETECTED || KEY01_DETECTED){
      Timer30=0;
      TSL_Tick_Flags.b.User1_Start_100ms=1;  
    }
  }
  // chenge PWM mode
  if(bChange){
    switch(OutState){
      case AUTO:
        setPwmByTemp();
        break;
      case MAN100:
        PWM=PWM100;
        break;      
      case MAN50:
        PWM=PWM50;
        break;      
      case MAN25:
        PWM=PWM25;
        break; 
      case OFF:
        PWM=PWM0;
        break;      
      }
    runPWM();
    setLeds();
    bChange=false;
  }
}

void setPwmByTemp(void){
  switch(OutTemperature){
  case TEMP0:
      PWM=PWM100;
      break;      
  case TEMP5:
      PWM=PWM50;
      break;      
  case TEMP10:
      PWM=PWM25;
      break;      
  case TEMP20:
      PWM=PWM0;
      break;      
  }
}

void runPWM(void ){
  switch(PWM){
  case PWM0:
    runTIM5(false);
 //   TIM5_Cmd(DISABLE);
    GPIO_WriteLow(GPIOC, GPIO_PIN_5);
 //   TIM5_SetCompare1(0);
 //   TIM5_Cmd(ENABLE);
    break;
  case PWM25:
    runTIM5(true);
    TIM5_Cmd(DISABLE);
    TIM5_SetCompare1(256);
    TIM5_Cmd(ENABLE);
    break;
  case PWM50:
    runTIM5(true);
    TIM5_Cmd(DISABLE);
    TIM5_SetCompare1(512);
    TIM5_Cmd(ENABLE);
    break;
  case PWM100:
    runTIM5(false);
//    TIM5_Cmd(DISABLE);
    GPIO_WriteHigh(GPIOC, GPIO_PIN_5);
//    TIM5_SetCompare1(1024);
//    TIM5_Cmd(ENABLE);
    break;
  }
}

void setLeds(void){
  LED2_OFF();
  LED3_OFF();
  switch (OutState){
  case OFF:
  case MAN100:
  case MAN50:
  case MAN25:
    LED1_OFF();
    break;
  case AUTO:
    LED1_ON();
    break;
  }
  switch(PWM){
  case PWM25:
    LED2_ON();
    break;
  case PWM50:
    LED3_ON();
    break;
  case PWM100:
    LED2_ON();
    LED3_ON();
    break;
    
  }
}
             
 void runADC(bool_T start){
   if (start){
     if(!IS_ADC_ON){
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
        
      GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_IN_FL_NO_IT);
      ADC1_DeInit();
      ADC1_Init(ADC1_CONVERSIONMODE_SINGLE, 
                   ADC1_CHANNEL_0,
                   ADC1_PRESSEL_FCPU_D8, 
                   ADC1_EXTTRIG_TIM, 
                   ENABLE, 
                   ADC1_ALIGN_RIGHT, 
                   ADC1_SCHMITTTRIG_ALL, 
                   DISABLE);
      ADC1_ExternalTriggerConfig(ADC1_EXTTRIG_TIM, DISABLE);
      ADC1_ITConfig(ADC1_IT_EOCIE, ENABLE);
      ADC1_Cmd(ENABLE);
     }
   }else{
      ADC1_Cmd(DISABLE);
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, DISABLE);
   }
 }
             
 void runTIM5(bool_T start){
   if (start){
     if(!IS_TIM5_ON){
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER5, ENABLE);
     
      TIM5_DeInit();
      TIM5_TimeBaseInit( TIM5_PRESCALER_32, 1023);
      TIM5_OC1Init(TIM5_OCMODE_PWM1,
                      TIM5_OUTPUTSTATE_ENABLE,
                      512,
                      TIM5_OCPOLARITY_HIGH);
      TIM5_OC1PreloadConfig(ENABLE);
      TIM5_ARRPreloadConfig(ENABLE);
      TIM5_Cmd(DISABLE);
     }
   }else{
      TIM5_Cmd(DISABLE);
            TIM5_OC1Init(TIM5_OCMODE_PWM1,
                      TIM5_OUTPUTSTATE_DISABLE,
                      512,
                      TIM5_OCPOLARITY_HIGH);
      CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER5, DISABLE);
   }
 }

/* Public functions ----------------------------------------------------------*/


