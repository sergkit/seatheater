typedef enum{ // arround temperature
  TEMP0=0,
  TEMP5=1,
  TEMP10=2,
  TEMP20=3
} OutTemp_T;
/**/
//0C work
#define TEMP_0_ADC (756)
#define TEMP_5_ADC (710)
#define TEMP_10_ADC (662)
/**/
/*
// 25C test
#define TEMP_0_ADC (512)
#define TEMP_5_ADC (460)
#define TEMP_10_ADC (410)
*/

#define TRU (1==1)
#define FAL (!TRU)
typedef enum{
  true=TRU,
  false=FAL
}bool_T;

extern volatile OutTemp_T OutTemperature;
extern  volatile bool_T bChange;
