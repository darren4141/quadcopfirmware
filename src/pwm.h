#define D0_PIN 0
#define D2_PIN 2
#define D3_PIN 21
#define D8_PIN 19

#define STEP_MS         500                
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT   
#define LEDC_FREQ_HZ    1000                

#define CH_D0 LEDC_CHANNEL_0
#define CH_D2 LEDC_CHANNEL_1
#define CH_D3 LEDC_CHANNEL_2
#define CH_D8 LEDC_CHANNEL_3

//PID PWM CONSTANTS
#define HOVER_FF 700
#define ASCEND_PWM 100
#define DESCEND_PWM -100
#define ROLL_PWM 25
#define LAUNCH_ANGLE_DEGREES 20.0
#define PID_UPPER_BOUND 200

extern volatile float targetPWMpct;
extern volatile float currentPWMpct;

typedef struct{
    float basePWM;
    float offset0;
    float offset1;
    float offset2;
    float offset3;

}pwmconfig;

void incrementTargetPWMpct(float size);

void setTargetPWMpct(float newTargetPWMpct);

float getTargetPWMpct();

void pwm_setter_task();
