#include "main.h"
#include "app_main.h"
#include "math.h"
#include "ARGB.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim1_ch1;

#define FRAME_DELAY_MS 50 

/* Easy definition of LEDs
#define PORT 0
#define BOW 1
#define STARBOARD 2
#define PORT 3
#define DISTANCE 4
#define FSK 5
*/

typedef enum {
    SHOW_OFF            =   0,
    SHOW_ON             =   1,
    SHOW_DANGERMED      =   2,
    SHOW_DANGERHIG      =   3,
    SHOW_COLLECTED      =   4,
    SHOW_FSK      =   5,
    NUM_OF_SHOWS    
} t_ShowType;

t_ShowType currentShow = 0;

bool buttonPressed = 0;


void app_init(){
    return;
}

// Utility function to make a simple "oscillating" brightness.
// t is a time/frame counter, period controls how fast it cycles,
// minVal and maxVal control the brightness range.
static uint8_t oscillateBrightness(float t, float period, uint8_t minVal, uint8_t maxVal) {
    // Create a normalized sine wave between minVal and maxVal
    float angle = (2.0f * M_PI * t) / period;
    float sineVal = (sin(angle) + 1.0f) / 2.0f; // map sine(-1..1) to (0..1)
    float range = (float)(maxVal - minVal);
    return (uint8_t)(minVal + sineVal * range);
}

void show_off (uint32_t frame){
    (void)frame;
    ARGB_FillRGB(0,0,0);
    ARGB_FillHSV(0,0,0);
    ARGB_FillWhite(0);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET); // Disable Indicator LED
}

void show_on(uint32_t frame){
    float t = (float)frame;

    // PORT

    uint8_t hue = 0; // Red
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(0, hue, sat, val);

    // BOW

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(1, hue, sat, val);

    // STARBOARD

    uint8_t hue =  85; // Green
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(2, hue, sat, val);

    // STERN

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(3, hue, sat, val);

    // DISTANCE

    uint8_t hue = 43; // Yellow
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(4, hue, sat, val);

    // FSK

    uint8_t hue = 0; // OFF
        uint8_t sat = 0;
        uint8_t val = 0;
        ARGB_SetHSV(5, hue, sat, val);
}


void show_dangermed(uint32_t frame){
    float t = (float)frame;

    // PORT

    uint8_t hue = 0; // Red
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(0, hue, sat, val);

    // BOW

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(1, hue, sat, val);

    // STARBOARD

    uint8_t hue =  85; // Green
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(2, hue, sat, val);

    // STERN

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(3, hue, sat, val);

    // DISTANCE

    uint8_t hue = 22; // Amber
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(4, hue, sat, val);

    // FSK

    uint8_t hue = 0; // OFF
        uint8_t sat = 0;
        uint8_t val = 0;
        ARGB_SetHSV(5, hue, sat, val);
}

void show_dangerhigh(uint32_t frame) {
    float t = (float)frame;

    // PORT

    uint8_t hue = 0; // Red
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(0, hue, sat, val);

    // BOW

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(1, hue, sat, val);

    // STARBOARD

    uint8_t hue =  85; // Green
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(2, hue, sat, val);

    // STERN

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(3, hue, sat, val);

    // DISTANCE

    uint8_t hue = 5; // Red
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(4, hue, sat, val);

    // FSK

    uint8_t hue = 0; // OFF
        uint8_t sat = 0;
        uint8_t val = 0;
        ARGB_SetHSV(5, hue, sat, val);
}

void show_collected(uint32_t frame) {
    float t = (float)frame;

    // PORT

    uint8_t hue = 0; // Red
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(0, hue, sat, val);

    // BOW

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(1, hue, sat, val);

    // STARBOARD

    uint8_t hue =  85; // Green
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(2, hue, sat, val);

    // STERN

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(3, hue, sat, val);

    // DISTANCE

    uint8_t hue = 35; // Gold
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(4, hue, sat, val);

    // FSK

    uint8_t hue = 0; // OFF
        uint8_t sat = 0;
        uint8_t val = 0;
        ARGB_SetHSV(5, hue, sat, val);
}

void show_fsk(uint32_t frame) {
    float t = (float)frame;

    // PORT

    uint8_t hue = 0; // Red
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(0, hue, sat, val);

    // BOW

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(1, hue, sat, val);

    // STARBOARD

    uint8_t hue =  85; // Green
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(2, hue, sat, val);

    // STERN

    uint8_t hue = 0; // White
        uint8_t sat = 0;
        uint8_t val = 128;
        ARGB_SetHSV(3, hue, sat, val);

    // DISTANCE

    uint8_t hue = 43; // Yellow
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(4, hue, sat, val);

    // FSK

    uint8_t hue = 43; // 170
        uint8_t sat = 255;
        uint8_t val = 128;
        ARGB_SetHSV(5, hue, sat, val);
}

void app_main(){
    
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    ARGB_Init();  // Initialization
    ARGB_SetBrightness(128); // Set a moderate global brightness (0-255)
    ARGB_Clear();
    ARGB_Show();
    uint32_t frame = 0;
    for(;;){
       // Wait for strip to be ready
        while (ARGB_Ready() != ARGB_READY) { }

        switch (currentShow) {
            case SHOW_OFF:
                show_off(frame);
                break;
            case SHOW_ON:
                show_on(frame);
                break;
            case SHOW_DANGERMED:
                show_dangermed(frame);
                break;
            case SHOW_DANGERHIG:
                show_dangerhig(frame);
                break;
            case SHOW_COLLECTED:
                show_collected(frame);
                break;
            case SHOW_FSK:
                show_fsk(frame);
                break;
            default:
                break;
        }

        if (buttonPressed) {
            buttonPressed = false;
            currentShow = (currentShow + 1) % NUM_OF_SHOWS;
        }

        ARGB_Show();
        HAL_Delay(FRAME_DELAY_MS);
        frame++;
        
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == USER_BUTTON_Pin){
        buttonPressed = true;
    }
}
