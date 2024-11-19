/* C Standard library */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/mpu9250.h"

/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

// Tilakoneen esittely
enum state { IDLE=1, READ_UART, READ_SENSOR, UPDATE };
enum state myState = IDLE;

// Globaalit muuttujat
float ax, ay, az, gx, gy, gz;
float thresholdVaaka = 0.7;
float thresholdPysty = 1.5;
char input[10];

//painonappien ja ledien RTOS-muuttujat ja alustus
static PIN_Handle buttonHandle;
static PIN_Handle button2Handle;
static PIN_State buttonState;
static PIN_State button2State;
static PIN_Handle ledHandle;
static PIN_State ledState;

// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;


PIN_Config buttonConfig[] =
{
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};

PIN_Config button2Config[] =
{
   Board_BUTTON1  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};

PIN_Config ledConfig[] =
{
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

// MPU power pin
static PIN_Config MpuPinConfig[] =
{
    Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg =
{
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

// Vaihdetaan led-pinnin tilaa negaatiolla ja siirrytään lukemaan sensorin dataa READ_SENSOR tilassa
void buttonFxn(PIN_Handle handle, PIN_Id pinId)
{
    uint_t pinValue = PIN_getOutputValue( Board_LED0 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );

    if (myState == IDLE)
    {
    myState = READ_SENSOR;
    }
    else
    {
        myState = IDLE;
    }
    Task_sleep(100000 / Clock_tickPeriod);
}

void button2Fxn(PIN_Handle handle, PIN_Id pinId)
{
    if (myState == READ_SENSOR)
    {
        printf(" Väli\n"); // Tulostaa väli
        snprintf(input, sizeof(input), " \r\n\0");
        System_flush();
        myState = UPDATE;
        Task_sleep(100000 / Clock_tickPeriod);
    }

}


/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1)
{

    // UARTin alustus
    // UART-kirjaston asetukset
    UART_Handle uart;
    UART_Params uartParams;

    // Alustetaan sarjaliikenne
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.baudRate = 9600; // nopeus 9600baud
    uartParams.dataLength = UART_LEN_8; // 8
    uartParams.parityType = UART_PAR_NONE; // n
    uartParams.stopBits = UART_STOP_ONE; // 1

    // Avataan yhteys laitteen sarjaporttiin vakiossa Board_UART0
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL)
    {
        System_abort("Error opening the UART");
    }

    while (1)
    {

        // Kun tila on oikea, tulosta sensoridata merkkijonossa debug-ikkunaan. Muista tilamuutos
        if (myState == UPDATE)
        {

            // Tulos terminaaliin
            UART_write(uart, input, strlen(input));

            // Odotus tilaan
            myState = READ_SENSOR;
        }



        // Just for sanity check for exercise, you can comment this out
        //System_printf("uartTask\n");
        //System_flush();

        // Once per second, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

Void sensorTaskFxn(UArg arg0, UArg arg1)
{

    //Alustetaan i2cMPU väylä taskille
    I2C_Handle      i2cMPU;
    I2C_Params      i2cMPUParams;

    // i2cMPU väylä taskin käyttöön
    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    // Note the different configuration below
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU power on
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2CMPU\n");
    }

    // MPU setup and calibration
    System_printf("MPU9250: Setup and calibration...\n");
    System_flush();

    Task_sleep(100000 / Clock_tickPeriod);
    mpu9250_setup(&i2cMPU);

    System_printf("MPU9250: Setup and calibration OK\n");
    System_flush();

    while (1)
    {
        if (myState == READ_SENSOR)
        {

            mpu9250_get_data(&i2cMPU, &ax, &ay, &az, &gx, &gy, &gz);

            // Tarkista laitteen asento kiihtyvyysarvojen perusteella
            if ((az > 0.5) || (az < -0.5))
            {
                if (fabs(ax) > thresholdVaaka && fabs(ay) < thresholdVaaka && fabs(az) < thresholdPysty)
                {
                    printf("- %.2f %.2f\n", ax, ay); // Tulostaa viivan
                    snprintf(input, sizeof(input), "-\r\n\0");
                    System_flush();
                    myState = UPDATE;
                }
                else if (fabs(ay) > thresholdVaaka && fabs(ax) < thresholdVaaka && fabs(az) < thresholdPysty)
                {
                    printf("- %.2f %.2f\n", ax, ay); // Tulostaa viivan
                    snprintf(input, sizeof(input), "-\r\n\0");
                    System_flush();
                    myState = UPDATE;
                }
                else if (fabs(az) >= thresholdPysty)
                {
                    printf(". %.2f\n", az); // Tulostaa pisteen
                    snprintf(input, sizeof(input), ".\r\n\0");
                    System_flush();
                    myState = UPDATE;
                }
            }

            else
            {
                printf("Liikettä ei tunnistettu, yritä uudelleen! Tarkista anturin asento!\n");
                System_flush();
            }

            // Just for sanity check for exercise, you can comment this out
            //System_printf("sensorTask\n");
            //System_flush();

        }
        Task_sleep(100000 / Clock_tickPeriod);
    }
}


Int main(void)
{

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    // Initialize board
    Board_initGeneral();

    // i2c väylä käyttöön
    Board_initI2C();

    // UART käyttöön
    Board_initUART();

    // Ledi käyttöön ohjelmassa
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle)
    {
       System_abort("Error initializing LED pin\n");
    }

    // Painonappi käyttöön ohjelmassa
    buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle)
    {
       System_abort("Error initializing button pin\n");
    }

    // Toinen nappi myös ohjelmaan
    button2Handle = PIN_open(&button2State, button2Config);
    if(!button2Handle)
    {
       System_abort("Error initializing button pin\n");
    }

    // Painonapille keskeytyksen käsittellijä
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0)
    {
       System_abort("Error registering button callback function");
    }

    // Toiselle painonapille myös keskeytyksen käsittellijä
    if (PIN_registerIntCb(button2Handle, &button2Fxn) != 0)
    {
       System_abort("Error registering button callback function");
    }

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL)
    {
     System_abort("Pin open failed!");
    }

    /* Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL)
    {
        System_abort("Task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL)
    {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
