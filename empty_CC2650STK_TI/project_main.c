// TEKIJÄT: Joona-Oskari Pekkala, Veeti Pitkänen, Lauri Tamminen
// Työssä on käytetty apuna ChatGPT tekoälyä. Jokainen tekoälyltä saatu vastaus on ihmisen tarkistama
// ja jokainen siltä saatu vastaus on ymmärretty. Tekoälyltä saatua koodia ei ole käytetty suoraan meidän työssä,
// vaan sitä on muokattu meidän työhön sopivaksi. Tekoälyn koodia on käytetty vain ideoimiseen.


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
#include "buzzer.h"

/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

// Tilakoneen esittely
enum state { IDLE=1, READ_SENSOR, UPDATE };
enum state myState = IDLE;

// Globaalit muuttujat ja liikkeiden raja-arvot
float ax, ay, az, gx, gy, gz;
float thresholdVaaka = 0.5;
float thresholdPysty = 1.5;

// Lähetysksen bufferi
char input[10];

// UART vastaanottobufferi
uint8_t uartBuffer[30];

// Morsetuksen ajoitukset
#define DOT_DURATION (200000 / Clock_tickPeriod)
#define DASH_DURATION (3 * DOT_DURATION)
#define SYMBOL_GAP (DOT_DURATION)
#define SPACE_GAP (3 * DOT_DURATION)

//painonappien, kaiuttimen ja ledien RTOS-muuttujat ja alustus
static PIN_Handle buttonHandle;
static PIN_Handle button2Handle;
static PIN_State buttonState;
static PIN_State button2State;
static PIN_Handle ledHandle;
static PIN_Handle led2Handle;
static PIN_State ledState;
static PIN_State led2State;
static PIN_Handle buzzerHandle;
static PIN_State buzzerState;

// MPU pinnin muuttujat
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

// Ledien, nappien, kaiuttimen ja MPU pinnin configit
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

PIN_Config led2Config[] =
{
 Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

PIN_Config buzzerConfig[] =
{
 Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

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

// Tulostetaan välilyönti
void button2Fxn(PIN_Handle handle, PIN_Id pinId)
{
    if (myState == READ_SENSOR)
    {
        printf(" \n");          // Konsoliin tulostus, jotta debug helpompaa.
        System_flush();
        input[0] = ' ';
        input[1] = '\r';
        input[2] = '\n';
        input[3] = '\0';
        myState = UPDATE;
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

// Käsittelijäfunktio, jossa käsitellään UARTin kautta tullut viesti
static void uartFxn(UART_Handle handle, void *rxBuf, size_t len)
{
    // Muutetaan vastaanotettu data merkkijonoksi
    char *data = (char *)rxBuf;
    uint32_t startTick;
    uint32_t endTick;

    // Piste/lyhyt
    if (data[0] == '.')
    {
        startTick = Clock_getTicks();
        endTick = startTick + DOT_DURATION;

        PIN_setOutputValue(led2Handle, Board_LED1, Board_LED_ON);
        while(Clock_getTicks() < endTick)
        {
            // Tyhjä while looppi, että ledi palaa halutun ajan
        }
        PIN_setOutputValue(led2Handle, Board_LED1, Board_LED_OFF);
    }

    // Viiva/pitkä
    else if (data[0] == '-')
    {
        startTick = Clock_getTicks();
        endTick = startTick + DASH_DURATION;

        PIN_setOutputValue(led2Handle, Board_LED1, Board_LED_ON);
        while(Clock_getTicks() < endTick)
        {
            // Tyhjä while looppi, että ledi palaa halutun ajan
        }
        PIN_setOutputValue(led2Handle, Board_LED1, Board_LED_OFF);
    }

    // Välilyönti
    else if (data[0] == ' ')
    {
        startTick = Clock_getTicks();
        endTick = startTick + SPACE_GAP;
        while(Clock_getTicks() < endTick)
        {
            // Tyhjä while looppi, että välilyönnin aika on haluttu
        }
    }

    // Merkkien välinen tauko
    startTick = Clock_getTicks();
    endTick = startTick + SYMBOL_GAP;
    while(Clock_getTicks() < endTick)
    {
        // Tyhjä while looppi, että symbolien välinen aika on haluttu
    }

    // Valmiina vastaanottamaan uusi viesti
    UART_read(handle, rxBuf, 1);
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
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = &uartFxn;
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

    UART_read(uart, uartBuffer, 1);

    while (1)
    {
        // Lähetetään viesti vain, kun sensorista on saatu dataa. Kaiutin soi lähetetyn merkin varmistamiseksi
        if (myState == UPDATE)
        {
            UART_write(uart, input, 4);

            buzzerOpen(buzzerHandle);
            buzzerSetFrequency(2000);
            Task_sleep(300000 / Clock_tickPeriod);
            buzzerClose();

            myState = READ_SENSOR;
        }
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
    PIN_setOutputValue(hMpuPin, Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
    Task_sleep(100000 / Clock_tickPeriod);
    System_printf("MPU9250: Power ON\n");
    System_flush();

    // MPU open i2c
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL)
    {
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
            if ((az > 0.5) || (az < -0.5))
            {
                if (fabs(ax) > thresholdVaaka && fabs(ay) < thresholdVaaka && fabs(az) < thresholdPysty)
                {
                    // Lähetetään viiva morse-koodina
                    System_printf("-\n"); // Tulostaa viivan konsoliin bebuggausta varten
                    System_flush();
                    input[0] = '-';
                    input[1] = '\r';
                    input[2] = '\n';
                    input[3] = '\0';
                    myState = UPDATE;

                    // Viive, jotta sensorin kerkeää siirtämään lähtöpisteeseen
                    Task_sleep(500000 / Clock_tickPeriod);
                }
                else if (fabs(ay) > thresholdVaaka && fabs(ax) < thresholdVaaka && fabs(az) < thresholdPysty)
                {
                    // Lähetetään viiva morse-koodina
                    System_printf("-\n"); // Tulostaa viivan konsoliin bebuggausta varten
                    System_flush();
                    input[0] = '-';
                    input[1] = '\r';
                    input[2] = '\n';
                    input[3] = '\0';
                    myState = UPDATE;

                    // Viive, jotta sensorin kerkeää siirtämään lähtöpisteeseen
                    Task_sleep(500000 / Clock_tickPeriod);
                }
                else if (fabs(az) >= thresholdPysty)
                {
                    // Lähetetään piste morse-koodina
                    System_printf(".\n"); // Tulostaa pisteen konsoliin bebuggausta varten
                    System_flush();
                    input[0] = '.';
                    input[1] = '\r';
                    input[2] = '\n';
                    input[3] = '\0';
                    myState = UPDATE;

                    // Viive, jotta sensorin kerkeää siirtämään lähtöpisteeseen
                    Task_sleep(500000 / Clock_tickPeriod);
                }
            }
            else
            {
                printf("Liikettä ei tunnistettu, yritä uudelleen! Tarkista anturin asento!\n");
                System_flush();
            }
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

    // Vihreä ledi käyttöön ohjelmassa
    ledHandle = PIN_open(&ledState, ledConfig);
    if(!ledHandle)
    {
       System_abort("Error initializing LED pin\n");
    }

    // Punainen ledi käyttöön ohjelmassa
    led2Handle = PIN_open(&led2State, led2Config);
    if(!led2Handle)
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

    // Kaiutin mukaan ohjelmaan
    buzzerHandle = PIN_open(&buzzerState, buzzerConfig);
    if(!buzzerHandle)
    {
        System_abort("Error initializing buzzer pin\n");
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
