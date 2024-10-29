/* C Standard library */
#include <stdio.h>
#include <string.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"

/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

// JTKJ: Tehtävä 3. Tilakoneen esittely
enum state { WAITING=1, DATA_READY };
enum state programState = WAITING;

// JTKJ: Tehtävä 3. Valoisuuden globaali muuttuja
double ambientLight = -1000.0;

//painonappien ja ledien RTOS-muuttujat ja alustus
static PIN_Handle buttonHandle;
static PIN_State buttonState;
static PIN_Handle ledHandle;
static PIN_State ledState;


PIN_Config buttonConfig[] =
{
   Board_BUTTON0  | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
   PIN_TERMINATE
};

PIN_Config ledConfig[] =
{
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE
};

// Vaihdetaan led-pinnin tilaa negaatiolla
void buttonFxn(PIN_Handle handle, PIN_Id pinId)
{
    uint_t pinValue = PIN_getOutputValue( Board_LED0 );
    pinValue = !pinValue;
    PIN_setOutputValue( ledHandle, Board_LED0, pinValue );
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
        char str[16];

        // Kun tila on oikea, tulosta sensoridata merkkijonossa debug-ikkunaan. Muista tilamuutos
        if (programState == DATA_READY)
        {
            int kokonaisosa = (int)ambientLight;
            int desimaaliosa = (int)ambientLight;
            desimaaliosa = desimaaliosa < 0 ? -desimaaliosa : desimaaliosa;

            // Luodaan ambientLightistä merkkijono
            sprintf(str, "%d.%02d\n\r", kokonaisosa, desimaaliosa);

            // Merkkijonon tulostus
            System_printf("UARTin ambientLight: %s\n", str);
            System_flush();

            // Odotus tilaan
            programState = WAITING;
        }

        // Lähetetään merkkijono UARTilla
        UART_write(uart, str, strlen(str));

        // Just for sanity check for exercise, you can comment this out
        //System_printf("uartTask\n");
        //System_flush();

        // Once per second, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

Void sensorTaskFxn(UArg arg0, UArg arg1)
{

    //Alustetaan i2c väylä taskille
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cMessage;

    System_printf("sensorTaskFxn käynnissä\n");
    System_flush();

    // i2c väylä taskin käyttöön
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C0, &i2cParams);
    if (i2c == NULL)
    {
      System_abort("Error Initializing I2C\n");
    }

    // i2c-viesteille lähetys- ja vastaanottopuskurit
    uint8_t txBuffer[1];
    uint8_t rxBuffer[2];

    i2cMessage.slaveAddress = Board_OPT3001_ADDR;
    txBuffer[0] = OPT3001_REG_RESULT;              // Rekisterin osoite lähetyspuskuriin
    i2cMessage.writeBuf = txBuffer;                // Lähetyspuskurin asetus
    i2cMessage.writeCount = 1;                     // Lähetetään 1 tavu
    i2cMessage.readBuf = rxBuffer;                 // Vastaanottopuskurin asetus
    i2cMessage.readCount = 2;                      // Vastaanotetaan 2 tavua


    //Sensorin alustus ja 100ms viive
    Task_sleep(100000 / Clock_tickPeriod);
    opt3001_setup(&i2c);

    while (1)
    {
        if (I2C_transfer(i2c, &i2cMessage))
        {
        //Lue sensorilta dataa ja tulosta se Debug-ikkunaan merkkijonona
            double data = opt3001_get_data(&i2c);
                if (data >= 0)
                {
                    // Luetaan rekisteristä raaka-data
                    uint16_t reg_0 = rxBuffer[0];
                    reg_0 = reg_0 << 8;
                    uint16_t reg_1 = reg_0 | rxBuffer[1];


                    // Tulostetaan raaka-data binäärinä konsoliin
                    int i;
                    System_printf("Sensortaskin raakadata: ");
                    for (i = 15; i >= 0; i--)
                    {
                        System_printf("%d", (reg_1 >> i) & 1);
                        if (i % 4 == 0 && i != 0)
                        {
                            System_printf(" ");
                        }
                    }
                    System_printf("\n");
                    System_flush();

                    //Tallenna mittausarvo globaaliin muuttujaan. Muista tilamuutos
                    ambientLight = data;
                    programState = DATA_READY;
                }


        // Just for sanity check for exercise, you can comment this out
        // System_printf("sensorTask\n");
        // System_flush();

        // Once per second, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
        }
    }
}

int main(void)
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

    // Painonapille keskeytyksen käsittellijä
    if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0)
    {
       System_abort("Error registering button callback function");
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
