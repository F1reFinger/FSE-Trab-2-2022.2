#include "bme280.h"
#include "uart_MOD.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <softPwm.h>
#include <wiringPi.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <signal.h>

#define TURN_OVER_ON 0xA1
#define TURN_OVER_OFF 0xA2
#define START_HEAT 0xA3
#define CANCEL_PROCESS 0xA4
#define MENU 0xA5

#define FRG "frg"
#define CRN "crn"
#define PZZ "pzz"
#define CST "custom"

double saida_medida, sinal_de_controle;
double referencia = 0.0;
double Kp = 30;  // Ganho Proporcional
double Ki = 0.2; // Ganho Integral
double Kd = 400; // Ganho Derivativo
int T = 1.0;     // Período de Amostragem (ms)
unsigned long last_time;
double erro_total, erro_anterior = 0.0;
int sinal_de_controle_MAX = 100.0;
int sinal_de_controle_MIN = -100.0;
void pid_configura_constantes(double Kp_, double Ki_, double Kd_);
void pid_atualiza_referencia(float referencia_);
double pid_controle(double saida_medida);

#define I2C_ADDR 0x27 // I2C endereço do modulo

#define RESISTOR 4
#define FAN 5

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

typedef struct
{
    int temperature;
    int time;
    char name[6];
} Option;
Option frango;
Option carne;
Option pizza;
Option userOption;
Option menuOptions[5];

int selectedOption = 0;

int uart0_filestream;
int fd;

time_t startTime;
int totalTime = 0;
int overOn = 0;
int heating = 0;
int preHeating = 0;
float internalTemperature = 0;
float referenceTemperature = 0;
float roomTemperature = 0;

void setup();
int openUart();

struct identifier
{
    /* Variable to hold device address */
    uint8_t dev_addr;

    /* Variable that contains file descriptor */
    int8_t fd;
};

void user_delay_us(uint32_t period, void *intf_ptr);
void print_sensor_data(struct bme280_data *comp_data);
int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);

sem_t mutex;
void *userInputHandler(void *vargp);
void *temperatureChecker(void *vargp);
void *lcdUpdater(void *vargp);
void *heatController(void *vargp);
void *cool(void *vargp);
void signalTreatment(int s);

struct bme280_dev dev;

struct identifier id;

/* Variable to define the result */
int8_t rslt = BME280_OK;
pthread_t userInputThreadId;
pthread_t temperatureCheckerThreadId;
pthread_t heatControllerThreadId;

int main(int argc, char *argv[]){

    sem_init(&mutex, 0, 1);

    char resultString[300];
    char size;

    setup(argc, argv);
    printf("\n --------------------- \n");

    signal(SIGINT, signalTreatment);
    signal(SIGHUP, signalTreatment);

    pthread_create(&userInputThreadId, NULL, userInputHandler, NULL);
    printf("Before Temperature Checker Thread\n");
    pthread_create(&temperatureCheckerThreadId, NULL, temperatureChecker, NULL);
    printf("Before Heat Controller Thread\n");
    pthread_create(&heatControllerThreadId, NULL, heatController, NULL);

    rslt = stream_sensor_data_forced_mode(&dev);
    if (rslt != BME280_OK){

        fprintf(stderr, "Failed to stream sensor data (code %+d).\n", rslt);
        exit(1);

    }

    pthread_join(userInputThreadId, NULL);
    pthread_join(temperatureCheckerThreadId, NULL);
    pthread_join(heatControllerThreadId, NULL);

    close(uart0_filestream);
    return 0;
}

void signalTreatment(int s){

    char buffer[4];
    printf("Signal %d\n");
    softPwmWrite(RESISTOR, 80);
    softPwmWrite(FAN, 80);
    int value = 0;
    memcpy(buffer, (char *)&value, sizeof(value));
    sendData(uart0_filestream, SEND_SYSTEM_STATE, buffer, 1);
    usleep(1000000);
    sendData(uart0_filestream, SEND_WORKING_STATE, buffer, 1);
    pthread_cancel(userInputThreadId);
    pthread_cancel(temperatureCheckerThreadId);
    pthread_cancel(heatControllerThreadId);
    printf("Encerrado\n");
    exit(0);

}

void *lcdUpdater(void *vargp){

    char lcdMessage[20];
    time_t currentTime;
    char menu[6];

    while (1){

        usleep(500000);
        if (overOn){

            if (preHeating == 1){

                currentTime = time(0);
                printf("Pre Aquecendo");

            }
            else if (heating == 1){

                currentTime = time(0);
                printf("Aquecendo: %d s", totalTime * 60 - (currentTime - startTime));

            }
            else{
                printf("%s: %ds %dg", menuOptions[selectedOption].name, menuOptions[selectedOption].time * 60, menuOptions[selectedOption].temperature);
            }
        }
    }
}

void *cool(void *vargp){

    printf("Cooling 0\n");
    time_t currentTime;

    int controle;
    char buffer[4] = {};
    int value = -100;

    while (1){

        usleep(3000000);
        printf("Cooling\n");

        if (internalTemperature > roomTemperature && heating != 1){// Se um novo aquecimento for iniciado ou a temperatura estiver baixa, para o resfriamento

            memcpy(buffer, (char *)&value, sizeof(value));
            sendData(uart0_filestream, SEND_CONTROL_SIGNAL, buffer, 4);
            softPwmWrite(FAN, 90);

        }
        else{

            value = 0;
            memcpy(buffer, (char *)&value, sizeof(value));
            sendData(uart0_filestream, SEND_CONTROL_SIGNAL, buffer, 4);
            softPwmWrite(FAN, 0);
            break;

        }
    }
}

void *heatController(void *vargp){

    time_t currentTime;
    pthread_t coolThreadId;
    int controle;
    char buffer[4];

    while (1){

        usleep(1000000);
        printf("heating: %d, preHeating: %d\n", heating, preHeating);

        if (preHeating == 1){
            startTime = time(0);
            if ((internalTemperature + 2.0) >= referenceTemperature){
                preHeating = 0;
            }
        }

        if (heating){

            usleep(1000000);
            currentTime = time(0);
            printf("time: %d\n", (currentTime - startTime));

            if ((currentTime - startTime) - (60 * totalTime) >= 0){

                printf("Heat Finished\n");
                heating = 0;
                softPwmWrite(RESISTOR, 0);
                softPwmWrite(FAN, 0);
                pthread_create(&coolThreadId, NULL, cool, NULL);
                printf("Await Cool\n");
                pthread_join(coolThreadId, NULL);

            }
            else{

                controle = pid_controle(internalTemperature);
                printf("Controle: %d\n", controle);
                memcpy(buffer, (char *)&controle, sizeof(controle));
                sendData(uart0_filestream, SEND_CONTROL_SIGNAL, buffer, 4);

                if (controle >= 0){

                    printf("Aquecendo, %d\n", controle);
                    softPwmWrite(RESISTOR, controle);
                    softPwmWrite(FAN, 0);

                }
                else{

                    printf("Resfriando, %d\n", MIN(controle, -40));
                    softPwmWrite(RESISTOR, 0);
                    softPwmWrite(FAN, MIN(controle, -40));

                }
            }
        }
    }
}

void *temperatureChecker(void *vargp){

    char readBuffer[30];
    int readBufferSize;
    int bmerslt;
    uint32_t req_delay;

    while (1){

        if (overOn){

            printf("fetching temperatures\n");

            sem_wait(&mutex);
            requestData(uart0_filestream, REQUEST_INTERNAL_TEMPERATURE);
            usleep(1000000);
            readData(uart0_filestream, readBuffer, 9);
            memcpy(&internalTemperature, &readBuffer[3], 4);
            sem_post(&mutex);

            sem_wait(&mutex);
            requestData(uart0_filestream, REQUEST_REFERENCE_TEMPERATURE);
            usleep(1000000);
            readData(uart0_filestream, readBuffer, 9);
            memcpy(&referenceTemperature, &readBuffer[3], 4);
            sem_post(&mutex);
            menuOptions[0].temperature = referenceTemperature;

            pid_atualiza_referencia(referenceTemperature);
        }
    }
}

void *userInputHandler(void *vargp){

    int userCommand;
    char inputBuffer[30];
    char resultBuffer[30];
    char buffer[4] = {0, 0, 0, 0};
    char value = 0;

    while (1){

        usleep(500000);
        sem_wait(&mutex);
        requestData(uart0_filestream, READ_USER_COMMANDS);
        usleep(1000000);
        readData(uart0_filestream, inputBuffer, 9);
        memcpy(&userCommand, &inputBuffer[3], 4);
        sem_post(&mutex);
        printf("User Command: %d\n", userCommand);

        switch (userCommand){
        case 0:
            printf("Nenhum comando\n");
            break;

        case 1:
            printf("Liga Forno\n");
            overOn = 1;
            value = 1;
            memcpy(buffer, (char *)&value, sizeof(value));
            sem_wait(&mutex);
            sendData(uart0_filestream, SEND_SYSTEM_STATE, buffer, 1);
            usleep(1000000);
            readData(uart0_filestream, inputBuffer, 9);
            sem_post(&mutex);
            break;

        case 2:
            printf("Desliga Forno\n");
            value = 0;
            softPwmWrite(RESISTOR, 0);
            softPwmWrite(FAN, 0);
            memcpy(buffer, (char *)&value, sizeof(value));
            sem_wait(&mutex);
            sendData(uart0_filestream, SEND_SYSTEM_STATE, buffer, 1);
            usleep(1000000);
            readData(uart0_filestream, inputBuffer, 9);
            sem_post(&mutex);
            totalTime = 0;
            overOn = 0;
            break;

        case 3:
            if (overOn){

                printf("Inicia Aquecimento\n");
                preHeating = 1;
                value = 1;
                memcpy(buffer, (char *)&value, sizeof(value));
                sendData(uart0_filestream, SEND_WORKING_STATE, buffer, 1);
                usleep(1000000);
                readData(uart0_filestream, inputBuffer, 9);
                sem_post(&mutex);
                heating = 1;

            }
            break;

        case 4:
            if (overOn){
                printf("Cancela Processo\n");
                value = 1;
                softPwmWrite(RESISTOR, 0);
                softPwmWrite(FAN, 0);
                memcpy(buffer, (char *)&value, sizeof(value));
                sem_wait(&mutex);
                sendData(uart0_filestream, SEND_WORKING_STATE, buffer, 1);
                usleep(1000000);
                readData(uart0_filestream, inputBuffer, 9);
                sem_post(&mutex);
                heating = 0;
                break;
            }
        case 5:

            if (overOn){

                printf("Adiciona Tempo\n");
                if (selectedOption == 0){

                    totalTime = totalTime + 1;
                    menuOptions[0].time = totalTime;
                    memcpy(buffer, (char *)&totalTime, sizeof(totalTime));
                    sem_wait(&mutex);
                    sendData(uart0_filestream, SEND_TIMER_VALUE, buffer, 4);
                    usleep(1000000);
                    readData(uart0_filestream, inputBuffer, 9);
                    sem_post(&mutex);

                }
            }
            break;

        case 6:
            if (overOn){

                printf("Diminui Tempo\n");
                if (selectedOption == 0){

                    totalTime = MAX(totalTime - 1, 0);
                    menuOptions[selectedOption].time = totalTime;
                    memcpy(buffer, (char *)&totalTime, sizeof(totalTime));
                    sem_wait(&mutex);
                    sendData(uart0_filestream, SEND_TIMER_VALUE, buffer, 4);
                    usleep(1000000);
                    readData(uart0_filestream, inputBuffer, 9);
                    sem_post(&mutex);

                }
            }
            break;

        case 7:
            printf("Menu\n");
            if (selectedOption == 3){
                selectedOption = 0;
            }
            else{
                selectedOption++;
            }

            totalTime = menuOptions[selectedOption].time;
            memcpy(buffer, (char *)&menuOptions[selectedOption].time, sizeof(int));
            sem_wait(&mutex);
            sendData(uart0_filestream, SEND_TIMER_VALUE, buffer, 4);
            usleep(1000000);
            readData(uart0_filestream, inputBuffer, 9);
            sem_post(&mutex);
            break;

        default:
            printf("Comando Inesperado\n");
            break;
        }
    }
}

void setup(int argc, char *argv[]){

    frango.temperature = 70;
    frango.time = 3;
    strcpy(frango.name, (char *)FRG);
        carne.temperature = 70;
        carne.time = 5;
    strcpy(carne.name, CRN);
        pizza.temperature = 60;
        pizza.time = 2;
    strcpy(pizza.name, PZZ);
        userOption.temperature = 0;
        userOption.time = 0;
    strcpy(userOption.name, CST);
        menuOptions[0] = userOption;
        menuOptions[1] = frango;
        menuOptions[2] = carne;
        menuOptions[3] = pizza;

    uart0_filestream = openUart();

    if (wiringPiSetup() == -1)
        exit(1);

    fd = wiringPiI2CSetup(I2C_ADDR);

    pinMode(RESISTOR, OUTPUT);
    pinMode(FAN, OUTPUT);

    softPwmCreate(RESISTOR, 0, 100);
    softPwmCreate(FAN, 0, 100);

    int8_t rslt = BME280_OK;

    if (argc < 2){
        fprintf(stderr, "Missing argument for i2c bus.\n");
        exit(1);
    }

    if ((id.fd = open(argv[1], O_RDWR)) < 0){
        fprintf(stderr, "Failed to open the i2c bus %s\n", argv[1]);
        exit(1);
    }

    id.dev_addr = BME280_I2C_ADDR_PRIM;

    if (ioctl(id.fd, I2C_SLAVE, id.dev_addr) < 0){
        fprintf(stderr, "Failed to acquire bus access and/or talk to slave.\n");
        exit(1);
    }

    /* Make sure to select BME280_I2C_ADDR_PRIM or BME280_I2C_ADDR_SEC as needed */

    dev.intf = BME280_I2C_INTF;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;
    dev.intf_ptr = &id;

    /* Initialize the bme280 */
    rslt = bme280_init(&dev);
    if (rslt != BME280_OK){
        fprintf(stderr, "Failed to initialize the device (code %+d).\n", rslt);
        exit(1);
    }
    printf("Temperature, Pressure, Humidity\n");
}

int openUart(){

    int filestream = -1;

    filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY); // Open in non blocking read/write mode
    if (filestream == -1){
        printf("Erro - Não foi possível iniciar a UART.\n");
    }
    else{
        printf("UART inicializada!\n");
    }

    struct termios options;
    tcgetattr(filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD; //<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(filestream, TCIFLUSH);
    tcsetattr(filestream, TCSANOW, &options);

    return filestream;
};

void pid_configura_constantes(double Kp_, double Ki_, double Kd_)
{
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
}

void pid_atualiza_referencia(float referencia_)
{
    referencia = (double)referencia_;
}

double pid_controle(double saida_medida)
{

    double erro = referencia - saida_medida;

    erro_total += erro; // Acumula o erro (Termo Integral)

    if (erro_total >= sinal_de_controle_MAX){
        erro_total = sinal_de_controle_MAX;
    }
    else if (erro_total <= sinal_de_controle_MIN){
        erro_total = sinal_de_controle_MIN;
    }

    double delta_error = erro - erro_anterior; // Diferença entre os erros (Termo Derivativo)

    sinal_de_controle = Kp * erro + (Ki * T) * erro_total + (Kd / T) * delta_error; // PID calcula sinal de controle

    if (sinal_de_controle >= sinal_de_controle_MAX){
        sinal_de_controle = sinal_de_controle_MAX;
    }
    else if (sinal_de_controle <= sinal_de_controle_MIN){
        sinal_de_controle = sinal_de_controle_MIN;
    }

    erro_anterior = erro;
    return sinal_de_controle;
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr){

    struct identifier id;
    id = *((struct identifier *)intf_ptr);
    write(id.fd, &reg_addr, 1);
    read(id.fd, data, len);
    return 0;
}

/*!
 * @brief This function provides the delay for required time (Microseconds) as per the input provided in some of the
 * APIs
 */

void user_delay_us(uint32_t period, void *intf_ptr){
    usleep(period);
}

/*!
 * @brief This function for writing the sensor's registers through I2C bus.
 */
int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr){

    uint8_t *buf;
    struct identifier id;
    id = *((struct identifier *)intf_ptr);
    buf = malloc(len + 1);
    buf[0] = reg_addr;
    memcpy(buf + 1, data, len);

    if (write(id.fd, buf, len + 1) < (uint16_t)len){
        return BME280_E_COMM_FAIL;
    }

    free(buf);
    return BME280_OK;
}

/*!
 * @brief This API used to print the sensor temperature, pressure and humidity data.
 */

void print_sensor_data(struct bme280_data *comp_data){

    float temp, press, hum;

#ifdef BME280_FLOAT_ENABLE
    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;
#else
#ifdef BME280_64BIT_ENABLE
    temp = 0.01f * comp_data->temperature;
    press = 0.0001f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#else
    temp = 0.01f * comp_data->temperature;
    press = 0.01f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#endif
#endif
    roomTemperature = temp;
    printf("room temperature: %f\n", temp);
}

/*!
 * @brief This API reads the sensor temperature, pressure and humidity data in forced mode.
 */

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev){

    /* Variable to define the result */
    int8_t rslt = BME280_OK;

    /* Variable to define the selecting sensors */
    uint8_t settings_sel = 0;

    /* Variable to store minimum wait time between consecutive measurement in force mode */
    uint32_t req_delay;

    /* Structure to get the pressure, temperature and humidity values */
    struct bme280_data comp_data;

    /* Recommended mode of operation: Indoor navigation */
    dev->settings.osr_h = BME280_OVERSAMPLING_1X;
    dev->settings.osr_p = BME280_OVERSAMPLING_16X;
    dev->settings.osr_t = BME280_OVERSAMPLING_2X;
    dev->settings.filter = BME280_FILTER_COEFF_16;

    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    /* Set the sensor settings */
    rslt = bme280_set_sensor_settings(settings_sel, dev);
    if (rslt != BME280_OK){
        fprintf(stderr, "Failed to set sensor settings (code %+d).", rslt);
        return rslt;
    }

    printf("Temperature, Pressure, Humidity\n");

    /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
     *  and the oversampling configuration. */
    req_delay = bme280_cal_meas_delay(&dev->settings);

    /* Continuously stream sensor data */
    while (1){
        usleep(2000000);
        /* Set the sensor to forced mode */
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        if (rslt != BME280_OK){
            fprintf(stderr, "Failed to set sensor mode (code %+d).", rslt);
            break;
        }

        /* Wait for the measurement to complete and print data */
        dev->delay_us(req_delay, dev->intf_ptr);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        if (rslt != BME280_OK){
            fprintf(stderr, "Failed to get sensor data (code %+d).", rslt);
            break;
        }

        print_sensor_data(&comp_data);
    }

    return rslt;
}