#include <Arduino.h>

/*
  При запуске:
  - Зажать кнопку - вход в меню настройки RPM. Измерить RPM, выставить энкодером, удержать кнопку для выхода

  В работе:
  - Клик - остановить или запустить мотор
  - Вращение энкодера - изменить целевой RPM
  - Удержание - переход к настройке Kp. Удержание - переход к настройке Ki. Удержание - выход из меню
*/

// =========================== CONFIG ===========================
#define DIV_R1 20000    // верхнее плечо делителя [Ом]
#define DIV_R2 4700     // нижнее плечо делителя [Ом]
#define REF1_1 1100     // напряжение опорного 1.1V [мВ]
#define MEAS_PRD 8      // период измерения BEMF [мс]
#define CALC_PRD 30     // период расчёта [мс]
#define WAIT_ADC 800    // ожидание после отключения ШИМ [мкс]
#define MIN_EMF 800     // минимальный EMF, ниже которого будет остановка [мВ]
#define STALL_TOUT 3000 // таймаут нахождения в режиме остановки [мс]
#define START_TOUT 500  // время плавного разгона [мс]
#define PWM_START 100   // ШИМ для настройки RPM при запуске

// измерение
#define VMOT_PIN A0 // пин измерения питания
#define BEMF_PIN A1 // пин измерения BEMF

// дисплей
#define SCLK_PIN 8
#define RCLK_PIN 7
#define DIO_PIN 6

// энкб
// #define ENCB_A 5
// #define ENCB_B 4
// #define ENCB_KEY 2

#define BTN_UP_PIN 5
#define BTN_DOWN_PIN 4
#define BTN_OK_PIN 2

// =========================== DATA ===========================
#include <GTimer.h>
#include <GyverPWM.h>
//
#include <EncButton.h>
// EncButton eb(ENCB_A, ENCB_B, ENCB_KEY);

// Создаем объекты для кнопок
Button btnOk(BTN_OK_PIN);
Button btnUp(BTN_UP_PIN);
Button btnDown(BTN_DOWN_PIN);

#include <GyverSegment.h>
Disp595_4 disp(DIO_PIN, SCLK_PIN, RCLK_PIN);

// VARS
uint8_t pwm;
int vref;
int targetEmf;
int setp;
bool firstStart = true;
// int direction = 0;
// Дополнительная переменная для контроля скорости изменения при удержании
unsigned long holdChangeInterval = 150; // Интервал изменения в миллисекундах при удержании (150 мс)
unsigned long lastChangeTime = 0;       // Время последнего изменения

#include "IntEMA.h"
IntEMA<int> vmot;
IntEMA<int> vemf;

#include "PIreg.h"
PIreg pi;

// DATA
struct Data
{
    int16_t rpm = 0;
    int16_t krpm = 0;
    float k = 0;
    float kp = 0.003;
    float ki = 0.03;
};
Data data;

#include <EEManager.h>
EEManager memory(data);

// STATE
enum class State
{
    Stop,
    Startup,
    Stabilize,
    Stall,
    Kp,
    Ki,
};
// State state = State::Startup;
State state = State::Stop; // При включении в режим Стоп

// =========================== FUNC ===========================

// чтение напряжения питания
uint16_t readVCC(uint16_t ref1v1)
{
    uint8_t muxt = ADMUX;
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(1);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC))
        ;
    uint16_t vcc = ref1v1 * 1023ul / ADC;
    ADMUX = muxt;
    return vcc;
}

// чтение с делителя напряжения
uint16_t readDivider(uint8_t apin)
{
    return (long)analogRead(apin) * vref * ((DIV_R1 + DIV_R2) / DIV_R2) / 1024;
}

// измерение BEFM
void measure()
{
    EVERY16_MS(MEAS_PRD)
    {
        PWM_16KHZ_D3(0);
        delayMicroseconds(WAIT_ADC);

        int curVCC = readDivider(VMOT_PIN);
        vmot.filter(curVCC, 2);

        int curBEMF = vmot - readDivider(BEMF_PIN);
        vemf.filter(curBEMF, 3);

        PWM_16KHZ_D3(pwm);
    }
}

///////////////////////////////////////////////////////////////////*

void startMenu()
{ // Функция вызывается только в сетапе, при первом запуске.

    // Опрашиваем кнопку OK
    btnOk.tick();

    if (btnOk.read())
    {
        disp.clearPrint("MENU");
        disp.delay(600);

        disp.clear();
        disp.home();
        disp.print("RPM");
        disp.update();

        // Ждем отпускания кнопки OK
        while (btnOk.read())
            disp.tick();

        disp.clearPrintR(data.krpm);

        pwm = PWM_START;
        /////////////////////////////////////////////////////////////////
        // --- ОСНОВНОЙ ЦИКЛ МЕНЮ ---
        while (true)
        {
            measure();
            disp.tick();
            btnOk.tick();
            btnUp.tick();
            btnDown.tick();
            int direction = 0;

            // --- ИЗМЕНЕНИЕ ПАРАМЕТРА (аналог поворотам энкодера) ---

            if (btnUp.click() || btnUp.hold() || btnUp.step())
                direction = 1;

            else if (btnDown.click() || btnDown.hold() || btnDown.step())
                direction = -1;

            // Логика изменения значения
            if (direction != 0)
            {
                data.krpm += 50 * direction;                // Используем direction как множитель
                data.krpm = constrain(data.krpm, 50, 5000); // Ограничение диапазона
                disp.clearPrintR(data.krpm);
            }

            // --- ВЫХОД ИЗ МЕНЮ ---
            // В  выход — при удержании кнопки энкодера.
            if (btnOk.hold())
            {
                data.k = vemf / (float)data.krpm; // вычисление коэффициента
                memory.updateNow();               // сохранение
                break;                            // выход из меню
            }
        }
    }
}

// расчёт управления
void calc()
{
    static uTimer16<millis> stateTmr(true);

    // таймер
    EVERY16_MS(CALC_PRD)
    {
        switch (state)
        {
        case State::Stop:
            return;

        case State::Startup:
            setp += targetEmf / (START_TOUT / CALC_PRD);
            if (setp >= targetEmf)
            {
                state = State::Stabilize;
                setp = targetEmf;
            }
            break;

        case State::Stabilize:
            setp = targetEmf;
            if (vemf < MIN_EMF)
            {
                state = State::Stall;
                setp = 0;
                disp.clearPrint("stop");
                stateTmr.start();
            }
            break;

        case State::Stall:
            if (stateTmr.timeout(STALL_TOUT))
            {
                stateTmr.start();
                // state = State::Startup;
                state = State::Stop;
            }
            break;

        default:
            break;
        }

        pwm = pi.compute(vemf, setp, CALC_PRD / 1000.0);

        Serial.print(vemf);
        Serial.print(',');
        Serial.print(pwm * 10);
        Serial.print(',');
        Serial.print(setp);
        Serial.println();
    }
}

void encbtn()
{
    // Опрашиваем все кнопки
    btnOk.tick();
    btnUp.tick();
    btnDown.tick();
    int direction = 0;

    // Обработка btnOk (замена click/encoder button)
    if (btnOk.click())
    {
        switch (state)
        {
        case State::Kp:
        case State::Ki:
            break;

        case State::Stop:
            state = State::Startup;
            disp.clearPrintR(data.rpm);
            break;

        default:
            state = State::Stop;
            pi.integral = 0;
            setp = 0;
            pwm = 0;
            disp.clearPrint("----");
            break;
        }
    }

    auto printKp = []()
    {
        disp.clear();
        disp.home();
        disp.print("p");
        disp.print(data.kp, 2);
        disp.update();
    };

    auto printKi = []()
    {
        disp.clear();
        disp.home();
        disp.print("i");
        disp.print(data.ki, 2);
        disp.update();
    };

    // Обработка удержания btnOk (замена hold)
    if (btnOk.hold())
    {

        // Serial.println("Hold OK detected");

        switch (state)
        {
        case State::Stall:
        case State::Startup:
        case State::Stabilize:
            state = State::Kp;
            setp = targetEmf;
            printKp();
            break;

        case State::Kp:
            state = State::Ki;
            printKi();
            break;

        case State::Ki:
            state = State::Stabilize;
            disp.clearPrintR(data.rpm);
            break;

        default:
            // Serial.println("Hold OK in unexpected state");
            break;
        }
    }

    /////////////////////////////////////////////////////////////////////////
    // Обработка btnUp и btnDown (замена turn/поворота энкодера)

    if (btnUp.click() || btnUp.hold() || btnUp.step())
        direction = 1;

    else if (btnDown.click() || btnDown.hold() || btnDown.step())
        direction = -1;

    // Логика изменения значения
    if (direction != 0)

    {
        // Serial.println("dir:" + String(direction) + " state:" + String(static_cast<int>(state)));
        switch (state)
        {

        case State::Stall:
        case State::Startup:
        case State::Stabilize:
            data.rpm += 50 * direction;
            data.rpm = constrain(data.rpm, 100, 3500);

            memory.update();
            targetEmf = data.rpm * data.k;
            disp.clearPrintR(data.rpm);
            break;

        case State::Kp:
            data.kp += 0.01 * direction;
            data.kp = max(0, data.kp);
            memory.update();
            pi.Kp = data.kp;
            printKp();
            break;

        case State::Ki:
            data.ki += 0.01 * direction;
            data.ki = max(0, data.ki);
            memory.update();
            pi.Ki = data.ki;
            printKi();
            break;

        default:
            break;
        }
    }
}

// =========================== SKETCH ===========================

void setup()
{
    Serial.begin(9600);
    vref = readVCC(REF1_1);
    vmot.init(readDivider(VMOT_PIN));

    // режим 3 кнопок явно (хотя конструктор выше уже это сделал)

    pinMode(BTN_OK_PIN, INPUT_PULLUP);
    pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
    pinMode(BTN_UP_PIN, INPUT_PULLUP);

    pinMode(3, OUTPUT);
    memory.begin(0, 'a');

    startMenu();

    targetEmf = data.rpm * data.k;
    pi.Kp = data.kp;
    pi.Ki = data.ki;

    disp.clearPrintR(data.rpm); // Показать установленные обороты
    disp.delay(500);
    disp.clearPrint("----"); // При включении в режим Стоп
}

void loop()
{

    encbtn();
    measure();
    calc();
    disp.tick();
    memory.tick();
}