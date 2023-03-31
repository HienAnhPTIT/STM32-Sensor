// /*
//  * Based on AnalogRead_DigitalRead example from: https://github.com/feilipu/Arduino_FreeRTOS_Library
//  * Modified by: Frederic Pillon <frederic.pillon (at) st.com>
//  */
// #include <DHT.h>
// #include <Wire.h>
// #include <ArduinoJson.h>
// #include <STM32FreeRTOS.h>
// #include <Adafruit_Sensor.h>
// // #include <Adafruit_BusIO_Register.h>
// HardwareSerial Serial1(USART1);

// #define DHTPIN PB1    // Chân GPIO được sử dụng để kết nối với cảm biến DHT
// #define DHTTYPE DHT11 // Loại cảm biến DHT (DHT11 hoặc DHT22)
// #define LEDPIN PC13
// #define LIGHTSENSOR PA1
// #define LED1 PC14
// #define LED2 PC15
// // #define SOUNDPIN PB0     // Chân GPIO được sử dụng để kết nối với cảm biến DHT

// DHT dht(DHTPIN, DHTTYPE);

// bool leda = 0;
// bool ledb = 0;

// // If no default pin for user button (USER_BTN) is defined, define it
// // #ifndef USER_BTN
// // #define USER_BTN 2
// // #endif

// // Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// // It will be used to ensure only only one Task is accessing this resource at any time.
// SemaphoreHandle_t xSerialSemaphore;

// // define two Tasks for DigitalRead & AnalogRead
// void TaskDHTRead(void *pvParameters);
// void TaskLightRead(void *pvParameters);
// void TaskServerSend(void *pvParameters);
// void TaskServerGive(void *pvParameters);
//     float temperature = 1.00 * dht.readTemperature();
//     float humidity = 1.00 * dht.readHumidity();
//     float lightValue = 1.00 * analogRead(LIGHTSENSOR);

// // the setup function runs once when you press reset or power the board
// void setup()
// {
//     // initialize serial communication at 9600 bits per second:
//     Serial.begin(9600);
//     dht.begin();
//     pinMode(LEDPIN, OUTPUT);
//     // pinMode(SOUNDPIN, INPUT_ANALOG);
//     pinMode(LED1, OUTPUT);
//     pinMode(LED2, OUTPUT);
//     digitalWrite(LED1, LOW);
//     digitalWrite(LED2, LOW);
//     pinMode(LIGHTSENSOR, INPUT_ANALOG);
//     while (!Serial)
//     {
//         ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
//     }

//     // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
//     // because it is sharing a resource, such as the Serial port.
//     // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
//     if (xSerialSemaphore == NULL) // Check to confirm that the Serial Semaphore has not already been created.
//     {
//         xSerialSemaphore = xSemaphoreCreateMutex(); // Create a mutex semaphore we will use to manage the Serial Port
//         if ((xSerialSemaphore) != NULL)
//             xSemaphoreGive((xSerialSemaphore)); // Make the Serial Port available for use, by "Giving" the Semaphore.
//     }

//     // Now set up two Tasks to run independently.
//     xTaskCreate(
//         TaskDHTRead, (const portCHAR *)"DHTRead" // A name just for humans
//         ,
//         128 // This stack size can be checked & adjusted by reading the Stack Highwater
//         ,
//         NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
//         ,
//         NULL);

//     xTaskCreate(
//         TaskLightRead, (const portCHAR *)"LightRead", 128 // Stack size
//         ,
//         NULL, 2 // Priority
//         ,
//         NULL);
//     xTaskCreate(
//         TaskServerSend, (const portCHAR *)"ServerSend" // A name just for humans
//         ,
//         128 // This stack size can be checked & adjusted by reading the Stack Highwater
//         ,
//         NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
//         ,
//         NULL);
//     xTaskCreate(
//         TaskServerGive, (const portCHAR *)"ServerGive" // A name just for humans
//         ,
//         128 // This stack size can be checked & adjusted by reading the Stack Highwater
//         ,
//         NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
//         ,
//         NULL);

//     // start scheduler
//     vTaskStartScheduler();
//     Serial.println("Insufficient RAM");
//     while (1)
//         ;
// }

// void loop()
// {
//     // Empty. Things are done in Tasks.
// }

// /*--------------------------------------------------*/
// /*---------------------- Tasks ---------------------*/
// /*--------------------------------------------------*/

// void TaskDHTRead(void *pvParameters __attribute__((unused))) // This is a Task.
// {
//     /*
//       DigitalReadSerial
//       Reads a digital input on pin defined with USER_BTN, prints the result to the serial monitor

//       This example code is in the public domain.
//     */

//     // defined USER_BTN digital pin  has a pushbutton attached to it. Give it a name:
//     //   uint8_t pushButton = USER_BTN;

//     // make the pushbutton's pin an input:
//     //   pinMode(pushButton, INPUT);

//     for (;;) // A Task shall never return or exit.
//     {
//         // read the input pin:
//         // int buttonState = digitalRead(pushButton);
        

//         // See if we can obtain or "Take" the Serial Semaphore.
//         // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
//         if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
//         {
//             // We were able to obtain or "Take" the semaphore and can now access the shared resource.
//             // We want to have the Serial Port for us alone, as it takes some time to print,
//             // so we don't want it getting stolen during the middle of a conversion.
//             // print out the state of the button:
//             //   Serial.print("Button state: ");
//             //   Serial.println(buttonState);
//             //   // Serial1.print("Nhiet do: ");
//             Serial1.print(temperature);
//             Serial1.print(" *C, Do am: ");
//             Serial1.print(humidity);
//             Serial1.println(" %RH");

//             xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
//         }

//         vTaskDelay(1); // one tick delay (15ms) in between reads for stability
//     }
// }

// void TaskLightRead(void *pvParameters __attribute__((unused))) // This is a Task.
// {

//     for (;;)
//     {
//         // read the input on analog pin 0:
//         // int sensorValue = analogRead(A0);

//         // See if we can obtain or "Take" the Serial Semaphore.
//         // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
//         if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
//         {
//             // We were able to obtain or "Take" the semaphore and can now access the shared resource.
//             // We want to have the Serial Port for us alone, as it takes some time to print,
//             // so we don't want it getting stolen during the middle of a conversion.
//             // print out the value you read:
//             Serial.println(lightValue);

//             xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
//         }

//         vTaskDelay(1); // one tick delay (15ms) in between reads for stability
//     }
// }

// void TaskServerGive(void *pvParameters __attribute__((unused))) // This is a Task.
// {
//     /*
//       DigitalReadSerial
//       Reads a digital input on pin defined with USER_BTN, prints the result to the serial monitor

//       This example code is in the public domain.
//     */

//     // defined USER_BTN digital pin  has a pushbutton attached to it. Give it a name:
//     //   uint8_t pushButton = USER_BTN;

//     // make the pushbutton's pin an input:
//     //   pinMode(pushButton, INPUT);

//     for (;;) // A Task shall never return or exit.
//     {

//         // See if we can obtain or "Take" the Serial Semaphore.
//         // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
//         if (Serial1.available() > 0 && xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
//         {
//             // We were able to obtain or "Take" the semaphore and can now access the shared resource.
//             // We want to have the Serial Port for us alone, as it takes some time to print,
//             // so we don't want it getting stolen during the middle of a conversion.
//             // print out the state of the button:
//             // Read from  STM module and send to serial monitor
//             String input = Serial1.readString();

//             /*
//             Phân tích chuỗi JSON và lưu trữ kết quả vào một DynamicJsonDocument.
//             Sau đó, chúng ta sử dụng các trường tương ứng của DynamicJsonDocument
//             để lấy ra giá trị của trường "temperature" và "humidity".
//             */
//             DynamicJsonDocument jsonDocSG(128); // Kích thước buffer là 128 byte

//             // Phân tích chuỗi JSON
//             deserializeJson(jsonDocSG, input);

//             // Lấy giá trị nhiệt độ và độ ẩm
//             bool ledac = jsonDocSG["led1"];
//             digitalWrite(LED1, (ledac == 1) ? HIGH : LOW);
//             if (ledac != leda)
//             {
//                 leda = !leda;
//             }
//             // if( leda == 1 ||  leda == true){
//             // digitalWrite(LED1, HIGH);
//             // }
//             bool ledbc = jsonDocSG["led2"];
//             digitalWrite(LED2, (ledbc == 1) ? HIGH : LOW);
//             if (ledbc != ledb)
//             {
//                 ledb = !ledb;
//             }
//             xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
//         }

//         vTaskDelay(20); // one tick delay (15ms) in between reads for stability
//     }
// }

// void TaskServerSend(void *pvParameters __attribute__((unused))) // This is a Task.
// {
//     /*
//       DigitalReadSerial
//       Reads a digital input on pin defined with USER_BTN, prints the result to the serial monitor

//       This example code is in the public domain.
//     */

//     // defined USER_BTN digital pin  has a pushbutton attached to it. Give it a name:
//     //   uint8_t pushButton = USER_BTN;

//     // make the pushbutton's pin an input:
//     //   pinMode(pushButton, INPUT);
   
//     for (;;) // A Task shall never return or exit.
//     {

//         // See if we can obtain or "Take" the Serial Semaphore.
//         // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
//         if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE)
//         {
//             String a = (String)leda;
//             String b = (String)ledb;
//             //////////////
//             // Định dạng chuỗi JSON
//             DynamicJsonDocument jsonDocSS(128);
//             jsonDocSS["temperature"] = 1.00 * temperature;
//             jsonDocSS["humidity"] = 1.00 * humidity;
//             jsonDocSS["lightValue"] = 1.00 * lightValue;
//             // jsonDocSS["soundValue"] = soundValue;
//             jsonDocSS["led1"] = leda;
//             jsonDocSS["led2"] = ledb;
//             String jsonStringSS;
//             serializeJson(jsonDocSS, jsonStringSS);
//             ////////////
//             // if (isnan(temperature) || isnan(humidity) || isnan(lightValue) || isnan(leda) || isnan(ledb)) {
//             Serial1.println(jsonStringSS);
//             delay(1000);
//             xSemaphoreGive(xSerialSemaphore); // Now free or "Give" the Serial Port for others.
//         }

//         vTaskDelay(1000); // one tick delay (15ms) in between reads for stability
//     }
// }
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_BusIO_Register.h>
#include <ArduinoJson.h>
HardwareSerial Serial1(USART1);

#define DHTPIN PB1     // Chân GPIO được sử dụng để kết nối với cảm biến DHT
// #define SOUNDPIN PB0     // Chân GPIO được sử dụng để kết nối với cảm biến DHT
#define DHTTYPE DHT11  // Loại cảm biến DHT (DHT11 hoặc DHT22)
#define LEDPIN PC13
#define LIGHTSENSOR PA1
#define LED1 PC14
#define LED2 PC15


DHT dht(DHTPIN, DHTTYPE);

  bool leda = 0;
  bool ledb = 0;

void setup() {
  Serial1.begin(9600);
  dht.begin();
  pinMode(LEDPIN, OUTPUT);
  // pinMode(SOUNDPIN, INPUT_ANALOG);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  pinMode(LIGHTSENSOR, INPUT_ANALOG);
}

void loop() {
  // digitalWrite(LEDPIN, LOW);
  // delay(1000);
  // digitalWrite(LEDPIN, HIGH);
  delay(1000);
  float temperature = 1.00*dht.readTemperature();
  float humidity = 1.00*dht.readHumidity();
  float lightValue = 1.00*analogRead(LIGHTSENSOR);
  // float soundValue = analogRead(SOUNDPIN);


  ///
  if (Serial1.available() > 0)
  {
    // Read from  STM module and send to serial monitor
    String input = Serial1.readString();

    /*
    Phân tích chuỗi JSON và lưu trữ kết quả vào một DynamicJsonDocument.
    Sau đó, chúng ta sử dụng các trường tương ứng của DynamicJsonDocument
    để lấy ra giá trị của trường "temperature" và "humidity".
    */
    DynamicJsonDocument jsonDocSG(128); // Kích thước buffer là 128 byte

    // Phân tích chuỗi JSON
    deserializeJson(jsonDocSG, input);

    // Lấy giá trị nhiệt độ và độ ẩm
    bool ledac = jsonDocSG["led1"];
    digitalWrite(LED1, (ledac == 1 ) ? HIGH : LOW);
    if( ledac != leda){
    leda = !leda;
    }
    // if( leda == 1 ||  leda == true){
    // digitalWrite(LED1, HIGH);
    // }
    bool ledbc = jsonDocSG["led2"];
    digitalWrite(LED2, (ledbc == 1 ) ? HIGH : LOW);
    if( ledbc != ledb){
    ledb = !ledb;
    }
    // if( ledb == 1 || ledb == ue){
    // digitalWrite(LED2, HIGH);tr
    // }
    // Serial.println(input);
    // Serial.print("led1: ");
    // Serial.println(leda);
    // Serial.print("led2: ");
    // Serial.println(ledb);
    delay(20);
  }
  ///
  String a = (String)leda;
  String b = (String)ledb;
  //////////////
  // Định dạng chuỗi JSON
  DynamicJsonDocument jsonDocSS(128);
  jsonDocSS["temperature"] = 1.00*temperature;
  jsonDocSS["humidity"] = 1.00*humidity;
  jsonDocSS["lightValue"] = 1.00*lightValue;
  // jsonDocSS["soundValue"] = soundValue;
  jsonDocSS["led1"] = leda;
  jsonDocSS["led2"] = ledb;
  String jsonStringSS;
  serializeJson(jsonDocSS, jsonStringSS);
  ////////////
  // if (isnan(temperature) || isnan(humidity) || isnan(lightValue) || isnan(leda) || isnan(ledb)) {
  Serial1.println(jsonStringSS);
  delay(1000);
  

  /////
  
  // if (isnan(lightValue) || isnan(leda) || isnan(ledb)) {
  
  //   Serial1.println("Khong the doc du lieu tu cam bien DHT!");
  //   delay(1000);
  // } 
  // // if (!isnan(temperature) || !isnan(humidity) || !isnan(lightValue) || !isnan(leda) || !isnan(ledb)) {
  // if (!isnan(lightValue) || !isnan(leda) || !isnan(ledb)) {
  
  //   // Serial1.print("Nhiet do: ");
  //   // Serial1.print(temperature);
  //   // Serial1.print(" *C, Do am: ");
  //   // Serial1.print(humidity);
  //   // Serial1.println(" %RH");
  //   // Gửi chuỗi JSON qua UART
  // }

  /////
  ///////////////////////
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// #include <DHT.h>
// #include <Wire.h>
// #include <STM32FreeRTOS.h>
// #include <Adafruit_BusIO_Register.h>
// HardwareSerial Serial1(USART1);

// // Cấu hình cảm biến DHT11
// #define DHTPIN PA0
// #define DHTTYPE DHT11
// DHT dht(DHTPIN, DHTTYPE);

// // Cấu hình cảm biến ánh sáng
// #define LDRPIN PA1

// // Task cho cảm biến DHT11
// void TaskDHT11(void *pvParameters) {
//   (void) pvParameters;
//   float humidity, temperature;
//   for (;;) {
//     humidity = dht.readHumidity();
//     temperature = dht.readTemperature();
//     if (!isnan(humidity) && !isnan(temperature)) {
//       Serial1.print("Humidity: ");
//       Serial1.print(humidity);
//       Serial1.print("% Temperature: ");
//       Serial1.print(temperature);
//       Serial1.println("C");
//     } else {
//       Serial1.println("Failed to read DHT11 sensor!");
//     }
//     vTaskDelay(pdMS_TO_TICKS(2000));
//   }
// }

// // Task cho cảm biến ánh sáng
// void TaskLDR(void *pvParameters) {
//   (void) pvParameters;
//   int ldr_value;
//   for (;;) {
//     ldr_value = analogRead(LDRPIN);
//     Serial1.print("LDR value: ");
//     Serial1.println(ldr_value);
//     vTaskDelay(pdMS_TO_TICKS(1000));
//   }
// }

// // Task cho đèn LED
// void TaskLED(void *pvParameters) {
//   (void) pvParameters;
//   for (;;) {
//     digitalWrite(PC13, HIGH);
//     vTaskDelay(pdMS_TO_TICKS(500));
//     digitalWrite(PC13, LOW);
//     vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }

// void setup() {
//   Serial1.begin(9600);
//   pinMode(PC13, OUTPUT);
//   dht.begin();
//   xTaskCreate(TaskDHT11, "DHT11", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
//   xTaskCreate(TaskLDR, "LDR", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//   xTaskCreate(TaskLED, "LED", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
// }

// void loop() {
//   vTaskStartScheduler();
// }

///////////////////////////////////////////

// /*
//  * Example to demonstrate thread definition, semaphores, and thread sleep.
//  */
// #include <STM32FreeRTOS.h>
// #include <Wire.h>

// // Define the LED pin is attached
// const uint8_t LED_PIN = PC13;

// // Declare a semaphore handle.
// SemaphoreHandle_t sem;
// //------------------------------------------------------------------------------
// /*
//  * Thread 1, turn the LED off when signalled by thread 2.
//  */
// // Declare the thread function for thread 1.
// static void Thread1(void* arg) {
//   UNUSED(arg);
//   while (1) {

//     // Wait for signal from thread 2.
//     xSemaphoreTake(sem, portMAX_DELAY);

//     // Turn LED off.
//     digitalWrite(LED_PIN, LOW);
//   }
// }
// //------------------------------------------------------------------------------
// /*
//  * Thread 2, turn the LED on and signal thread 1 to turn the LED off.
//  */
// // Declare the thread function for thread 2.
// static void Thread2(void* arg) {
//   UNUSED(arg);
//   pinMode(LED_PIN, OUTPUT);

//   while (1) {
//     // Turn LED on.
//     digitalWrite(LED_PIN, HIGH);

//     // Sleep for 200 milliseconds.
//     vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);

//     // Signal thread 1 to turn LED off.
//     xSemaphoreGive(sem);

//     // Sleep for 200 milliseconds.
//     vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
//   }
// }
// //------------------------------------------------------------------------------
// void setup() {
//   portBASE_TYPE s1, s2;

//   Serial.begin(9600);

//   // initialize semaphore
//   sem = xSemaphoreCreateCounting(1, 0);

//   // create task at priority two
//   s1 = xTaskCreate(Thread1, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);

//   // create task at priority one
//   s2 = xTaskCreate(Thread2, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

//   // check for creation errors
//   if (sem== NULL || s1 != pdPASS || s2 != pdPASS ) {
//     Serial.println(F("Creation problem"));
//     while(1);
//   }

//   // start scheduler
//   vTaskStartScheduler();
//   Serial.println("Insufficient RAM");
//   while(1);
// }

// //------------------------------------------------------------------------------
// // WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// // loop must never block
// void loop() {
//   // Not used.
// }

////////////////////////////////////////////////////////////////////////



// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BusIO_Register.h>
// #include <Adafruit_Serial.h>
// // #include <STM32FreeRTOS.h>
// #include <HardwareSerial.h>

// #define Serial1_RX PA10
// #define Serial1_TX PA9

// HardwareSerial Serial1(USART1);

// void setup() {
//   Serial1.begin(115200);
  
//   pinMode(Serial1_RX, INPUT_PULLUP);
//   pinMode(Serial1_TX, OUTPUT);

//   GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
//   USART_DeInit(USART1);

//   USART_InitTypeDef USART_InitStructure;
//   USART_StructInit(&USART_InitStructure);
//   USART_InitStructure.USART_BaudRate = 115200;
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;
//   USART_InitStructure.USART_Parity = USART_Parity_No;
//   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//   USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//   USART_Init(USART1, &USART_InitStructure);
//   USART_Cmd(USART1, ENABLE);
// }


//////////////////////////////////////////////////


