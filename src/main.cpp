#include <Arduino.h>
#include "HX711.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <pins.h>
#include <Parameters.h>
#include <genieArduino.h>

HX711 scaleX;
HX711 scaleY;
//TMC2130Stepper xdriver(X_CS_PIN, R_SENSE, 0); 
TMC2130Stepper xdriver = TMC2130Stepper(X_CS_PIN, R_SENSE, 13, 12, 14); // Software SPI 
TMC2130Stepper ydriver = TMC2130Stepper(Y_CS_PIN, R_SENSE, 13, 12, 14); // Software SPI 

AccelStepper stepperX = AccelStepper(stepperX.DRIVER, xSTEP_PIN, xDIR_PIN);
AccelStepper stepperY = AccelStepper(stepperY.DRIVER, ySTEP_PIN, yDIR_PIN);

Genie genie;

static QueueHandle_t scaleX_queue;
static QueueHandle_t scaleY_queue;

static QueueHandle_t pos_X_queue;
static QueueHandle_t pos_Y_queue;

void TaskSensors(void* pvParameters);
void TaskSteppers(void* pvParameters);
void TaskStateMachine(void* pvParameters);

void CalibrateScale(){
  scaleX.begin(dataPinX, clockPin, true);
  scaleX.tare(10);
  Serial.println("\n\nCALIBRATION\n===========");
  Serial.println("remove all weight from the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();
  Serial.println("and press enter\n");
  while (Serial.available() == 0);

  Serial.println("Determine zero weight offset");
  //scaleX.tare(20);  // average 20 measurements.
  uint32_t offset = scaleX.read();
  Serial.print("OFFSET: ");
  Serial.println(offset);
  Serial.println();


  Serial.println("place a weight on the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();
  Serial.println("and press enter\n");
  while (Serial.available() == 0);

  // Serial.println("enter the weight in (whole) grams and press enter");
  // uint32_t weight = 0;
  // while (Serial.peek() != '\n')
  // {
  //   if (Serial.available())
  //   {
  //     char ch = Serial.read();
  //     if (isdigit(ch))
  //     {
  //       weight *= 10;
  //       weight = weight + (ch - '0');
  //     }
  //   }
  // }
  // Serial.print("WEIGHT: ");
  // Serial.println(weight);
  //scaleX.calibrate_scale(weight, 20);
  scaleX.calibrate_scale(226.796, 20);
 
  float scale = scaleX.get_scale();

  Serial.print("SCALE:  ");
  Serial.println(scale, 6);

  Serial.print("\nuse scale.set_offset(");
  Serial.print(offset);
  Serial.print("); and scale.set_scale(");
  Serial.print(scale, 6);
  Serial.print(");\n");
  Serial.println("in the setup of your project");
  Serial.println("\n\n");

  scaleX.set_offset(offset);  

    while (Serial.available()) Serial.read();
  Serial.println("and press enter\n");
  while (Serial.available() == 0);
  scaleX.tare(10);
}

void TaskSensors(void* pvParameters) {
    (void)pvParameters;
    //static int step = 0;
    //float forceX, forceY;
    int forceX, forceY;
    Serial.println("SCALE");

    scaleX.begin(dataPinX, clockPin);                                   
    scaleX.set_offset(22008);
    scaleX.set_scale(3325.150146);  

    scaleY.begin(dataPinY, clockPin);                                   
    scaleY.set_offset(22008);
    scaleY.set_scale(3325.150146);  

    while (1) {
      if (scaleX.is_ready())
        {
          forceX = int(scaleX.get_units(1));
          xQueueReset(scaleX_queue);
          xQueueSend(scaleX_queue, (void*)&forceX, 10);
          //Serial.println(forceX);
          continue;
        }
      if (scaleY.is_ready())
        {
          forceY = int(scaleY.get_units(1));
          xQueueReset(scaleY_queue);
          xQueueSend(scaleY_queue, (void*)&forceY, 10);
          //Serial.println(forceY);
          continue;
        }

      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}
static TaskHandle_t task_sensors= NULL;

void TaskSteppers(void* pvParameters){
    (void)pvParameters;
    pinMode(EN_PIN, OUTPUT);
    pinMode(xSTEP_PIN, OUTPUT);
    pinMode(xDIR_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);      // Enable driver in hardware
    xdriver.begin();             // Initiate pins and registeries
    ydriver.begin();
    //xdriver.tbl(1); //blank_time(24);  //number of clock cycles to wait for current sense. (16, 24, 36, 54)
    //driver.freewheel(1);    //leads shorted.
    xdriver.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    //xdriver.ihold(1);            // 0to31 scale for holding current
    //xdriver.iholddelay(0);      //min delay before hold current applied 0...15
    //xdriver.TPOWERDOWN(255);     //max delay before powering down
    //xdriver.toff(4);     //? 0 no good, 1 no good, 2 works
    xdriver.en_pwm_mode(1);      // Enable extremely quiet stepping
    xdriver.pwm_autoscale(1);    //enable automatic PWM current control
    xdriver.microsteps(16);   //0 means full step... (0,2,4,)
    //xdriver.TCOOLTHRS(0xFFFFF); // 20bit max  set to MAX because is disables stall guard.

    xdriver.THIGH(0x0000F);    // 0 upper threshold for operation with stallguard
    xdriver.vhighchm(true);  //full stepping on high velocity commands
    xdriver.semin(5);    //cool step lower threshold 
    xdriver.semax(2);    //cool step upper
    xdriver.sedn(0b01);
    xdriver.sgt(STALL_VALUE);

    // xdriver.en_pwm_mode(1);      // Enable extremely quiet stepping
    // xdriver.pwm_autoscale(1);
    // xdriver.microsteps(16);
    Serial.print("DRV_STATUS=0b");
	  Serial.println(xdriver.DRV_STATUS(), BIN);

    stepperX.setMaxSpeed(maxSpeed); 
    stepperX.setAcceleration(maxAccel); 
    stepperX.setEnablePin(EN_PIN);
    stepperX.setPinsInverted(false, false, true);
    stepperX.enableOutputs();

    stepperX.move(-xInsert);
    // do
    // {
    //   stepperX.run();
    // } while (!xdriver.stallguard());

    while (1)
    {
      stepperX.run();
      stepperY.run();

      // if (xdriver.stallguard()) {
      //   Serial.println("Stall");
      //   vTaskDelay(100 / portTICK_PERIOD_MS);
      // }
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
}
static TaskHandle_t task_steppers= NULL;

void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event); // Remove the next queued event from the buffer, and process it below
  //int slider_val = 0;

  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)                // If the Reported Message was from a Slider
    {
      switch (Event.reportObject.index )
      {
      case 0:
        //genie.WriteObject(GENIE_OBJ_SOUND,0,1);
        Serial.println("Home");
        break;
      case 1:
        Serial.println("Start");
        break;
      case 2:
        Serial.println("Stop");
        break;
      default:
        break;
      }
      }
    }

  //If the cmd received is from a Reported Object, which occurs if a Read Object (genie.ReadOject) is requested in the main code, reply processed here.
  if (Event.reportObject.cmd == GENIE_REPORT_OBJ)
  {
    // if (Event.reportObject.object == GENIE_OBJ_USER_LED)              // If the Reported Message was from a User LED
    // {
    //   if (Event.reportObject.index == 0)                              // If UserLed0 (Index = 0)
    //   {
    //     bool UserLed0_val = genie.GetEventData(&Event);               // Receive the event data from the UserLed0
    //     UserLed0_val = !UserLed0_val;                                 // Toggle the state of the User LED Variable
    //     genie.WriteObject(GENIE_OBJ_USER_LED, 0, UserLed0_val);       // Write UserLed0_val value back to UserLed0
    //   }
    // }
  }

  /********** This can be expanded as more objects are added that need to be captured *************
  *************************************************************************************************
  Event.reportObject.cmd is used to determine the command of that event, such as an reported event
  Event.reportObject.object is used to determine the object type, such as a Slider
  Event.reportObject.index is used to determine the index of the object, such as Slider0
  genie.GetEventData(&Event) us used to save the data from the Event, into a variable.
  *************************************************************************************************/
}

void TaskStateMachine(void* pvParameters){
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  genie.Begin(Serial2);  //Software Serial
  genie.WriteContrast(1); // About 2/3 Max Brightness
  //genie.WriteStr(0, (String) "Hello 4D World");
  genie.AttachEventHandler(myGenieEventHandler);

  Serial.println("Main");

  while(1){
    genie.DoEvents();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
static TaskHandle_t task_statemachine= NULL;

void setup() {
#pragma region PinSetup
  pinMode(valve_PIN, OUTPUT);
  digitalWrite(valve_PIN, LOW);
#pragma endregion 

  Serial.begin(115200);
  //CalibrateScale();

  //Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);

  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  xTaskCreatePinnedToCore(TaskSensors, "Sensors Readings", 2048, NULL, 2, &task_sensors, 0);
  vTaskSuspend(task_sensors);
  scaleX_queue = xQueueCreate(1, sizeof(int));
  scaleY_queue = xQueueCreate(1, sizeof(int));  

  //xTaskCreatePinnedToCore(TaskSteppers, "Stepper Motors", 2048, NULL, 2, &task_steppers, 1);
  //vTaskSuspend(task_steppers);
  pos_X_queue = xQueueCreate(1, sizeof(int));
  pos_Y_queue = xQueueCreate(1, sizeof(int));  

  xTaskCreatePinnedToCore(TaskStateMachine, "Main Loop", 4096, NULL, 1, &task_statemachine, 1);
  //vTaskSuspend(task_statemachine)

  vTaskDelete(NULL); // Delete "setup and loop" task

}

void loop() {
  //vTaskDelay(10 / portTICK_PERIOD_MS);
  // int force;
  // if (xQueuePeek(scaleX_queue, (void*)&force, 0) == pdTRUE) {
  //   Serial.println(force);
  // }
  
}

