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

static uint32_t state = 0;

static QueueHandle_t scaleX_queue;
static QueueHandle_t scaleY_queue;

static QueueHandle_t pos_X_queue;
static QueueHandle_t pos_Y_queue;

static QueueHandle_t state_queue;

static bool start_flag = false;
static bool stop_flag = false;
static bool home_flag = false;
static bool homed_flag = false;
static bool running = false;
static bool yForce_Limit_Hit = false;

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
          if (forceX > xForce_MAX)
          {
            
          }
          continue;
        }

      if (scaleY.is_ready())
        {
          forceY = int(scaleY.get_units(1));
          //Serial.println(forceY);
          if (forceY > yForce_MAX)
          {
            yForce_Limit_Hit = true;
          }
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
    xdriver.rms_current(800);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    xdriver.toff(4);     //? 0 no good, 1 no good, 2 works
    xdriver.blank_time(24);
    xdriver.ihold(0);
    xdriver.dc_time(0);
    xdriver.dc_sg(0);
    xdriver.en_pwm_mode(1);      // Enable extremely quiet stepping
    xdriver.pwm_autoscale(1);    //enable automatic PWM current control
    xdriver.microsteps(16);   //0 means full step... (0,2,4,)
    xdriver.TCOOLTHRS(0); // 20bit max  set to MAX because is disables stall guard.
    xdriver.THIGH(0);    // 0 upper threshold for operation with stallguard
    xdriver.vhighchm(true);  //full stepping on high velocity commands
    xdriver.semin(5);    //cool step lower threshold 
    xdriver.semax(2);    //cool step upper
    xdriver.sedn(0b01);
    xdriver.sgt(STALL_VALUE);

    ydriver.begin();             // Initiate pins and registeries
    ydriver.rms_current(900);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    ydriver.toff(4);     //? 0 no good, 1 no good, 2 works
    ydriver.blank_time(24);
    ydriver.ihold(0);
    ydriver.dc_time(0);
    ydriver.dc_sg(0);
    ydriver.en_pwm_mode(1);      // Enable extremely quiet stepping
    ydriver.pwm_autoscale(1);    //enable automatic PWM current control
    ydriver.microsteps(16);   //0 means full step... (0,2,4,)
    ydriver.TCOOLTHRS(0); // 20bit max  set to MAX because is disables stall guard.
    ydriver.THIGH(0);    // 0 upper threshold for operation with stallguard
    ydriver.vhighchm(true);  //full stepping on high velocity commands
    ydriver.semin(5);    //cool step lower threshold 
    ydriver.semax(2);    //cool step upper
    ydriver.sedn(0b01);
    ydriver.sgt(STALL_VALUE);

    Serial.print("X_DRV_STATUS=0b");
	  Serial.println(xdriver.DRV_STATUS(), BIN);

    Serial.print("Y_DRV_STATUS=0b");
	  Serial.println(ydriver.DRV_STATUS(), BIN);

    stepperX.setMaxSpeed(maxSpeed); 
    stepperX.setAcceleration(maxAccel); 
    stepperX.setEnablePin(EN_PIN);
    stepperX.setPinsInverted(false, false, true);
    stepperX.enableOutputs();

    stepperY.setMaxSpeed(maxSpeed); 
    stepperY.setAcceleration(maxAccel); 
    stepperY.setEnablePin(EN_PIN);
    stepperY.setPinsInverted(false, false, true);
    stepperY.enableOutputs();

    //stepperX.move(-xInsert);
    // do
    // {
    //   stepperX.run();
    // } while (!xdriver.stallguard());

    while (1)
    {
      stepperX.run();
      stepperY.run();
      if (!running) vTaskDelay(1);

      if (yForce_Limit_Hit)
      {
        stepperY.setAcceleration(10000000);
        yForce_Limit_Hit = false;
        uint64_t tempYpos = stepperY.targetPosition();
        stepperY.move(-5);
        stepperY.runToPosition();
        stepperY.moveTo(tempYpos);
        stepperY.setAcceleration(maxAccel);
      }
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
          home_flag = true;
          break;
        case 1:
          Serial.println("Start");
          start_flag = true;
          break;
        case 2:
          Serial.println("Stop");
          stop_flag = true;
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

void HomeSteppers()
{
  uint8_t xStall = 0;
  uint8_t yStall = 0;
  running = true;
  home_flag = false;
  stepperX.setMaxSpeed(homeSpeed);
  stepperY.setMaxSpeed(homeSpeed);
  stepperX.move(xHome);
  stepperY.move(yHome);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  while(!(xdriver.stst() && ydriver.stst()))
  {
    genie.DoEvents();
    vTaskDelay(1 / portTICK_PERIOD_MS);

    //AUTO HOME TESTING
    if (!xdriver.ola()) xStall++;
    else xStall = 0;

    if (!ydriver.ola()) yStall++;
    else yStall = 0;

    if (xStall >= xHomeSense) stepperX.stop();
    if (yStall >= yHomeSense) stepperY.stop();

    if (stop_flag or home_flag)
    {
      stepperX.stop();
      stepperY.stop();
      break;
    }
  }
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  home_flag = false;
  homed_flag = !stop_flag;
  stepperX.setMaxSpeed(maxSpeed);
  stepperY.setMaxSpeed(maxSpeed);
  stepperX.moveTo(xStartPos);
  stepperY.moveTo(yStartPos);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  while(!(xdriver.stst() && ydriver.stst()))
  {
    genie.DoEvents();
    //vTaskDelay(1 / portTICK_PERIOD_MS);

    if (stop_flag)
    {
      stepperX.stop();
      stepperY.stop();
      break;
    }
  }
  Serial.println("Done");
  running = false;
}

void TaskStateMachine(void* pvParameters){
    (void)pvParameters;
    uint32_t state = 5;
    bool startMove = true;

    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    genie.Begin(Serial2);  //Software Serial
    genie.WriteContrast(1); // About 2/3 Max Brightness
    //genie.WriteStr(0, (String) "Hello 4D World");
    genie.AttachEventHandler(myGenieEventHandler);

    Serial.println("Main");

    while (1)
    {
      genie.DoEvents();

      //Check if home button pressed
      if (home_flag && !running)
      {
        HomeSteppers();
      }

      //Check if stop button pressed
      if (stop_flag)
      {
        stepperX.stop();
        stepperY.stop();
        startMove = true;
        state = 5;
      }

      //State Machine
      switch (state)
      {

        case 0:                                                   //Homing
            if (startMove && !homed_flag)
            {
              HomeSteppers();
            }
            state = 1;
            startMove = true;
            break;


        case 1:                                                   //Go to start position
            if (startMove)
            {
              running = true;
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              stepperY.moveTo(yStartPos);
              stepperX.moveTo(xStartPos);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (xdriver.stst() && ydriver.stst())
            {
              state = 2;
              startMove = true;
            }
            break;


        case 2:                                                    //Insert X
            if (startMove)
            {
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              stepperX.moveTo(xInsert);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (xdriver.stst())
            {
              state = 3;
              startMove = true;
            }
            break;


        case 3:                                                   //Retract X and Insert Y
            if (startMove)
            {
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              stepperY.moveTo(yInsert);
              vTaskDelay(500 / portTICK_PERIOD_MS);
              stepperX.moveTo(xStartPos);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (xdriver.stst() && ydriver.stst())
            {
              state = 4;
              startMove = true;
            }
            break;


        case 4:                                                     //Retract Y (Go To Home)
            if (startMove)
            {
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              stepperY.moveTo(yStartPos);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (ydriver.stst())
            {
              
              state = 5;
              startMove = true;
            }
            break;


        case 5:                                                     //Wait for button press and reset button states
            if (startMove)
            {
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              start_flag = false;
              home_flag = false;
              stop_flag = false;
              running = false;
            }
            if (start_flag)
            {
              vTaskDelay(1000 / portTICK_PERIOD_MS);
              start_flag = false;
              state = 0;
              startMove = true;
            }
            break;
      }
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
  //vTaskSuspend(task_sensors);
  scaleX_queue = xQueueCreate(1, sizeof(int));
  scaleY_queue = xQueueCreate(1, sizeof(int));  

  xTaskCreatePinnedToCore(TaskSteppers, "Stepper Motors", 2048, NULL, 0, &task_steppers, 0);
  //vTaskSuspend(task_steppers);
  pos_X_queue = xQueueCreate(1, sizeof(int));
  pos_Y_queue = xQueueCreate(1, sizeof(int));  

  xTaskCreatePinnedToCore(TaskStateMachine, "State Machine", 2048, NULL, 2, &task_statemachine, 1);
  //vTaskSuspend(task_sensors);
  state_queue = xQueueCreate(1, sizeof(int));

  //xTaskCreatePinnedToCore(TaskStateMachine, "Main Loop", 2048, NULL, 0, &task_statemachine, 1);
  //vTaskSuspend(task_statemachine)

  //vTaskDelete(NULL); // Delete "setup and loop" task



}

void loop() {
  //vTaskDelay(10 / portTICK_PERIOD_MS);
  // int force;
  // if (xQueuePeek(scaleX_queue, (void*)&force, 0) == pdTRUE) {
  //   Serial.println(force);
  // }
  
}

