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

typedef enum
{
    e_NoError = 0,
    e_UnknownError = 1,
    e_ValveOpen = 2,
    e_XForceLimit = 3,
    e_AlreadyStarted = 4,
    e_Num_ErrorCodes
} Error_Codes_t;

typedef enum
{
    e_Homeing = 0,
    e_Start = 1,
    e_Stabalize = 2,
    e_Insert = 3,
    e_Retract = 4,
    e_Idle = 5,
    e_XPlus = 6,
    e_XMinus = 7,
    e_YPlus = 8,
    e_YMinus = 9,
    e_Origin = 10,
    e_Backup = 11,
    e_Num_States
} States_t;

EventGroupHandle_t xEventGroupMain = xEventGroupCreate();
EventGroupHandle_t xEventGroupManual = xEventGroupCreate();

EventBits_t START_BIT = (1<<0);
EventBits_t STOP_BIT = (1<<1);
EventBits_t HOME_BIT = (1<<2);
EventBits_t HOMED_BIT = (1<<3);
EventBits_t RUNNING_BIT = (1<<4);
EventBits_t YFORCE_BIT = (1<<5);
EventBits_t VALVE_BIT = (1<<6);
EventBits_t BACKUP_BIT = (1<<7);

EventBits_t XJOGPLUS_BIT = (1<<0);
EventBits_t XJOGMINUS_BIT = (1<<1);
EventBits_t YJOGPLUS_BIT = (1<<2);
EventBits_t YJOGMINUS_BIT = (1<<3);
EventBits_t ORIGIN_BIT = (1<<4);
EventBits_t XFORCE_BIT = (1<<5);


static bool start_flag = false;
static bool stop_flag = false;
static bool home_flag = false;
static bool homed_flag = false;
static bool running = false;
static bool yForce_Limit_Hit = false;
static bool xJogPlus_flag = false;
static bool xJogMinus_flag = false;
static bool yJogPlus_flag = false;
static bool yJogMinus_flag = false;
static bool origin_flag = false;
static bool valveState = false;

static int JogSteps = stepspermm;
static int JogMM = 0;
static String JogStr = "";

void TaskSensors(void* pvParameters);
void TaskSteppers(void* pvParameters);
void TaskStateMachine(void* pvParameters);
void Jog_X(int steps);
void Jog_Y(int steps);

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
    int forceX, forceY, count;
    Serial.println("SCALE");

    scaleX.begin(dataPinX, clockPin);                                   
    scaleX.set_offset(22008);
    scaleX.set_scale(3325.150146);  

    scaleY.begin(dataPinY, clockPin);                                   
    scaleY.set_offset(22008);
    scaleY.set_scale(3325.150146);  

    while (1) {
      count++;
      vTaskDelay(1 / portTICK_PERIOD_MS);
      if (scaleX.is_ready())
        {
          forceX = int(scaleX.get_units(1));
          if (count >= force_refresh) genie.WriteObject(GENIE_OBJ_METER, 1, max(0, forceX * 100/2516));
          if (forceX > xForce_MAX)
          {
            
          }
          continue;
        }

      if (scaleY.is_ready())
        {
          forceY = int(scaleY.get_units(1));
          if (count >= force_refresh) genie.WriteObject(GENIE_OBJ_METER, 0, max(0, forceY) * 100/2516);
          if (forceY > yForce_MAX)
          {
            Serial.println("backup2");
            xEventGroupSetBits(xEventGroupMain, YFORCE_BIT);
          }
          continue;
        }
      if (count >= force_refresh) count = 0;
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
    xdriver.rms_current(900);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    xdriver.toff(4);     //? 0 no good, 1 no good, 2 works
    xdriver.blank_time(24);
    xdriver.ihold(200);
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
    ydriver.ihold(200);
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

    while (1)
    {
      /*if (xEventGroupGetBits(xEventGroupMain) & YFORCE_BIT)
      {
        xEventGroupSetBits(xEventGroupMain, BACKUP_BIT);
        stepperY.setAcceleration(100000);
        //xEventGroupClearBits(xEventGroupMain, YFORCE_BIT);
        uint64_t tempYpos = stepperY.targetPosition();
        stepperY.move(1000);
        //vTaskDelay(10 / portTICK_PERIOD_MS);
        stepperY.runToPosition();
        if (xEventGroupGetBits(xEventGroupMain) & RUNNING_BIT) stepperY.moveTo(tempYpos);
        stepperY.setAcceleration(maxAccel);
      }
      else
      {
        xEventGroupClearBits(xEventGroupMain, BACKUP_BIT);
      }*/

      stepperX.run();
      stepperY.run();
      if (!(xEventGroupGetBits(xEventGroupMain) & RUNNING_BIT)) vTaskDelay(1);
    }
}
static TaskHandle_t task_steppers= NULL;

void Jog_X(int steps)
{
  xEventGroupClearBits(xEventGroupManual, XJOGPLUS_BIT);//xJogPlus_flag = false;
  xEventGroupClearBits(xEventGroupManual, XJOGMINUS_BIT);//xJogMinus_flag = false;
  xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);//running = true;
  stepperX.move(steps);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  while(!xdriver.stst())
  {
    genie.DoEvents();
    vTaskDelay(1 / portTICK_PERIOD_MS);

    if (xEventGroupGetBits(xEventGroupMain) & STOP_BIT)
    {
      stepperX.stop();
      stepperY.stop();
    }
  }
  xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);//running = false;
}

void Jog_Y(int steps)
{
  xEventGroupClearBits(xEventGroupManual, YJOGPLUS_BIT);//yJogPlus_flag = false;
  xEventGroupClearBits(xEventGroupManual, YJOGMINUS_BIT);//yJogMinus_flag = false;
  xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);//running = true;
  stepperY.move(steps);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  while(!ydriver.stst() || (xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))
  {
    genie.DoEvents();
    vTaskDelay(1 / portTICK_PERIOD_MS);

    if (xEventGroupGetBits(xEventGroupMain) & STOP_BIT)
    {
      stepperX.stop();
      stepperY.stop();
    }
  }
  xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);//running = false;
}

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
          xEventGroupSetBits(xEventGroupMain, HOME_BIT);//home_flag = true;
          break;
        case 1:
          Serial.println("Start");
          xEventGroupSetBits(xEventGroupMain, START_BIT);//start_flag = true;
          break;
        case 2:
          Serial.println("Stop");
          xEventGroupSetBits(xEventGroupMain, STOP_BIT);//stop_flag = true;
          break;
        case 4:
          Serial.print("Valve ");
          (xEventGroupGetBits(xEventGroupMain) & VALVE_BIT) ? xEventGroupClearBits(xEventGroupMain, VALVE_BIT) : xEventGroupSetBits(xEventGroupMain, VALVE_BIT);//valveState = !valveState;
          digitalWrite(valve_PIN, (xEventGroupGetBits(xEventGroupMain) & VALVE_BIT));
          Serial.println((xEventGroupGetBits(xEventGroupMain) & VALVE_BIT) ? "OPEN" : "CLOSED");
        default:
          break;
      }
    }

    else if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)
    {
      switch (Event.reportObject.index)
      {
        case 0:
          Serial.print("JogX +");
          Serial.println(JogSteps);
          xEventGroupSetBits(xEventGroupManual, XJOGPLUS_BIT);//xJogPlus_flag = true;
          break;
        case 1:
          Serial.print("JogX -");
          Serial.println(JogSteps);
          xEventGroupSetBits(xEventGroupManual, XJOGMINUS_BIT);//xJogMinus_flag = true;
          break;
        case 3:
          Serial.print("JogY +");
          Serial.println(JogSteps);
          xEventGroupSetBits(xEventGroupManual, YJOGPLUS_BIT);//yJogPlus_flag = true;
          break;
        case 2:
          Serial.print("JogY -");
          Serial.println(JogSteps);
          xEventGroupSetBits(xEventGroupManual, YJOGMINUS_BIT);//yJogMinus_flag = true;
          break;
        case 5:
          Serial.println("Increase JogSteps");
          JogSteps *= 10;
          JogMM = JogSteps / stepspermm;
          if (JogMM < 10) JogStr = "  " + (String)JogMM + " ";
          else if (JogMM < 100) JogStr = " " + (String)JogMM + " ";
          else if (JogMM < 1000) JogStr = " " + (String)JogMM;
          else JogStr = (String)JogMM;
          genie.WriteStr(1, JogStr);
          break;
        case 6:
          Serial.println("Decrease JogSteps");
          JogSteps /= 10;
          if (JogSteps < 1) JogSteps = 1;
          JogMM = JogSteps / stepspermm;
          if (JogMM < 10) JogStr = "  " + (String)JogMM + " ";
          else if (JogMM < 100) JogStr = " " + (String)JogMM + " ";
          else if (JogMM < 1000) JogStr = " " + (String)JogMM;
          else JogStr = (String)JogMM;
          genie.WriteStr(1, JogStr);
          break;
        case 7:
          Serial.println("Go to origin");
          xEventGroupSetBits(xEventGroupManual, ORIGIN_BIT);//origin_flag = true;
        default:
          break;
      }
    }

    else if (Event.reportObject.object == GENIE_OBJ_FORM)
    {
      Serial.println(Event.reportObject.index);
      switch (Event.reportObject.index)
      {
        case 2:
          JogMM = JogSteps / stepspermm;
          if (JogMM < 10) JogStr = "  " + (String)JogMM + " ";
          else if (JogMM < 100) JogStr = " " + (String)JogMM + " ";
          else if (JogMM < 1000) JogStr = " " + (String)JogMM;
          else JogStr = (String)JogMM;
          genie.WriteStr(1, JogStr);
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
  if (xEventGroupGetBits(xEventGroupMain) & RUNNING_BIT) return;
  uint8_t xStall = 0;
  uint8_t yStall = 0;
  int forceX, forceY;
  int temphome = xEventGroupGetBits(xEventGroupMain) & HOME_BIT;
  xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);
  xEventGroupSetBits(xEventGroupMain, HOMED_BIT);
  xEventGroupClearBits(xEventGroupMain, HOME_BIT);
  stepperX.setMaxSpeed(homeSpeed);
  stepperY.setMaxSpeed(homeSpeed);
  stepperX.move(xHome);
  stepperY.move(yHome);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  while(!(xdriver.stst() && ydriver.stst()))
  {
    genie.DoEvents();
    vTaskDelay(1 / portTICK_PERIOD_MS);

    if (!xdriver.olb() && !xdriver.ola()) xStall++;
    else xStall = 0;

    if (!ydriver.olb() && !ydriver.ola()) yStall++;
    else yStall = 0;

    Serial.println(xdriver.stallguard());

    //if (xStall >= xHomeSense) stepperX.stop();
    //if (yStall >= yHomeSense) stepperY.stop();

    if ((xEventGroupGetBits(xEventGroupMain) & STOP_BIT) || (xEventGroupGetBits(xEventGroupMain) & HOME_BIT))
    {
      stepperX.stop();
      stepperY.stop();
    }

    //Read sensors and update graph
    forceX = int(scaleX.get_units(1));
    forceY = int(scaleY.get_units(1));

    genie.WriteObject(GENIE_OBJ_SCOPE, 2, max(0, forceX * 100/2516));
    genie.WriteObject(GENIE_OBJ_SCOPE, 2, max(0, forceY * 100/2516));
  }
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  //xEventGroupClearBits(xEventGroupMain, HOME_BIT);//home_flag = false;
  (xEventGroupGetBits(xEventGroupMain) & STOP_BIT) ? xEventGroupClearBits(xEventGroupMain, HOMED_BIT) : xEventGroupSetBits(xEventGroupMain, HOMED_BIT);//homed_flag = !stop_flag;
  stepperX.setMaxSpeed(maxSpeed);
  stepperY.setMaxSpeed(maxSpeed);
  stepperX.moveTo(xStartPos);
  stepperY.moveTo(yStartPos);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  while(!(xdriver.stst() && ydriver.stst()))
  {
    genie.DoEvents();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (xEventGroupGetBits(xEventGroupMain) & STOP_BIT)
    {
      stepperX.stop();
      stepperY.stop();
    }

    //Read sensors and update graph
    forceX = int(scaleX.get_units(1));
    forceY = int(scaleY.get_units(1));

    genie.WriteObject(GENIE_OBJ_SCOPE, 2, max(0, forceX * 100/2516));
    genie.WriteObject(GENIE_OBJ_SCOPE, 2, max(0, forceY * 100/2516));
  }
  if (temphome) xEventGroupSetBits(xEventGroupMain, HOME_BIT);
  else xEventGroupClearBits(xEventGroupMain, HOME_BIT);
  Serial.println("Done");
  xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);//running = false;
}

void GoTo_Origin()
{
  xEventGroupClearBits(xEventGroupManual, ORIGIN_BIT);//origin_flag = false;
  xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);//running = true;
  stepperX.moveTo(xStartPos);
  stepperY.moveTo(yStartPos);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  while(!(ydriver.stst() && xdriver.stst()) || (xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))
  {
    genie.DoEvents();

    if (xEventGroupGetBits(xEventGroupMain) & STOP_BIT)
    {
      stepperX.stop();
      stepperY.stop();
    }
  }
  xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);//running = false;
}



void TaskStateMachine(void* pvParameters){
    (void)pvParameters;
    States_t state = e_Idle;
    States_t temp_state = e_Idle;
    bool startMove = true;
    bool setup_backup = true;
    uint64_t tempYpos = 0;

    Error_Codes_t active_error = e_NoError;
    bool setup_error = false;
    String errorstr = "";

    int forceX = 0;
    int forceY = 0;
    Serial.println("SCALE");

    scaleX.begin(dataPinX, clockPin);                                   
    scaleX.set_offset(22008);
    scaleX.set_scale(3325.150146);  

    scaleY.begin(dataPinY, clockPin);                                   
    scaleY.set_offset(22008);
    scaleY.set_scale(3325.150146);

    vTaskDelay(2500 / portTICK_PERIOD_MS);

    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    genie.Begin(Serial2);  //Software Serial
    genie.WriteContrast(1); // About 2/3 Max Brightness
    genie.AttachEventHandler(myGenieEventHandler);
    JogMM = JogSteps / stepspermm;
    if (JogMM < 10) JogStr = "  " + (String)JogMM + " ";
    else if (JogMM < 100) JogStr = " " + (String)JogMM + " ";
    else if (JogMM < 1000) JogStr = " " + (String)JogMM;
    else JogStr = (String)JogMM;
    Serial.println("JOG:" + JogStr);
    genie.WriteStr(1, JogStr);

    Serial.println("Main");

    while (1)
    {
      //Handle Sensor Readings
      forceX = int(scaleX.get_units(1));
       if (forceX > xForce_MAX)
       {
          xEventGroupSetBits(xEventGroupManual, XFORCE_BIT);
       }

       forceY = int(scaleY.get_units(1));
       if (forceY > yForce_MAX)
       {
          xEventGroupSetBits(xEventGroupMain, YFORCE_BIT);
       }

       else xEventGroupClearBits(xEventGroupMain, YFORCE_BIT);
      

      genie.WriteObject(GENIE_OBJ_SCOPE, 2, max(0, forceX * 100/2516));
      genie.WriteObject(GENIE_OBJ_SCOPE, 2, max(0, forceY * 100/2516));
      
      //Handle Genie Events
      genie.DoEvents();
      vTaskDelay(1);  //  / portTICK_PERIOD_MS

      //Check for X force limit reached
      if ((xEventGroupGetBits(xEventGroupManual) & XFORCE_BIT))
      {
        xEventGroupClearBits(xEventGroupManual, XFORCE_BIT);
        xEventGroupClearBits(xEventGroupMain, BACKUP_BIT);
        if (xEventGroupGetBits(xEventGroupMain) & RUNNING_BIT)
        {
          stepperX.moveTo(0);
          stepperY.moveTo(0);
          //while(!(xdriver.stst() && ydriver.stst())) vTaskDelay(1 / portTICK_PERIOD_MS);
          startMove = true;
          state = e_Idle;
          active_error = e_XForceLimit;
        }
      }

      //Check for Y limit reached
      if ((xEventGroupGetBits(xEventGroupMain) & YFORCE_BIT) && (xEventGroupGetBits(xEventGroupMain) & RUNNING_BIT) && !(xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))  // 
      {
        temp_state = state;
        state = e_Backup;
        setup_backup = true;
      }

      //Check for button presses if not running
      if (!(xEventGroupGetBits(xEventGroupMain) & RUNNING_BIT))
      {
        if (xEventGroupGetBits(xEventGroupMain) & HOME_BIT) state = e_Homeing;//HomeSteppers();
        else if ((xEventGroupGetBits(xEventGroupManual) & XJOGPLUS_BIT) && state == e_Idle) state = e_XPlus;
        else if ((xEventGroupGetBits(xEventGroupManual) & XJOGMINUS_BIT) && state == e_Idle) state = e_XMinus;
        else if ((xEventGroupGetBits(xEventGroupManual) & YJOGPLUS_BIT) && state == e_Idle) state = e_YPlus;
        else if ((xEventGroupGetBits(xEventGroupManual) & YJOGMINUS_BIT) && state == e_Idle) state = e_YMinus;
        else if ((xEventGroupGetBits(xEventGroupManual) & ORIGIN_BIT) && state == e_Idle) GoTo_Origin();
      }

      //Check for button presses while running
      else
      {
        if ((xEventGroupGetBits(xEventGroupMain) & START_BIT)) active_error = e_AlreadyStarted;
      }

      //Check for errors
      if (active_error != e_NoError)
      {
        //generate error message
        switch (active_error)
        {
          case e_UnknownError:
            errorstr = "\nAn Unknown Error Occured\n\nSomething went wrong...";
            break;

          case e_ValveOpen:
            errorstr = "\nClamp is not engaged\n\nEngage the clamp before starting.";
            break;

          case e_XForceLimit:
            errorstr = "\nX force limit reached\n\nToo much force on X load sensor.";
            break;

          case e_AlreadyStarted:
            errorstr = "\nProcedure is running\n\nWait until the procedure is done.";
            xEventGroupClearBits(xEventGroupMain, START_BIT);
            break;

          default:
            errorstr = "If you see this something is wrong";
            break;
        }

        Serial.println(errorstr);

        genie.WriteObject(GENIE_OBJ_FORM, 4, 1);
        genie.WriteStr(0, errorstr);
        active_error = e_NoError;
      }

      //Check if stop button pressed
      if (xEventGroupGetBits(xEventGroupMain) & STOP_BIT)
      {
        stepperX.setAcceleration(maxAccel * 4);
        stepperY.setAcceleration(maxAccel * 4);
        stepperX.stop();
        stepperY.stop();
        while(!(xdriver.stst() && ydriver.stst())) vTaskDelay(1 / portTICK_PERIOD_MS);
        stepperX.setAcceleration(maxAccel);
        stepperY.setAcceleration(maxAccel);
        startMove = true;
        state = e_Idle;
      }

      //State Machine
      switch (state)
      {

        case e_Homeing:                                                //Homing
            Serial.print("state: ");
            Serial.println(state);
            Serial.println(startMove);
            Serial.println(xEventGroupGetBits(xEventGroupMain) & HOMED_BIT);
            Serial.println(xEventGroupGetBits(xEventGroupMain) & HOME_BIT);
            if (startMove && (!(xEventGroupGetBits(xEventGroupMain) & HOMED_BIT)) || (xEventGroupGetBits(xEventGroupMain) & HOME_BIT))
            {
              Serial.println("Homing...");
              HomeSteppers();
            }
            state = (xEventGroupGetBits(xEventGroupMain) & HOME_BIT) ? e_Idle : e_Start;
            xEventGroupClearBits(xEventGroupMain, HOME_BIT);
            startMove = true;
            break;


        case e_Start:                                                   //Go to start position
            if (startMove)
            {
              xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);//running = true;
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              stepperY.moveTo(yStartPos);
              stepperX.moveTo(xStartPos);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (xdriver.stst() && ydriver.stst() && !(xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))
            {
              if (xEventGroupGetBits(xEventGroupMain) & VALVE_BIT)
              {
                state = e_Stabalize;
                startMove = true;
              }

              else
              {
                state = e_Idle;
                active_error = e_ValveOpen;
                startMove = true;
              }
              
            }
            break;


        case e_Stabalize:                                                    //Insert X
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
              state = e_Insert;
              startMove = true;
            }
            break;


        case e_Insert:                                                   //Retract X and Insert Y
            if (startMove)
            {
              stepperY.setAcceleration(maxAccel);
              stepperX.setAcceleration(maxAccel);
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              stepperY.moveTo(yInsert);
              vTaskDelay((20000000 / maxSpeed) / portTICK_PERIOD_MS);
              stepperX.moveTo(xStartPos);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (xdriver.stst() && ydriver.stst() && !(xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))
            {
              state = e_Retract;
              startMove = true;
            }
            break;


        case e_Retract:                                                     //Retract Y (Go To Home)
            if (startMove)
            {
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              stepperY.moveTo(yStartPos);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if (ydriver.stst() && !(xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))
            {
              state = e_Idle;
              startMove = true;
            }
            break;


        case e_Idle:                                                     //Reset button states and wait for button press
            if (startMove)
            {
              Serial.print("state: ");
              Serial.println(state);
              startMove = false;
              xEventGroupClearBits(xEventGroupMain, (START_BIT + HOME_BIT + STOP_BIT));
              xEventGroupClearBits(xEventGroupManual, (XJOGMINUS_BIT + XJOGPLUS_BIT + YJOGMINUS_BIT + YJOGPLUS_BIT));
            }
            if (xdriver.stst() && ydriver.stst()) xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);
            if (xEventGroupGetBits(xEventGroupMain) & START_BIT)
            {
              vTaskDelay(100 / portTICK_PERIOD_MS);
              xEventGroupClearBits(xEventGroupMain, START_BIT);//start_flag = false;
              state = e_Homeing;
              startMove = true;
            }
            break;

        case e_XPlus:
            if (startMove)
            {
              xEventGroupClearBits(xEventGroupManual, XJOGPLUS_BIT);
              xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);
              startMove = false;
              stepperX.move(JogSteps);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if(xdriver.stst())
            {
              state = e_Idle;
              startMove = true;
              xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);
            }
            break;
        
        case e_XMinus:
        if (startMove)
            {
              xEventGroupClearBits(xEventGroupManual, XJOGMINUS_BIT);
              xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);
              startMove = false;
              stepperX.move(-JogSteps);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if(xdriver.stst())
            {
              state = e_Idle;
              startMove = true;
              xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);
            }
            break;

        case e_YPlus:
            if (startMove)
            {
              xEventGroupClearBits(xEventGroupManual, YJOGPLUS_BIT);
              xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);
              startMove = false;
              stepperY.move(JogSteps);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if(ydriver.stst() && !(xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))
            {
              state = e_Idle;
              startMove = true;
              xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);
            }
            break;

        case e_YMinus:
            if (startMove)
            {
              xEventGroupClearBits(xEventGroupManual, YJOGMINUS_BIT);
              xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);
              startMove = false;
              stepperY.move(-JogSteps);
              vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            else if(ydriver.stst() && !(xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))
            {
              state = e_Idle;
              startMove = true;
              xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);
            }
            break;

        case e_Origin:
          if (startMove)
          {
            xEventGroupClearBits(xEventGroupManual, ORIGIN_BIT);
            xEventGroupSetBits(xEventGroupMain, RUNNING_BIT);
            stepperX.moveTo(xStartPos);
            stepperY.moveTo(yStartPos);
            startMove = false;
            vTaskDelay(100 / portTICK_PERIOD_MS);
          }
          else if (xdriver.stst() && ydriver.stst() && !(xEventGroupGetBits(xEventGroupMain) & BACKUP_BIT))
          {
            state = e_Idle;
            startMove = true;
            xEventGroupClearBits(xEventGroupMain, RUNNING_BIT);
          }

        case e_Backup:
            if (setup_backup)
            {
              setup_backup = false;
              xEventGroupSetBits(xEventGroupMain, BACKUP_BIT);
              stepperY.setAcceleration(100000);
              tempYpos = stepperY.targetPosition();
              stepperY.move(200);
              vTaskDelay(1 / portTICK_PERIOD_MS);
            }

            else if (ydriver.stst())
            {
              if (xEventGroupGetBits(xEventGroupMain) & RUNNING_BIT) stepperY.moveTo(tempYpos);
              stepperY.setAcceleration(maxAccel);
              xEventGroupClearBits(xEventGroupMain, BACKUP_BIT);
              state = temp_state;
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
  //xTaskCreatePinnedToCore(TaskSensors, "Sensors Readings", 2048, NULL, 3, &task_sensors, 1);
  //vTaskSuspend(task_sensors);
  scaleX_queue = xQueueCreate(1, sizeof(int));
  scaleY_queue = xQueueCreate(1, sizeof(int));  

  xTaskCreatePinnedToCore(TaskSteppers, "Stepper Motors", 2048, NULL, 3, &task_steppers, 1);
  //vTaskSuspend(task_steppers);
  pos_X_queue = xQueueCreate(1, sizeof(int));
  pos_Y_queue = xQueueCreate(1, sizeof(int));  

  xTaskCreatePinnedToCore(TaskStateMachine, "State Machine", 2048, NULL, 3, &task_statemachine, 0);
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

