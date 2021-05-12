/* mbed */
#include "mbed.h"
#include <iostream>
#include <math.h>
#define PI 3.14159265
using namespace std;
/* Lab9 RPC */
#include "mbed_rpc.h"
/* HW2 uLCD */
#include "uLCD_4DGL.h"
/* Lab8 ML */
#include "tfconfig.h"
#include "magic_wand_model_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "accelerometer_handler.h"
/* Lab10 MQTT & Accelerometer */
#include "stm32l475e_iot01_accelero.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"


////////////////////////////////////////////////////////////
/* Global Thread */
Thread gesture_thread;
Thread detection_thread;
Thread mqtt_thread(osPriorityHigh);
Thread publish_thread;

////////////////////////////////////////////////////////////
/* Global EventQueue */
EventQueue gesture_queue;
EventQueue detection_queue;
EventQueue mqtt_queue;
EventQueue publish_queue;

////////////////////////////////////////////////////////////
/* RPC variable */
RpcDigitalOut myled1(LED1,"myled1");
RpcDigitalOut myled2(LED2,"myled2");
RpcDigitalOut myled3(LED3,"myled3");
BufferedSerial pc(USBTX, USBRX);
void LEDControl(Arguments *in, Reply *out);
RPCFunction rpcLED(&LEDControl, "LEDControl");
double x, y;

////////////////////////////////////////////////////////////
/* enable variable */
DigitalOut if_gesture_mode(LED1);
DigitalOut if_detection_mode(LED2);
DigitalOut led3(LED3);

////////////////////////////////////////////////////////////
/* uLCD variable */
uLCD_4DGL uLCD(D1, D0, D2);

////////////////////////////////////////////////////////////
/* ML variable */
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
////////////////////////////////////////////////////////////
/* accelerometer variable */
int16_t acc_data_XYZ_pre[3] = {0};
int16_t acc_data_XYZ_aft[3] = {0};

////////////////////////////////////////////////////////////
/* MQTT variable */
WiFiInterface *wifi;
InterruptIn btn2(USER_BUTTON);
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
const char* topic = "Mbed";
MQTT::Client<MQTTNetwork, Countdown> *global_client;

////////////////////////////////////////////////////////////
/* capture mode */
int tfanalysis[10];
int myanalysis[10];
int xyzdata[2][3];
bool mysensor = 0;

////////////////////////////////////////////////////////////
/* thres angel */
#define thres_angle_mode_max 3
int thres_angle_mode = 0;
int thres_angle_table[thres_angle_mode_max] = {30, 45, 60};
int thres_over_counter = 0;

void readRPCCommand();
void uLCDInit();
void uLCDDisplay(double inform);
void publish_MQTT();
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client);
void messageArrived(MQTT::MessageData& md);
void close_mqtt();
void gestureMode();
int  gestureMode_gestureVerify();
int  PredictGesture(float* output);
void detectionMode();

int main() {
   gesture_queue.call(&gestureMode);
   detection_queue.call(&detectionMode);
   publish_queue.call(&publish_MQTT);
   gesture_thread.start(callback(&gesture_queue, &EventQueue::dispatch_forever));
   detection_thread.start(callback(&detection_queue, &EventQueue::dispatch_forever));
   publish_thread.start(callback(&publish_queue, &EventQueue::dispatch_forever));
   uLCDDisplay(0);
   readRPCCommand();
}

void readRPCCommand() {
   char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }
}
void uLCDInit() {
   uLCD.background_color(BLACK);
   uLCD.cls();
   uLCD.text_width(3);
   uLCD.text_height(3);
}
void uLCDDisplay(double inform) {
   if (if_gesture_mode) {
      uLCD.color(GREEN);
      uLCD.locate(2, 2);
      uLCD.printf("%d", int(inform));
   }
   else if (if_detection_mode) {
      int count = thres_over_counter;
      uLCD.color(BLUE);
      uLCD.locate(2, 2);
      uLCD.printf("%2d", int(inform));
      uLCD.color(RED);
      uLCD.locate(2, 3);
      uLCD.printf("%2d", count);
   }
   else {
      uLCDInit();
      uLCD.color(WHITE);
      uLCD.locate(1, 2);
      uLCD.printf("RPC");
      uLCD.locate(1, 3);
      uLCD.printf("loop");
   }
}
void gestureMode() {
   int temp;
   while (1) {
      if (if_gesture_mode) {
         if_detection_mode = 0;
         uLCDDisplay(0);
         BSP_ACCELERO_Init();
         for (int i = 0; i < 10; i++) {
            BSP_ACCELERO_AccGetXYZ(acc_data_XYZ_pre);
            temp = gestureMode_gestureVerify();
            BSP_ACCELERO_AccGetXYZ(acc_data_XYZ_aft);
            tfanalysis[i] = temp;
            myanalysis[i] = (acc_data_XYZ_aft[2] > acc_data_XYZ_pre[2]);
         }
         cout << "\ntfanalysis:";
         for (int i = 0; i < 10; i++) {
            cout << tfanalysis[i] << " ";
         }
         cout << "\nmyanalysis:";
         for (int i = 0; i < 10; i++) {
            cout << myanalysis[i] << " ";
         }
         cout << endl;
         publish_message(global_client);
      }
   }
}
void detectionMode() {
   while (1) {
      if (if_detection_mode) {
         if_gesture_mode = 0;
            ThisThread::sleep_for(3000ms);
            cout << endl;
            for (int i = 0; i < 10; i++) {
               cout << tfanalysis[i] << endl;
            }
            for (int i = 0; i < 10; i++) {
               cout << myanalysis[i] << endl;
            }
            cout << endl;
         if_detection_mode = 0;
      }
   }
}
void publish_MQTT()  {
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return;
    }


    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return;
    }


    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);
    global_client = &client;

    //TODO: revise host to your IP
    const char* host = "192.168.31.205";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    btn2.rise(mqtt_queue.event(&publish_message, &client));
    //btn3.rise(&close_mqtt);

    int num = 0;
    while (num != 5) {
            client.yield(100);
            ++num;
    }

    while (1) {
            if (closed) break;
            client.yield(500);
            ThisThread::sleep_for(500ms);
    }

    printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
    printf("Failed: rc from disconnect was %d\n", rc);
    }
    
    //mqttNetwork.disconnect();
    //printf("Successfully closed!\n");
    return;
}
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    MQTT::Message message;
    char buff[100];
    if (if_gesture_mode) {
       sprintf(buff, "0 (capture mode -> RPC loop)\r\n");
       printf("/LEDControl/run 1 0\r\n");
       if_gesture_mode = 0;
    }
    else if (if_detection_mode) {
       sprintf(buff, "%d (overtilt times)\r\n", thres_over_counter);
    }
    else {
       sprintf(buff, "???\r\n");
    }
   
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);

    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}
int gestureMode_gestureVerify() {
   // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return;
  }

  error_reporter->Report("Set up successful...\n");

  while (if_gesture_mode) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;
      
      //change thres angel
      if (gesture_index == 0) {
         if (thres_angle_mode < thres_angle_mode_max - 1) thres_angle_mode++;
         else thres_angle_mode = thres_angle_mode_max - 1;
         cout << "tf = 0\n";
         uLCDInit();
         uLCDDisplay(gesture_index);
         return 0;
      }
      else if (gesture_index == 1) {
         if (thres_angle_mode > 0) thres_angle_mode--;
         else thres_angle_mode = 0;
         cout << "tf = 1\n";
         uLCDInit();
         uLCDDisplay(gesture_index);
         return 1;
      }
      if (gesture_index == 2) {
         if (thres_angle_mode < thres_angle_mode_max - 1) thres_angle_mode++;
         else thres_angle_mode = thres_angle_mode_max - 1;
         cout << "tf = 2\n";
         uLCDInit();
         uLCDDisplay(gesture_index);
         return 2;
      }
      
    // Produce an output
    if (gesture_index < label_num) {
      error_reporter->Report(config.output_message[gesture_index]);
    }
  }
}
void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}
void close_mqtt() {
    closed = true;
}
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}
void LEDControl (Arguments *in, Reply *out) {
    bool success = true;

    // In this scenario, when using RPC delimit the two arguments with a space.
    x = in->getArg<double>();
    y = in->getArg<double>();

    // Have code here to call another RPC function to wake up specific led or close it.
    char buffer[200], outbuf[256];
    char strings[20];
    int led = x;
    int on = y;
    sprintf(strings, "/myled%d/write %d", led, on);
    strcpy(buffer, strings);
    RPC::call(buffer, outbuf);
    if (success) {
        out->putData(buffer);
    } else {
        out->putData("Failed to execute LED control.");
    }
}