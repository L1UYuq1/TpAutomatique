// NETTOYER REGULIEREMENT LE GROS FICHIER SUIVANT :!!!!
// sudo rm /var/log/uvcdynctrl-udev.log
// solution definitive :
//sudo apt remove uvcdynctrl
/* NE PAS OUBLIER DE FIXER LD_LIBRARY_PATH DANS LE TERMINAL POUR QUE CA FONCTIONNE
// Alt Shif F to Auto indent
export LD_LIBRARY_PATH="/home/ubuntu/kobuki/install/lib":"/home/ubuntu/realsense/librealsense/build/release/"
//DOC DES CLASSES ICI :
 https://yujinrobot.github.io/kobuki/index.html

// au tout debut c'est ici
 cd ~/kobuki
 source install/setup.sh
// pour compiler c'est ici :
sh ./compile.sh
// ICI POUR FIRMWARE UPDATE TO 1.2.0
*/
//  POUR POUVOIR MODIFIER GAINS DU CONTROLLER PID INTERNE SI ON DOIT TOUT RECOMPILER
// dans le fichier /home/ubuntu/kobuki/src/kobuki_core/src/driver/kobuki.cpp
//  dans la fct bool Kobuki::setControllerGain ligne 604 du fichier
// et pour la fct suivante Kobuki::setControllerGain
// IGNORER LES WARNINGS ET COMMENTER return false
// cd
/*******************************************************************************
** YGORRA includes
******************************************************************************/
 #include <control.hpp>
/*****************************************************************************
 ** kobuki Includes
 *****************************************************************************/
//#define SAVE_VIDEO_TO_FILE
#include <iostream>
#include <fstream>
#include <string>
#include <csignal>
#include <termios.h> // for keyboard input
#include <ecl/command_line.hpp>
#include <ecl/console.hpp>
#include <ecl/geometry.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/time.hpp>
#include <ecl/threads.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/exceptions.hpp>

#include <kobuki_core/kobuki.hpp>

/*****************************************************************************
 ** realsense2  Includes
 *****************************************************************************/
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

/* Include the librealsense C header files */
#include <librealsense2/rs.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_frame.h>

#include <stdlib.h>
#include <stdint.h>
#include <cstdio>
#include <fstream>  // File IO
#include <iostream> // Terminal IO
#include <sstream>  // Stringstreams

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "example.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STREAM RS2_STREAM_COLOR // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT RS2_FORMAT_RGB8  // rs2_format identifies how binary data is encoded within a frame      //
#define WIDTH 640               // Defines the number of columns for each frame                         //
#define HEIGHT 480              // Defines the number of lines for each frame                           //
#define FPS 30                  // Defines the rate of frames per second                                //
#define STREAM_INDEX 0          // Defines the stream index, used for multiple streams of the same type //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t myCheck_error(rs2_error *e);

/*****************************************************************************
** Classes
*****************************************************************************/

/**
 * @brief Keyboard remote control for our robot core (mobile base).
 *
 */

class KobukiManager
{
public:
  /*********************
   ** C&D
   **********************/
  KobukiManager();
  ~KobukiManager();
   
  bool init(const std::string &device);
  ControlStruct control;
  /*********************
   ** Callbacks
   **********************/
  void mainLoopStep();

  /*********************
   ** Accessor
   **********************/
  const ecl::linear_algebra::Vector3d &getPose() { return pose; };
  bool isShutdown() { return quit_requested || kobuki.isShutdown(); }

private:
  // realsense2 data
  uint8_t *rgb_frame_data;
  int16_t nbBitPerPixel, width, height;
  bool measureOk;
  float measureMeter; // ecart centre ligne / robot, a une distance de ? metres
  // keyboard data
  double vxKeyboard, wzKeyboard;
  ecl::linear_algebra::Vector3d pose;
  kobuki::Kobuki kobuki;

  double linear_vel_step, linear_vel_max;
  double angular_vel_step, angular_vel_max;
  std::string name;
  ecl::Slot<> slot_stream_data;

  /*********************
   ** Commands
   **********************/
  void incrementLinearVelocity();
  void decrementLinearVelocity();
  void incrementAngularVelocity();
  void decrementAngularVelocity();
  void resetVelocity();
  void showAllData(kobuki::CoreSensors::Data sensorData, ControlStruct control);

  /*********************
   ** Debugging
   **********************/
  void relayWarnings(const std::string &message);
  void relayErrors(const std::string &message);

  /*********************
   ** Keylogging
   **********************/
  void keyboardInputLoop();
  void realSense2Loop();
  void updateMeasure(uint8_t *rgb_frame_data);
  void processKeyboardInput(char c);
  void restoreTerminal();
  bool quit_requested;
  int key_file_descriptor;
  struct termios original_terminal_state;
  ecl::Thread threadKeyboard;
  ecl::Thread threadRealSense2;

  ecl::Mutex mutex;
};

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */
KobukiManager::KobukiManager() : vxKeyboard(0.0), wzKeyboard(0.0),
                                 linear_vel_step(0.2),
                                 linear_vel_max(1.0),
                                 angular_vel_step(0.66),
                                 angular_vel_max(6.6),
                                 slot_stream_data(&KobukiManager::mainLoopStep, *this),
                                 quit_requested(false),
                                 key_file_descriptor(0)
{
  tcgetattr(key_file_descriptor, &original_terminal_state); // get terminal properties
}

KobukiManager::~KobukiManager()
{
  endControl(control);
  kobuki.setBaseControl(0, 0); // linear_velocity, angular_velocity in (m/s), (rad/s)
  kobuki.disable();
  std::cout << std::endl
            << "]; " << std::endl;
  tcsetattr(key_file_descriptor, TCSANOW, &original_terminal_state);
}

/**
 * @brief Initialises the node.
 */
bool KobukiManager::init(const std::string &device)
{
  /*********************
   ** Parameters
   **********************/
  std::cout << "Parameters" << std::endl;
  std::cout << "----------" << std::endl;
  std::cout << " - linear_vel_max   [" << linear_vel_max << "]" << std::endl;
  std::cout << " - linear_vel_step  [" << linear_vel_step << "]" << std::endl;
  std::cout << " - angular_vel_max  [" << angular_vel_max << "]" << std::endl;
  std::cout << " - angular_vel_step [" << angular_vel_step << "]" << std::endl;
  std::cout << std::endl;

  /******************************
   ** Velocities and measure
   ******************************/
  vxKeyboard = 0.0;
  wzKeyboard = 0.0;
  measureMeter = 0.0;
  /*********************
   ** Kobuki
   **********************/
  kobuki::Parameters parameters;
  parameters.sigslots_namespace = "/kobuki";
  parameters.device_port = device;
  parameters.enable_acceleration_limiter = false;
  initControl(control);
  kobuki.init(parameters);
  kobuki.enable();
  slot_stream_data.connect("/kobuki/stream_data");

  /*********************
   ** Wait for connection
   **********************/
  threadKeyboard.start(&KobukiManager::keyboardInputLoop, *this);
  threadRealSense2.start(&KobukiManager::realSense2Loop, *this);

  return true;
}
void KobukiManager::updateMeasure(uint8_t *rgb_frame_data)
{
  // index of pixels passing treeshold
  int row = height - 1;
  int k0 = row * width * 3;
  int seuil_rgb = 150;
  // compute mean and sd of cols index greater than treeshold
  int mean_col = 0, pow_col = 0, nb_ok = 0;
  for (int col = 0, k = k0; col < width; col++)
  {
    uint8_t r = rgb_frame_data[k++];
    uint8_t g = rgb_frame_data[k++];
    uint8_t b = rgb_frame_data[k++];
    bool ok = (r > seuil_rgb) && (g > seuil_rgb) && (b > seuil_rgb);
    if (ok)
    {
      nb_ok++;
      pow_col += col * col;
      mean_col += col;
    }
  }
  // just keep measure larger than a given treeshold
  bool ok = nb_ok > (width / 30);
  if (!ok)
  {
    measureOk = false;
    return;
  }
  // refine measure, just keep samples < alpha. variance
  pow_col /= nb_ok;
  mean_col /= nb_ok;
  int var_col = pow_col - mean_col * mean_col;
  int mean_col2 = 0, nb_ok2 = 0;
  for (int col2 = 0, k = k0; col2 < width; col2++)
  {
    uint8_t r = rgb_frame_data[k++];
    uint8_t g = rgb_frame_data[k++];
    uint8_t b = rgb_frame_data[k++];
    ok = (r > seuil_rgb) && (g > seuil_rgb) && (b > seuil_rgb);
    if (ok)
    {
      ok = (col2 - mean_col) * (col2 - mean_col) < var_col * 1;
    }
    if (ok)
    {
      nb_ok2++;
      mean_col2 += col2;
    }
  }
  ok = (nb_ok2 > width / 100);
  if (!ok)
  {
    measureOk = false;
    return;
  }
  mean_col2 /= nb_ok2;
  // scale measure on the base of a rouleau de sopalin at the beginning of image (last column, I don't know why ...)
  float scaleMeter = 0.3;                       // 0.3m
  int scalePixel = ((589 - 140) * width) / 640; // (589-140) columns for a width of 640 pixels
  measureMeter = (scaleMeter * (mean_col2 - width / 2)) / scalePixel;
  // measureMeter=(scaleMeter*(mean_col-(width/2)))/scalePixel;
  // measureMeter=(float)mean_col;
  measureOk = true;
}

void KobukiManager::realSense2Loop()
{
  uint8_t deviceOk;
  rs2_error *e = 0;
  rs2_context *ctx;
  rs2_device_list *device_list;
  int dev_count;
  rs2_device *dev;
  rs2_pipeline *pipeline;
  rs2_config *config;
  // Create a context object. This object owns the handles to all connected realsense devices.
  // The returned object should be released with rs2_delete_context(...)

  rs2_pipeline_profile *pipeline_profile;
  // loop variables
  rs2_frame *frames;
  int num_of_frames;
  rs2_frame *frame;
  unsigned long long frame_number;
  rs2_time_t frame_timestamp;
  rs2_timestamp_domain frame_timestamp_domain;
  rs2_metadata_type frame_metadata_time_of_arrival;
  const char *frame_timestamp_domain_str;
  ctx = rs2_create_context(RS2_API_VERSION, &e);
  deviceOk = myCheck_error(e);

  /* Get a list of all the connected devices. */
  // The returned object should be released with rs2_delete_device_list(...)
  if (deviceOk)
  {
    device_list = rs2_query_devices(ctx, &e);
    deviceOk = myCheck_error(e);
  }
  if (deviceOk)
  {
    dev_count = rs2_get_device_count(device_list, &e);
    deviceOk = myCheck_error(e);
  }
  printf("There are %d connected RealSense devices.\n", dev_count);
  if (0 == dev_count)
    deviceOk = 0;
  // Get the first connected device
  // The returned object should be released with rs2_delete_device(...)
  if (deviceOk)
  {
    dev = rs2_create_device(device_list, 0, &e);
    deviceOk = myCheck_error(e);
    // print_device_info(dev);
  }
  // Create a pipeline to configure, start and stop camera streaming
  // The returned object should be released with rs2_delete_pipeline(...)

  if (deviceOk)
  {
    pipeline = rs2_create_pipeline(ctx, &e);
    deviceOk = myCheck_error(e);
  }
  // Create a config instance, used to specify hardware configuration
  // The retunred object should be released with rs2_delete_config(...)
  if (deviceOk)
  {
    config = rs2_create_config(&e);
    deviceOk = myCheck_error(e);
  }
  // Request a specific configuration

  if (deviceOk)
  {

    rs2_config_enable_stream(config, STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);
    deviceOk = myCheck_error(e);
    width = WIDTH;
    height = HEIGHT;
    nbBitPerPixel = 3;
  }

  // Start the pipeline streaming
  // The retunred object should be released with rs2_delete_pipeline_profile(...)
  if (deviceOk)
  {
    pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
    deviceOk = myCheck_error(e);
  }
  if (!deviceOk)
  {
    printf("The connected device doesn't support color streaming!\n");
  }

  int numFrame = 0;
  char imgFileName[100];
  while (1)
  {
    deviceOk = 1; // reset deviceOk at each loop
    // This call waits until a new composite_frame is available
    // composite_frame holds a set of frames. It is used to prevent frame drops
    // The returned object should be released with rs2_release_frame(...)
    if (deviceOk)
    {
      frames = rs2_pipeline_wait_for_frames(pipeline, RS2_DEFAULT_TIMEOUT, &e);
      deviceOk = myCheck_error(e);
    }
    // Returns the number of frames embedded within the composite frame
    if (deviceOk)
    {
      num_of_frames = rs2_embedded_frames_count(frames, &e);
      deviceOk = myCheck_error(e);
    }
    int i = num_of_frames - 1; // use only last frame

    if (i >= 0)
    {
      // The retunred object should be released with rs2_release_frame(...)
      frame = rs2_extract_frame(frames, i, &e);
      deviceOk = myCheck_error(e);

      if (deviceOk)
      {
        rgb_frame_data = (uint8_t *)(rs2_get_frame_data(frame, &e));
        deviceOk = myCheck_error(e);
      }
      if (deviceOk)
      {
        unsigned long long frame_number = rs2_get_frame_number(frame, &e);
        deviceOk = myCheck_error(e);
      }
      if (deviceOk)
      {
        frame_timestamp = rs2_get_frame_timestamp(frame, &e);
        deviceOk = myCheck_error(e);
      }
      // Specifies the clock in relation to which the frame timestamp was measured
      if (deviceOk)
      {
        frame_timestamp_domain = rs2_get_frame_timestamp_domain(frame, &e);
        deviceOk = myCheck_error(e);
        frame_timestamp_domain_str = rs2_timestamp_domain_to_string(frame_timestamp_domain);
      }
      if (deviceOk)
      {
        frame_metadata_time_of_arrival = rs2_get_frame_metadata(frame, RS2_FRAME_METADATA_TIME_OF_ARRIVAL, &e);
        deviceOk = myCheck_error(e);
      }
      if (deviceOk)
      {
        updateMeasure(rgb_frame_data);
      }

#ifdef SAVE_VIDEO_TO_FILE
      if (deviceOk)
      {
        /*printf("RGB frame arrived.\n");
        printf("First 10 bytes: ");
        int k;
        printf("firstBytes=[");
        for (k = 0; k < (WIDTH + 1) * 3; ++k)
          printf("'%02x';", rgb_frame_data[k]);
        printf("];\n");
        printf("\nFrame No: %llu\n", frame_number);
        printf("Timestamp: %f\n", frame_timestamp);
        printf("Timestamp domain: %s\n", frame_timestamp_domain_str);
        printf("Time of arrival: %lld\n\n", frame_metadata_time_of_arrival);
        */
        sprintf(imgFileName, "img_%05d.raw", numFrame++);
        FILE *fileToWrite = fopen(imgFileName, "wb+");
        size_t sizeImage = WIDTH * HEIGHT * 3;
        if (fileToWrite != NULL)
        {
          fwrite(rgb_frame_data, 1, sizeImage, fileToWrite);
          fclose(fileToWrite);
        }
      }
#endif
      rs2_release_frame(frame);
    }

    rs2_release_frame(frames);
  }

  // Stop the pipeline streaming
  if (deviceOk)
  {
    rs2_pipeline_stop(pipeline, &e);
    deviceOk = myCheck_error(e);
  }

  // Release resources
  if (deviceOk)
  {
    rs2_delete_pipeline_profile(pipeline_profile);
    rs2_delete_config(config);
    rs2_delete_pipeline(pipeline);
    rs2_delete_device(dev);
    rs2_delete_device_list(device_list);
    rs2_delete_context(ctx);
  }
}
/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/
/**
 * @brief The worker threadKeyboard function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */

void KobukiManager::keyboardInputLoop()
{
  struct termios raw;
  memcpy(&raw, &original_terminal_state, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor, TCSANOW, &raw);

  char c;
  while (!quit_requested)
  {
    if (read(key_file_descriptor, &c, 1) < 0)
    {
      perror("read char failed():");
      exit(-1);
    }
    processKeyboardInput(c);
  }
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void KobukiManager::processKeyboardInput(char c)
{
  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {
  
  case 'q':
  {
    quit_requested = true;
    break;
  }
  default:
  {
    handleKeyboard(control,c);
    break;
  }
  }
}

/*****************************************************************************
 ** Implementation [Commands]
 *****************************************************************************/


void KobukiManager::showAllData(kobuki::CoreSensors::Data sensorData, ControlStruct control)
{

  std::cout << ecl::TimeStamp() << ",";
  std::cout << sensorData.time_stamp << ",";
  std::cout << control.left_encoder << ",";
  std::cout << control.right_encoder << ",";
  std::cout << control.left_pwm << ",";
  std::cout << control.right_pwm << ",";
  std::cout << (int)sensorData.battery << ",";
  std::cout << (int)sensorData.over_current << "";
  bool showRefs = false;
  if (showRefs)
  {
    double wheel_left_angle, wheel_left_angle_rate, wheel_right_angle, wheel_right_angle_rate;
    kobuki.getWheelJointStates(wheel_left_angle, wheel_left_angle_rate, wheel_right_angle,
                               wheel_right_angle_rate);
    std::cout << "," << wheel_left_angle << ",";
    std::cout << wheel_right_angle << ",";
    std::cout << wheel_left_angle_rate << ",";
    std::cout << wheel_right_angle_rate << "";
  }
  bool showCurrent = true;
  if (showCurrent)
  {
    kobuki::Current::Data currentData = kobuki.getCurrentData();
    std::cout << "," << (int)(currentData.current[0]) << ",";
    std::cout << (int)(currentData.current[1]);
  }
  std::cout << "," << control.vx << ",";
  std::cout << control.wz << ",";
  std::cout << control.measure << "";
  std::cout << "" << std::endl;
}

void KobukiManager::mainLoopStep()
{
  static int count = 0;
  static uint16_t last_left_encoder;
  static uint16_t last_right_encoder;
  static int32_t left_enc_offset = 0;
  static int32_t right_enc_offset = 0;

  if (count == 0)
  {
    // std::cout << "%%init controller gain " << std::endl;
    //  1 for user-configured PID gain
    //  P gain 	4 	  	  	Kp * 1000 (default: 100*1000)
    //  I gain 	4 	  	  	Ki * 1000 (default: 0.1*1000)
    //  D gain 	4 	  	  	Kd * 1000 (default: 2*1000)
    unsigned char type = 1;
    unsigned int p_gain = 1 * 100 * 1000;
    unsigned int i_gain = 0.1 * 1000;
    unsigned int d_gain = 2*1000;
    kobuki.setControllerGain(type, p_gain, i_gain, d_gain);
    printf("p_gain =%u;i_gain =%u;d_gain=%u\n", p_gain, i_gain, d_gain);
  }
  /* YGORRA probably unused
  ecl::linear_algebra::Vector3d pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);
  ecl::concatenate_poses(pose, pose_update);
  */
  // TODO(daniel.stonier): this needs a mutex
  // This callback triggers in Kobuki's threadKeyboard, however
  // vxKeyboard, wzKeyboard are updated in the keyboard input threadKeyboard.
  kobuki::CoreSensors::Data sensorData = kobuki.getCoreSensorData();
  control.measure = measureMeter;
  if (count == 0)
  {
    last_right_encoder = sensorData.right_encoder; // between 0 and 65535
    last_left_encoder = sensorData.left_encoder;   // between 0 and 65535
    right_enc_offset=-sensorData.right_encoder;
    left_enc_offset=-sensorData.left_encoder;
    
  }
  if (sensorData.right_encoder < last_right_encoder)
  {
    if (last_right_encoder - sensorData.right_encoder > 32000)
    {
      right_enc_offset += 65536;
    }
  }
  else
  {
    if (sensorData.right_encoder - last_right_encoder > 32000)
    {
      right_enc_offset -= 65536;
    }
  }
  control.right_encoder = right_enc_offset + sensorData.right_encoder;
  if (sensorData.left_encoder < last_left_encoder)
  {
    if (last_left_encoder - sensorData.left_encoder > 32000)
    {
      left_enc_offset += 65536;
    }
  }
  else
  {
    if (sensorData.left_encoder - last_left_encoder > 32000)
    {
      left_enc_offset -= 65536;
    }
  }
  control.left_encoder = left_enc_offset + sensorData.left_encoder;
  
  last_right_encoder = sensorData.right_encoder; // between 0 and 65535
  last_left_encoder = sensorData.left_encoder;   // between 0 and 65535

  int8_t *add_left = (int8_t *)&sensorData.left_pwm;
  control.left_pwm = (int)*add_left;
  int8_t *add_right = (int8_t *)&sensorData.right_pwm;
  control.right_pwm = (int)*add_right;
  control.measure = measureMeter;
  oneStepControl(control);
  mutex.lock();
  kobuki.setBaseControl(control.vx, -control.wz);
  mutex.unlock();
  showAllData(sensorData, control);

  count++;
}

/*****************************************************************************
** Signal Handler
*****************************************************************************/

bool signal_shutdown_requested = false;
void signalHandler(int /* signum */)
{
  signal_shutdown_requested = true;
}

/*****************************************************************************
** Main
*****************************************************************************/
uint8_t myCheck_error(rs2_error *e)
{
  if (e)
  {
    printf("rs_error was raised when calling %s(%s):\n", rs2_get_failed_function(e), rs2_get_failed_args(e));
    printf("    %s\n", rs2_get_error_message(e));
    return 0;
  }
  return 1;
}
int main(int argc, char **argv)
{

  ecl::CmdLine cmd_line("simple_keyop program", ' ', "0.3");
  ecl::UnlabeledValueArg<std::string> device_port("device_port", "Path to device file of serial port to open, connected to the kobuki", false, "/dev/kobuki", "string");
  cmd_line.add(device_port);
  cmd_line.parse(argc, argv);

  signal(SIGINT, signalHandler);

  std::cout << ecl::bold << "\nSimple Keyop : Utility for driving kobuki by keyboard.\n"
            << ecl::reset << std::endl;

  KobukiManager kobuki_manager;
  kobuki_manager.init(device_port.getValue());

  ecl::Sleep sleep_one_second(1);
  // redirect cout to file data_kobuki.txt
  std::ofstream out("./data_kobuki.txt");
  // std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
  std::cout.rdbuf(out.rdbuf()); // redirect std::cout to out.txt!

  try
  {
    while (!signal_shutdown_requested && !kobuki_manager.isShutdown())
    {
      sleep_one_second();
    }
  }
  catch (ecl::StandardException &e)
  {
    std::cout << e.what();
  }
  return 0;
}
