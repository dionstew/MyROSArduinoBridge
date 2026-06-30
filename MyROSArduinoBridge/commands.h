/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ     'a'
#define GET_BAUDRATE    'b'
#define PIN_MODE        'c'
#define DIGITAL_READ    'd'
#define READ_ENCODERS   'e'
#define MOTOR_SPEEDS    'm'
#define MOTOR_RAW_PWM   'o'
#define PING            'p'
#define RESET_ENCODERS  'r'
#define SERVO_WRITE     's'
#define SERVO_READ      't'
#define UPDATE_PID      'u'
#define DIGITAL_WRITE   'w'
#define ANALOG_WRITE    'x'
#define LEFT              0
#define RIGHT             1
#define CEK_DATA        'z'
#define ALL_BRAKE_OFF   'q'
#define TIMING_TRANSACTION 'f'   // f <seq_id> <left_cmd> <right_cmd> -> fb ...
//#define INTERVAL        'i'
#endif
