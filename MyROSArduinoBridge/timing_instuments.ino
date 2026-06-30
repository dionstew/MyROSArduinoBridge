#ifdef USE_BASE
void readEncoderPairForFeedback(long &leftTicks, long &rightTicks) {
  #if defined(MEGA_I2C_NANO_ENC_COUNTER)
    readEncoder_I2C(ADDR_NANO_LEFT,  encoderLeft);
    readEncoder_I2C(ADDR_NANO_RIGHT, encoderRight);
    leftTicks  = encoderLeft.ticks;
    rightTicks = encoderRight.ticks;
  #elif defined(ENCODER_DISK_20PPR)
    readEncoder_20PPR(LEFT, encoderLeft);
    readEncoder_20PPR(RIGHT, encoderRight);
    leftTicks  = encoderLeft.ticks;
    rightTicks = encoderRight.ticks;
  #else
    leftTicks  = readEncoder(LEFT);
    rightTicks = readEncoder(RIGHT);
  #endif

  g_last_encoder_read_us = micros();
}

void applyDiffDriveCommand(long leftCmd, long rightCmd) {
  lastMotorCommand = millis();
  g_last_command_us = micros();
  g_in_timeout_safe_stop = false;
  g_status_code = STATUS_NORMAL;

  if (leftCmd == 0 && rightCmd == 0) {
    setMotorSpeeds(0, 0);
    resetPID();
    moving = 0;
    setAllBrakes(true, true);
  }
  else {
    moving = 1;
    setAllBrakes(false, false);
  }

  // This is the time when the command target is applied to the firmware.
  // In PID mode, the physical PWM update still occurs in updatePID().
  leftPID.TargetTicksPerFrame   = leftCmd;
  rightPID.TargetTicksPerFrame  = rightCmd;
  g_last_target_update_us = micros();
}

void sendTimingFeedback(unsigned long seqId, long leftCmd, long rightCmd) {
  long leftTicks = 0;
  long rightTicks = 0;
  readEncoderPairForFeedback(leftTicks, rightTicks);

  g_feedback_tx_count++;
  g_last_feedback_tx_us = micros();

  // Format expected by the ROS-side transaction parser:
  // fb <seq_id> <cmd_rx_us> <motor_update_us> <encoder_read_us> <feedback_tx_us> <left_ticks> <right_ticks> <status_code>
  // Here motor_update_us is the target-update timestamp. For PID-based mode, actual PWM update is tracked in g_last_pid_update_us.
  Serial.print("fb ");
  Serial.print(seqId);
  Serial.print(" ");
  Serial.print(g_cmd_rx_us);
  Serial.print(" ");
  Serial.print(g_last_target_update_us);
  Serial.print(" ");
  Serial.print(g_last_encoder_read_us);
  Serial.print(" ");
  Serial.print(g_last_feedback_tx_us);
  Serial.print(" ");
  Serial.print(leftTicks);
  Serial.print(" ");
  Serial.print(rightTicks);
  Serial.print(" ");
  Serial.println(g_status_code);
}
#endif