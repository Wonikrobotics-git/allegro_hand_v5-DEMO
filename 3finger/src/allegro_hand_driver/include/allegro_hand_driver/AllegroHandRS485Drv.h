/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Wonik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Wonik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  @file rs485drv.cpp
 *  @brief to support RS-485 interface
 *
 *  Created on:         Dec 16, 2024
 *  Added to Project:   Dec 18, 2024
 *  Author:             Soohoon Yang(Hibo)
 *  Maintained by:      Soohoon Yang(sh-yang@wonik.com)
 */

#ifndef RS485_DRIVER_H
#define RS485_DRIVER_H

#include "ros/ros.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <errno.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <cstdint>
#include <std_msgs/String.h>
#include <vector>

// ==========================
// Constants and Definitions
// ==========================

// Command ID Definitions
constexpr uint8_t CMD_PLACE[]      = {0x08};       // Place command
constexpr uint8_t CMD_PICK_3F[]    = {0x04};       // Pick 3-finger command
constexpr uint8_t CMD_TORQUE_OFF[] = {0x0C};       // Torque off command
constexpr uint8_t CMD_GRAVITY[]    = {0x09};       // Gravity compensation
constexpr uint8_t CMD_POS_READ_1[] = {0x40};       // Read position of finger 1
constexpr uint8_t CMD_POS_READ_2[] = {0x41};       // Read position of finger 2
constexpr uint8_t CMD_POS_READ_3[] = {0x42};       // Read position of finger 3
constexpr uint8_t SAV_POS[]        = {0x10};
constexpr uint8_t GO_POS[]         = {0x11};
///////////

enum CommandLength {
    COMMANDREAD = 0x00,
    DEVICEINFO = 0x05,
    POSREAD = 0x06,
    WRITEPOS = 0x01,
    READPOS = 0x26
};

// ==========================
// Function Prototypes
// ==========================

/**
 * @brief Get the baud rate value
 * @param baudrate Desired baud rate
 * @return Corresponding baud rate setting
 */
int getBaudrate(int baudrate);

/**
 * @brief Handle errors with a message
 * @param msg Error message
 */
void handleError(const std::string& msg);

/**
 * @brief Set the serial port status
 * @param fd File descriptor of the serial port
 */
void setSerialPortStatus(int fd);

/**
 * @brief Send data through the serial port
 * @param fd File descriptor of the serial port
 * @param command Pointer to the command array
 * @param commandSize Size of the command array
 * @param msg Log message for the operation
 * @param isPosRead Indicates if the operation is a position read
 */
void sendData(int fd, const uint8_t* command, size_t commandSize, CommandLength length);

/**
 * @brief Read data from the serial port
 * @param fd File descriptor of the serial port
 */
void readData(int fd);

/**
 * @brief Initialize the serial port
 * @param portName Name of the serial port
 * @param baudrate Baud rate for communication
 * @return File descriptor of the initialized serial port, or -1 on error
 */
int RS485init(const char* portName, int baudrate);

bool running = true;

#endif // RS485_DRIVER_H
