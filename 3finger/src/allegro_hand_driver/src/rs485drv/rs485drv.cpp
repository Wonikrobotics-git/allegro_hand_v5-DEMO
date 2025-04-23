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

#include <allegro_hand_driver/AllegroHandRS485Drv.h>

void handleError(const std::string& msg) {
    std::cerr << msg << ": " << strerror(errno) << std::endl;
    std::exit(EXIT_FAILURE);
}

void setSerialPortStatus(int fd) {
    int status;
    if (ioctl(fd, TIOCMGET, &status) == -1) {
        handleError("Error getting modem control lines");
    }
    status |= TIOCM_DTR;
    if (ioctl(fd, TIOCMSET, &status) == -1) {
        handleError("Error setting modem control lines");
    }
}

void sendData(int fd, const uint8_t* command, size_t commandSize, CommandLength length) {
    uint8_t header[] = {0xFF, 0xFD, static_cast<uint8_t>(length), 0x01};
    size_t totalSize = sizeof(header) + commandSize;
    uint8_t* fullCommand = new uint8_t[totalSize];

    // Copy header & command
    memcpy(fullCommand, header, sizeof(header));
    memcpy(fullCommand + sizeof(header), command, commandSize);

    ////Print the data being sent (in hex format with 0x prefix)
    // std::cout << "Sending Data: ";
    // for (size_t i = 0; i < totalSize; ++i) {
    //     std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') 
    //               << static_cast<int>(fullCommand[i]) << " ";
    // }
    // std::cout << std::dec << std::endl; // reset to decimal

    // Write data
    ssize_t bytesWritten = write(fd, fullCommand, totalSize);
    if (bytesWritten != static_cast<ssize_t>(totalSize)) {
        handleError("Error writing data to serial port");
    }

    delete[] fullCommand;
}

void readData(int fd) {
    uint8_t rxBuffer[256];
    ssize_t bytesRead;
    while (running) {
        memset(rxBuffer, 0, sizeof(rxBuffer));
        bytesRead = read(fd, rxBuffer, sizeof(rxBuffer) - 1);
         if (bytesRead > 0) {
            //Check raw read data
            // std::cout << "Received data (" << bytesRead << " bytes): ";
            // for (ssize_t i = 0; i < bytesRead; ++i) {
            //     printf("0x%02X ", rxBuffer[i]);
            // }
            // std::cout << std::endl;

            uint8_t length = rxBuffer[2];
            uint8_t id = rxBuffer[4];
        
          switch(length){
            case DEVICEINFO:{
              int16_t data1 = static_cast<int16_t>((rxBuffer[6] >> 4));
              int16_t data2 = static_cast<int16_t>((rxBuffer[8] >> 4));
              int16_t data3 = static_cast<int16_t>(rxBuffer[9]);
              std::cout << std ::endl;
              std::cout << ">>RS-485(): AllegroHand hardware version: 0x0"<< data1 << std::endl;
              std::cout << "                        firmware version: 0x0"<< data2 << std::endl;
              std::cout << "                              handedness: "<< "3-finger model" << std ::endl;
              break;
            }

            case POSREAD: {
              int16_t data1 = static_cast<int16_t>((rxBuffer[6] << 8) | rxBuffer[5]);
              int16_t data2 = static_cast<int16_t>((rxBuffer[8] << 8) | rxBuffer[7]);
              int16_t data3 = static_cast<int16_t>((rxBuffer[10] << 8) | rxBuffer[9]);
              switch(id)
              {
                  case CMD_POS_READ_1[0] :
                    std::cout << "Index-finger Position " << std::endl;
                    std::cout << "Position x: " << data1 <<" "
                              << "Position y: " << data2 <<" "
                              << "Position z: " << data3 <<  std::endl;                  
                  break;
                  case CMD_POS_READ_2[0] :
                    std::cout << "Middle-finger Position " << std::endl;
                    std::cout << "Position x: " << data1 <<" "
                              << "Position y: " << data2 <<" "
                              << "Position z: " << data3 <<  std::endl;
                  break;
                  case CMD_POS_READ_3[0] :
                    std::cout << "Pinky-finger Position " << std::endl;
                    std::cout << "Position x: " << data1 <<" "
                              << "Position y: " << data2 <<" "
                              << "Position z: " << data3 <<  std::endl;                
                  break;
              }
              break;
            }   

            case WRITEPOS:{
            int16_t num = static_cast<int16_t>(rxBuffer[5]);
            switch(id)
            {
              case SAV_POS[0] :
                std::cout << "Custom Pose"<<num<<" saved"<<std::endl;
              break;
              case GO_POS[0] :
                std::cout << "Custom Pose"<<num<<" completely moved "<<std::endl;
              break;
            }
            break;
            }

			  }


      } else if (bytesRead < 0) {
            handleError("Error reading data");
      }
    }
}

int getBaudrate(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}

int RS485init(const char* portName, int baudrate) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        handleError("Error opening serial port");
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        handleError("Error getting serial port attributes");
    }

    tty.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        handleError("Error setting serial port attributes");
    }
    std::cout << "Port configured successfully." << std::endl;

    setSerialPortStatus(fd);

    uint8_t open485[]= {0x00, 0x2B, 0xD2};
    sendData(fd, open485, sizeof(open485),DEVICEINFO);

    std::thread readThread(readData, fd);
    readThread.detach();
    std::cout << "Data reading start." << std::endl;
    return fd;
}