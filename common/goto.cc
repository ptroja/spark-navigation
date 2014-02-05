/*
Copyright (c) 2005, Brad Kratochvil, Toby Collett, Brian Gerkey, Andrew Howard, ...
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the Player Project nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * goto.cc - a simple (and bad) goto program
 *
 * but, it demonstrates one multi-threaded structure
 *
 * @todo: this has been ported to libplayerc++, but not tested AT ALL
 */

#include <libplayerc++/playerc++.h>

#include <iostream>
#include <cmath>
#include <cstdlib> // for atof()
#if !defined (WIN32)
  #include <unistd.h>
#endif

#include <string>

#include <boost/signal.hpp>
#include <boost/bind.hpp>

#define USAGE \
  "USAGE: goto [-x <x>] [-y <y>] [-h <host>] [-p <port>] [-m]\n" \
  "       -x <x>: set the X coordinate of the target to <x>\n"\
  "       -y <y>: set the Y coordinate of the target to <y>\n"\
  "       -h <host> : connect to Player on this host\n" \
  "       -p <port> : connect to Player on this TCP port\n" \
  "       -i <index>: connect to position2d interface with this index\n" \
  "       -d        : enable debug\n" \
  "       -m        : turn on motors (be CAREFUL!)"

PlayerCc::PlayerClient* robot;
PlayerCc::Position2dProxy* pp;

bool         gMotorEnable(false);
bool         gGotoDone(false);
std::string  gHostname(PlayerCc::PLAYER_HOSTNAME);
uint32_t        gPort(PlayerCc::PLAYER_PORTNUM);
uint32_t        gIndex(0);
uint32_t        gDebug(0);
uint32_t        gFrequency(10); // Hz

player_pose2d_t gTarget = {0, 0, 0};

void
print_usage(int argc, char** argv)
{
  std::cout << USAGE << std::endl;
}

int
parse_args(int argc, char** argv)
{
  const char* optflags = "h:p:i:d:u:x:y:m";
  int ch;

  while(-1 != (ch = getopt(argc, argv, optflags)))
  {
    switch(ch)
    {
      /* case values must match long_options */
      case 'h':
          gHostname = optarg;
          break;
      case 'p':
          gPort = atoi(optarg);
          break;
      case 'i':
          gIndex = atoi(optarg);
          break;
      case 'd':
          gDebug = atoi(optarg);
          break;
      case 'u':
          gFrequency = atoi(optarg);
          break;
      case 'x':
          gTarget.px = atof(optarg);
          break;
      case 'y':
          gTarget.py = atof(optarg);
          break;
      case 'm':
          gMotorEnable = true;
          break;
      case '?':
      case ':':
      default:
        print_usage(argc, argv);
        return (-1);
    }
  }

  return (0);
}

template<typename T>
void
Print(T t)
{
  std::cout << *t << std::endl;
}

int
main(int argc, char **argv)
{
  try
  {
    using namespace PlayerCc;

    parse_args(argc,argv);

    // Connect to Player server
    robot = new PlayerClient(gHostname, gPort);

    // Request sensor data
    pp = new Position2dProxy(robot, gIndex);

    if(gMotorEnable)
      pp->SetMotorEnable(true);

    std::cout << "goto starting, target: " << gTarget.px
              << ", " << gTarget.py << std::endl;

    robot->Read();

    pp->GoTo(gTarget);

    robot->Read();

    for (;;)
    {
      //timespec sleep = {0, 100000000}; // 100 ms
      //nanosleep(&sleep, NULL);
      robot->Read();
      const double dist = hypot(pp->GetXPos()-gTarget.px, pp->GetYPos()-gTarget.py);
      printf("speed(x,y,yaw): %+.2f %+.2f %+.2f\tpose(x,y,yaw): %+.2f %+.2f %+.2f\tdist: %.2f\n",
	pp->GetXSpeed(), pp->GetYSpeed(), pp->GetYawSpeed(),
	pp->GetXPos(), pp->GetYPos(), pp->GetYaw(),
        dist
      );

      if (dist < 0.5 &&
          std::abs(pp->GetXSpeed()) == 0.0 &&
          std::abs(pp->GetYSpeed()) == 0.0 &&
          std::abs(pp->GetYawSpeed()) == 0.0)
         break;
    }
  }
  catch (PlayerCc::PlayerError & e)
  {
    std::cerr << e << std::endl;
    return -1;
  }

  return(0);
}

