// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cstdint>
#include <vector>
#include <chrono>
#include <thread>
#include <msp/FlightController.hpp>

#define DEVICE "/dev/ttyUSB0"
#define BAUDRATE 115200

int main(int argc, char * argv[])
{
  fcu::FlightController fcu;
  fcu.connect(DEVICE, BAUDRATE);
  using ChannelVector = std::vector<uint16_t>;

  // CHANNELS ARE IN THE ORDER: ROLL, PITCH, THROTTLE, YAW, AUX1, AUX2, AUX3, AUX4
  // ChannelVector channels = {1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000};

  std::vector<ChannelVector> channels_vectors;
  channels_vectors.push_back({1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000});
  channels_vectors.push_back({2000, 1500, 1000, 1500, 1000, 1000, 1000, 1000});
  channels_vectors.push_back({1750, 1500, 1000, 1500, 1000, 1000, 1000, 1000});
  channels_vectors.push_back({1500, 2000, 1000, 1500, 1000, 1000, 1000, 1000});
  channels_vectors.push_back({1500, 1750, 1000, 1500, 1000, 1000, 1000, 2000});
  channels_vectors.push_back({1500, 1500, 1000, 2000, 1000, 1000, 1000, 1000});
  channels_vectors.push_back({1500, 1500, 1000, 1750, 1000, 1000, 1000, 1000});
  channels_vectors.push_back({1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000});
  channels_vectors.push_back({1500, 1500, 1500, 1500, 1000, 1000, 1000, 1000});
  channels_vectors.push_back({1500, 1500, 2000, 1500, 1000, 1000, 1000, 1000});

  for (auto & channel_vector : channels_vectors) {
    for (int i = 0; i < 200; i++) {
      fcu.setRc(channel_vector);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  fcu.disconnect();

  return 0;
}
