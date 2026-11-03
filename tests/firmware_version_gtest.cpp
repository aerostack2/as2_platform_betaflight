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

/**
 * @file firmware_version_gtest.cpp
 *
 * Tests for the firmware version helpers
 */

#include <gtest/gtest.h>

#include "as2_platform_betaflight/betaflight_platform.hpp"

namespace as2_platform_betaflight
{

TEST(RawImuAcc1G, ScaledUpToBetaflight44) {
  EXPECT_DOUBLE_EQ(rawImuAcc1G(1045), 512.0);
}

TEST(RawImuAcc1G, RawFromBetaflight45) {
  EXPECT_DOUBLE_EQ(rawImuAcc1G(1046), 2048.0);
  EXPECT_DOUBLE_EQ(rawImuAcc1G(1048), 2048.0);
}

TEST(FirmwareVersionString, SemanticVersion) {
  EXPECT_EQ(firmwareVersionString(4, 5, 2), "4.5.2");
}

TEST(FirmwareVersionString, CalendarVersion) {
  EXPECT_EQ(firmwareVersionString(26, 6, 2), "2026.6.2");
}

}  // namespace as2_platform_betaflight
