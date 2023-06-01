//
// Created by treideme on 01/06/23.
//
#include <PvConfigurationReader.h>
#include <gtest/gtest.h>

namespace {
  TEST(Test, A) {
    PvConfigurationReader lReader;
    PvResult res = lReader.Load("config/parameters.pvcfg");;
    ASSERT_TRUE(res.IsOK()) << "Parameter Loading failed";

    for(size_t i = 0; i < lReader.GetGenParameterArrayCount(); i++) {
      PvString name;
      res = lReader.GetGenParameterArrayName(i, name);
      ASSERT_TRUE(res.IsOK());
      std::cerr << "GetGenParameterArrayName: " << name.GetAscii() << std::endl;
    }
    for(size_t i = 0; i < lReader.GetStringCount(); i++) {
      PvString name;
      res = lReader.GetStringName(i, name);
      ASSERT_TRUE(res.IsOK());
      std::cerr << "GetStringName: " << name.GetAscii() << std::endl;
    }
  }
}