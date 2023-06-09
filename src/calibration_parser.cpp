/******************************************************************************
 *  Copyright 2023 Labforge Inc.                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this project except in compliance with the License.        *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 ******************************************************************************

@file calibration_parser.cpp Implementation of Calibration Parser for Bottlenose
@author Thomas Reidemeister <thomas@labforge.ca>
*/
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

std::unordered_map<std::string, double> LoadFlatYamlParameters(const std::string &fname = "", int sensors = 0) {
  std::unordered_map<std::string, double> kdata;

  if (fname.empty() || (fname.length() >= 5 &&
                        (fname.substr(fname.length() - 5) == ".yaml" || fname.substr(fname.length() - 4) == ".yml"))) {
    return kdata;
  }

  if (sensors == 0) {
    return kdata;
  }

  try {
    YAML::Node calib = YAML::LoadFile(fname);

    int nCameras = calib.size();
    if (nCameras != sensors) {
      return kdata;
    }

    int tvec_count = 0;
    int rvec_count = 0;

    for (const auto &cam: calib) {
      std::string camKey = cam.first.as<std::string>();

      if (camKey != "cam0" && camKey != "cam1") {
        kdata.clear();
        return kdata;
      }

      std::string id = camKey.substr(camKey.length() - 1);

      kdata["fx" + id] = cam.second["fx"].as<double>();
      kdata["fy" + id] = cam.second["fy"].as<double>();
      kdata["cx" + id] = cam.second["cx"].as<double>();
      kdata["cy" + id] = cam.second["cy"].as<double>();

      kdata["k1" + id] = cam.second["k1"].as<double>();
      kdata["k2" + id] = cam.second["k2"] ? cam.second["k2"].as<double>() : 0.0;
      kdata["k3" + id] = cam.second["k3"] ? cam.second["k3"].as<double>() : 0.0;
      kdata["p1" + id] = cam.second["p1"] ? cam.second["p1"].as<double>() : 0.0;
      kdata["p2" + id] = cam.second["p2"] ? cam.second["p2"].as<double>() : 0.0;

      std::vector<double> tvec = {0.0, 0.0, 0.0};
      if (cam.second["tvec"]) {
        tvec = cam.second["tvec"].as<std::vector<double>>();
        tvec_count++;
      }

      std::vector<double> rvec = {0.0, 0.0, 0.0};
      if (cam.second["rvec"]) {
        rvec = cam.second["rvec"].as<std::vector<double>>();
        rvec_count++;
      }

      kdata["tx" + id] = tvec[0];
      kdata["ty" + id] = tvec[1];
      kdata["tz" + id] = tvec[2];
      kdata["rx" + id] = rvec[0];
      kdata["ry" + id] = rvec[1];
      kdata["rz" + id] = rvec[2];

      kdata["kWidth"] = cam.second["width"].as<double>();
      kdata["kHeight"] = cam.second["height"].as<double>();
    }

    if (tvec_count < (sensors - 1) || rvec_count < (sensors - 1)) {
      kdata.clear();
      return kdata;
    }
  } catch (const std::exception &e) {
    kdata.clear();
    return kdata;
  }

  return kdata;
}
