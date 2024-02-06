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

@file bottlenose_chunk_parser.cpp Parse chunk data.
@author G. M. Tchamgoue <martin@labforge.ca>, Thomas Reidemeister <thomas@labforge.ca>
*/
#include <iomanip>
#include "bottlenose_chunk_parser.hpp"
#include "rclcpp/node.hpp"

/**
 * Check if a defined chunk ID is present.
 * @param buffer
 * @param chunkID
 * @return
 */
static bool hasChunkData(PvBuffer *buffer, uint32_t chunkID) {
  if ((buffer == nullptr) || (!buffer->HasChunks())) {
    return false;
  }

  for (size_t i = 0; i < buffer->GetChunkCount(); ++i) {
    uint32_t id;
    PvResult res = buffer->GetChunkIDByIndex(i, id);
    if ((id == chunkID) && res.IsOK()) {
      return true;
    }
  }

  return false;
}

/**
 * Decode uint with variable length from bytes with configurable order.
 * @param bytes Input bytes
 * @param len Number of bytes to decode
 * @param little Little endian, or big endian
 * @return Decoded uint
 */
static uint32_t uintFromBytes(const uint8_t *bytes, uint32_t len, bool little=true){
  uint32_t intv = 0;
  if((bytes == nullptr) || (len == 0) || (len > 4)){
    return intv;
  }
  if(little){
    for(int32_t i = len-1; i >= 0; i--){
      intv = (intv << 8) | bytes[i];
    }
  } else{
    for(uint32_t i = 0; i < len; ++i){
      intv = (intv << 8) | bytes[i];
    }
  }
  return intv;
}

/**
 * Get the pointer to the chunk data by ID.
 * @param chunkID
 * @param rawdata
 * @param dataSize
 * @return
 */
static uint8_t *getChunkDataByID(uint32_t chunkID, const uint8_t *rawdata, uint32_t dataSize){
  uint8_t *chunk_data = nullptr;
  if((rawdata == nullptr) || (dataSize == 0)) {
    return chunk_data;
  }

  int32_t pos = dataSize - 4;
  while( pos >= 0) {
    uint32_t chunk_len = uintFromBytes(&rawdata[pos], 4, false);
    if((chunk_len > 0) && ((pos - 4 - chunk_len) > 0)){
      pos -= 4;
      uint32_t chunk_id = uintFromBytes(&rawdata[pos], 4, false);
      pos -= chunk_len;
      if( chunk_id == chunkID){
        chunk_data = (uint8_t*)&rawdata[pos];
        break;
      }
    }
    pos -= 4;
  }

  return chunk_data;
}

static uint8_t *getChunkRawData(PvBuffer *buffer, chunk_type_t chunkID){
  uint8_t *rawdata = nullptr;
  if(buffer == nullptr){
    return rawdata;
  }

  PvPayloadType payload = buffer->GetPayloadType();
  if(payload == PvPayloadTypeImage){
    if(hasChunkData(buffer, chunkID)){
      rawdata = (uint8_t*)buffer->GetChunkRawDataByID(chunkID);
    }
  } else if (payload == PvPayloadTypeMultiPart){
    if(buffer->GetMultiPartContainer()->GetPartCount() == 3){
      IPvChunkData *chkbuffer = buffer->GetMultiPartContainer()->GetPart(2)->GetChunkData();
      if((chkbuffer != nullptr) && (chkbuffer->HasChunks())){
        uint8_t *dataptr = buffer->GetMultiPartContainer()->GetPart(2)->GetDataPointer();
        uint32_t dataSize = chkbuffer->GetChunkDataSize();
        rawdata = getChunkDataByID(chunkID, dataptr, dataSize);
      }
    }
  }
  return rawdata;
}

static bool parseMetaInformation(uint8_t *data, info_t *info, uint32_t *offset){
  if(data == nullptr)
    return false;
  if(info == nullptr)
    return false;
  if(offset == nullptr)
    return false;

  memcpy(info, data, sizeof(info_t));
  *offset += sizeof(info_t);

  return true;
}

bool chunkDecodeMetaInformation(PvBuffer *buffer, info_t *info) {
  uint8_t *data = getChunkRawData(buffer, CHUNK_ID_INFO);
  if(data == nullptr) {
    return false;
  }
  if(info == nullptr) {
    return false;
  }

  uint32_t offset = 0;
  return parseMetaInformation(data, info, &offset);
}

static bool parseChunkKeypointData(uint8_t *data, keypoints_t *kp, uint32_t *offset){
  if(data == nullptr) return false;
  if(kp == nullptr) return false;
  if(offset == nullptr) return false;

  uint32_t num_keypoints = uintFromBytes(data, 2, true);
  uint32_t frame_id = uintFromBytes(&data[2], 2, true);
  if((num_keypoints == 0) || (num_keypoints > MAX_KEYPOINTS)){
    return false;
  }
  if(frame_id > 3) return false;

  kp->count = num_keypoints;
  kp->fid = frame_id;
  memcpy(kp->points, &data[4], sizeof(point_u16_t) * kp->count);
  *offset = (frame_id > 1)?((num_keypoints + 1) * 4):0;

  return true;
}

bool chunkDecodeKeypoints(PvBuffer *buffer, std::vector<keypoints_t> &keypoints) {
  uint8_t *data = getChunkRawData(buffer, CHUNK_ID_FEATURES);
  if(data == nullptr)
    return false;

  keypoints.clear();

  keypoints_t kp;
  uint32_t offset = 0;
  bool parsed = parseChunkKeypointData(data, &kp, &offset);
  if(parsed){
    keypoints.push_back(kp);
    if(offset > 0) {
      parsed = parseChunkKeypointData(&data[offset], &kp, &offset);
      if(parsed)
        keypoints.push_back(kp);
    }
  }
  return parsed;
}

bool parseChunkBoundingBoxData(uint8_t *data, bboxes_t *boxes){
  if(data == nullptr) return false;
  if(boxes == nullptr) return false;

  uint32_t frame_id = uintFromBytes(data, 4, true);
  uint32_t num_boxes = uintFromBytes(&data[4], 4, true);

  if((num_boxes == 0) || (num_boxes > MAX_BBOXES)){
    return false;
  }
  if(frame_id > 3) return false;

  boxes->count = num_boxes;
  boxes->fid = frame_id;
  memcpy(boxes->box, &data[8], sizeof(bbox_t) * boxes->count);

  return true;
}

bool chunkDecodeBoundingBoxes(PvBuffer *buffer, bboxes_t &bboxes) {
  uint8_t *data = getChunkRawData(buffer, CHUNK_ID_DNNBBOXES);

  if(data == nullptr) return false;

  bzero(&bboxes, sizeof(bboxes_t));
  bool parsed = parseChunkBoundingBoxData(data, &bboxes);

  return parsed;
}

bool chunkDecodePointCloud(PvBuffer *buffer, pointcloud_t &pointcloud) {
  uint8_t *data = getChunkRawData(buffer, CHUNK_ID_POINTCLOUD);
  if(data == nullptr) return false;

  bzero(&pointcloud, sizeof(pointcloud_t));

  pointcloud.count = uintFromBytes(data, 4, true);

  uint32_t offset = 1 * sizeof(uint32_t);
  memcpy(pointcloud.points, &data[offset], sizeof(vector3f_t) * pointcloud.count);

  return true;
}

size_t validMatches(const matches_t &matches) {
  size_t count = 0;
  for (size_t i = 0; i < matches.count; ++i) {
    if (matches.points[i].x != matches.unmatched) {
      count++;
    }
  }
  return count;
}

bool chunkDecodeMatches(PvBuffer *buffer, matches_t &matches) {
  uint8_t *data = getChunkRawData(buffer, CHUNK_ID_MATCHES);
  if(data == nullptr) return false;

  bzero(&matches, sizeof(matches_t));

  matches.count = uintFromBytes(data, 4, true);
  matches.layout = uintFromBytes(&data[4], 4, true);
  matches.unmatched = uintFromBytes(&data[8], 4, true);

  uint32_t offset = 3 * sizeof(uint32_t);
  if(matches.layout < 2){
    point_u16_t *points = (point_u16_t*)&data[offset];
    for(uint32_t i = 0; i < matches.count; ++i){
      matches.points[i].x = points[i].x;
      matches.points[i].y = points[i].y;
    }
  } else {
    memcpy(matches.points, &data[offset], sizeof(hamat_matches_8xu16_t) * matches.count);
  }

  return true;
}

std::string ms_to_date_string(uint64_t ms) {
  // Convert milliseconds to seconds
  std::chrono::seconds seconds(ms / 1000);

  // Convert to time_t
  std::time_t time = seconds.count();

  // Convert to tm struct
  std::tm *tm_time = std::localtime(&time); // Use std::gmtime for GMT

  // Format the time into a string
  std::stringstream ss;
  ss << std::put_time(tm_time, "%Y-%m-%d %H:%M:%S");

  return ss.str();
}

