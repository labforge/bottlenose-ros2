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

