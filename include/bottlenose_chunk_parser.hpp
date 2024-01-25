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

@file bottlenose_chunk_parser.hpp Parse chunk data.
@author G. M. Tchamgoue <martin@labforge.ca>, Thomas Reidemeister <thomas@labforge.ca>
*/
#ifndef __BOTTLENOSE_CHUNK_PARSER_HPP__
#define __BOTTLENOSE_CHUNK_PARSER_HPP__

#include <PvBuffer.h>
#include <vector>
#include <stdint.h>

#define MAX_KEYPOINTS 0xFFFF

/**
 * @brief ChunkIDs for possible buffers appended to the GEV buffer.
 */
typedef enum {
  CHUNK_ID_FEATURES = 0x4001,    ///< Keypoints
  CHUNK_ID_DESCRIPTORS = 0x4002, ///< Descriptors
  CHUNK_ID_DNNBBOXES = 0x4003,   ///< Bounding boxes for detected targets
  CHUNK_ID_EMBEDDINGS = 0x4004,  ///< Embeddings
  CHUNK_ID_INFO = 0x4005,        ///< Meta information
} chunk_type_t;

/**
 * @brief Chunk data representation of a keypoint.
 */
typedef struct point_u16 {
  uint16_t x;  ///< x coordinate  of the point
  uint16_t y;  ///< y coordinate of the point
} point_u16_t; ///< a 2D uint16 point representation

/**
 * @brief Chunk data representation of keypoints.
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint32_t count;
  uint32_t fid;
  point_u16_t points[MAX_KEYPOINTS];
} keypoints_t;

/**
 * @brief Meta information chunk data to decode timestamps.
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint64_t real_time;
  uint32_t count;
} info_t;

/**
 * Decode meta information from buffer, if present.
 * @param buffer Buffer received on GEV interface
 * @param info Meta information
 * @return
 */
bool chunkDecodeMetaInformation(PvBuffer *buffer, info_t *info);

/**
 * Decode keypoints from buffer, if present.
 * @param buffer Buffer received on GEV interface
 * @param keypoints Vector of keypoints.
 * @return
 */
bool chunkDecodeKeypoints(PvBuffer *buffer, std::vector<keypoints_t> &keypoints);

std::string ms_to_date_string(uint64_t ms);


#endif // __BOTTLENOSE_CHUNK_PARSER_HPP__