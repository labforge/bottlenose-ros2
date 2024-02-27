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

#include <vector>
#include <cstdint>
#include <PvBuffer.h>

#define MAX_KEYPOINTS 0xFFFF
#define MAX_BBOXES (100)

/**
 * @brief ChunkIDs for possible buffers appended to the GEV buffer.
 */
typedef enum {
  CHUNK_ID_FEATURES = 0x4001,    ///< Keypoints
  CHUNK_ID_DESCRIPTORS = 0x4002, ///< Descriptors
  CHUNK_ID_DNNBBOXES = 0x4003,   ///< Bounding boxes for detected targets
  CHUNK_ID_EMBEDDINGS = 0x4004,  ///< Embeddings
  CHUNK_ID_INFO = 0x4005,        ///< Meta information
  CHUNK_ID_MATCHES = 0x4006,     ///< Matches
  CHUNK_ID_POINTCLOUD = 0x4007   ///< Point cloud
} chunk_type_t;

/**
 * @brief Chunk data representation of a keypoint.
 */
typedef struct point_u16 {
  uint16_t x;  ///< x coordinate  of the point
  uint16_t y;  ///< y coordinate of the point
} point_u16_t;

/**
 * @brief Chunk data representation of point matches.
 */
typedef struct hamat_matches_8xu16 {
  uint16_t x; ///< x coordinate or index of the first point
  uint16_t y; ///< y coordinate of the first point
  uint16_t x2; ///< x coordinate or index of the second point
  uint16_t y2; ///< y coordinate of the second point
  uint16_t d2; ///< next minimum distance
  uint16_t d1; ///< first minimum distance
  uint16_t n2; ///< number of reference points that calculated the second Hamming distance
  uint16_t n1; ///< number of reference points that calculated the first Hamming distance
} hamat_matches_8xu16_t;

typedef struct __attribute__((packed, aligned(4))) {
  uint32_t count;
  uint32_t layout;
  uint32_t unmatched;
  hamat_matches_8xu16_t points[MAX_KEYPOINTS];
} matches_t;

/**
 * @brief Chunk data a 3-dimensional coordinate.
 */
typedef struct vector3f {
  float x;  ///< x coordinate  of the point
  float y;  ///< y coordinate  of the point
  float z;  ///< z coordinate  of the point
} vector3f_t;

/**
 * @brief Chunk data representation of keypoint AKAZE descriptors.
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint8_t data[64];         ///< up to 486 bits of descriptor data LSB first
} descriptor_t;

/**
 * @brief Chunk data representation of keypoints.
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint32_t count;
  uint32_t fid;
  point_u16_t points[MAX_KEYPOINTS];
} keypoints_t;

/**
 * @brief Chunk data representation of keypoint descriptors.
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint32_t count;
  uint32_t length;
  uint32_t fid;
  descriptor_t descriptors[MAX_KEYPOINTS];
} descriptors_t;

/**
 * @brief Chunk data representation of a bounding box.
 */
typedef struct bbox {
  int32_t cid;    ///< box class id
  float score;    ///< detection score
  int32_t left;   ///< left-most point of the box
  int32_t top;    ///< top-most point of the box
  int32_t right;  ///< right-most point of the box
  int32_t bottom; ///< bottom-most point of the box
  char label[24]; ///< box label;
} bbox_t;

/**
 * @brief Chunk data representation of bounding boxes.
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint32_t fid;
  uint32_t count;
  bbox_t box[MAX_BBOXES];
} bboxes_t;

typedef struct __attribute__((packed, aligned(4))) {
  uint32_t count;
  vector3f_t points[MAX_KEYPOINTS];
} pointcloud_t;

/**
 * @brief Meta information chunk data to decode timestamps.
 */
typedef struct __attribute__((packed, aligned(4))) {
  uint64_t real_time; ///< Realtime in milliseconds
  uint32_t count;     ///< Frame count
  float gain;         ///< Gain value
  float exposure;     ///< Exposure value
} info_t;

/**
 * Decode meta information from buffer, if present.
 * @param buffer Buffer received on GEV interface
 * @param info Meta information
 * @return true if present, false if not present or corrupted.
 */
bool chunkDecodeMetaInformation(PvBuffer *buffer, info_t *info);

/**
 * Decode keypoints from buffer, if present.
 * @param buffer Buffer received on GEV interface
 * @param keypoints Vector of keypoints.
 * @return true if present, false if not present or corrupted.
 */
bool chunkDecodeKeypoints(PvBuffer *buffer, std::vector<keypoints_t> &keypoints);

/**
 * Decode bounding boxes from buffer, if present.
 * @param buffer Buffer received on GEV interface
 * @param bboxes Decoded bounding boxes.
 * @return true if present, false if not present or corrupted.
 */
bool chunkDecodeBoundingBoxes(PvBuffer *buffer, bboxes_t &bboxes);

/**
 * Decode point cloud from buffer, if present.
 * @param buffer Buffer received on GEV interface
 * @param pointcloud Decoded point cloud
 * @return true if present, false if not present or corrupted.
 */
bool chunkDecodePointCloud(PvBuffer *buffer, pointcloud_t &pointcloud);

/**
 * Decode matches from image stream, if present.
 * @param buffer Buffer received on GEV interface
 * @param matches Decoded matches
 * @return true if present, false if not present or corrupted.
 */
bool chunkDecodeMatches(PvBuffer *buffer, matches_t &matches);

/**
 * Count the number of valid matches in the matches structure.
 * @param matches Decoded matches
 * @return Number of valid matches
 */
size_t validMatches(const matches_t &matches);

std::string ms_to_date_string(uint64_t ms);


#endif // __BOTTLENOSE_CHUNK_PARSER_HPP__