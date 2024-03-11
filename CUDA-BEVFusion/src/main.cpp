/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <cuda_runtime.h>
#include <string.h>

#include <vector>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "bevfusion/bevfusion.hpp"
#include "common/check.hpp"
#include "common/tensor.hpp"
#include "common/timer.hpp"
#include "common/visualize.hpp"

static std::vector<unsigned char*> load_images(
  const std::vector<std::string>& root, 
  const std::vector<std::string>& modality_prefix,
  int seqNum, 
  int frameIdx) {

  std::vector<unsigned char*> images;

  // For 5 cameras on vehicle push_back full path of image
  for (int i = 0; i < 5; i++) {
    char path[200];

    // example = 2d_raw/cam0/1/2d_raw_cam0_1_4.jpg
    sprintf(path, "%s/%s%d/%d/%s%d_%d_%d.jpg", root[i].c_str(), "cam", i, seqNum, modality_prefix[i].c_str(), i, seqNum, frameIdx);
    std::cout << path << std::endl;
    int width, height, channels;
    images.push_back(stbi_load(path, &width, &height, &channels, 0));
    // printf("Image info[%d]: %d x %d : %d\n", i, width, height, channels);
  }
  char path[200];
  // Duplicate last frame for compatibility
  sprintf(path, "%s/%s%d/%d/%s%d_%d_%d.jpg", root[4].c_str(), "cam", 4, seqNum, modality_prefix[4].c_str(), 4, seqNum, frameIdx);
  std::cout << path << std::endl;
  int width, height, channels;
  images.push_back(stbi_load(path, &width, &height, &channels, 0));
  return images;
}

static void free_images(std::vector<unsigned char*>& images) {
  for (size_t i = 0; i < images.size(); ++i) stbi_image_free(images[i]);

  images.clear();
}

static void visualize(const std::vector<bevfusion::head::transbbox::BoundingBox>& bboxes, const nv::Tensor& lidar_points,
                      const std::vector<unsigned char*> images, const nv::Tensor& lidar2image, const std::string& save_path,
                      cudaStream_t stream) {
  printf("Entered visualize");
  std::vector<nv::Prediction> predictions(bboxes.size());
  memcpy(predictions.data(), bboxes.data(), bboxes.size() * sizeof(nv::Prediction));
  printf("Copied predictions");
  int padding = 300;
  int lidar_size = 1024;
  int content_width = lidar_size + padding * 3;
  int content_height = 1080;
  nv::SceneArtistParameter scene_artist_param;
  scene_artist_param.width = content_width;
  scene_artist_param.height = content_height;
  scene_artist_param.stride = scene_artist_param.width * 3;
  
  nv::Tensor scene_device_image(std::vector<int>{scene_artist_param.height, scene_artist_param.width, 3}, nv::DataType::UInt8);
  scene_device_image.memset(0x00, stream);
  printf("Created scene device");
  scene_artist_param.image_device = scene_device_image.ptr<unsigned char>();
  auto scene = nv::create_scene_artist(scene_artist_param);
  printf("Created scene artist");
  nv::BEVArtistParameter bev_artist_param;
  bev_artist_param.image_width = content_width;
  bev_artist_param.image_height = content_height;
  bev_artist_param.rotate_x = 70.0f;
  bev_artist_param.norm_size = lidar_size * 0.5f;
  bev_artist_param.cx = content_width * 0.5f;
  bev_artist_param.cy = content_height * 0.5f;
  bev_artist_param.image_stride = scene_artist_param.stride;

  auto points = lidar_points.to_device();
  auto bev_visualizer = nv::create_bev_artist(bev_artist_param);
  printf("Started draw points");
  bev_visualizer->draw_lidar_points(points.ptr<nvtype::half>(), points.size(0));
  printf("Started draw prediction");
  bev_visualizer->draw_prediction(predictions, false);
  printf("Started draw ego");
  bev_visualizer->draw_ego();
  printf("Started apply");
  bev_visualizer->apply(scene_device_image.ptr<unsigned char>(), stream);

  printf("Started memcpy");
  nv::ImageArtistParameter image_artist_param;
  image_artist_param.num_camera = images.size();
  image_artist_param.image_width = 960;
  image_artist_param.image_height = 600;
  image_artist_param.image_stride = image_artist_param.image_width * 3;
  image_artist_param.viewport_nx4x4.resize(images.size() * 4 * 4);
  memcpy(image_artist_param.viewport_nx4x4.data(), lidar2image.ptr<float>(),
         sizeof(float) * image_artist_param.viewport_nx4x4.size());

  int gap = 0;
  int camera_width = 500;
  int camera_height = static_cast<float>(camera_width / (float)image_artist_param.image_width * image_artist_param.image_height);
  int offset_cameras[][3] = {
      {-camera_width / 2, -content_height / 2 + gap, 0},
      {content_width / 2 - camera_width - gap, -content_height / 2 + camera_height / 2, 0},
      {-content_width / 2 + gap, -content_height / 2 + camera_height / 2, 0},
      {-camera_width / 2, +content_height / 2 - camera_height - gap, 1},
      {-content_width / 2 + gap, +content_height / 2 - camera_height - camera_height / 2, 0},
      {content_width / 2 - camera_width - gap, +content_height / 2 - camera_height - camera_height / 2, 1}};

  std::cout << "Started for loop" << std::endl;
  auto visualizer = nv::create_image_artist(image_artist_param);
  for (size_t icamera = 0; icamera < images.size(); ++icamera) {
    int ox = offset_cameras[icamera][0] + content_width / 2;
    int oy = offset_cameras[icamera][1] + content_height / 2;
    bool xflip = static_cast<bool>(offset_cameras[icamera][2]);
    printf("Started draw prediction");
    visualizer->draw_prediction(icamera, predictions, xflip);
    nv::Tensor device_image(std::vector<int>{600, 960, 3}, nv::DataType::UInt8);
    device_image.copy_from_host(images[icamera], stream);
    printf("Finished draw prediction");
    if (xflip) {
      auto clone = device_image.clone(stream);
      scene->flipx(clone.ptr<unsigned char>(), clone.size(1), clone.size(1) * 3, clone.size(0), device_image.ptr<unsigned char>(),
                   device_image.size(1) * 3, stream);
      checkRuntime(cudaStreamSynchronize(stream));
    }
    visualizer->apply(device_image.ptr<unsigned char>(), stream);

    scene->resize_to(device_image.ptr<unsigned char>(), ox, oy, ox + camera_width, oy + camera_height, device_image.size(1),
                     device_image.size(1) * 3, device_image.size(0), 0.8f, stream);
    checkRuntime(cudaStreamSynchronize(stream));
  }

  printf("Save to %s\n", save_path.c_str());
  stbi_write_jpg(save_path.c_str(), scene_device_image.size(1), scene_device_image.size(0), 3,
                 scene_device_image.to_host(stream).ptr(), 100);
}

std::shared_ptr<bevfusion::Core> create_core(const std::string& model, const std::string& precision) {

  printf("Create by %s, %s\n", model.c_str(), precision.c_str());
  bevfusion::camera::NormalizationParameter normalization;
  normalization.image_width = 960;
  normalization.image_height = 600;
  normalization.output_width = 704;
  normalization.output_height = 256;
  normalization.num_camera = 6;
  // normalization.resize_lim = 1.0;
  normalization.resize_lim = 0.48f;
  normalization.interpolation = bevfusion::camera::Interpolation::Bilinear;

  float mean[3] = {0.485, 0.456, 0.406};
  float std[3] = {0.229, 0.224, 0.225};
  normalization.method = bevfusion::camera::NormMethod::mean_std(mean, std, 1 / 255.0f, 0.0f);

  bevfusion::lidar::VoxelizationParameter voxelization;
  voxelization.min_range = nvtype::Float3(-54.0f, -54.0f, -5.0);
  voxelization.max_range = nvtype::Float3(+54.0f, +54.0f, +3.0);
  voxelization.voxel_size = nvtype::Float3(0.075f, 0.075f, 0.2f);
  voxelization.grid_size =
      voxelization.compute_grid_size(voxelization.max_range, voxelization.min_range, voxelization.voxel_size);
  voxelization.max_points_per_voxel = 10;
  voxelization.max_points = 300000;
  voxelization.max_voxels = 160000;
  voxelization.num_feature = 5;

  bevfusion::lidar::SCNParameter scn;
  scn.voxelization = voxelization;
  scn.model = nv::format("model/%s/lidar.backbone.xyz.onnx", model.c_str());
  scn.order = bevfusion::lidar::CoordinateOrder::XYZ;

  if (precision == "int8") {
    scn.precision = bevfusion::lidar::Precision::Int8;
  } else {
    scn.precision = bevfusion::lidar::Precision::Float16;
  }

  bevfusion::camera::GeometryParameter geometry;
  geometry.xbound = nvtype::Float3(-54.0f, 54.0f, 0.3f);
  geometry.ybound = nvtype::Float3(-54.0f, 54.0f, 0.3f);
  geometry.zbound = nvtype::Float3(-10.0f, 10.0f, 20.0f);
  geometry.dbound = nvtype::Float3(1.0, 60.0f, 0.5f);
  geometry.image_width = 704;
  geometry.image_height = 256;
  geometry.feat_width = 88;
  geometry.feat_height = 32;
  geometry.num_camera = 6;
  geometry.geometry_dim = nvtype::Int3(360, 360, 80);

  bevfusion::head::transbbox::TransBBoxParameter transbbox;
  transbbox.out_size_factor = 8;
  transbbox.pc_range = {-54.0f, -54.0f};
  transbbox.post_center_range_start = {-61.2, -61.2, -10.0};
  transbbox.post_center_range_end = {61.2, 61.2, 10.0};
  transbbox.voxel_size = {0.075, 0.075};
  transbbox.model = nv::format("model/%s/build/head.bbox.plan", model.c_str());
  transbbox.confidence_threshold = 0.12f;
  transbbox.sorted_bboxes = true;

  bevfusion::CoreParameter param;
  param.camera_model = nv::format("model/%s/build/camera.backbone.plan", model.c_str());
  param.normalize = normalization;
  param.lidar_scn = scn;
  param.geometry = geometry;
  param.transfusion = nv::format("model/%s/build/fuser.plan", model.c_str());
  param.transbbox = transbbox;
  param.camera_vtransform = nv::format("model/%s/build/camera.vtransform.plan", model.c_str());
  return bevfusion::create_core(param);
}

int main(int argc, char** argv) {

  const char* data      = "example-data/leva_tensors";
  const char* model     = "resnet50int8";
  const char* precision = "int8";

  if (argc > 1) data      = argv[1];
  if (argc > 2) model     = argv[2];
  if (argc > 3) precision = argv[3];

  auto core = create_core(model, precision);
  if (core == nullptr) {
    printf("Core has been failed.\n");
    return -1;
  }

  cudaStream_t stream;
  cudaStreamCreate(&stream);
 
  core->print();
  core->set_timer(true);

  // TODO: Convert camera lidar calibrations from .yaml to .tensor (In separate python program) [Arnav]
  // 1. Understand contents of camera2lidar, camera_intrinsics, lidar2image, img_aug_matrix tensors
  // 2. Convert .yaml to .tensor for the following
  // 3. Change load path to where our .tensor files are stored [may involve changing .cpp files]

  // Load matrix to host
  auto camera2lidar = nv::Tensor::load(nv::format("%s/camera2lidar.tensor", data), false);
  auto camera_intrinsics = nv::Tensor::load(nv::format("%s/camera_intrinsics.tensor", data), false);
  auto lidar2image = nv::Tensor::load(nv::format("%s/lidar2image.tensor", data), false);
  auto img_aug_matrix = nv::Tensor::load(nv::format("%s/img_aug_matrix.tensor", data), false);
  camera2lidar.print("camera2lidar", 0, 4, 8);
  camera_intrinsics.print("camera_intrinsics", 0, 4, 8);
  lidar2image.print("lidar2image", 0, 4, 8);
  img_aug_matrix.print("img_aug_matrix", 0, 4, 8);
  core->update(camera2lidar.ptr<float>(), camera_intrinsics.ptr<float>(), lidar2image.ptr<float>(), img_aug_matrix.ptr<float>(),
              stream);
  // core->free_excess_memory();


  // std::string img_root_dir = "/media/warthog/Art_SSD/ecocar_processed/CACCDataset/2d_raw";
  std::vector<std::string> img_root_dir = {
    "/media/warthog/Art_SSD/ecocar_processed/CACCDataset/2d_raw",
    "/media/warthog/Art_SSD/ecocar_processed/CACCDataset/2d_raw",
    "/media/warthog/Art_SSD/ecocar_processed/CACCDataset/2d_undistort",
    "/media/warthog/Art_SSD/ecocar_processed/CACCDataset/2d_undistort",
    "/media/warthog/Art_SSD/ecocar_processed/CACCDataset/2d_undistort",
  };
  std::vector<std::string> modality_prefix = {
    "2d_raw_cam",
    "2d_raw_cam",
    "2d_undistort_cam",
    "2d_undistort_cam",
    "2d_undistort_cam",
  };
  std::string pc_root_dir = std::string(data) + "/points";

  std::vector<int> seqList = {0};
  std::vector<int> num_of_frames_per_seq = {10};
  int num_of_frames_per_seq_iter = 0;

  for(int seqNum : seqList) {
      // Get the frame number for the sequences
      for (int frame_idx=0; frame_idx < num_of_frames_per_seq[num_of_frames_per_seq_iter]; ++frame_idx) {
        // TODO: Format strings for current frames' images and point cloud [Arsh]
        
        // TODO: Modify load_images(cam_dir, camid, seq, frame) to load images from our dataset [Arsh]
        // Load image and lidar to host
        
        auto images = load_images(img_root_dir, modality_prefix, seqNum, frame_idx);
        printf("Loaded images for frame %i\n", frame_idx);
        // TODO: Add load lidar_points(lidar_dir, seq, frame) to load point cloud from our dataset [Arnav]
        std::string pc_path = nv::format("%s/%d/%d.tensor", pc_root_dir.c_str(), seqNum, frame_idx);
        std::cout << "Loading point cloud for frame " << pc_path << std::endl;
        auto lidar_points = nv::Tensor::load(pc_path, false);

        lidar_points.print("lidar_points", 0, 5, 8);
        printf("Loaded point cloud for frame %i\n", frame_idx);
        // warmup
        auto bboxes =
            core->forward((const unsigned char**)images.data(), lidar_points.ptr<nvtype::half>(), lidar_points.size(0), stream);
        printf("Ran forward pass for frame %i\n", frame_idx); 
        // evaluate inference time
        // for (int i = 0; i < 5; ++i) {
        core->forward((const unsigned char**)images.data(), lidar_points.ptr<nvtype::half>(), lidar_points.size(0), stream);
        // }
        printf("Ran double forward pass for frame %i\n", frame_idx);
        // visualize and save to jpg
        // TODO: Modify visualize to save frames individually for our dataset [Arnav]
        visualize(bboxes, lidar_points, images, lidar2image, "build/cuda-bevfusion.jpg", stream);
        printf("Visualized frame %i\n", frame_idx);

        // destroy memory
        free_images(images);
        // checkRuntime(cudaStreamDestroy(stream));
        printf("Destroyed frame %i\n", frame_idx);
        break;
      }
      num_of_frames_per_seq_iter++;
  }
  checkRuntime(cudaStreamDestroy(stream));
  
  return 0;
}