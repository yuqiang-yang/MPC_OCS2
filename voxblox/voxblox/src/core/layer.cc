//
// Created by johannes on 19.12.19.
//
#include <voxblox/core/layer.h>
#include <voxblox/interpolator/interpolator.h>
using namespace voxblox;

template <>
void Layer<EsdfCachingVoxel>::cacheGradients(){

  int cnt = 0;
  BlockIndexList blocksIdxs;
  getAllAllocatedBlocks(&blocksIdxs);

  Interpolator<EsdfCachingVoxel> interpolator(this);
  // precompute gradients
  for (const BlockIndex& blockIdx : blocksIdxs) {
    const Block<EsdfCachingVoxel>& block = getBlockByIndex(blockIdx);
    for (int i = 0; i < block.num_voxels(); i++) {
      cnt++;

      const Point& point = block.computeCoordinatesFromLinearIndex(i);
      Eigen::Vector3f gradient = Eigen::Vector3f::Zero();

      // for(int i = 0;i < 100; i++)
      // {
      //   for(int j = 0;j < 200; j++)
      //   {
      //     int z = 0.1+i;
      //     if (i == 50 && j == 50)
      //     std::cerr << "";
      //     z *= 0.1;
      //     int y = z;
      //   }
      // }



      if (interpolator.getGradient(point, &gradient)) {

        getVoxelPtrByCoordinates(point)->gradient = gradient;
      }
    }
  }

}

template <>
void Layer<EsdfCachingVoxel>::cacheHessians(){
  BlockIndexList blocksIdxs;
  getAllAllocatedBlocks(&blocksIdxs);

  Interpolator<EsdfCachingVoxel> interpolator(this);
  // precompute hessians
  for (const BlockIndex& blockIdx : blocksIdxs) {
    const Block<EsdfCachingVoxel>& block = getBlockByIndex(blockIdx);
    for (int i = 0; i < block.num_voxels(); i++) {
      const Point& point = block.computeCoordinatesFromLinearIndex(i);
      Eigen::Matrix3f hessian;
      if (interpolator.getHessian(point, &hessian)) {
        getVoxelPtrByCoordinates(point)->hessian = hessian;
      }
    }
  }
}