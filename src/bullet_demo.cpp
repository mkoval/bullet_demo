#include <array>
#include <iostream>
#include <vector>
#include <string>
#include <CL/cl.hpp>
#include <Bullet3OpenCL/Initialize/b3OpenCLUtils.h>
#include <Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h>
#include <Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h>
#include <Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h>
#include <Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvhBroadphase.h>
#include <Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h>

int main(int argc, char** argv)
{
#if 0
  // Select an OpenCL platform. This is necessary because the NVidia OpenCL
  // does not support choosing a default platform by passing NULL.
  std::vector<cl::Platform> all_platforms;
  cl::Platform::get(&all_platforms);
  if (all_platforms.empty())
  {
    std::cerr << "No OpenCL platforms found." << std::endl;
    return 1;
  }

  cl::Platform platform = all_platforms.front();
  std::cout << "Using first OpenCL platform of " << all_platforms.size()
            << " options: " << platform.getInfo<CL_PLATFORM_NAME>()
            << std::endl;

  // Select an OpenCL device.
  std::vector<cl::Device> all_devices;
  platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
  if (all_devices.empty())
  {
    std::cerr << "No OpenCL devices found." << std::endl;
    return 1;
  }

  cl::Device device = all_devices.front();
  std::cout << "Using first OpenCL device of " << all_devices.size()
            << " options: " << device.getInfo<CL_DEVICE_NAME>() << std::endl;

  cl::Context context(device);
  cl::CommandQueue queue(context, device);
  std::cout << "Created OpenCL context and command queue." << std::endl;

  std::cout << "context = " << context() << "\n"
            << "device = " << device() << "\n"
            << "queue = " <<  queue() << "\n"
            << std::flush;

  // Create the Bullet 3 rigid body pipeline.
#else
  int ciErrNum = 0;
  cl_platform_id platform_id;

  cl_context context = b3OpenCLUtils::createContextFromType(
    CL_DEVICE_TYPE_GPU, &ciErrNum, 0, 0, -1, -1, &platform_id);
  if (ciErrNum != CL_SUCCESS)
  {
    std::cerr << "Failed creating OpenCL context." << std::endl;
    return 1;
  }

  int numDev = b3OpenCLUtils::getNumDevices(context);
  if (numDev == 0)
  {
    std::cerr << "No OpenCL devices are available." << std::endl;
    return 1;
  }

  cl_device_id device = b3OpenCLUtils::getDevice(context, 0);
  cl_command_queue queue = clCreateCommandQueue(context, device, 0, &ciErrNum);

  if (ciErrNum != CL_SUCCESS)
  {
    std::cerr << "Failed creating OpenCL command queue." << std::endl;
    return 1;
  }
#endif

  b3Config config;
  config.m_maxConvexBodies = 2;
  config.m_maxConvexShapes = config.m_maxConvexBodies;
  config.m_maxBroadphasePairs = 16 * config.m_maxConvexBodies;
  config.m_maxBroadphasePairs = config.m_maxBroadphasePairs;

  b3GpuNarrowPhase narrow_phase(context, device, queue, config);
  b3GpuSapBroadphase broad_phase(context, device, queue);
  b3DynamicBvhBroadphase bvh_broad_phase(2 /* max convex bodies */);
  b3GpuRigidBodyPipeline pipeline(context, device, queue, &narrow_phase,
    &broad_phase, &bvh_broad_phase, config);
  std::cout << "Created b3GpuRigidBodyPipeline." << std::endl;

  return 0;
}
