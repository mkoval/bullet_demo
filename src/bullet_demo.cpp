#include <array>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <CL/cl.hpp>
#include <Bullet3OpenCL/Initialize/b3OpenCLUtils.h>
#include <Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h>
#include <Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h>
#include <Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h>
#include <Bullet3OpenCL/BroadphaseCollision/b3GpuParallelLinearBvhBroadphase.h>
#include <Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h>
#include <Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.h>

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
  pipeline.setGravity(b3MakeVector3(0., 0., -9.81));

  std::cout << "Created b3GpuRigidBodyPipeline." << std::endl;

  const std::vector<b3Vector3> pusher_vertices {
    b3MakeVector3(-0.05, -0.3, -0.1),
    b3MakeVector3(-0.05, -0.3, +0.1),
    b3MakeVector3(-0.05, +0.3, -0.1),
    b3MakeVector3(-0.05, +0.3, +0.1),
    b3MakeVector3(+0.05, -0.3, -0.1),
    b3MakeVector3(+0.05, -0.3, +0.1),
    b3MakeVector3(+0.05, +0.3, -0.1),
    b3MakeVector3(+0.05, +0.3, +0.1)
  };
  std::unique_ptr<b3ConvexUtility> pusher_convex(new b3ConvexUtility);
  if (!pusher_convex->initializePolyhedralFeatures(
        pusher_vertices.data(), pusher_vertices.size()))
  {
    std::cerr << "Failed creating pusher b3ConvexUtility." << std::endl;
    return 1;
  }
  std::cout << "Created pusher b3ConvexUtility." << std::endl;

  const int pusher_shape = narrow_phase.registerConvexHullShape(
    pusher_convex.release());
  if (pusher_shape < 0)
  {
    std::cerr << "Failed registering pusher shape." << std::endl;
    return 1;
  }
  std::cout << "Registered pusher shape." << std::endl;

  const float pusher_mass(1.f);
  const int pusher_userdata = 1;
  const b3Vector3 pusher_position = b3MakeVector3(0., 0., 0.05);
  const b3Quaternion pusher_orientation(0., 0., 0., 1.);

  const int pusher_physics = pipeline.registerPhysicsInstance(
    pusher_mass, pusher_position, pusher_orientation, pusher_shape,
    pusher_userdata, false);
  if (pusher_physics < 0)
  {
    std::cerr << "Failed registering pusher physics interface." << std::endl;
    return 1;
  }
  std::cout << "Registered pusher physics interface." << std::endl;

  pipeline.writeAllInstancesToGpu();
  narrow_phase.writeAllBodiesToGpu();
  broad_phase.writeAabbsToGpu();
  std::cout << "Wrote data to GPU." << std::endl;

  std::cout << "Simulating " << pipeline.getNumBodies() << " bodies."
            << std::endl;
  const double timestep = 0.01;

  for (int istep = 0; istep < 100; ++istep)
  {
    narrow_phase.readbackAllBodiesToCpu();

    b3Vector3 position;
    b3Quaternion orientation;
    if (!narrow_phase.getObjectTransformFromCpu(
          position, orientation, pusher_shape))
    {
      std::cerr << "Failed getting pusher transform." << std::endl;
      return 1;
    }

    std::cout << "position = "
      << position[0] << " " << position[1] << " " << position[2] << std::endl;

    pipeline.stepSimulation(timestep);
  }


  return 0;
}
