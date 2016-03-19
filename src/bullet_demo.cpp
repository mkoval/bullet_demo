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

struct BodyIds
{
  int shape;
  int physics;
};

BodyIds create_box(
  b3GpuNarrowPhase& narrow_phase, b3GpuRigidBodyPipeline& pipeline,
  const b3Vector3& origin, const b3Vector3& size, float mass,
  const b3Vector3& position, const b3Quaternion& orientation,
  int userdata)
{
  BodyIds ids;

  const b3Vector3 s = 0.5 * size;
  const std::vector<b3Vector3> vertices {
    origin + b3MakeVector3(-s[0], -s[1], -s[2]),
    origin + b3MakeVector3(-s[0], -s[1], +s[2]),
    origin + b3MakeVector3(-s[0], +s[1], -s[2]),
    origin + b3MakeVector3(-s[0], +s[1], +s[2]),
    origin + b3MakeVector3(+s[0], -s[1], -s[2]),
    origin + b3MakeVector3(+s[0], -s[1], +s[2]),
    origin + b3MakeVector3(+s[0], +s[1], -s[2]),
    origin + b3MakeVector3(+s[0], +s[1], +s[2])
  };

  std::unique_ptr<b3ConvexUtility> pusher_convex(new b3ConvexUtility);
  if (!pusher_convex->initializePolyhedralFeatures(
        vertices.data(), vertices.size()))
    throw std::runtime_error("Failed creating polyhedron.");

  ids.shape = narrow_phase.registerConvexHullShape(
    pusher_convex.release());
  if (ids.shape < 0)
    throw std::runtime_error("Failed registering shape.");

  ids.physics = pipeline.registerPhysicsInstance(
    mass, position, orientation, ids.shape, userdata, false);
  if (ids.physics < 0)
    throw std::runtime_error("Failed registering physics instance.");

  return ids;
}

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

  BodyIds pusher = create_box(narrow_phase, pipeline,
    b3MakeVector3(0., 0., 0.), b3MakeVector3(0.1, 0.6, 0.2),
    1., b3MakeVector3(0., 0., 0.05), b3Quaternion(0., 0., 0., 1.), 0);
  std::cout << "Created pusher." << std::endl;

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
          position, orientation, pusher.shape))
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
