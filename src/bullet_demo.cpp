#include <array>
#include <iostream>
#include <vector>
#include <string>
#include <CL/cl.hpp>
#include <Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h>

#define OPENCL_CHECK(error_code, message)                                     \
do {                                                                          \
  if (error_code != CL_SUCCESS)                                               \
  {                                                                           \
    std::cerr << message << ": " << getCLErrorCode(error_code) << std::endl;  \
    return 1;                                                                 \
  }                                                                           \
} while(0)

const constexpr int OPENCL_DEVICE_INDEX = -1;
const constexpr int OPENCL_PLATFORM_INDEX = -1;

std::string getCLErrorCode(cl_int error_code)
{
  switch (error_code)
  {
  case CL_INVALID_PLATFORM:
    return "invalid platform";
  case CL_INVALID_VALUE:
    return "invalid value";
  case CL_DEVICE_NOT_AVAILABLE:
    return "not vailable";
  case CL_DEVICE_NOT_FOUND:
    return "device not found";
  case CL_OUT_OF_HOST_MEMORY:
    return "out of host memory";
  case CL_INVALID_DEVICE_TYPE:
    return "invalid device type";
  default:
    return "unknown";
  }
}

int main(int argc, char** argv)
{
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
  std::cout << "Using OpenCL platform: "
            << platform.getInfo<CL_PLATFORM_NAME>() << std::endl;

  // Select an OpenCL device.
  std::vector<cl::Device> all_devices;
  platform.getDevices(CL_DEVICE_TYPE_ALL, &all_devices);
  if (all_devices.empty())
  {
    std::cerr << "No OpenCL devices found." << std::endl;
    return 1;
  }

  cl::Device device = all_devices.front();
  std::cout << "Using OpenCL device: "
            << device.getInfo<CL_DEVICE_NAME>() << std::endl;

  return 0;
}
