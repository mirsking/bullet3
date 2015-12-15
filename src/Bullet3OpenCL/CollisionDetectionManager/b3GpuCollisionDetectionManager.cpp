/*
Copyright (c) 2015 mirsking.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Mirs King

#include "b3GpuCollisionDetectionManager.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"

#include "Bullet3OpenCL/RigidBody/kernels/integrateKernel.h"
#include "Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.h"
#define B3_RIGIDBODY_INTEGRATE_PATH "src/Bullet3OpenCL/RigidBody/kernels/integrateKernel.cl"
#define B3_RIGIDBODY_UPDATEAABB_PATH "src/Bullet3OpenCL/RigidBody/kernels/updateAabbsKernel.cl"

b3GpuCollisionDetectionManager::b3GpuCollisionDetectionManager(const b3Config& config)
{
    m_data = new b3GpuCollisionDetectionInternalData;
    m_data->m_config = config;
    initCL();

    m_data->m_allAabbsGPU = new b3OpenCLArray<b3SapAabb>(m_data->m_context, m_data->m_queue, config.m_maxConvexBodies);
    m_data->m_overlappingPairsGPU = new b3OpenCLArray<b3BroadphasePair>(m_data->m_context, m_data->m_queue, config.m_maxBroadphasePairs);

    m_data->m_broadphase = new b3GpuSapBroadphase(m_data->m_context, m_data->m_device, m_data->m_queue);
    m_data->m_narrowphase = new b3GpuNarrowPhase(m_data->m_context, m_data->m_device, m_data->m_queue, m_data->m_config);

    cl_int errNum=0;

    {
        cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,integrateKernelCL,&errNum,"",B3_RIGIDBODY_INTEGRATE_PATH);
        b3Assert(errNum==CL_SUCCESS);
        m_data->m_integrateTransformsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,integrateKernelCL, "integrateTransformsKernel",&errNum,prog);
        b3Assert(errNum==CL_SUCCESS);
        clReleaseProgram(prog);
    }
    {
        cl_program prog = b3OpenCLUtils::compileCLProgramFromString(m_data->m_context,m_data->m_device,updateAabbsKernelCL,&errNum,"",B3_RIGIDBODY_UPDATEAABB_PATH);
        b3Assert(errNum==CL_SUCCESS);
        m_data->m_updateAabbsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,updateAabbsKernelCL, "initializeGpuAabbsFull",&errNum,prog);
        b3Assert(errNum==CL_SUCCESS);


        m_data->m_clearOverlappingPairsKernel = b3OpenCLUtils::compileCLKernelFromString(m_data->m_context, m_data->m_device,updateAabbsKernelCL, "clearOverlappingPairsKernel",&errNum,prog);
        b3Assert(errNum==CL_SUCCESS);

        clReleaseProgram(prog);
    }
}

b3GpuCollisionDetectionManager::~b3GpuCollisionDetectionManager()
{
    if (m_data->m_integrateTransformsKernel)
        clReleaseKernel(m_data->m_integrateTransformsKernel);

    if (m_data->m_updateAabbsKernel)
        clReleaseKernel(m_data->m_updateAabbsKernel);

    if (m_data->m_clearOverlappingPairsKernel)
        clReleaseKernel(m_data->m_clearOverlappingPairsKernel);

    delete m_data->m_allAabbsGPU;
    delete m_data->m_overlappingPairsGPU;
    delete m_data->m_broadphase;
    delete m_data->m_narrowphase;

    exitCL();
    delete m_data;
}

