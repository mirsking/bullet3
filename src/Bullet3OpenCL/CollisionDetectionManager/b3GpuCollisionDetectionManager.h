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

#ifndef B3GPUCOLLISIONDETECTIONMANAGER_H
#define B3GPUCOLLISIONDETECTIONMANAGER_H

#include "b3GpuCollisionDetectionManagerInternalData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"

class b3GpuCollisionDetectionManager
{
public:
    b3GpuCollisionDetectionManager(const b3Config& config);
    ~b3GpuCollisionDetectionManager();

private:
    void initCL(int preferredDeviceIndex=-1, int preferredPlatformIndex=-1)
    {
        if(!m_data)
        {
            b3Error("Before init CL, you should init m_data in CollisionDetectionManager!\n");
            return;
        }
        int ciErrNum = 0;

        cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
        m_data->m_context = b3OpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex,&m_data->m_platformId);

        oclCHECKERROR(ciErrNum, CL_SUCCESS);

        int numDev = b3OpenCLUtils::getNumDevices(m_data->m_context);

        if (numDev>0)
        {
            m_data->m_device = b3OpenCLUtils::getDevice(m_data->m_context, 0);
            m_data->m_queue = clCreateCommandQueue(m_data->m_context, m_data->m_device, 0, &ciErrNum);
            oclCHECKERROR(ciErrNum, CL_SUCCESS);

            b3OpenCLDeviceInfo info;
            b3OpenCLUtils::getDeviceInfo(m_data->m_device, &info);
            b3Printf("Open CL device %s successfully!\n", info.m_deviceName);
            m_data->m_clInitialized = true;
        }
    }

    void exitCL()
    {

        if (m_data && m_data->m_clInitialized)
        {
            clReleaseCommandQueue(m_data->m_queue);
            clReleaseContext(m_data->m_context);
            m_data->m_clInitialized = false;
        }
    }

protected:
    struct b3GpuCollisionDetectionInternalData* m_data;
};

#endif // B3GPUCOLLISIONDETECTIONMANAGER_H
