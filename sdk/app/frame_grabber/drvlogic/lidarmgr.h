/*
 * Copyright (C) 2014  RoboPeak
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 *  RoboPeak Lidar System
 *  Win32 Demo Application
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *
 */

#pragma once
#include "common.h"
#include "..\..\sdk\src\hal\locker.h"

class LidarMgr {

public:
    static LidarMgr & GetInstance();
    
    LidarMgr();
    ~LidarMgr();
    bool onConnect(const char * port);
    bool isConnected() const { return _isConnected; }
    void onDisconnect();
    bool checkDeviceHealth(int * errorCode = NULL);

public:
    rplidar_response_device_info_t devinfo;

	static RPlidarDriver * lidar_drv;
    
protected:
    static LidarMgr * g_instance;
    static rp::hal::Locker g_oplocker;
    bool  _isConnected;
};