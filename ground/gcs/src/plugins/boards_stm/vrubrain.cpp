/**
 ******************************************************************************
 *
 * @file       vrubrain.cpp
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013
 *
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup Boards_OpenPilotPlugin OpenPilot boards support Plugin
 * @{
 * @brief Plugin to support boards by the OP project
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <uavobjectmanager.h>
#include "uavobjectutil/uavobjectutilmanager.h"
#include <extensionsystem/pluginmanager.h>
#include "vrubrain.h"
#include "hwvrubrain.h"

/**
 * @brief Vrubrain::Vrubrain
 *  This is the Vrubrain board definition
 */
Vrubrain::Vrubrain(void)
{
    // Initialize our USB Structure definition here:
    USBInfo board;
    board.vendorID = 0x20A0;
    board.productID = 0x4195;

    setUSBInfo(board);

    boardType = 0x87;

    // Define the bank of channels that are connected to a given timer
    channelBanks.resize(6);
    channelBanks[0] = QVector<int> () << 1 << 3;
    channelBanks[1] = QVector<int> () << 4 << 6;
    channelBanks[2] = QVector<int> () << 7 << 8;
}

Vrubrain::~Vrubrain()
{

}


QString Vrubrain::shortName()
{
    return QString("VR μBrain");
}

QString Vrubrain::boardDescription()
{
    return QString("The VRuBrain project VRuBrain boards");
}

//! Return which capabilities this board has
bool Vrubrain::queryCapabilities(BoardCapabilities capability)
{
    switch(capability) {
    case BOARD_CAPABILITIES_GYROS:
        return true;
    case BOARD_CAPABILITIES_ACCELS:
        return true;
    case BOARD_CAPABILITIES_MAGS:
        return true;
    case BOARD_CAPABILITIES_BAROS:
        return true;
    case BOARD_CAPABILITIES_RADIO:
        return false;
    }
    return false;
}

/**
 * @brief Vrubrain::getSupportedProtocols
 *  TODO: this is just a stub, we'll need to extend this a lot with multi protocol support
 * @return
 */
QStringList Vrubrain::getSupportedProtocols()
{
    return QStringList("uavtalk");
}

QPixmap Vrubrain::getBoardPicture()
{
    return QPixmap(":/stm/images/vrubrain.png");
}

QString Vrubrain::getHwUAVO()
{
    return "HwVrubrain";
}

int Vrubrain::queryMaxGyroRate()
{
    ExtensionSystem::PluginManager *pm = ExtensionSystem::PluginManager::instance();
    UAVObjectManager *uavoManager = pm->getObject<UAVObjectManager>();
    HwVrubrain *hwVrubrain = HwVrubrain::GetInstance(uavoManager);
    Q_ASSERT(hwVrubrain);
    if (!hwVrubrain)
        return 0;

    HwVrubrain::DataFields settings = hwVrubrain->getData();

    switch(settings.GyroRange) {
    case HwVrubrain::GYRORANGE_250:
        return 250;
    case HwVrubrain::GYRORANGE_500:
        return 500;
    case HwVrubrain::GYRORANGE_1000:
        return 1000;
    case HwVrubrain::GYRORANGE_2000:
        return 2000;
    default:
        return 500;
    }
}
