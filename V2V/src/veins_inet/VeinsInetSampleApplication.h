//
// Copyright (C) 2018 Christoph Sommer <sommer@ccs-labs.org>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#pragma once

#include "veins_inet/veins_inet.h"
#include <fstream>
#include <string>
#include <set>

#include "veins_inet/VeinsInetApplicationBase.h"
#include "veins_inet/LogHandler.h"
#include "veins_inet/AccidentCreator.h"
#include "veins_inet/PacketProcessor.h"
#include "veins_inet/PacketSender.h"
#include "veins_inet/ApplicationLifecycle.h"

class VEINS_INET_API VeinsInetSampleApplication : public veins::VeinsInetApplicationBase {
protected:
    bool haveForwarded = false;
    std::set<std::string> seenMessages; // Track messages we've already seen/relayed

public:
    bool hasSentOwnAccident = false;

    // Public accessor methods for protected members
    veins::TraCICommandInterface::Vehicle* getTraciVehicle() { return traciVehicle; }
    veins::VeinsInetMobility* getMobility() { return mobility; }
    void callTimestampPayload(inet::Ptr<inet::Chunk> payload) { timestampPayload(payload); }
    std::unique_ptr<inet::Packet> callCreatePacket(std::string name) { return createPacket(name); }
    veins::TimerManager& getTimerManager() { return timerManager; }
    void callBaseSendPacket(std::unique_ptr<inet::Packet> pk) { VeinsInetApplicationBase::sendPacket(std::move(pk)); }

protected:
    virtual bool startApplication() override;
    virtual bool stopApplication() override;
    virtual void processPacket(std::shared_ptr<inet::Packet> pk) override;
    virtual void sendPacket(std::unique_ptr<inet::Packet> pk) override;

public:
    VeinsInetSampleApplication();
    ~VeinsInetSampleApplication();
};
