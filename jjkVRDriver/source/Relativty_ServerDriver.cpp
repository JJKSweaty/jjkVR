// Copyright (C) 2020  Max Coutte, Gabriel Combe
// Copyright (C) 2020  Relativty.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "openvr_driver.h"

#include "driverlog.h"

#include "Relativty_ServerDriver.hpp"
#include "Relativty_HMDDriver.hpp"

#include <algorithm>
#include <cmath>
#include <Windows.h>

namespace {
	vr::HmdQuaternion_t multiplyQuaternion(const vr::HmdQuaternion_t& left,
										 const vr::HmdQuaternion_t& right) {
		return {
			left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
			left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
			left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
			left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w
		};
	}

	vr::HmdQuaternion_t mouseAimQuaternion(double yawDegrees, double pitchDegrees) {
		constexpr double degreesToRadians = 3.14159265358979323846 / 180.0;
		const double halfYaw = yawDegrees * degreesToRadians * 0.5;
		const double halfPitch = pitchDegrees * degreesToRadians * 0.5;
		const vr::HmdQuaternion_t yaw = { std::cos(halfYaw), 0.0, std::sin(halfYaw), 0.0 };
		const vr::HmdQuaternion_t pitch = { std::cos(halfPitch), std::sin(halfPitch), 0.0, 0.0 };
		return multiplyQuaternion(yaw, pitch);
	}
}

class Relativty::MouseController : public RelativtyDevice<false> {
public:
	MouseController(HMDDriver* hmd) : RelativtyDevice("mouse", "jjkvr_"), hmd(hmd) {
		static const char* const mouseSection = "jjkvr_mouse";
		m_sRenderModelPath = "generic_controller";
		m_sBindPath = "{jjkvr}/input/jjkvr_mouse_profile.json";
		screenX = vr::VRSettings()->GetInt32(mouseSection, "screenX");
		screenY = vr::VRSettings()->GetInt32(mouseSection, "screenY");
		screenWidth = vr::VRSettings()->GetInt32(mouseSection, "screenWidth");
		screenHeight = vr::VRSettings()->GetInt32(mouseSection, "screenHeight");
		if (screenWidth < 2) screenWidth = 2;
		if (screenHeight < 2) screenHeight = 2;
		horizontalDegrees = vr::VRSettings()->GetFloat(mouseSection, "horizontalDegrees");
		verticalDegrees = vr::VRSettings()->GetFloat(mouseSection, "verticalDegrees");
	}

	vr::EVRInitError Activate(uint32_t unObjectId) override {
		const auto error = RelativtyDevice::Activate(unObjectId);
		if (error != vr::VRInitError_None) {
			return error;
		}

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer,
			vr::Prop_ControllerType_String, "jjkvr_mouse");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer,
			vr::Prop_TrackingSystemName_String, "jjkvr");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer,
			vr::Prop_ManufacturerName_String, "JJKVR");
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer,
			vr::Prop_RegisteredDeviceType_String, "jjkvr/mouse_controller");
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer,
			vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);

		const auto triggerError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/trigger/click", &trigger);
		const auto rightClickError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/grip/click", &rightClick);
		const auto systemError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/system/click", &system);
		if (triggerError != vr::VRInputError_None ||
			rightClickError != vr::VRInputError_None ||
			systemError != vr::VRInputError_None) {
			ServerDriver::Log("Mouse: unable to create one or more input components.\n");
		}

		return vr::VRInitError_None;
	}

	void frameUpdate() {
		const vr::DriverPose_t hmdPose = hmd->GetPose();
		POINT cursor = { screenX + screenWidth / 2, screenY + screenHeight / 2 };
		GetCursorPos(&cursor);

		const double normalizedX = std::clamp(
			static_cast<double>(cursor.x - screenX) / (screenWidth - 1), 0.0, 1.0);
		const double normalizedY = std::clamp(
			static_cast<double>(cursor.y - screenY) / (screenHeight - 1), 0.0, 1.0);
		const double yawDegrees = -(normalizedX - 0.5) * horizontalDegrees;
		const double pitchDegrees = -(normalizedY - 0.5) * verticalDegrees;

		m_Pose = hmdPose;
		m_Pose.qRotation = multiplyQuaternion(
			hmdPose.qRotation, mouseAimQuaternion(yawDegrees, pitchDegrees));
		m_Pose.deviceIsConnected = true;
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
			m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));

		if (trigger != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				trigger, (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0, 0.0);
		}
		if (rightClick != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				rightClick, (GetAsyncKeyState(VK_RBUTTON) & 0x8000) != 0, 0.0);
		}
		if (system != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				system, (GetAsyncKeyState(VK_MBUTTON) & 0x8000) != 0, 0.0);
		}
	}

private:
	HMDDriver* hmd;
	int32_t screenX;
	int32_t screenY;
	int32_t screenWidth;
	int32_t screenHeight;
	float horizontalDegrees;
	float verticalDegrees;
	vr::VRInputComponentHandle_t trigger = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t rightClick = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t system = vr::k_ulInvalidInputComponentHandle;
};

vr::EVRInitError Relativty::ServerDriver::Init(vr::IVRDriverContext* DriverContext) {

	vr::EVRInitError eError = vr::InitServerDriverContext(DriverContext);
		if (eError != vr::VRInitError_None) {
			return eError;
	}
	#ifdef DRIVERLOG_H
	InitDriverLog(vr::VRDriverLog());
	DriverLog("JJKVR driver version 0.1.0 (based on Relativty 0.1.1)");
	DriverLog("Thread1: hid quaternion packet listener loop");
	DriverLog("Thread2: update driver pose loop");
	DriverLog("Thread3: receive positional data from python loop");
	#endif

	this->Log("JJKVR Init successful.\n");
	
	this->HMDDriver = new Relativty::HMDDriver("zero");
	vr::VRServerDriverHost()->TrackedDeviceAdded(HMDDriver->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_HMD, this->HMDDriver);
	// GetSerialNumber() is there for a reason!

	this->mouseController = new Relativty::MouseController(this->HMDDriver);
	vr::VRServerDriverHost()->TrackedDeviceAdded(mouseController->GetSerialNumber().c_str(),
		vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, this->mouseController);

	return vr::VRInitError_None;
}

void Relativty::ServerDriver::Cleanup() {
	delete this->mouseController;
	this->mouseController = nullptr;

	delete this->HMDDriver;
	this->HMDDriver = nullptr;

	#ifdef DRIVERLOG_H
	CleanupDriverLog();
	#endif

	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* Relativty::ServerDriver::GetInterfaceVersions() {
	return vr::k_InterfaceVersions;
}

void Relativty::ServerDriver::RunFrame() {
	if (this->HMDDriver != nullptr) {
		this->HMDDriver->frameUpdate();
	}
	if (this->mouseController != nullptr) {
		this->mouseController->frameUpdate();
	}
}

bool Relativty::ServerDriver::ShouldBlockStandbyMode() {
	return false;
}

void Relativty::ServerDriver::EnterStandby() {

}

void Relativty::ServerDriver::LeaveStandby() {

}

void Relativty::ServerDriver::Log(std::string log) {
	vr::VRDriverLog()->Log(log.c_str());
}
