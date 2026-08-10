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

#include "JJKVR_ServerDriver.hpp"
#include "JJKVR_HMDDriver.hpp"

#include <cmath>
#include <cstdint>
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

	constexpr bool cursorIsInsideScreen(long x, long y, int32_t screenX, int32_t screenY,
										 int32_t screenWidth, int32_t screenHeight) {
		// Right and bottom are exclusive, matching Windows monitor rectangles.
		return static_cast<int64_t>(x) >= screenX &&
			static_cast<int64_t>(x) < static_cast<int64_t>(screenX) + screenWidth &&
			static_cast<int64_t>(y) >= screenY &&
			static_cast<int64_t>(y) < static_cast<int64_t>(screenY) + screenHeight;
	}

	static_assert(cursorIsInsideScreen(1920, 0, 1920, 0, 2880, 1440));
	static_assert(!cursorIsInsideScreen(1919, 0, 1920, 0, 2880, 1440));
	static_assert(!cursorIsInsideScreen(4800, 0, 1920, 0, 2880, 1440));
}

class JJKVR::MouseController : public JJKVRDevice<false> {
public:
	MouseController(HMDDriver* hmd) : JJKVRDevice("mouse", "jjkvr_"), hmd(hmd) {
		static const char* const mouseSection = "jjkvr_mouse";
		m_sRenderModelPath = "generic_controller";
		m_sBindPath = "{jjkvr}/input/jjkvr_mouse_profile.json";
		screenX = vr::VRSettings()->GetInt32(
			k_pch_ExtDisplay_Section, k_pch_ExtDisplay_WindowX_Int32);
		screenY = vr::VRSettings()->GetInt32(
			k_pch_ExtDisplay_Section, k_pch_ExtDisplay_WindowY_Int32);
		screenWidth = vr::VRSettings()->GetInt32(
			k_pch_ExtDisplay_Section, k_pch_ExtDisplay_WindowWidth_Int32);
		screenHeight = vr::VRSettings()->GetInt32(
			k_pch_ExtDisplay_Section, k_pch_ExtDisplay_WindowHeight_Int32);
		if (screenWidth < 2) screenWidth = 2;
		if (screenHeight < 2) screenHeight = 2;
		horizontalDegrees = vr::VRSettings()->GetFloat(mouseSection, "horizontalDegrees");
		verticalDegrees = vr::VRSettings()->GetFloat(mouseSection, "verticalDegrees");
	}

	vr::EVRInitError Activate(uint32_t unObjectId) override {
		const auto error = JJKVRDevice::Activate(unObjectId);
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
		m_Pose = hmdPose;

		POINT cursor = {};
		const bool mouseActive = GetPhysicalCursorPos(&cursor) && cursorIsInsideScreen(
			cursor.x, cursor.y, screenX, screenY, screenWidth, screenHeight) &&
			hmdPose.poseIsValid && hmdPose.deviceIsConnected;
		if (mouseActive) {
			const double normalizedX =
				static_cast<double>(cursor.x - screenX) / (screenWidth - 1);
			const double normalizedY =
				static_cast<double>(cursor.y - screenY) / (screenHeight - 1);
			const double yawDegrees = -(normalizedX - 0.5) * horizontalDegrees;
			const double pitchDegrees = -(normalizedY - 0.5) * verticalDegrees;
			m_Pose.qRotation = multiplyQuaternion(
				hmdPose.qRotation, mouseAimQuaternion(yawDegrees, pitchDegrees));
		}
		m_Pose.poseIsValid = mouseActive;
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
			m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));

		if (trigger != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				trigger, mouseActive && (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0, 0.0);
		}
		if (rightClick != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				rightClick, mouseActive && (GetAsyncKeyState(VK_RBUTTON) & 0x8000) != 0, 0.0);
		}
		if (system != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				system, mouseActive && (GetAsyncKeyState(VK_MBUTTON) & 0x8000) != 0, 0.0);
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

vr::EVRInitError JJKVR::ServerDriver::Init(vr::IVRDriverContext* DriverContext) {

	vr::EVRInitError eError = vr::InitServerDriverContext(DriverContext);
		if (eError != vr::VRInitError_None) {
			return eError;
	}
	#ifdef DRIVERLOG_H
	InitDriverLog(vr::VRDriverLog());
	DriverLog("JJKVR driver version 0.1.0");
	DriverLog("Thread1: hid quaternion packet listener loop");
	DriverLog("Thread2: update driver pose loop");
	DriverLog("Thread3: receive positional data from python loop");
	#endif

	this->Log("JJKVR Init successful.\n");
	
	this->HMDDriver = new JJKVR::HMDDriver("zero");
	vr::VRServerDriverHost()->TrackedDeviceAdded(HMDDriver->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_HMD, this->HMDDriver);
	// GetSerialNumber() is there for a reason!

	this->mouseController = new JJKVR::MouseController(this->HMDDriver);
	vr::VRServerDriverHost()->TrackedDeviceAdded(mouseController->GetSerialNumber().c_str(),
		vr::ETrackedDeviceClass::TrackedDeviceClass_Controller, this->mouseController);

	return vr::VRInitError_None;
}

void JJKVR::ServerDriver::Cleanup() {
	delete this->mouseController;
	this->mouseController = nullptr;

	delete this->HMDDriver;
	this->HMDDriver = nullptr;

	#ifdef DRIVERLOG_H
	CleanupDriverLog();
	#endif

	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* JJKVR::ServerDriver::GetInterfaceVersions() {
	return vr::k_InterfaceVersions;
}

void JJKVR::ServerDriver::RunFrame() {
	if (this->mouseController != nullptr) {
		this->mouseController->frameUpdate();
	}
}

bool JJKVR::ServerDriver::ShouldBlockStandbyMode() {
	return false;
}

void JJKVR::ServerDriver::EnterStandby() {

}

void JJKVR::ServerDriver::LeaveStandby() {

}

void JJKVR::ServerDriver::Log(std::string log) {
	vr::VRDriverLog()->Log(log.c_str());
}
