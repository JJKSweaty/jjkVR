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
#include <Xinput.h>

#pragma comment(lib, "Xinput9_1_0.lib")

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

	struct Vector3 {
		double x;
		double y;
		double z;
	};

	constexpr Vector3 rotateVector(const vr::HmdQuaternion_t& q, Vector3 v) {
		const double tx = 2.0 * (q.y * v.z - q.z * v.y);
		const double ty = 2.0 * (q.z * v.x - q.x * v.z);
		const double tz = 2.0 * (q.x * v.y - q.y * v.x);
		return {
			v.x + q.w * tx + (q.y * tz - q.z * ty),
			v.y + q.w * ty + (q.z * tx - q.x * tz),
			v.z + q.w * tz + (q.x * ty - q.y * tx)
		};
	}

	constexpr vr::HmdQuaternion_t identity = { 1.0, 0.0, 0.0, 0.0 };
	constexpr Vector3 identityResult = rotateVector(identity, { 0.25, -0.20, -0.45 });
	static_assert(identityResult.x == 0.25 && identityResult.y == -0.20 &&
		identityResult.z == -0.45);

	float applyThumbDeadzone(SHORT value, SHORT deadzone)
	{
		if (value > deadzone)
		{
			return static_cast<float>(value - deadzone) /
				static_cast<float>(32767 - deadzone);
		}
		if (value < -deadzone)
		{
			return static_cast<float>(value + deadzone) /
				static_cast<float>(32768 - deadzone);
		}
		return 0.0f;
	}

	bool keyDown(int key)
	{
		return (GetAsyncKeyState(key) & 0x8000) != 0;
	}
}

class JJKVR::MouseController : public JJKVRDevice<true> {
public:
	MouseController(HMDDriver* hmd) : JJKVRDevice("mouse", "jjkvr_"), hmd(hmd) {
		static const char* const mouseSection = "jjkvr_mouse";
		// SteamVR owns this animated model, so updates do not require a bundled mesh.
		m_sRenderModelPath = "oculus_quest2_controller_right";
		m_sBindPath = "{jjkvr}/input/jjkvr_mouse_profile.json";
		horizontalDegrees = vr::VRSettings()->GetFloat(mouseSection, "horizontalDegrees");
		verticalDegrees = vr::VRSettings()->GetFloat(mouseSection, "verticalDegrees");
		virtualCursorX = 0.5;
		virtualCursorY = 0.5;
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
		const auto triggerTouchError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/trigger/touch", &triggerTouch);
		const auto triggerValueError = vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trigger/value", &triggerValue,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
		const auto rightClickError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/grip/click", &rightClick);
		const auto gripTouchError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/grip/touch", &gripTouch);
		const auto gripValueError = vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/grip/value", &gripValue,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);
		const auto systemError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/system/click", &system);
		const auto menuError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/application_menu/click", &applicationMenu);
		const auto trackpadClickError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/trackpad/click", &trackpadClick);
		const auto trackpadTouchError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/trackpad/touch", &trackpadTouch);
		const auto trackpadXError = vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trackpad/x", &trackpadX,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		const auto trackpadYError = vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/trackpad/y", &trackpadY,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		const auto joystickClickError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/joystick/click", &joystickClick);
		const auto joystickTouchError = vr::VRDriverInput()->CreateBooleanComponent(
			m_ulPropertyContainer, "/input/joystick/touch", &joystickTouch);
		const auto joystickXError = vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/joystick/x", &joystickX,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		const auto joystickYError = vr::VRDriverInput()->CreateScalarComponent(
			m_ulPropertyContainer, "/input/joystick/y", &joystickY,
			vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
		if (triggerError != vr::VRInputError_None ||
			triggerTouchError != vr::VRInputError_None ||
			triggerValueError != vr::VRInputError_None ||
			rightClickError != vr::VRInputError_None ||
			gripTouchError != vr::VRInputError_None ||
			gripValueError != vr::VRInputError_None ||
			systemError != vr::VRInputError_None ||
			menuError != vr::VRInputError_None ||
			trackpadClickError != vr::VRInputError_None ||
			trackpadTouchError != vr::VRInputError_None ||
			trackpadXError != vr::VRInputError_None ||
			trackpadYError != vr::VRInputError_None ||
			joystickClickError != vr::VRInputError_None ||
			joystickTouchError != vr::VRInputError_None ||
			joystickXError != vr::VRInputError_None ||
			joystickYError != vr::VRInputError_None) {
			ServerDriver::Log("Mouse: unable to create one or more input components.\n");
		}

		return vr::VRInitError_None;
	}

	void frameUpdate() {
		if (m_unObjectId == vr::k_unTrackedDeviceIndexInvalid) {
			return;
		}

		const vr::DriverPose_t hmdPose = hmd->GetPose();
		m_Pose = hmdPose;
		const bool trackingValid = hmdPose.poseIsValid && hmdPose.deviceIsConnected;
		const DWORD nowMs = GetTickCount();
		if (lastFrameMs == 0U) {
			lastFrameMs = nowMs;
		}
		const float dt = static_cast<float>(nowMs - lastFrameMs) / 1000.0f;
		lastFrameMs = nowMs;

		XINPUT_STATE gamepadState = {};
		const bool gamepadConnected = XInputGetState(0U, &gamepadState) == ERROR_SUCCESS;
		const float moveX = (keyDown(VK_A) || keyDown(VK_LEFT) ? -1.0f : 0.0f) +
			(keyDown(VK_D) || keyDown(VK_RIGHT) ? 1.0f : 0.0f);
		const float moveY = (keyDown(VK_W) || keyDown(VK_UP) ? 1.0f : 0.0f) +
			(keyDown(VK_S) || keyDown(VK_DOWN) ? -1.0f : 0.0f);
		const bool keyboardMoving = (moveX != 0.0f) || (moveY != 0.0f);

		POINT cursor = {};
		MONITORINFO monitorInfo = { sizeof(MONITORINFO) };
		const bool cursorAvailable = GetPhysicalCursorPos(&cursor) &&
			GetMonitorInfo(MonitorFromPoint(cursor, MONITOR_DEFAULTTONEAREST), &monitorInfo);
		double normalizedX = 0.5;
		double normalizedY = 0.5;
		if (gamepadConnected) {
			if (!virtualCursorLocked) {
				if (cursorAvailable) {
					const LONG width = monitorInfo.rcMonitor.right - monitorInfo.rcMonitor.left;
					const LONG height = monitorInfo.rcMonitor.bottom - monitorInfo.rcMonitor.top;
					normalizedX = static_cast<double>(cursor.x - monitorInfo.rcMonitor.left) /
						(width > 1 ? width - 1 : 1);
					normalizedY = static_cast<double>(cursor.y - monitorInfo.rcMonitor.top) /
						(height > 1 ? height - 1 : 1);
				}
				virtualCursorX = normalizedX;
				virtualCursorY = normalizedY;
				virtualCursorLocked = true;
			}

			const float stickX = applyThumbDeadzone(gamepadState.Gamepad.sThumbRX,
				XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);
			const float stickY = applyThumbDeadzone(gamepadState.Gamepad.sThumbRY,
				XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);
			virtualCursorX = std::fmin(std::fmax(virtualCursorX + stickX * dt * 1.35, 0.0), 1.0);
			virtualCursorY = std::fmin(std::fmax(virtualCursorY - stickY * dt * 1.35, 0.0), 1.0);
			normalizedX = virtualCursorX;
			normalizedY = virtualCursorY;
		}
		else if (cursorAvailable) {
			const LONG width = monitorInfo.rcMonitor.right - monitorInfo.rcMonitor.left;
			const LONG height = monitorInfo.rcMonitor.bottom - monitorInfo.rcMonitor.top;
			normalizedX = static_cast<double>(cursor.x - monitorInfo.rcMonitor.left) /
				(width > 1 ? width - 1 : 1);
			normalizedY = static_cast<double>(cursor.y - monitorInfo.rcMonitor.top) /
				(height > 1 ? height - 1 : 1);
			virtualCursorX = normalizedX;
			virtualCursorY = normalizedY;
			virtualCursorLocked = false;
			const double yawDegrees = -(normalizedX - 0.5) * horizontalDegrees;
			const double pitchDegrees = -(normalizedY - 0.5) * verticalDegrees;
			m_Pose.qRotation = multiplyQuaternion(
				hmdPose.qRotation, mouseAimQuaternion(yawDegrees, pitchDegrees));
		}
		else {
			virtualCursorLocked = false;
		}

		if (gamepadConnected) {
			const double yawDegrees = -(normalizedX - 0.5) * horizontalDegrees;
			const double pitchDegrees = -(normalizedY - 0.5) * verticalDegrees;
			m_Pose.qRotation = multiplyQuaternion(
				hmdPose.qRotation, mouseAimQuaternion(yawDegrees, pitchDegrees));
		}

		m_Pose.vecPosition[0] = 0.0;
		m_Pose.vecPosition[1] = 0.0;
		m_Pose.vecPosition[2] = 0.0;
		m_Pose.vecVelocity[0] = 0.0;
		m_Pose.vecVelocity[1] = 0.0;
		m_Pose.vecVelocity[2] = 0.0;
		m_Pose.vecAcceleration[0] = 0.0;
		m_Pose.vecAcceleration[1] = 0.0;
		m_Pose.vecAcceleration[2] = 0.0;
		m_Pose.poseIsValid = trackingValid;
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(
			m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));

		const bool leftPressed = trackingValid && (gamepadConnected ?
			((gamepadState.Gamepad.wButtons & XINPUT_GAMEPAD_A) != 0) :
			((GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0));
		if (trigger != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(trigger, leftPressed, 0.0);
		}
		if (triggerTouch != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(triggerTouch, leftPressed, 0.0);
		}
		if (triggerValue != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateScalarComponent(
				triggerValue, leftPressed ? 1.0f : 0.0f, 0.0);
		}
		const bool rightPressed = trackingValid && (gamepadConnected ?
			((gamepadState.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0) :
			((GetAsyncKeyState(VK_RBUTTON) & 0x8000) != 0));
		if (rightClick != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				rightClick, rightPressed, 0.0);
		}
		if (gripTouch != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(gripTouch, rightPressed, 0.0);
		}
		if (gripValue != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateScalarComponent(
				gripValue, rightPressed ? 1.0f : 0.0f, 0.0);
		}
		if (system != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				system, trackingValid && (gamepadConnected ?
				((gamepadState.Gamepad.wButtons & XINPUT_GAMEPAD_START) != 0) :
				((GetAsyncKeyState(VK_MBUTTON) & 0x8000) != 0)), 0.0);
		}
		if (applicationMenu != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(applicationMenu,
				trackingValid && (gamepadConnected ?
				((gamepadState.Gamepad.wButtons & XINPUT_GAMEPAD_BACK) != 0) :
				((GetAsyncKeyState(VK_XBUTTON1) & 0x8000) != 0)), 0.0);
		}
		if (trackpadClick != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(trackpadClick,
				trackingValid && (gamepadConnected ?
				((gamepadState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) != 0) :
				((GetAsyncKeyState(VK_XBUTTON2) & 0x8000) != 0)), 0.0);
		}
		if (trackpadTouch != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				trackpadTouch, trackingValid && (cursorAvailable || gamepadConnected), 0.0);
		}
		if (trackpadX != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateScalarComponent(
				trackpadX, static_cast<float>(normalizedX * 2.0 - 1.0), 0.0);
		}
		if (trackpadY != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateScalarComponent(
				trackpadY, static_cast<float>(1.0 - normalizedY * 2.0), 0.0);
		}
		const bool stickPressed = trackingValid && (gamepadConnected ?
			((gamepadState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) != 0) :
			((GetAsyncKeyState(VK_XBUTTON2) & 0x8000) != 0));
		if (joystickClick != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(joystickClick, stickPressed || keyboardMoving, 0.0);
		}
		if (joystickTouch != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateBooleanComponent(
				joystickTouch, trackingValid && (cursorAvailable || gamepadConnected || keyboardMoving), 0.0);
		}
		if (joystickX != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateScalarComponent(
				joystickX, keyboardMoving ? moveX : 0.0f, 0.0);
		}
		if (joystickY != vr::k_ulInvalidInputComponentHandle) {
			vr::VRDriverInput()->UpdateScalarComponent(
				joystickY, keyboardMoving ? moveY : 0.0f, 0.0);
		}
	}

private:
	HMDDriver* hmd;
	float horizontalDegrees;
	float verticalDegrees;
	double virtualCursorX;
	double virtualCursorY;
	bool virtualCursorLocked = false;
	DWORD lastFrameMs = 0U;
	vr::VRInputComponentHandle_t trigger = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t triggerTouch = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t triggerValue = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t rightClick = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t gripTouch = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t gripValue = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t system = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t applicationMenu = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t trackpadClick = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t trackpadTouch = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t trackpadX = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t trackpadY = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t joystickClick = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t joystickTouch = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t joystickX = vr::k_ulInvalidInputComponentHandle;
	vr::VRInputComponentHandle_t joystickY = vr::k_ulInvalidInputComponentHandle;
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
