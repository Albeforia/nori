#pragma once

#include <nori/common.h>
#include <embree3/rtcore_device.h>

NORI_NAMESPACE_BEGIN

/**
@brief Singleton that holds the Embree device
*/
class EmbreeDevice {

public:
	static EmbreeDevice& instance() {
		static EmbreeDevice _instance;
		return _instance;
	}

	EmbreeDevice(const EmbreeDevice&) = delete;
	EmbreeDevice& operator=(const EmbreeDevice&) = delete;

	~EmbreeDevice() {
		rtcReleaseDevice(m_device);
	}

	RTCDevice device() const { return m_device; }

private:
	EmbreeDevice() {
		m_device = rtcNewDevice(nullptr);
	}

	RTCDevice m_device;
};

NORI_NAMESPACE_END