#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <cmath>

#include "spatialsdk/an_packet_protocol.h"
#include "spatialsdk/spatial_packets.h"
#include "spatialsdk/rs232/rs232.h"

#define RAD2DEG (180.0/M_PI)

namespace py = pybind11;

class PyHALAdvancedNavigationSpatial{
	private:
		an_decoder_t an_decoder;
		an_packet_t *in_packet;
		system_state_packet_t system_state_packet;
		char rtcm_buf[255];
		size_t rtcm_size;
		int bytes_received;


	public:
	PyHALAdvancedNavigationSpatial(){};
	~PyHALAdvancedNavigationSpatial(){
		if (ComportIsOpen()){
			CloseComport();
		}
	};

	bool init(std::string port){
		return OpenComport(&port[0], 115200);
	}

	std::string get_gnss_name(){
		return "AdvancedNavigation-Spatial";
	}

	bool shutdown(){
		if (ComportIsOpen()){
			CloseComport();
		}
		return true;
	}

	void get_gnss_info(py::array_t<double, py::array::c_style | py::array::forcecast> gnss_status){
		auto gnss_status_ptr = (double*)gnss_status.request().ptr;
		auto gil_release = py::gil_scoped_release();
		bool packet_found = false;
		an_decoder_initialise(&an_decoder);
		while (not packet_found){
			if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0){
				an_decoder_increment(&an_decoder, bytes_received);
				while ((in_packet = an_packet_decode(&an_decoder)) != nullptr){
					if (in_packet->id == packet_id_system_state){
						packet_found = true;
						decode_system_state_packet(&system_state_packet, in_packet);
					} else {
						an_packet_free(&in_packet);
					}
				}
			}
		}

		gnss_status_ptr[0] = system_state_packet.latitude * RAD2DEG;
		gnss_status_ptr[1] = system_state_packet.longitude * RAD2DEG;
		gnss_status_ptr[2] = system_state_packet.height;
		gnss_status_ptr[3] = system_state_packet.standard_deviation[0];
		gnss_status_ptr[4] = system_state_packet.standard_deviation[1];
		gnss_status_ptr[5] = system_state_packet.standard_deviation[2];

		an_packet_free(&in_packet);
	}

	bool has_fix(){
		auto gil_release = py::gil_scoped_release();
		bool packet_found = false;
		an_decoder_initialise(&an_decoder);
		while (not packet_found){
			if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0){
				an_decoder_increment(&an_decoder, bytes_received);
				while ((in_packet = an_packet_decode(&an_decoder)) != nullptr){
					if (in_packet->id == packet_id_system_state){
						packet_found = true;
						decode_system_state_packet(&system_state_packet, in_packet);
					} else {
						an_packet_free(&in_packet);
					}
				}
			}
		}

		bool fix = system_state_packet.filter_status.b.gnss_fix_type > 0;
		an_packet_free(&in_packet);
		return fix;
	}

	constexpr bool has_rtcm() const {
		return true;
	}

	void put_rtcm_msg(py::array_t<unsigned char, py::array::c_style | py::array::forcecast> rtcm_msg, int rtcm_size){
		auto rtcm_buf = (unsigned char*)rtcm_msg.request().ptr;
		auto gil_release = py::gil_scoped_release();
		an_packet_t *out_packet = an_packet_allocate(rtcm_size, packet_id_rtcm_corrections);
		if(out_packet != NULL){
			memcpy(out_packet->data, rtcm_buf, rtcm_size);
			an_packet_encode(out_packet);
			SendBuf(an_packet_pointer(out_packet), an_packet_size(out_packet));
			an_packet_free(&out_packet);
		}
	}
};

PYBIND11_MODULE(advancednavigation_spatial, m) {
    py::class_<PyHALAdvancedNavigationSpatial>(m, "HAL")
		.def(py::init<>())
		.def("init", &PyHALAdvancedNavigationSpatial::init)
		.def("get_gnss_name", &PyHALAdvancedNavigationSpatial::get_gnss_name)
		.def("shutdown", &PyHALAdvancedNavigationSpatial::shutdown)
		.def("get_gnss_info", &PyHALAdvancedNavigationSpatial::get_gnss_info)
		.def("has_fix", &PyHALAdvancedNavigationSpatial::has_fix)
		.def("has_rtcm", &PyHALAdvancedNavigationSpatial::has_rtcm)
		.def("put_rtcm_msg", &PyHALAdvancedNavigationSpatial::put_rtcm_msg);
}