#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <functional>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cassert>

#include "rclcpp/rclcpp.hpp"

#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <serial/serial.h>

using std::placeholders::_1;

class HALSpatialNode : public rclcpp::Node {
 public:

  HALSpatialNode() : Node("hal_an_spatial") {
    port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud = (uint32_t) declare_parameter<int>("baud", 115200);
	gnss_rate = (uint32_t) declare_parameter<int>("gnss_rate", 1);
	imu_rate = (uint32_t) declare_parameter<int>("imu_rate", 50);
	imu_topic = declare_parameter<std::string>("publishers.imu_topic", "/imu");
	mag_topic = declare_parameter<std::string>("publishers.mag_topic", "/mag");
	nav_topic = declare_parameter<std::string>("publishers.nav_topic", "/gnss");
	rtcm_topic = declare_parameter<std::string>("publishers.rtcm_topic", "/rtcm");
	reference_frame = declare_parameter<std::string>("frame_id", "base_link");
	try {
     ser = std::make_shared<serial::Serial>();
     ser->setPort(port);
     ser->setBaudrate(baud);
     auto t = serial::Timeout::simpleTimeout(serial::Timeout::max());
     ser->setTimeout(t);
     ser->open();
    } catch(...){
      RCLCPP_ERROR(get_logger(), "Unable to open port ", port);
    }

	an_packet_t *out_packet;

	packet_timer_period_packet_t timer_packet = {
		.permanent = 0,
		.utc_synchronisation = 0,
		.packet_timer_period = 1000 //1000 us (1000 Hz)
	};

	out_packet = encode_packet_timer_period_packet(&timer_packet);
	if(out_packet != nullptr){
		an_packet_encode(out_packet);
		ser->write(an_packet_pointer(out_packet), an_packet_size(out_packet));
		an_packet_free(&out_packet);
	}

	// Packet Rate = 1000000/(Packet Period x Packet Timer Period) Hz
	// Packet Period = 1000000/(Packet Timer Period x Packet Rate)
	packet_periods_packet_t periods_packet = {
		.permanent = 0,
		.clear_existing_packets = 1,
		.packet_periods = {
			{ .packet_id = 20,	.period = 1000000/(1000 * gnss_rate /*Hz*/) }, 
			{ .packet_id = 28,	.period = 1000000/(1000 * imu_rate  /*Hz*/) }
		}
	};

	out_packet = encode_packet_periods_packet(&periods_packet);
	if(out_packet != nullptr){
		an_packet_encode(out_packet);
		ser->write(an_packet_pointer(out_packet), an_packet_size(out_packet));
		an_packet_free(&out_packet);
	}

    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1);
    mag_publisher_ = create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 1);
    nav_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(nav_topic, 1);
	rtcm_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(rtcm_topic, 10, std::bind(&HALSpatialNode::rtcm_callback, this, _1));

	worker_thread = std::thread([this]{this->feedback_loop();});
  }

  ~HALSpatialNode(){
	  stop = true;
	  worker_thread.join();
	  ser->close();
  }
  
 private:
	std::thread worker_thread;
 	std::string port;
	std::string reference_frame;
    uint32_t baud, imu_rate, gnss_rate;
	std::shared_ptr<serial::Serial> ser;
	std::string imu_topic, mag_topic, nav_topic, rtcm_topic;
	an_decoder_t an_decoder;
	int bytes_received;
	bool stop = false;

  	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  	rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_publisher_;
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rtcm_subscription_;

	void rtcm_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		an_packet_t *out_packet = an_packet_allocate(msg->layout.dim[0].size , packet_id_rtcm_corrections);
		if(out_packet != nullptr){
			memcpy(out_packet->data, msg->data.data(), msg->layout.dim[0].size );
			an_packet_encode(out_packet);
			ser->write(an_packet_pointer(out_packet), an_packet_size(out_packet));
			an_packet_free(&out_packet);
		}
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		RCLCPP_INFO(get_logger(), "RTCM - Time difference = %d [ms]", std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
	}

  void feedback_loop(){
	an_packet_t *in_packet;
	sensor_msgs::msg::Imu imu_msg;
	sensor_msgs::msg::MagneticField mag_msg;
	sensor_msgs::msg::NavSatFix nav_msg;
	raw_sensors_packet_t sensor_packet;
	system_state_packet_t system_packet;
	packet_id_e type;

	while(not stop){
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		bool packet_found = false;
		an_decoder_initialise(&an_decoder);
		while (not packet_found){
			if ((bytes_received = ser->read(an_decoder_pointer(&an_decoder), 1))){
				an_decoder_increment(&an_decoder, bytes_received);
				while ((in_packet = an_packet_decode(&an_decoder)) != nullptr){
					if (in_packet->id == packet_id_raw_sensors){
						type = packet_id_raw_sensors;
						packet_found = true;
						decode_raw_sensors_packet(&sensor_packet, in_packet);
					} else if (in_packet->id == packet_id_system_state){
						type = packet_id_system_state;
						packet_found = true;
						decode_system_state_packet(&system_packet, in_packet);
					} else {
						an_packet_free(&in_packet);
					}
				}
			}
		}

		if (type == packet_id_raw_sensors){
			imu_msg.angular_velocity.x = sensor_packet.gyroscopes[0];
			imu_msg.angular_velocity.y = sensor_packet.gyroscopes[1];
			imu_msg.angular_velocity.z = sensor_packet.gyroscopes[2];
			imu_msg.linear_acceleration.x = sensor_packet.accelerometers[0];
			imu_msg.linear_acceleration.y = sensor_packet.accelerometers[1];
			imu_msg.linear_acceleration.z = sensor_packet.accelerometers[2];
			imu_publisher_->publish(imu_msg);
			mag_msg.magnetic_field.x = sensor_packet.magnetometers[0];
			mag_msg.magnetic_field.y = sensor_packet.magnetometers[1];
			mag_msg.magnetic_field.z = sensor_packet.magnetometers[2];
			mag_publisher_->publish(mag_msg);
		} else {
			switch(system_packet.filter_status.b.gnss_fix_type){
				case 1:
				case 2:
					nav_msg.status.status = nav_msg.status.STATUS_FIX;
					break;
				case 3:
				case 4:
				case 5:
					nav_msg.status.status = nav_msg.status.STATUS_SBAS_FIX;
					break;
				case 6:
				case 7:
					nav_msg.status.status = nav_msg.status.STATUS_GBAS_FIX;
					break;
				default:
					nav_msg.status.status = nav_msg.status.STATUS_NO_FIX;
			}
			RCLCPP_INFO(get_logger(), "Fix type: %d", int(system_packet.filter_status.b.gnss_fix_type));
			nav_msg.status.service = nav_msg.status.SERVICE_GPS | nav_msg.status.SERVICE_GLONASS;
			nav_msg.header.frame_id = reference_frame;
			nav_msg.longitude = system_packet.longitude;
			nav_msg.latitude = system_packet.latitude;
			nav_msg.altitude = system_packet.height;
			// covariances in ENU order (ROS2), while SPATIAL order is lat/lon/elev
			nav_msg.position_covariance[0] = system_packet.standard_deviation[1];
			nav_msg.position_covariance[4] = system_packet.standard_deviation[0];
			nav_msg.position_covariance[8] = system_packet.standard_deviation[2];
			nav_msg.position_covariance_type = nav_msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
			nav_publisher_->publish(nav_msg);
		}

		an_packet_free(&in_packet);
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		RCLCPP_INFO(get_logger(), "Time difference = %d [ms]", std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
	}
  }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<HALSpatialNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}

/* DO NOT REMOVE: SNIPPET FOR RTCM CORRECTIONS (TODO)
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
*/