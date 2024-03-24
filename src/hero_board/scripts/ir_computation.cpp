#include <iostream>
#include <vector>
#include <jsoncpp/json/json.h>

namespace std;

class ircomp_warning : public std::exception
{
	public:
	     ircomp_warning(const std::string& warn_msg) {}
	     const char* what() { return warn_msg.c_str(); } //message of warning
	private:
	     std::string warn_msg;
};

struct ir_distances{
	float[] sensor1;
	float[] sensor2;
	float[] sensor3;
	float[] angles;
}


int main(int argc, char *argv[]){
	struct ir_distances* ir_data;
	uint8 num_fields = 4;
	if (argc > 1 || argc == 0){
		throw ircomp_warning("Too many or too few arguments provided. Cannot process IR Sensor feeds.");
	}
	std::string serialized_data = argv[1];
	Json::Value msg_json;
    	Json::Reader reader;
    	if (!reader.parse(serialized_data, msg_json)) {
        	std::cerr << "Failed to parse IR Vector messag as JSON\n";
        	return 1;
    	}
	int sizeof_msg = sizeof(msg_json);
	int sizeof_sensor_payload = sizeof_msg - 1; //Subract off byte for is_depth_data boolean
	int sizeof_single_array = sizeof_sensor_payload/4;
	
	

