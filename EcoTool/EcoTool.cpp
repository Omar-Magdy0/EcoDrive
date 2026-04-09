#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <functional>
#include <aebfStream.h>
#include <sstream>

#include <type_traits>
#include <limits>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <boost/asio.hpp>

// Templated parser that accepts both decimal and hexadecimal
// Works with: uint8_t, uint16_t, uint32_t, int, unsigned int, etc.
template<typename T>
bool parseNumber(const std::string& str, T& value) {
    if (str.empty()) {
        return false;
    }

    std::string s = str;
    bool isHex = false;

    // Detect hex prefix (0x or 0X)
    if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        isHex = true;
        s = s.substr(2);
    }

    // Remove leading zeros
    s.erase(0, s.find_first_not_of('0'));
    if (s.empty()) s = "0";

    std::stringstream ss(s);
    T temp = 0;

    if (isHex) {
        ss >> std::hex >> temp;
    } else {
        ss >> std::dec >> temp;
    }

    // Check for parsing failure or extra characters
    if (ss.fail() || !ss.eof()) {
        return false;
    }

    // Check range (prevent overflow)
    if (temp < std::numeric_limits<T>::min() || 
        temp > std::numeric_limits<T>::max()) {
        return false;
    }

    value = temp;
    return true;
}

// Helper function to split input line into words
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    while (ss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

class CLI {
private:
    // Command dispatcher: command name → function that receives arguments
    std::unordered_map<std::string, std::function<void(const std::vector<std::string>&)>> commands;
    bool running = true;

    // ====================== Your Commands ======================
    void cmd_portConnect(const std::vector<std::string>& args){
        if(args.size < 1)
        {

        }
    }

    void cmd_aebfSend(const std::vector<std::string>& args) {
        if (args.size() < 3)
        {   // device ID + command ID and payload are required
            std::cout << "Usage: aebfSend <deviceID_hex> <commandID_hex> [payload...]\n";
            return;
        }
        int device_id = 0;
        int service_id = 0;
        int payload_len = 0;
        uint8_t payload[256] = {0};
        bool device_id_good = parseNumber<int>(args[0], device_id);
        bool service_id_good = parseNumber<int>(args[1], service_id);

        //Normal string passing for now , a payload parser can be added later
        payload_len = args[2].length();
        bool payload_len_good = payload_len <= 240;
        if(payload_len_good)std::memcpy(payload + 5, args[2].data(), payload_len);

        // Sanity checks
        device_id_good |= (device_id >= 0 && device_id <= 31);
        service_id_good |= (service_id >= 0 && service_id <= 1023);
        if(!device_id_good){std::cout << "err: device ID isnt a valid number, (min: 0, max: 31)"; return;}
        if(!service_id_good){std::cout << "err: service ID isnt a valid number, (min: 0, max: 1023)"; return;}
        if(!payload_len_good){std::cout << "err: Payload size bad, payload size (min: 1 and max: 240) bytes"; return;}
        
        aebf_encode_frame(payload, (uint8_t)device_id, (uint16_t)service_id, (uint8_t)payload_len);

        const uint16_t frame_len = AEBF_FRAMED_SIZE(payload_len);
    }

    void cmd_fsave(const std::vector<std::string>& args) {
        if (args.size() < 2) {
            std::cout << "Usage:\n"
                      << "  fsave start [filename]   - Start saving received data\n"
                      << "  fsave end                - Stop saving\n";
            return;
        }

        if (args[0] == "start")
        {
            std::string filename;
            if (args.size() > 1)
            {
                filename = args[1];
                std::cout << "Started logging to: " << filename << "\n";
            }
            else
            {
                std::cout << "err: no file name added, usage fsave start [filename]" << std::endl;
            }
            // TODO: call your startLogging(filename);
        }
        else if (args[0] == "end") {
            std::cout << "Stopped file logging.\n";

        }
        else {
            std::cout << "Unknown fsave option: " << args[0] << "\n";
        }
    }

    void cmd_help(const std::vector<std::string>& /*args*/) {
        std::cout << "=== EcoTool CLI Commands ===\n"
                  << "aebfSend <devID> <cmdID> [payload ...]   Send AEBF command over connected port\n"
                  << "fsave start [filename]                   Start saving received data to file\n"
                  << "fsave end                                Stop saving data\n"
                  << "help                                     Show this help\n"
                  << "exit  or  quit                           Exit the program\n\n"
    ;}

    void cmd_exit(const std::vector<std::string>& /*args*/) {
        running = false;
        std::cout << "Exiting CLI...\n";
    }

public:
    CLI() {
        // Register commands (this is the clean way - easy to extend)
        commands["portCon"] = [this](const auto& args) { cmd_portConnect(args); };
        commands["aebfS"] = [this](const auto& args) { cmd_aebfSend(args); };
        commands["fsave"]    = [this](const auto& args) { cmd_fsave(args); };
        commands["help"]     = [this](const auto& args) { cmd_help(args); };
        commands["exit"]     = [this](const auto& args) { cmd_exit(args); };
        commands["quit"]     = [this](const auto& args) { cmd_exit(args); };
    }

    void run() {
        std::string line;
        std::cout << "EcoTool V0.0 CLI ready (type 'help' for commands)\n> ";
        while (running) {
            if (!std::getline(std::cin, line)) break;
            if (line.empty()) {
                std::cout << "> ";
                continue;
            }

            auto tokens = split(line);
            if (tokens.empty()) continue;

            std::string cmd_name = tokens[0];
            std::vector<std::string> args(tokens.begin() + 1, tokens.end());

            auto it = commands.find(cmd_name);
            if (it != commands.end()) {
                it->second(args);           // Execute the registered function
            } 
            else {
                // Unknown command
                std::cout << "Unknown command" << line << "\n";
            }
            if (running) std::cout << "> ";
        }
    }
};


int main()
{
    CLI cli = CLI();
    cli.run();
}
