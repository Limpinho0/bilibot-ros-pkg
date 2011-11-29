#include <string>

class Serial
{
    public:
        Serial();
        Serial(std::string device);
        int openPort();
        int closePort();
        uint8_t readByte();
        void writeByte(uint8_t ch);
    private:
        int fd;
        std::string device;
};
