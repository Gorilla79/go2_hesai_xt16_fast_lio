#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <chrono>

int main()
{
    const char* port = "/dev/ttyUSB0";
    int baudrate = B115200;

    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("Failed to open serial port");
        return 1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr failed");
        return 1;
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8bit
    tty.c_cflag |= CLOCAL | CREAD;               // enable receiver
    tty.c_cflag &= ~(PARENB | PARODD);            // no parity
    tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                      // no flow control

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr failed");
        return 1;
    }

    std::cout << "Listening on " << port << " ...\n";

    char buf[256];
    auto last = std::chrono::steady_clock::now();
    int count = 0;

    while (true)
    {
        int n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0)
        {
            buf[n] = '\0';
            std::cout << buf;
            count++;

            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - last).count();
            if (dt >= 1.0)
            {
                std::cout << "\n[Rate] " << count << " packets/sec\n";
                count = 0;
                last = now;
            }
        }
    }

    close(fd);
    return 0;
}
