/* *** ADuC boot loader v. 0.1 ***
* 03 jan 2021
*
* from:
* https://www.analog.com/media/en/evaluation-boards-kits/evaluation-software/AN-1074.pdf
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

typedef unsigned char uint8_t;
typedef unsigned int uint32_t;

typedef enum {
  ERASE_CODE = 0x43,
  ERASE_ALL = 0x41,
  PROGRAM_BLOCK= 0x57,
  READ_PAGE = 0x56,
  LOAD_PAGE = 0x51,
  RUN_CODE = 0x55,
  LOAD_DATA_BLOCK = 0x45,
  ENABLE_BOOTLOAD = 0x46
} commands;

int set_interface_attribs (int fd, int speed, int parity)
{
  struct termios tty;
  if (tcgetattr (fd, &tty) != 0) {
    printf("error %d from tcgetattr", errno);
    return EXIT_FAILURE;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf("error %d from tcsetattr", errno);
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

void set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf("error %d from tggetattr", errno);
    return ;
  }
  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout
  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf("error %d setting term attributes", errno);
  }
}

int uart_read(int port, char *buf, size_t size)
{
  size_t pos = 0;
  while (size--) {
    if (!read(port, buf+pos, 1))
      return EXIT_FAILURE;
    pos++;
  }
  return EXIT_SUCCESS;
}

uint32_t hex2int(char *hex, int n)
{
  uint32_t val = 0;
  while (*hex && n--) {
    uint8_t byte = *hex++;
    if (byte >= '0' && byte <= '9')
      byte = byte - '0';
    else if (byte >= 'a' && byte <='f')
      byte = byte - 'a' + 10;
    else if (byte >= 'A' && byte <='F')
      byte = byte - 'A' + 10;
    val = (val << 4) | (byte & 0x0f);
  }
  return val;
}

int send_command(int port, commands cmd, uint8_t *data, uint8_t data_size)
{
  uint8_t buf[25] = {0x07, 0x0E, 1+data_size, cmd};
  uint8_t checksum = cmd+buf[2];
  for (int i = 0; i < data_size; i++) {
    buf[4+i] = data[i];
    checksum += buf[4+i];
  }
  buf[3+buf[2]] = 0x100 - checksum;
  write(port, buf, 4+buf[2]);
  int bytes_readed = read(port, buf, 1); //read ack
  if (buf[0] == 0x06 && bytes_readed == 1) {
    return EXIT_SUCCESS;
  }
  return EXIT_FAILURE;
}

int main(int argc, char *argv[])
{
  char buf[100];
  if (argc > 1) {
    printf("Opening file %s\n", argv[1]);
    //FILE *hex_file = fopen("./adbtldr/main.hex", "r");
    FILE *hex_file = fopen(argv[1], "r");
    if (hex_file != NULL) {
      char *portname = "/dev/ttyUSB0";
      if (argc == 3) {
        portname = argv[2];
      }
      int serial_port = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
      if (serial_port < 0) {
        printf("error %d opening %s: %s", errno, portname, strerror (errno));
        return EXIT_FAILURE;
      }
      set_interface_attribs (serial_port, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
      set_blocking (serial_port, 1);                  // set blocking
      //checking bootloader (only for ver > 2.0)
      uint8_t tmp[4] = { 0x21, 0x5A, 0x00, 0xA6 };
      write (serial_port, tmp, 4);
      if (uart_read(serial_port, buf, 25) == EXIT_SUCCESS) {
        buf[7] = 0;
        printf("Detected device: %s.\n", buf);
      }
      else {
        printf("Error: Target not responding.\n");
        return EXIT_FAILURE;
      }
      if (send_command(serial_port, ERASE_ALL, 0, 0) == EXIT_FAILURE) {
        printf("Erasing program memory chip error. \n");
        return EXIT_FAILURE;
      }
      int program_bytes = 0;
      for (int line = 1; fgets(buf, 100, hex_file); line++) {
        if (buf[0] == ':') {
          int data_size = hex2int(buf + 1, 2);
          program_bytes += data_size;
          uint8_t data[3+16] = { 0, hex2int(buf+3, 2), hex2int(buf+5, 2) }; //address and data. first always byte is zero?
          if (hex2int(buf+7, 2) == 0) { // check end of file
            uint8_t checksum = data_size+data[1]+data[2];
            for (int i = 0; i < data_size; i++) {
              data[i+3] = hex2int(buf + 9+2*i, 2);
              checksum += data[i+3];
            }
            if ((uint8_t)(checksum+hex2int(buf + 9 + 2*data_size, 2)) != 0) {
              printf("Check sum error at line %d: %s", line, buf);
              return EXIT_FAILURE;
            }
            if (send_command(serial_port, PROGRAM_BLOCK, data, data_size+3) == EXIT_FAILURE) {
              printf("Error program at line %d\n", line);
              return EXIT_FAILURE;
            }
          }
        }
      }
      printf("Writed %d bytes. \n", program_bytes);
      tmp[0] = 0; tmp[1] = 0; tmp[2] = 0;
      if (send_command(serial_port, RUN_CODE, tmp, 3) == EXIT_FAILURE) {
        printf("Error run user code.\n");
      }
      fclose(hex_file);
      close(serial_port);
    }
    else {
      printf("Cannot open file: %s", argv[1]);
      return EXIT_FAILURE;
    }
  }
  else {
    printf("Usage: adbtldr [hex-file] [optional: serial port name (Default port is /dev/ttyUSB0)] \n");
    printf("Example: adbtldr ld.hex /dev/ttyUSB1\n");
  }
  return EXIT_SUCCESS;
}
