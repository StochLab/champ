#ifndef B3M_H
#define B3M_H

#include <stdint.h>
#include <stdbool.h>

#define RECV_TIMEOUT_ERR    -1
#define LEN_ERR 	-5
#define RCMD_ERR 	-6
#define CHKSUM_ERR 	-7

// OPERATION MODES
#define NORMAL_MODE 	0
#define FREE_MODE 	2
#define HOLD_MODE 	3

// CONTROL MODES
#define POSITION_MODE 	0
#define VELOCITY_MODE 	4
#define CURRENT_MODE 	8
#define FF_MODE 	12

// PRESET
#define PRESET_POSITION 0
#define PRESET_VELOCITY 1
#define PRESET_TORQUE   2

// Buffer Length
#define BUFFER_LEN 		60
#define ADDRESS_ID 		0x00 // The starting address for reading the memory block for the IDs
#define ADDRESS_PARAM 	0x2A // The starting address for reading the memory block for the Various Parameters

namespace io {
class B3M{
	private:

		int 		fd; // File-descriptor for the device file
		int8_t 		id; // Motor ID

		int verify_response(uint8_t*, uint8_t*, uint8_t);

		int b3mLoad();
		int b3mSave();
		int b3mRead(uint8_t addr, uint8_t len, uint8_t *buf);
		int b3mWrite(uint8_t addr, uint8_t len, uint8_t *buf);
		int b3mReset(uint8_t time);
		int b3mPosition(int16_t pos, int16_t time, int16_t *retPos);
		int b3mMultiWrite(uint8_t addr, uint8_t len, uint8_t *buf);

	public:
		B3M(int8_t id, int fd);
		int8_t readMotorId(uint8_t*);
		int16_t getPos();
		int16_t getCur();
		int16_t getVel();
		int setPos(int16_t pos);
		int setMultiMotorPos(int16_t *pos, uint8_t *id, int *size);
		int setMode(uint8_t mode);
		int setGainPreset(uint8_t gainPreset);
		int getMemoryBlock(uint8_t addr, uint8_t *buf);
}; // class B3M


}; // namespace B3M


#if 0

typedef struct Motor {
	uint8_t     bus; // Bus number
	uint8_t     id; // Motor ID
	int16_t     ref; // Reference value for 0
	int16_t     pos; // Position of motor
	int16_t     vel;
	int16_t     current;
	uint8_t     mode;
	uint8_t     gainPreset;
} sMotor;


int setPos(sMotor m);
int setVel(sMotor m);
int setCurrent(sMotor m);
int setMode(sMotor m);
int setGainPreset(sMotor m);
int setCalibrationOffset(sMotor m);

double getPos();
int getVel(sMotor* m);
int getCurrent(sMotor* m);

void setModePositionControl(sMotor* m);
#endif

#endif // B3M_H
