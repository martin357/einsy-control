#pragma once

#include <inttypes.h>

class SimplePrint {
public:
	SimplePrint();
	void print(uint16_t number, uint16_t denom = 10000, unsigned char filler = ' ');
	void print(uint8_t number, uint8_t denom = 100, unsigned char filler = ' ');
	void print(int16_t number, int16_t denom = 10000, unsigned char filler = ' ');
	void print(int8_t number, int8_t denom = 100, unsigned char filler = ' ');
	void print(float number, float denom = 1000, unsigned char filler = ' ');
	void print(const char*);
	void print_P(const char*);
	void buffer_init(char* buffer_start, uint8_t buffer_size);
	virtual void write(uint8_t c);
	char* get_position();
private:
	char* _buffer_position;
	uint8_t _buffer_size;

};
