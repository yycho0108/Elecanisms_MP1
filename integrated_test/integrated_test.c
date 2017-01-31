//#include <p24FJ128GB206.h>
#include <stdint.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "usb.h"
#include "pin.h"
#include "spi.h"

// USB INTERFACE
#define MAX_REG_FUNC 32
typedef void (*ReqFun)(void);
ReqFun registeredFunctions[MAX_REG_FUNC];
uint8_t registeredFlags[MAX_REG_FUNC];

void addRequestHandler(ReqFun req, uint8_t flag);
extern uint8_t CUR_HANDLER_IDX;

extern uint8_t CUR_HANDLER_IDX = 0;

void addRequestHandler(ReqFun req, uint8_t flag){
	registeredFunctions[CUR_HANDLER_IDX] = req;
	registeredFlags[CUR_HANDLER_IDX] = flag;
}

void VendorRequests(void){
	for(int i=0; i<CUR_HANDLER_IDX; ++i){
		if(USB_setup.bRequest == registeredFlags[CUR_HANDLER_IDX]){
			registeredFunctions[i]();
		}
	}
}

// ENCODER INTERFACE
#define ENC_READ_REG 5
extern ReqFun enc_readReg();
extern _PIN *ENC_SCK, *ENC_MISO, *ENC_MOSI, *ENC_NCS;

// ENCODER IMPL
WORD enc_readReg(WORD address) {
    WORD cmd, result;
    cmd.w = 0x4000|address.w; //set 2nd MSB to 1 for a read
    cmd.w |= parity(cmd.w)<<15; //calculate even parity for

    pin_clear(ENC_NCS); // ACTIVE LOW
    spi_transfer(&spi1, cmd.b[1]);
    spi_transfer(&spi1, cmd.b[0]);
    pin_set(ENC_NCS);

    pin_clear(ENC_NCS);
    result.b[1] = spi_transfer(&spi1, 0);
    result.b[0] = spi_transfer(&spi1, 0);
    pin_set(ENC_NCS);
	return result;
	//return (WORD)(result.w & MASK);
}void init_enc(){

    ENC_MISO = &D[1];
    ENC_MOSI = &D[0];
    ENC_SCK = &D[2];
    ENC_NCS = &D[3];

    pin_digitalOut(ENC_NCS);
    pin_set(ENC_NCS);
    spi_open(&spi1, ENC_MISO, ENC_MOSI, ENC_SCK, 2e6, 1);

	addRequestHandler(enc_readReg);
}

int16_t main(void){
    init_clock();
    init_ui();
    init_pin();
	init_timer();
    init_spi();
	init_enc();

	InitUSB();

}
