#ifndef __ADDRESS_H_
#define __ADDRESS_H_

#define VALID_ADDRESS_SIG 0xDEADC0DE

//
struct AddressData
{
	uint8_t src;
	uint8_t dst;
    uint32_t signature;
};

extern AddressData addressData;

void LoadAddresses(uint8_t defaultSrc, uint8_t defaultDst);
void DumpAddresses();
void StoreAddresses();

#endif
