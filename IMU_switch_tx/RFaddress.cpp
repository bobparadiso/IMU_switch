#include <FlashStorage.h>
#include "Trace.h"
#include "RFaddress.h"

FlashStorage(address_store, AddressData);
AddressData addressData;

//
void DumpAddresses()
{
	tracef("Address Data; src:%d   dst:%d\r\n", addressData.src, addressData.dst);
}

//
void StoreAddresses()
{
	Serial.println("storing addresses:");
	DumpAddresses();

    address_store.write(addressData);	
}

//
void LoadAddresses(uint8_t defaultSrc, uint8_t defaultDst)
{
	addressData = address_store.read();	

	//apply defaults
	if (addressData.signature != VALID_ADDRESS_SIG)
	{
		addressData.src = defaultSrc;
		addressData.dst = defaultDst;
		addressData.signature = VALID_ADDRESS_SIG;
		StoreAddresses();
	}
	
	DumpAddresses();
}
