/*
 * csprng.h
 *
 *  Created on: 9. maj 2017
 *      Author: pm
 */

#ifndef COMMON_SERVICES_USB_CLASS_CDC_DEVICE_EXAMPLE_CSPRNG_H_
#define COMMON_SERVICES_USB_CLASS_CDC_DEVICE_EXAMPLE_CSPRNG_H_

int csprng_get(unsigned char *dst, int bytes);
void csprng_start(void);

#endif /* COMMON_SERVICES_USB_CLASS_CDC_DEVICE_EXAMPLE_CSPRNG_H_ */
