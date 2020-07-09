/*
 * make_cert.c
 *
 *  Created on: 17/07/2018
 *      Author: pm
 */




#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "xtea.h"
#include "uECC.h"

#define use_sha2_256 1
#define use_sha3_256 0

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#if use_sha2_256
	#if use_sha3_256
		#error You can not use both sha digest at the same time
	#endif
#endif

#if use_sha2_256 == 0
	#if use_sha3_256 == 0
		#error You must select a sha digest
	#endif
#endif

#if use_sha2_256
#include "sha256.h"
#define SHA_INIT(a)			sha256_init(a)
#define SHA_UPDATE(a,b,c)	sha256_update(a,b,c)
#define SHA_FINAL(a,b)		sha256_final(a,b)
#define SHA_CTX				SHA256_CTX
#endif

#if use_sha3_256
#include "sha3.h"
#define SHA_INIT(a)			rhash_sha3_256_init(a)
#define SHA_UPDATE(a,b,c)	rhash_sha3_update(a,b,c)
#define SHA_FINAL(a,b)		rhash_sha3_final(a,b)
#define SHA_CTX				sha3_ctx
#endif

uint8_t private[32] = {0};
uint8_t public[64] = {0};
uint8_t hash[32] = {0};

uint8_t IV_crypt[8] = {
		0xf4, 0x3b, 0x75, 0x11, 0x39, 0x89, 0x39, 0xfa
};

typedef struct {
	 unsigned char private_key[32];
	 unsigned char masterpublic_key[64];
	 struct {
		  unsigned char public_key[64];
		  union {
			   struct {
					uint16_t typeBE; // 00 01
					uint8_t snlen;
					unsigned char snr[20]; //P-NET definerer snr som string20
					unsigned char modul[9];
			   };
			   unsigned char payloadfield_size_control[32];
		  };
		  unsigned char signature[64];
	 } crt; //public cert
} devicecert_t;

devicecert_t device_certificate = {0};
uint8_t sig_tmp[64];

static inline void print_sig(uint8_t *sig)
{
	uint8_t i;
	//printf("%s",KGRN);
	for(i=0;i<64;i++){
		printf("%2.2x", *sig++);
		if(i==31)
			printf("\n");
	}
	//printf("%s",KNRM);
	printf("\n");// flush
}


// c style friendly for little endian
static inline void print_public(uint8_t *sig)
{
	uint8_t i;
	printf("static const unsigned char public_signer[64]={\n");
	sig+=31;
	for(i=0;i<(32);i++){
		printf("0x%2.2x", *sig--);
		printf(",");
	}
	printf("\n");
	sig+=64;
	for(i=0;i<(31);i++){
		printf("0x%2.2x", *sig--);
		printf(",");
	}
	printf("0x%2.2x", *sig--);
	printf("};");
	// flush
	printf("\n");
}

static inline void print_hash(uint8_t *hash)
{
	uint8_t i;
	//printf("%s",KGRN);
	for(i=0;i<32;i++)
		printf("%2.2x", *hash++);

	//printf("%s",KNRM);
	printf("\n");// flush
}
static inline void print_seperator(char g)
{
	uint8_t i;

	for(i=0;i<64;i++)
		printf("%c",g);
	// flush
	printf("\n");
}

int load_keys(void)
{
	FILE *masterkeys_fp;
	uint8_t HASH_KEY_result[32];
	uint8_t HASH_KEY_file[32];
	SHA_CTX KEY;

	SHA_INIT(&KEY);
	SHA_UPDATE(&KEY, (uint8_t *)"public_signer", sizeof("public_signer"));
	SHA_FINAL(&KEY,HASH_KEY_result);


	masterkeys_fp = fopen("../../../../keys.pri","r");
	if(masterkeys_fp != NULL){
		fread(public,sizeof(public),1,masterkeys_fp);
		fread(private,sizeof(private), 1, masterkeys_fp);

		decipher_payload_xtea(public,(uint32_t *)HASH_KEY_result, sizeof(public),*(uint64_t *)IV_crypt);
		decipher_payload_xtea(private,(uint32_t *)HASH_KEY_result, sizeof(private),*(uint64_t *)IV_crypt);

		SHA_INIT(&KEY);
		SHA_UPDATE(&KEY, public, sizeof(public));
		SHA_UPDATE(&KEY, private, sizeof(private));
		SHA_FINAL(&KEY,HASH_KEY_result);

		fread(HASH_KEY_file,sizeof(HASH_KEY_file), 1, masterkeys_fp);
		fclose(masterkeys_fp);

		// if hash does not match, someone has altered the file.
		return memcmp(HASH_KEY_file,HASH_KEY_result,sizeof(HASH_KEY_result));
	}

	return 1;
}
// pub[64],pri[32],hash[32]
// keys are crypted and hash is of uncrypted keys
int save_keys(void)
{
	FILE *masterkeys_fp;
	uint8_t HASH_KEY_result[32];
	uint8_t pri[32];
	uint8_t pub[64];
	SHA_CTX KEY;

	SHA_INIT(&KEY);
	SHA_UPDATE(&KEY, (uint8_t *)"public_signer", sizeof("public_signer"));
	SHA_FINAL(&KEY,HASH_KEY_result);

	masterkeys_fp = fopen("../../../../keys.pri","w");
	if(masterkeys_fp != NULL){
		memcpy(pub,public,sizeof(pub));
		memcpy(pri,private,sizeof(pri));

		encipher_payload_xtea(pub,(uint32_t *)HASH_KEY_result, sizeof(pub),*(uint64_t *)IV_crypt);
		encipher_payload_xtea(pri,(uint32_t *)HASH_KEY_result, sizeof(pri),*(uint64_t *)IV_crypt);

		fwrite(pub,sizeof(pub), 1, masterkeys_fp);
		fwrite(pri,sizeof(pri), 1, masterkeys_fp);

		SHA_INIT(&KEY);
		SHA_UPDATE(&KEY, public, sizeof(public));
		SHA_UPDATE(&KEY, private, sizeof(private));
		SHA_FINAL(&KEY,HASH_KEY_result);

		fwrite(HASH_KEY_result,sizeof(HASH_KEY_result), 1, masterkeys_fp);

		fclose(masterkeys_fp);

		return 0;
	}
	return 1;
}


int main(int argc, char **argv)
{
	FILE *masterkeys_fp, *Firmware_fp, *Firmware_out_fp;
	SHA_CTX CTX;
	unsigned char i,k;
	int lok[32];
	const char lok_find[32] = "Replace point";

	char *UUID_nr = argv[1];
	char *FileName = argv[2];

	if (argc != 3) { // Normal error handling
        printf("usage: %s [UUID] file \n", argv[0]);
        return 1;
    }
	// Normal error handling
	device_certificate.crt.snlen = strlen(UUID_nr);
	if(device_certificate.crt.snlen > sizeof(device_certificate.crt.snr)){
		printf("Bad serial number length\n");
		return 1;
	}
	// Get the keys
	masterkeys_fp = fopen("../../../../keys.pri","r");
	if(masterkeys_fp== NULL){
		printf("%s",KRED);
		print_seperator('*');
        printf(	"\tWARNING:\n"
				"\tFile not found: %s \n"
				"\tMaking a new key pair\n", "keys.pri");
		print_seperator('*');
		printf("%s",KNRM);
		// ONLY ONCE
		if (!uECC_make_key(public, private, uECC_secp256r1())) {
			printf("uECC_make_key() failed\n");
			return 1;
		}
        if(save_keys()){
			printf("save_keys() failed\n");
			return 1;
		}
    }
	else{
		if(load_keys()){
			printf("load_keys() failed\n");
			return 1;
		}
	}

	device_certificate.crt.typeBE = 1;
	memcpy(device_certificate.crt.snr,UUID_nr,device_certificate.crt.snlen);
	memcpy(device_certificate.crt.modul,"PD956-v1",sizeof("PD956-v1"));
	memcpy(device_certificate.masterpublic_key,public,sizeof(public));
	if (!uECC_make_key(device_certificate.crt.public_key, device_certificate.private_key, uECC_secp256r1())) {
		printf("uECC_make_key() failed\n");
		return 1;
	}
	// Debug test hardcoded values
	print_seperator('=');
	printf("UUID:    %s \n", device_certificate.crt.snr);
	print_seperator('-');

	// Make HASH of certificate
    SHA_INIT  (&CTX);
	SHA_UPDATE(&CTX, (uint8_t *)&device_certificate.crt.payloadfield_size_control, sizeof(device_certificate.crt.payloadfield_size_control));
	SHA_FINAL (&CTX, hash);

    // Debug display result
	printf("\nHASH of the certificate\n");
    print_hash(hash);
	printf("\n");

	// Sign firmware
	if (!uECC_sign(private, hash, sizeof(hash), device_certificate.crt.signature, uECC_secp256r1())) {
		printf("uECC_sign() failed\n");
		fclose(masterkeys_fp);
		return 1;
	}

	// Debug verify signature
	if (!uECC_verify(public, hash, sizeof(hash), device_certificate.crt.signature, uECC_secp256r1())) {
		printf("uECC_verify() failed\n");
		fclose(masterkeys_fp);
		return 1;
	}
	print_seperator('-');
	printf("\nSignature\n");
	print_sig(device_certificate.crt.signature);
	printf("\n");
	print_seperator('=');

	Firmware_fp=fopen(FileName,"r");
	rewind(Firmware_fp);
	Firmware_out_fp = fopen("temp.elf","w");
	i=0;
	while(1){
		lok[i] = fgetc(Firmware_fp);

		if(lok[i] == EOF)
			break;

		if(lok[i] == lok_find[i])
		{
			i++;
			if(i== 13)
			{
				fwrite((void *)&device_certificate,(sizeof(device_certificate)),1,Firmware_out_fp);
				k = sizeof(device_certificate) - i;
				while(k--)
					lok[i] = fgetc(Firmware_fp);
				i=0;
			}
		}
		else{
			k=0;
			i++;
			while(i--)
				fwrite((void *)&lok[k++],1,1,Firmware_out_fp);
			i=0;
		}
	}

	fclose(Firmware_out_fp);
	fclose(masterkeys_fp);
	
	memset(private,0,sizeof(private));

	remove( FileName);
	rename( "temp.elf", FileName );
	
    return 0;
}
