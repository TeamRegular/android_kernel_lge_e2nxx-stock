/*
 * Copyright LG Electronics (c) 2011
 * All rights reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sw3800.h>

SHA1Context sha;

/*
 * Start SW3800
 *  Define the SHA1 circular left shift macro
 *
 */
#define SHA1CircularShift(bits,word) \
                (((word) << (bits)) | ((word) >> (32-(bits))))

u8 Message_Digest[20] = {0x00, };
u8 Secret_Digest[20];

u8 ReadData[20] = {0x00, };
//uint8_t *RD = &ReadData[0];
int data[360] = {0x00, };
int logic_zero = 0;
int logic_one = 0;

int cover_type = 0;

int SW3800_DETECT;
int SW3800_PULLUP;

// set GPIO output mode
void set_cover_id_pin_output_mode(void)
{
  gpio_tlmm_config(GPIO_CFG(SW3800_DETECT, 0, GPIO_CFG_OUTPUT,
      GPIO_CFG_NO_PULL,GPIO_CFG_10MA), GPIO_CFG_DISABLE);
}

void set_cover_pullup_pin_output_mode(void)
{
  gpio_tlmm_config(GPIO_CFG(SW3800_PULLUP, 0, GPIO_CFG_OUTPUT,
      GPIO_CFG_NO_PULL,GPIO_CFG_10MA), GPIO_CFG_DISABLE);
}

// set GPIO input mode
void set_cover_id_pin_input_mode(void)
{
  gpio_tlmm_config(GPIO_CFG(SW3800_DETECT, 0, GPIO_CFG_INPUT,
      GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);
}

//set GPIO 0
void set_cover_id_pin_low(void)
{
  gpio_set_value(SW3800_DETECT, 0);
 }


//set GPIO 1
void set_cover_id_pin_high(void)
{
  gpio_set_value(SW3800_DETECT, 1);
}

void set_cover_pullup_pin_high(void)
{
  gpio_set_value(SW3800_PULLUP, 1);
}
void set_cover_pullup_pin_low(void)
{
  gpio_set_value(SW3800_PULLUP, 0);
 }

// check GPIO
unsigned char get_cover_id_value(void)
{
  return gpio_get_value(SW3800_DETECT);
}

//#ifdef USE_INTERNAL_RAND
unsigned long int _rand_seed = 1;

/* not required to be thread safe by posix */
int get_random_4byte(void)    // int rand_internal(void);
{
    _rand_seed = _rand_seed * 1103515245 + 12345;
    return (unsigned int)(_rand_seed / 65536) % 32768;
}
//#endif

void cover_challenge_data_init(aka_cover_info *ci)
{
  int random;

  random = get_random_4byte();

  ci->challenge_data[0] = (u8)(random & 0xff);
  ci->challenge_data[1] = (u8)((random & 0xff00) >> 8);
  ci->challenge_data[2] = (u8)((random & 0xff0000) >> 16);
  ci->challenge_data[3] = (u8)((random & 0xff000000) >> 24);

  random = get_random_4byte();

  ci->challenge_data[4] = (u8)(random & 0xff);
  ci->challenge_data[5] = (u8)((random & 0xff00) >> 8);
  ci->challenge_data[6] = (u8)((random & 0xff0000) >> 16);
  ci->challenge_data[7] = (u8)((random & 0xff000000) >> 24);
}


int SW3800_Authentication(void)
{
	aka_cover_info ci;
	cover_challenge_data_init(&ci);
	set_cover_pullup_pin_output_mode();
	set_cover_pullup_pin_high();
	set_cover_id_pin_output_mode();
	set_cover_id_pin_low();
	udelay(1500);
	set_cover_id_pin_high();
	mdelay(7);
	bifTrans_Command(SDA, 0x01);				//SDA command address : 0x01
	udelay(STOP_TAU);
	bifTrans_Command(RRA, 0x04);				//RRA command address : 0x03

	set_cover_id_pin_input_mode();

	cover_type = bifTransReadUint();
	printk("%s : [Slide] cover_type = %x \n ", __func__, cover_type);

	if( cover_type > 0 ){
	} else{
		set_cover_id_pin_output_mode();
		set_cover_id_pin_low();
		printk("%s : [Slide] COVER_SW3800_TYPE_ERR\n ", __func__);
		return COVER_SW3800_TYPE_ERR;
	}

	set_cover_id_pin_output_mode();
	Apply_SHA1(0xE0873BC1, 0x70D9104B,&ci);			//user sectet key 64bit
	udelay(200);
	bifTrans_Command(WRA, 0x10);				//WRA command address : 0x10
	udelay(STOP_TAU);
	bifTrans_Command(WD, ci.challenge_data[0]);		//WD command Challenge Data0 Byte
	udelay(STOP_TAU);
	bifTrans_Command(WD, ci.challenge_data[1]);		//WD command Challenge Data1 Byte
	udelay(STOP_TAU);
	bifTrans_Command(WD, ci.challenge_data[2]);		//WD command Challenge Data2 Byte
	udelay(STOP_TAU);
	bifTrans_Command(WD, ci.challenge_data[3]);		//WD command Challenge Data3 Byte
	udelay(STOP_TAU);
	bifTrans_Command(WD, ci.challenge_data[4]);		//WD command Challenge Data4 Byte
	udelay(STOP_TAU);
	bifTrans_Command(WD, ci.challenge_data[5]);		//WD command Challenge Data5 Byte
	udelay(STOP_TAU);
	bifTrans_Command(WD, ci.challenge_data[6]);		//WD command Challenge Data6 Byte
	udelay(STOP_TAU);
	bifTrans_Command(WD, ci.challenge_data[7]);		//WD command Challenge Data7 Byte
	mdelay(20);
	bifTrans_Command(RBE, 0x30 | 0x01);			//RBE command length : 0x01
	udelay(STOP_TAU);
	bifTrans_Command(RBL, 0x20 | 0x04);			//RBL command length : 0x04
	udelay(STOP_TAU);
	bifTrans_Command(RRA, 0x20);				//RRA command address : 0x20

	set_cover_id_pin_input_mode();

	bifTransReadMultUint8();

	set_cover_id_pin_output_mode();
	set_cover_id_pin_low();
	set_cover_pullup_pin_low();

	return SiliconWorks_Authentication();
}

void bifTrans_Command(u8 Transaction, u8 addr_data)
{
	int i;
	int Inversion_Bit_Cnt=0;
	u8 MIPI_bif_Packet[DATA_WORD]={0x00,};
	int GPIO_flag = 0;
// Transaction Element select
	switch(Transaction){
		case 0x01 ://SDA
			MIPI_bif_Packet[BCF]	= 1;
			MIPI_bif_Packet[nBCF]	= 0;
			MIPI_bif_Packet[D9]	= 1;
			MIPI_bif_Packet[D8]	= 0;
		break;
		case 0x02 ://WRA
			MIPI_bif_Packet[BCF]	= 0;
			MIPI_bif_Packet[nBCF]	= 1;
			MIPI_bif_Packet[D9]	= 1;
			MIPI_bif_Packet[D8]	= 0;
		break;
		case 0x03 ://WD
			MIPI_bif_Packet[BCF]	= 0;
			MIPI_bif_Packet[nBCF]	= 1;
			MIPI_bif_Packet[D9]	= 0;
			MIPI_bif_Packet[D8]	= 0;
		break;
		case 0x04 ://RBE
			MIPI_bif_Packet[BCF]	= 1;
			MIPI_bif_Packet[nBCF]	= 0;
			MIPI_bif_Packet[D9]	= 0;
			MIPI_bif_Packet[D8]	= 0;
		break;
		case 0x05 ://RRA
			MIPI_bif_Packet[BCF]	= 0;
			MIPI_bif_Packet[nBCF]	= 1;
			MIPI_bif_Packet[D9]	= 1;
			MIPI_bif_Packet[D8]	= 1;
		break;
	}
//Payload
	MIPI_bif_Packet[D7] = (addr_data >> 7) & 0x01;
	MIPI_bif_Packet[D6] = (addr_data >> 6) & 0x01;
	MIPI_bif_Packet[D5] = (addr_data >> 5) & 0x01;
	MIPI_bif_Packet[D4] = (addr_data >> 4) & 0x01;
	MIPI_bif_Packet[D3] = (addr_data >> 3) & 0x01;
	MIPI_bif_Packet[D2] = (addr_data >> 2) & 0x01;
	MIPI_bif_Packet[D1] = (addr_data >> 1) & 0x01;
	MIPI_bif_Packet[D0] = (addr_data & 0x01);
// Parity Bits
	MIPI_bif_Packet[P3] =((MIPI_bif_Packet[nBCF] + MIPI_bif_Packet[D9] + MIPI_bif_Packet[D8] + \
		MIPI_bif_Packet[D7] + MIPI_bif_Packet[D6] + MIPI_bif_Packet[D5] + MIPI_bif_Packet[D4]) % 2);

	MIPI_bif_Packet[P2] = ((MIPI_bif_Packet[nBCF] + MIPI_bif_Packet[D9] + MIPI_bif_Packet[D8] + \
		MIPI_bif_Packet[D7] + MIPI_bif_Packet[D3] + MIPI_bif_Packet[D2] + MIPI_bif_Packet[D1]) % 2);

	MIPI_bif_Packet[P1] = ((MIPI_bif_Packet[nBCF] + MIPI_bif_Packet[D9] + MIPI_bif_Packet[D6] + \
		MIPI_bif_Packet[D5] + MIPI_bif_Packet[D3] + MIPI_bif_Packet[D2] + MIPI_bif_Packet[D0]) % 2);

	MIPI_bif_Packet[P0] = ((MIPI_bif_Packet[nBCF] + MIPI_bif_Packet[D8] + MIPI_bif_Packet[D6] + \
		MIPI_bif_Packet[D4] + MIPI_bif_Packet[D3] + MIPI_bif_Packet[D1] + MIPI_bif_Packet[D0]) % 2);
// Inversion Bits
	for(i=2 ; i<INVERSION_CHK ; i++){
		if(MIPI_bif_Packet[i] == 0x01)
			++Inversion_Bit_Cnt;
	}
	if(Inversion_Bit_Cnt > HALF_DATA_BIT){
		for(i=2 ; i<INVERSION_CHK ; i++){
			MIPI_bif_Packet[i] = !(MIPI_bif_Packet[i]);
		}
		MIPI_bif_Packet[INV] = 0x01;
	}
	else{
		MIPI_bif_Packet[INV] = 0x00;
	}

// Transmmit Data Words
	for(i=0 ; i<DATA_WORD ; i++){								//GPIO_Pin Toggle
		if(GPIO_flag == 0)
			set_cover_id_pin_low();
		else
			set_cover_id_pin_high();

		if(MIPI_bif_Packet[i] == 0x00)
			udelay (ONE_TAU);
		else
			udelay (THREE_TAU);

		GPIO_flag = !GPIO_flag;
	}
// BCL Idle
	set_cover_id_pin_high();
}

void bifTransReadMultUint8(void)
{
	int flag = 0;
	int i = 0;
	int j = 1;
	int jj = 0;
	int k = 0;
	int tau_unit = 0;
	unsigned char Temp_ReadData[18]={0x00, };
	logic_zero = 0;
	logic_one = 0;
	for(k=0;k<20;k++){
		ReadData[k]=0x00;
	}

	while(1){
	//udelay(1);
	if(get_cover_id_value() == 0){
		logic_zero++;
			if(flag == 0){
				data[i] = logic_one;
				logic_one = 0;
				i++;
				flag = 1;
			}
		}
	else{
		logic_one++;
			if(flag == 1){
				data[i] = logic_zero;
				logic_zero = 0;
				i++;
				flag = 0;
			}
		}
	if(i == DATA_BIT || logic_one>=ERROR_CNT || logic_zero>=ERROR_CNT)
		break;
	}

	for(jj = 0 ; jj < RESPONSE_AUTH ; jj++){
		tau_unit = (data[j] + data[j+1]) / TRAINING_SEQ;

// Measured time / delay for current high or low phase
		for(i=j; i<j+DATA_WORD ; i++){
			if(((tau_unit*ONE_TAU_MIN)/10 <= data[i]) && (data[i] <= (tau_unit*ONE_TAU_MAX)/10))
				Temp_ReadData[i - j] = 0;
			else if(((tau_unit*THREE_TAU_MIN)/10 <= data[i]) && (data[i] <= (tau_unit*THREE_TAU_MAX)/10))
				Temp_ReadData[i - j] = 1;
		}
// Check the maximum number of physically transmitted logic ones is limited.
		if(Temp_ReadData[INV] == 1){						//Inversion BIT 'High'
			for(i=2 ; i<INVERSION_CHK ; i++){
				Temp_ReadData[i] = !(Temp_ReadData[i]);
			}
		}

// Assemble words
		if(Temp_ReadData[D7] == 0x01)	ReadData[jj] |= 0x80;
		if(Temp_ReadData[D6] == 0x01)	ReadData[jj] |= 0x40;
		if(Temp_ReadData[D5] == 0x01)	ReadData[jj] |= 0x20;
		if(Temp_ReadData[D4] == 0x01)	ReadData[jj] |= 0x10;
		if(Temp_ReadData[D3] == 0x01)	ReadData[jj] |= 0x08;
		if(Temp_ReadData[D2] == 0x01)	ReadData[jj] |= 0x04;
		if(Temp_ReadData[D1] == 0x01)	ReadData[jj] |= 0x02;
		if(Temp_ReadData[D0] == 0x01)	ReadData[jj] |= 0x01;

//		*RD++;
		j += TOTAL_WORD;
	}
}

int bifTransReadUint(void)
{
	int flag = 0;
	int i = 0;
	int j = 1;
	int tau_unit = 0;
	unsigned char user_code = 0;
	unsigned char Temp_ReadData[18]={0x00, };
	logic_zero = 0;
	logic_one = 0;

	while(1){
	//udelay(1);
	if(get_cover_id_value() == 0){
		logic_zero++;
			if(flag == 0){
				data[i] = logic_one;
				logic_one = 0;
				i++;
				flag = 1;
			}
		}
	else{
		logic_one++;
			if(flag == 1){
				data[i] = logic_zero;
				logic_zero = 0;
				i++;
				flag = 0;
			}
		}
	if(i == TOTAL_WORD || logic_one>=ERROR_CNT || logic_zero >= ERROR_CNT)
		break;
	}

	tau_unit = (data[j] + data[j+1]) / TRAINING_SEQ;

// Measured time / delay for current high or low phase
	for(i=j; i<j+DATA_WORD ; i++){
		if(((tau_unit*ONE_TAU_MIN)/10 <= data[i]) && (data[i] <= (tau_unit*ONE_TAU_MAX)/10))
			Temp_ReadData[i - j] = 0;
		else if(((tau_unit*THREE_TAU_MIN)/10 <= data[i]) && (data[i] <= (tau_unit*THREE_TAU_MAX)/10))
			Temp_ReadData[i - j] = 1;
	}

// Check the maximum number of physically transmitted logic ones is limited.
	if(Temp_ReadData[INV] == 1){						//Inversion BIT 'High'
		for(i=2 ; i<INVERSION_CHK ; i++){
			Temp_ReadData[i] = !(Temp_ReadData[i]);
		}
	}

// Assemble words
	if(Temp_ReadData[D7] == 0x01)	user_code |= 0x80;
	if(Temp_ReadData[D6] == 0x01)	user_code |= 0x40;
	if(Temp_ReadData[D5] == 0x01)	user_code |= 0x20;
	if(Temp_ReadData[D4] == 0x01)	user_code |= 0x10;
	if(Temp_ReadData[D3] == 0x01)	user_code |= 0x08;
	if(Temp_ReadData[D2] == 0x01)	user_code |= 0x04;
	if(Temp_ReadData[D1] == 0x01)	user_code |= 0x02;
	if(Temp_ReadData[D0] == 0x01)	user_code |= 0x01;


	return user_code;
}

int SiliconWorks_Authentication(void){
	int i=0;

	for(i=0 ; i<RESPONSE_AUTH ; i++){
		if(ReadData[i] != Message_Digest[i]){
			printk("%s : [Slide] COVER_SW3800_SHA_ERR\n ", __func__);
			return COVER_SW3800_SHA_ERR;
		}
	}
	return cover_type;
}

/*
 *  sha1.c
 *
 *  Description:
 *      This file implements the Secure Hashing Algorithm 1 as
 *      defined in FIPS PUB 180-1 published April 17, 1995.
 *
 *      The SHA-1, produces a 160-bit message digest for a given
 *      data stream.  It should take about 2**n steps to find a
 *      message with the same digest as a given message and
 *      2**(n/2) to find any two messages with the same digest,
 *      when n is the digest size in bits.  Therefore, this
 *      algorithm can serve as a means of providing a
 *      "fingerprint" for a message.
 *
 *  Portability Issues:
 *      SHA-1 is defined in terms of 32-bit "words".  This code
 *      uses <stdint.h> (included via "sha1.h" to define 32 and 8
 *      bit unsigned integer types.  If your C compiler does not
 *      support 32 bit unsigned integers, this code is not
 *      appropriate.
 *
 *  Caveats:
 *      SHA-1 is designed to work with messages less than 2^64 bits
 *      long.  Although SHA-1 allows a message digest to be generated
 *      for messages of any number of bits less than 2^64, this
 *      implementation only works with messages with a length that is
 *      a multiple of the size of an 8-bit character.
 *
 */

/*
 *  Define the SHA1 circular left shift macro
 */


/*
 *  SHA1Reset
 *
 *  Description:
 *      This function will initialize the SHA1Context in preparation
 *      for computing a new SHA1 message digest.
 *
 *  Parameters:
 *      context: [in/out]
 *          The context to reset.
 *
 *  Returns:
 *      sha Error Code.
 *
 */
int SHA1Reset(SHA1Context *context)
{
    if (!context) {
        return shaNull;
    }

    context->Length_Low             = 0;
    context->Length_High            = 0;
    context->Message_Block_Index    = 0;

    context->Intermediate_Hash[0]   = 0x67452301;
    context->Intermediate_Hash[1]   = 0xEFCDAB89;
    context->Intermediate_Hash[2]   = 0x98BADCFE;
    context->Intermediate_Hash[3]   = 0x10325476;
    context->Intermediate_Hash[4]   = 0xC3D2E1F0;

    context->Computed   = 0;
    context->Corrupted  = 0;

    return shaSuccess;
}

/*
 *  SHA1Result
 *
 *  Description:
 *      This function will return the 160-bit message digest into the
 *      Message_Digest array  provided by the caller.
 *      NOTE: The first octet of hash is stored in the 0th element,
 *            the last octet of hash in the 19th element.
 *
 *  Parameters:
 *      context: [in/out]
 *          The context to use to calculate the SHA-1 hash.
 *      Message_Digest: [out]
 *          Where the digest is returned.
 *
 *  Returns:
 *      sha Error Code.
 *
 */
int SHA1Result( SHA1Context *context, u8 Message_Digest[SHA1HashSize], u32 Secret_key0, u32 Secret_key1)
{
    int i;

    if (!context || !Message_Digest) {
        return shaNull;
    }

    if (context->Corrupted) {
        return context->Corrupted;
    }

    if (!context->Computed)
    {
        SHA1PadMessage(context, Secret_key0, Secret_key1);
        for(i=0; i<64; ++i)
        {
            context->Message_Block[i] = 0;			// message may be sensitive, clear it out
        }
        context->Length_Low = 0;    				// and clear length
        context->Length_High = 0;
        context->Computed = 1;
    }

    for(i = 0; i < SHA1HashSize; ++i) {
        Message_Digest[i] = context->Intermediate_Hash[i>>2] >> 8 * ( 3 - ( i & 0x03 ) );
    }

    return shaSuccess;
}

/*
 *  SHA1Input
 *
 *  Description:
 *      This function accepts an array of octets as the next portion
 *      of the message.
 *
 *  Parameters:
 *      context: [in/out]
 *          The SHA context to update
 *      message_array: [in]
 *          An array of characters representing the next portion of
 *          the message.
 *      length: [in]
 *          The length of the message in message_array
 *
 *  Returns:
 *      sha Error Code.
 *
 */
int SHA1Input(SHA1Context *context, const unsigned char *message_array, unsigned length)
{
	if (!length) { return shaSuccess; }
	if (!context || !message_array) { return shaNull; }
	if (context->Computed) {
        context->Corrupted = shaStateError;
		return shaStateError;
    }
	if (context->Corrupted) { return context->Corrupted; }

    while(length-- && !context->Corrupted)
    {
		context->Message_Block[context->Message_Block_Index++] = (*message_array & 0xFF);

		context->Length_Low += 8;

		if (context->Length_Low == 0)
		{
			context->Length_High++;
			if (context->Length_High == 0)
			{
				context->Corrupted = 1;			/* Message is too long */
			}
		}

		if (context->Message_Block_Index == 64)
		{
			SHA1ProcessMessageBlock(context);
		}

		message_array++;
    }

	// Insert
	context->Message_Block_Index = 4;
	context->Message_Block[context->Message_Block_Index+44] = context->Message_Block[context->Message_Block_Index];

	context->Message_Block_Index = 5;
	context->Message_Block[context->Message_Block_Index+44] = context->Message_Block[context->Message_Block_Index];

	context->Message_Block_Index = 6;
	context->Message_Block[context->Message_Block_Index+44] = context->Message_Block[context->Message_Block_Index];

	context->Message_Block_Index = 7;
	context->Message_Block[context->Message_Block_Index+44] = context->Message_Block[context->Message_Block_Index];

    return shaSuccess;
}

/*
 *  SHA1ProcessMessageBlock
 *
 *  Description:
 *      This function will process the next 512 bits of the message
 *      stored in the Message_Block array.
 *
 *  Parameters:
 *      None.
 *
 *  Returns:
 *      Nothing.
 *
 *  Comments:

 *      Many of the variable names in this code, especially the
 *      single character names, were used because those were the
 *      names used in the publication.
 *
 *
 */
void SHA1ProcessMessageBlock(SHA1Context *context)
{
	const uint32_t K[] = {   0x5A827999, 0x6ED9EBA1, 0x8F1BBCDC, 0xCA62C1D6 };	// Constants defined in SHA-1

    int           t;                // Loop counter
    u32      temp;             // Temporary word value
    u32      W[80];            // Word sequence
    u32      A, B, C, D, E;    // Word buffers

    for(t = 0; t < 16; t++)			// Initialize the first 16 words in the array W
    {
        W[t] = context->Message_Block[t * 4] << 24;
        W[t] |= context->Message_Block[t * 4 + 1] << 16;
        W[t] |= context->Message_Block[t * 4 + 2] << 8;
        W[t] |= context->Message_Block[t * 4 + 3];
    }

    for(t = 16; t < 80; t++)
    {
       W[t] = SHA1CircularShift(1,W[t-3] ^ W[t-8] ^ W[t-14] ^ W[t-16]);
    }

    A = context->Intermediate_Hash[0];
    B = context->Intermediate_Hash[1];
    C = context->Intermediate_Hash[2];
    D = context->Intermediate_Hash[3];
    E = context->Intermediate_Hash[4];

    for(t = 0; t < 20; t++)
    {
        temp =  SHA1CircularShift(5,A) + ((B & C) | ((~B) & D)) + E + W[t] + K[0];
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t = 20; t < 40; t++)
    {
        temp = SHA1CircularShift(5,A) + (B ^ C ^ D) + E + W[t] + K[1];
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t = 40; t < 60; t++)
    {
        temp = SHA1CircularShift(5,A) +
               ((B & C) | (B & D) | (C & D)) + E + W[t] + K[2];
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t = 60; t < 80; t++)
    {
        temp = SHA1CircularShift(5,A) + (B ^ C ^ D) + E + W[t] + K[3];
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    context->Intermediate_Hash[0] += A;
    context->Intermediate_Hash[1] += B;
    context->Intermediate_Hash[2] += C;
    context->Intermediate_Hash[3] += D;
    context->Intermediate_Hash[4] += E;

    context->Message_Block_Index = 0;
}

/*
 *  SHA1PadMessage
 *

 *  Description:
 *      According to the standard, the message must be padded to an even
 *      512 bits.  The first padding bit must be a '1'.  The last 64
 *      bits represent the length of the original message.  All bits in
 *      between should be 0.  This function will pad the message
 *      according to those rules by filling the Message_Block array
 *      accordingly.  It will also call the ProcessMessageBlock function
 *      provided appropriately.  When it returns, it can be assumed that
 *      the message digest has been computed.
 *
 *  Parameters:
 *      context: [in/out]
 *          The context to pad
 *      ProcessMessageBlock: [in]
 *          The appropriate SHA*ProcessMessageBlock function
 *  Returns:
 *      Nothing.
 *
 */
void SHA1PadMessage(SHA1Context *context, u32 secret_key, u32 secret_key1)
{
	context->Message_Block_Index = 4;

	context->Message_Block[context->Message_Block_Index++] = 0x74;
	context->Message_Block[context->Message_Block_Index++] = 0x61;
	context->Message_Block[context->Message_Block_Index++] = 0x6D;
	context->Message_Block[context->Message_Block_Index++] = 0x6E;

	// User Secret Key [63:32] (32bit)/////////////////////////////////////
	context->Message_Block[context->Message_Block_Index++] = (secret_key >> 24) & 0xFF;
	context->Message_Block[context->Message_Block_Index++] = (secret_key >> 16) & 0xFF;
	context->Message_Block[context->Message_Block_Index++] = (secret_key >> 8) & 0xFF;
	context->Message_Block[context->Message_Block_Index++] = secret_key & 0xFF;
        ///////////////////////////////////////////////////////////////////////
	context->Message_Block[context->Message_Block_Index++] = 0x69;
	context->Message_Block[context->Message_Block_Index++] = 0x70;
	context->Message_Block[context->Message_Block_Index++] = 0x2D;
	context->Message_Block[context->Message_Block_Index++] = 0x64;

	context->Message_Block[context->Message_Block_Index++] = 0x6F;
	context->Message_Block[context->Message_Block_Index++] = 0x6E;
	context->Message_Block[context->Message_Block_Index++] = 0x67;
	context->Message_Block[context->Message_Block_Index++] = 0x79;

	context->Message_Block[context->Message_Block_Index++] = 0x75;
	context->Message_Block[context->Message_Block_Index++] = 0x73;
	context->Message_Block[context->Message_Block_Index++] = 0x65;
	context->Message_Block[context->Message_Block_Index++] = 0x6F;

	// User Secret Key [31:0] (32bit)/////////////////////////////////////
	context->Message_Block[context->Message_Block_Index++] = (secret_key1 >> 24) & 0xFF;
	context->Message_Block[context->Message_Block_Index++] = (secret_key1 >> 16) & 0xFF;
	context->Message_Block[context->Message_Block_Index++] = (secret_key1 >> 8) & 0xFF;
	context->Message_Block[context->Message_Block_Index++] = secret_key1 & 0xFF;
        //////////////////////////////////////////////////////////////////////
	context->Message_Block[context->Message_Block_Index++] = 0x6E;
	context->Message_Block[context->Message_Block_Index++] = 0x67;
	context->Message_Block[context->Message_Block_Index++] = 0x67;
	context->Message_Block[context->Message_Block_Index++] = 0x75;

	context->Message_Block[context->Message_Block_Index++] = 0x64;
	context->Message_Block[context->Message_Block_Index++] = 0x61;
	context->Message_Block[context->Message_Block_Index++] = 0x65;
	context->Message_Block[context->Message_Block_Index++] = 0x6A;

	context->Message_Block[context->Message_Block_Index++] = 0x65;
	context->Message_Block[context->Message_Block_Index++] = 0x6F;
	context->Message_Block[context->Message_Block_Index++] = 0x6E;
	context->Message_Block[context->Message_Block_Index++] = 0x73;

	context->Message_Block[context->Message_Block_Index++] = 0x69;
	context->Message_Block[context->Message_Block_Index++] = 0x6C;
	context->Message_Block[context->Message_Block_Index++] = 0x69;
	context->Message_Block[context->Message_Block_Index++] = 0x63;

	context->Message_Block[context->Message_Block_Index++] = 0x6F;
	context->Message_Block[context->Message_Block_Index++] = 0x6E;
	context->Message_Block[context->Message_Block_Index++] = 0x77;
	context->Message_Block[context->Message_Block_Index++] = 0x6F;

	context->Message_Block_Index += 4;

	context->Message_Block[context->Message_Block_Index++] = 0x72;
	context->Message_Block[context->Message_Block_Index++] = 0x6B;
	context->Message_Block[context->Message_Block_Index++] = 0x73;
	context->Message_Block[context->Message_Block_Index++] = 0x80;

	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x00;

	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x01;
	context->Message_Block[context->Message_Block_Index++] = 0xB8;

    SHA1ProcessMessageBlock(context);
}

int SHA2Result( SHA1Context *context, u8 Message_Digest[SHA1HashSize], u8 Secret_Digest[SHA1HashSize])
{
    int i;

    if (!context || !Message_Digest)
    {
        return shaNull;
    }

    if (context->Corrupted)
    {
        return context->Corrupted;
    }

    if (!context->Computed)
    {
        SHA2PadMessage(context, Secret_Digest);
        for(i=0; i<64; ++i)
        {
            context->Message_Block[i] = 0;			// message may be sensitive, clear it out
        }
        context->Length_Low = 0;    				// and clear length
        context->Length_High = 0;
        context->Computed = 1;
    }

    for(i = 0; i < SHA1HashSize; ++i)
    {
        Message_Digest[i] = context->Intermediate_Hash[i>>2] >> 8 * ( 3 - ( i & 0x03 ) );
    }

    return shaSuccess;
}

void SHA2PadMessage(SHA1Context *context, u8 Secret_Digest[SHA1HashSize])
{
	context->Message_Block_Index = 4;

	context->Message_Block[context->Message_Block_Index++] = 0x69;
	context->Message_Block[context->Message_Block_Index++] = 0x64;
	context->Message_Block[context->Message_Block_Index++] = 0x31;
	context->Message_Block[context->Message_Block_Index++] = 0x74;

	context->Message_Block[context->Message_Block_Index++] = Message_Digest[0];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[1];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[2];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[3];

	context->Message_Block[context->Message_Block_Index++] = 0x62;
	context->Message_Block[context->Message_Block_Index++] = 0x61;
	context->Message_Block[context->Message_Block_Index++] = 0x74;
	context->Message_Block[context->Message_Block_Index++] = 0x74;

	context->Message_Block[context->Message_Block_Index++] = Message_Digest[4];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[5];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[6];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[7];

	context->Message_Block[context->Message_Block_Index++] = 0x65;
	context->Message_Block[context->Message_Block_Index++] = 0x72;
	context->Message_Block[context->Message_Block_Index++] = 0x79;
	context->Message_Block[context->Message_Block_Index++] = 0x61;

	context->Message_Block[context->Message_Block_Index++] = Message_Digest[8];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[9];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[10];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[11];

	context->Message_Block[context->Message_Block_Index++] = 0x75;
	context->Message_Block[context->Message_Block_Index++] = 0x74;
	context->Message_Block[context->Message_Block_Index++] = 0x68;
	context->Message_Block[context->Message_Block_Index++] = 0x65;

	context->Message_Block[context->Message_Block_Index++] = Message_Digest[12];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[13];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[14];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[15];

	context->Message_Block[context->Message_Block_Index++] = 0x6E;
	context->Message_Block[context->Message_Block_Index++] = 0x74;
	context->Message_Block[context->Message_Block_Index++] = 0x69;
	context->Message_Block[context->Message_Block_Index++] = 0x63;

	context->Message_Block[context->Message_Block_Index++] = Message_Digest[16];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[17];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[18];
	context->Message_Block[context->Message_Block_Index++] = Message_Digest[19];

	context->Message_Block[context->Message_Block_Index++] = 0x61;
	context->Message_Block[context->Message_Block_Index++] = 0x74;
	context->Message_Block[context->Message_Block_Index++] = 0x69;
	context->Message_Block[context->Message_Block_Index++] = 0x6F;

	context->Message_Block_Index += 4;

	context->Message_Block[context->Message_Block_Index++] = 0x6E;
	context->Message_Block[context->Message_Block_Index++] = 0x69;
	context->Message_Block[context->Message_Block_Index++] = 0x63;
	context->Message_Block[context->Message_Block_Index++] = 0x80;

	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x00;

	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x00;
	context->Message_Block[context->Message_Block_Index++] = 0x01;
	context->Message_Block[context->Message_Block_Index++] = 0xB8;

	SHA1ProcessMessageBlock(context);
}

void Apply_SHA1(u32 Secret_key0, u32 Secret_key1, aka_cover_info *ci)
{

    //----------------------------------------------------SW3800 SHA-1 First Input start
    SHA1Reset(&sha);

    SHA1Input(&sha, (const unsigned char *) ci, 8);

    SHA1Result(&sha, Message_Digest, Secret_key0, Secret_key1);

    //----------------------------------------------------SW3800 SHA-1 First Input end
    //----------------------------------------------------SW3800 SHA-1 Second Input start
    SHA1Reset(&sha);

    SHA1Input(&sha, (const unsigned char *) ci, 8);

    SHA2Result(&sha, Message_Digest, Secret_Digest);
}
/* End SW3800 */


MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("sw3800 vailidation driver");
