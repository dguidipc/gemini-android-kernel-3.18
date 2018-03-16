/*********************************************************************
 * FileName:        FUSB300.h
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC32
 * Compiler:        XC32
 * Company:         Fairchild Semiconductor
 *
 * Author           Date          Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * M. Smith         12/04/2014    Initial Version
 *
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * Software License Agreement:
 *
 * The software supplied herewith by Fairchild Semiconductor (the “Company?
 * is supplied to you, the Company's customer, for exclusive use with its
 * USB Type C / USB PD products.  The software is owned by the Company and/or
 * its supplier, and is protected under applicable copyright laws.
 * All rights are reserved. Any use in violation of the foregoing restrictions
 * may subject the user to criminal sanctions under applicable laws, as well
 * as to civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS?CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ********************************************************************/

#ifndef FUSB300_H
#define	FUSB300_H

    /* /////////////////////////////////////////////////////////////////////////// */
    /* Required headers */
    /* /////////////////////////////////////////////////////////////////////////// */
#define TRUE true
#define FALSE false
#define BOOL bool
#define UINT8 u8
#define UINT16 u16
#define UINT32 u32

    /* FUSB300 Register Addresses */
#define regDeviceID     0x01
#define regMode     0x02
#define regControl     0x03
#define regManual    0x04
#define regReset    0x05
#define regMask         0x10
#define regStatus      0x11
#define regType      0x12
#define regInterrupt    0x13

typedef enum {
	USBTypeC_Sink = 0,
	USBTypeC_Source,
	USBTypeC_DRP
} USBTypeCPort;

typedef union {
	UINT8 byte;
	struct {
		unsigned REVISION:3;
		unsigned VERSION:5;
	};
} regDeviceID_t;

typedef union {
	UINT8 byte;
	struct {
		/* Control0 */
		unsigned SOURCE:1;
		unsigned SOURCE_ACC:1;
		unsigned SINK:1;
		unsigned SINK_ACC:1;
		unsigned DRP:1;
		unsigned DRP_ACC:1;
		unsigned :2;
	};
} regModes_t;

typedef union {
	UINT8 byte;
	struct {
		/* Control0 */
		unsigned INT_MASK:1;
		unsigned HOST_CUR:2;
		unsigned :1;
		unsigned DRPTOGGLE:2;
		unsigned :2;
	};
} regControl_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned ERROR_REC:1;
		unsigned DISABLED:1;
		unsigned UNATT_SRC:1;
		unsigned UNATT_SNK:1;
		unsigned :4;
	};
} regManual_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned M_ATTACH:1;
		unsigned M_DETACH:1;
		unsigned M_BC_LVL:1;
		unsigned M_ACC_CH:1;
		unsigned :4;
	};
} regMask_t;

typedef union {
	UINT8 byte;
	struct {
		unsigned SW_RES:1;
		unsigned:7;
	};
} regReset_t;

typedef union {
	UINT8 byte;
	struct {
		/* Status0a */
		unsigned ATTACH:1;
		unsigned BC_LVL:2;
		unsigned VBUSOK:1;
		unsigned ORIENT:2;
		unsigned :2;
	};
} regStatus_t;

typedef struct {
	regDeviceID_t DeviceID;
	regModes_t Mode;
	regControl_t Control;
	regManual_t Manual;
	regReset_t Reset;
	UINT8 dummy;
	regMask_t Mask;
	regStatus_t Status;
} FUSB300reg_t;

/* /////////////////////////////////////////////////////////////////////////// */
/* LOCAL PROTOTYPES */
/* /////////////////////////////////////////////////////////////////////////// */
void InitializeFUSB300Interrupt(BOOL blnEnable);
void InitializeFUSB300Timer(BOOL blnEnable);
void InitializeFUSB300Variables(void);
void InitializeFUSB300(void);
void DisableFUSB300StateMachine(void);
void EnableFUSB300StateMachine(void);
void StateMachineFUSB300(struct usbtypc *typec);
void StateMachineDisabled(void);
void StateMachineErrorRecovery(void);
void StateMachineDelayUnattached(void);
void StateMachineUnattached(void);
void StateMachineAttachWaitSnk(void);
void StateMachineAttachWaitSrc(void);
void StateMachineAttachWaitAcc(void);
void StateMachineAttachedSink(void);
void StateMachineAttachedSource(void);
void StateMachineTryWaitSnk(void);
void StateMachineTrySrc(void);
void StateMachineDebugAccessory(void);
void StateMachineAudioAccessory(void);
void StateMachinePoweredAccessory(void);
void StateMachineUnsupportedAccessory(void);
void SetStateDisabled(void);
void SetStateErrorRecovery(void);
void SetStateDelayUnattached(void);
void SetStateUnattached(void);
void SetStateAttachWaitSnk(void);
void SetStateAttachWaitSrc(void);
void SetStateAttachWaitAcc(void);
void SetStateAttachedSrc(void);
void SetStateAttachedSink(void);
void RoleSwapToAttachedSink(void);
void RoleSwapToAttachedSource(void);
void SetStateTryWaitSnk(void);
void SetStateTrySrc(void);
void SetStateDebugAccessory(void);
void SetStateAudioAccessory(void);
void SetStatePoweredAccessory(void);
void SetStateUnsupportedAccessory(void);
void UpdateSourcePowerMode(void);
void ToggleMeasureCC1(void);
void ToggleMeasureCC2(void);
//CCTermType DecodeCCTermination(void);
//void UpdateSinkCurrent(CCTermType Termination);
void ConfigurePortType(unsigned char Control);
void UpdateCurrentAdvert(unsigned char Current);
void GetFUSB300TypeCStatus(unsigned char abytData[]);
unsigned char GetTypeCSMControl(void);
unsigned char GetCCTermination(void);

BOOL FUSB300Write(unsigned char regAddr, unsigned char length, unsigned char *data);
BOOL FUSB300Read(unsigned char regAddr, unsigned char length, unsigned char *data);

void fusb300_i2c_w_reg8(struct i2c_client *client, u8 addr, u8 var);
u8 fusb300_i2c_r_reg(struct i2c_client *client, u8 addr);
#endif	/* FUSB300_H */
