/*
 * ramn_msg_types.h -- message type codes for the ESP32 <-> STM32 SPI link.
 *
 * Deliberately free of HAL and project dependencies: the host-test build
 * replaces ramn_telematics.h with a stub, and a second copy of these
 * constants living in that stub is exactly the drift the codes exist to
 * prevent. Both include this file instead.
 */
#ifndef INC_RAMN_MSG_TYPES_H_
#define INC_RAMN_MSG_TYPES_H_

/* Message type codes -- byte 2 of a typed poll response from the ESP32.
 *
 * Every code is >= 0x20 on purpose. Byte 2 of an ordinary CAN poll response
 * is ID[31:24], and an extended CAN identifier caps at 0x1FFFFFFF, so that
 * byte never exceeds 0x1F. Keeping type codes above the ceiling makes byte 2
 * self-describing: one byte says whether this is a CAN frame or a typed
 * message, with no ambiguity to resolve.
 *
 * These were 0x01..0x04 and 0x10..0x11, all inside the identifier range, and
 * the collision was not theoretical. Reading byte 2 as a type shifted every
 * field of a CAN poll response by one; ID 0x100 with DLC 8 became identifier
 * 0x00010008, which the HAL truncates to 0x008, and the misread FLAGS byte
 * set the remote bit, so the frame went out empty. Every cansend from the UI
 * appeared on the bus as ID 0x008 with no payload.
 *
 * The values are defined by the ramn-protocol repository. The host tests
 * assert these equal RAMN_MSG_TYPE_* from its generated vector header, so
 * the two definitions cannot drift apart silently.
 */
#define RAMN_MSG_TYPE_CAN              0x00U
#define RAMN_MSG_TYPE_IMG_START        0x81U
#define RAMN_MSG_TYPE_IMG_CHUNK        0x82U
#define RAMN_MSG_TYPE_IMG_END          0x83U
#define RAMN_MSG_TYPE_IMG_ABORT        0x84U
#define RAMN_MSG_TYPE_DELTA_FRAME      0x90U
#define RAMN_MSG_TYPE_DELTA_FRAME_END  0x91U

/* The lowest legal type code, and the highest byte an extended identifier can
 * put in the same position. The gap between them is the dispatch rule. */
#define RAMN_MSG_TYPE_MIN              0x20U
#define RAMN_MAX_ID_HIGH_BYTE          0x1FU

/* Largest CAN FD payload, in bytes. The DLC byte on the SPI link carries a
 * byte count in this range, NOT an FDCAN DLC enum -- do not pass it through
 * DLCtoUINT8, which indexes a sixteen-entry table. */
#define CAN_MAX_PAYLOAD_BYTES          64U

#endif /* INC_RAMN_MSG_TYPES_H_ */
