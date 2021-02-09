/**
  ******************************************************************************
  * @file           : usbd_midi_if.h
  * @brief          : Header for usbd_midi_if file.
  ******************************************************************************
*/

#ifndef __USBD_MIDI_IF_H
#define __USBD_MIDI_IF_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#define USBMIDIBUFSIZE 2048 //must be power of 2
#define USBMIDIMASK (USBMIDIBUFSIZE-1)
   
#include "usbd_midi.h"
#include "usbd_desc.h"

//circuit buffer for midi rx data
typedef struct
{ 
  uint16_t curidx; //current pointer position
  uint16_t rdidx;  //reading pointer position
  uint8_t buf[USBMIDIBUFSIZE];
}tUsbMidiCable; 

//tUsbMidiCable usbmidicable1; //rx buf for virtual midi cable 1
//tUsbMidiCable usbmidicable2; //rx buf for vortual midi cable 2

extern USBD_MIDI_ItfTypeDef  USBD_Interface_fops_HS;

//Create SysEx buffer
void USBD_AddSysExMessage(uint8_t cable, uint8_t *msg, uint8_t length);

//Create NoteOn buffer
void USBD_AddNoteOn(uint8_t cable, uint8_t ch, uint8_t note, uint8_t vel);

//Create NoteOff buffer
void USBD_AddNoteOff(uint8_t cable, uint8_t ch, uint8_t note);

//Start transfer buffer
void USBD_SendMidiMessages(void);

void OTG_FS_IRQHandler(void);


#ifdef __cplusplus
}
#endif
  
#endif /* __USBD_MIDI_IF_H */
