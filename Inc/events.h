/*
 * events.h
 *
 *  Created on: 6 Jan 2017
 *      Author: David
 */

#ifndef SRC_EVENTS_H_
#define SRC_EVENTS_H_

// Event numbers, used to say what we need to do when a field is touched
// *** MUST leave value 0 free to mean "no event"
enum Event : uint8_t
{
	evNull = 0,						// value must match nullEvent declared in display.h

	// Page selection
	evTabControl, evTabPrint, evTabMsg, evTabSetup,

	// Heater control
	evSelectHead, evAdjustActiveTemp, evAdjustStandbyTemp,

	// Control functions
	evMovePopup, evExtrudePopup, evFan, evListMacros,
	evMoveX, evMoveY, evMoveZ, evMoveU, evMoveV, evMoveW,	// these 6 must be contiguous and in this order
	evExtrudeAmount, evExtrudeRate, evExtrude, evRetract,

	// Print functions
	evExtrusionFactor,
	evAdjustFan,
	evAdjustInt,
	evSetInt,
	evListFiles,

	evFile, evMacro,
	evPrint,
	evSendCommand,
	evFactoryReset,
	evAdjustSpeed,

	evScrollFiles, evFilesUp, evMacrosUp, evChangeCard,

	evKeyboard,

	// Setup functions
	evCalTouch, evSetBaudRate, evRotate, evAdjustBaudRate, evSetVolume, evSaveSettings, evReset,

	evYes,
	evCancel,
	evDeleteFile,
	evPausePrint,
	evResumePrint,
	evBabyStepPopup, evBabyStepAmount,

	evKey, evBackspace, evSendKeyboardCommand, evUp, evDown,

	evAdjustLanguage, evSetLanguage,
	evAdjustColours, evSetColours,

	evRestart, evEmergencyStop,

#ifdef OEM_LAYOUT
	evOemJogPage, evOemSetIncrement, evOemAdjustUpDown, evOemDone, evOemSetBrightness, evOemAxisSelect, evOemDistanceSelect, evOemCardSelect
#endif
};

#endif /* SRC_EVENTS_H_ */
