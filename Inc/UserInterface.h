/*
 * UserInterface.h
 *
 *  Created on: 7 Jan 2017
 *      Author: David
 */

#ifndef SRC_USERINTERFACE_HPP_
#define SRC_USERINTERFACE_HPP_

#include "mks_conf.h"
#include "UTFT.h"
#include "PrinterStatus.h"
#include "Display.h"

typedef uint32_t FirmwareFeatures;

#define noGcodesFolder 0x0001		// gcodes files are in 0:/ not 0:/gcodes
#define noStandbyTemps 0x0002		// firmware does not support separate tool active and standby temperatures
#define noG10Temps     0x0004   	// firmware does not support using G10 to set temperatures
#define noDriveNumber  0x0008		// firmware does not handle drive numbers at the start of file paths
#define noM20M36       0x0010		// firmware does not handle M20 S2 or M36 commands. Use M408 S20 and M408 S36 instead.

const size_t NumColourSchemes = 2;

#if DISPLAY_X == 800

const unsigned int maxHeaters = 7;
# define MAX_AXES   (6)

const PixelNumber margin = 4;
const PixelNumber textButtonMargin = 1;
const PixelNumber iconButtonMargin = 2;
const PixelNumber outlinePixels = 3;
const PixelNumber fieldSpacing = 12;
const PixelNumber statusFieldWidth = 228;
const PixelNumber bedColumn = 160;

const PixelNumber rowTextHeight = 32;	// height of the font we use
const PixelNumber rowHeight = 48;
const PixelNumber moveButtonRowSpacing = 20;
const PixelNumber extrudeButtonRowSpacing = 20;
const PixelNumber fileButtonRowSpacing = 12;
const PixelNumber keyboardButtonRowSpacing = 12;

const PixelNumber speedTextWidth = 105;
const PixelNumber efactorTextWidth = 45;
const PixelNumber percentageWidth = 90;
const PixelNumber e1FactorXpos = 220, e2FactorXpos = 375;

const PixelNumber messageTimeWidth = 90;

const PixelNumber popupY = 345;
const PixelNumber popupSideMargin = 20;
const PixelNumber popupTopMargin = 20;
const PixelNumber keyboardTopMargin = 20;
const PixelNumber popupFieldSpacing = 20;

const PixelNumber axisLabelWidth = 40;
const PixelNumber firstMessageRow = margin + rowHeight;		// adjust this to get a whole number of message rows below the keyboard

const PixelNumber progressBarHeight = 16;
const PixelNumber closeButtonWidth = 66;

const PixelNumber touchCalibMargin = 22;

#else /* DISPLAY_X == 800 */

const unsigned int maxHeaters = 5;
#define MAX_AXES	(3)

const PixelNumber margin = 1;
const PixelNumber textButtonMargin = 1;
const PixelNumber iconButtonMargin = 1;
const PixelNumber outlinePixels = 1;
const PixelNumber fieldSpacing = 1;
const PixelNumber statusFieldWidth = 156;
const PixelNumber bedColumn = 114;

const PixelNumber rowTextHeight = 21;	// height of the font we use
const PixelNumber rowHeight = 27;
const PixelNumber moveButtonRowSpacing = 12;
const PixelNumber extrudeButtonRowSpacing = 12;
const PixelNumber fileButtonRowSpacing = 4;
const PixelNumber keyboardButtonRowSpacing = 6;		// small enough to show 2 lines of messages

const PixelNumber speedTextWidth = 70;
const PixelNumber efactorTextWidth = 30;
const PixelNumber percentageWidth = 60;
const PixelNumber e1FactorXpos = 140, e2FactorXpos = 250;

const PixelNumber messageTimeWidth = 60;

const PixelNumber popupY = 192;
const PixelNumber popupSideMargin = 10;
const PixelNumber popupTopMargin = 10;
const PixelNumber keyboardTopMargin = 8;

const PixelNumber popupFieldSpacing = 10;

const PixelNumber axisLabelWidth = 24;
const PixelNumber firstMessageRow = margin + rowHeight + 3;		// adjust this to get a whole number of message rows below the keyboard

const PixelNumber progressBarHeight = 10;
const PixelNumber closeButtonWidth = 40;

const PixelNumber touchCalibMargin = 6;

#endif /* DISPLAY_X == 800 */

#if LARGE_FONT
# define DEFAULT_FONT	glcd28x32
#else
# define DEFAULT_FONT	glcd17x22
#endif

extern const uint8_t DEFAULT_FONT[]; 				// declare which fonts we will be using

const PixelNumber buttonHeight = rowTextHeight + 4;
const PixelNumber tempButtonWidth = (DISPLAY_X + fieldSpacing - bedColumn)/maxHeaters - fieldSpacing;

const PixelNumber row1 = 0;										// we don't need a top margin
const PixelNumber row2 = row1 + rowHeight - 2;					// the top row never has buttons so it can be shorter
const PixelNumber row3 = row2 + rowHeight;
const PixelNumber row4 = row3 + rowHeight;
const PixelNumber row5 = row4 + rowHeight;
const PixelNumber row6 = row5 + rowHeight;
const PixelNumber row6p3 = row6 /* + (rowHeight/6) */;
const PixelNumber row7 = row6 + rowHeight;
const PixelNumber row7p7 = row7 + 1;
const PixelNumber row8 = row7 + rowHeight;
const PixelNumber row8p7 = row8 + 2;
const PixelNumber row9 = row8 + rowHeight;
const PixelNumber rowTabs = DISPLAY_Y - rowTextHeight - 2;			// place at bottom of screen with no margin
const PixelNumber labelRowAdjust = 2;							// how much to drop non-button fields to line up with buttons

const PixelNumber speedColumn = margin;
const PixelNumber fanColumn = DISPLAY_X/4 + 20;

const PixelNumber pauseColumn = DISPLAY_X/2 + 10 + fieldSpacing;
const PixelNumber resumeColumn = pauseColumn;
const PixelNumber cancelColumn = pauseColumn + (DISPLAY_X - pauseColumn - fieldSpacing - margin)/2 + fieldSpacing;
const PixelNumber babystepColumn = cancelColumn;

const PixelNumber fullPopupWidth = DISPLAY_X - (2 * margin);
const PixelNumber fullPopupHeight = DISPLAY_X - (2 * margin);
const PixelNumber popupBarHeight = buttonHeight + (2 * popupTopMargin);

const PixelNumber tempPopupBarWidth = (5 * fullPopupWidth)/6;
const PixelNumber fileInfoPopupWidth = fullPopupWidth - (4 * margin),
				  fileInfoPopupHeight = (8 * rowTextHeight) + buttonHeight + (2 * popupTopMargin);
const PixelNumber areYouSurePopupWidth = DISPLAY_X - 80,
				  areYouSurePopupHeight = (3 * rowHeight) + (2 * popupTopMargin);

const PixelNumber movePopupWidth = fullPopupWidth;
const PixelNumber movePopupHeight = ((MAX_AXES + 1) * buttonHeight) + (MAX_AXES * moveButtonRowSpacing) + (2 * popupTopMargin);

const PixelNumber extrudePopupWidth = fullPopupWidth;
const PixelNumber extrudePopupHeight = (5 * buttonHeight) + (4 * extrudeButtonRowSpacing) + (2 * popupTopMargin);

const PixelNumber keyboardButtonWidth = DISPLAY_X/5;
const PixelNumber keyboardPopupWidth = fullPopupWidth;
#if DISPLAY_X == 800
const PixelNumber keyButtonWidth = (keyboardPopupWidth - 2 * popupSideMargin)/16;
#else
const PixelNumber keyButtonWidth = (keyboardPopupWidth - 2 * popupSideMargin)/12;
#endif
const PixelNumber keyButtonHStep = (keyboardPopupWidth - 2 * popupSideMargin - keyButtonWidth)/11;
const PixelNumber keyButtonVStep = buttonHeight + keyboardButtonRowSpacing;
const PixelNumber keyboardPopupHeight = (5 * keyButtonVStep) + (2 * keyboardTopMargin) + buttonHeight;
const PixelNumber keyboardPopupY = margin;

const unsigned int numFileColumns = 1;
const unsigned int numFileRows = 5;
const unsigned int numDisplayedFiles = numFileColumns * numFileRows;
const PixelNumber fileListPopupWidth = fullPopupWidth;
const PixelNumber fileListPopupHeight = ((numFileRows + 1) * buttonHeight) + (numFileRows * fileButtonRowSpacing) + (2 * popupTopMargin);

const uint32_t numMessageRows = (rowTabs - margin - rowHeight)/rowTextHeight - 1;
const PixelNumber messageTextX = margin + messageTimeWidth + 2;
const PixelNumber messageTextWidth = DISPLAY_X - margin - messageTextX;

const PixelNumber alertPopupWidth = fullPopupWidth - 6 * margin;
const PixelNumber alertPopupHeight = 3 * rowTextHeight + 2 * popupTopMargin;

const PixelNumber babystepPopupWidth = (2 * fullPopupWidth)/3;
const PixelNumber babystepPopupHeight = 3 * rowHeight + 2 * popupTopMargin;
const PixelNumber babystepRowSpacing = rowHeight;


extern IntegerField *freeMem;
extern TextButton *filenameButtons[];
extern StaticTextField *debugField;
extern StaticTextField *touchCalibInstruction;
extern StaticTextField *messageTextFields[], *messageTimeFields[];
extern TextField *fwVersionField;

namespace UI
{
	extern unsigned int GetNumLanguages();
	extern void CreateFields(uint32_t language, const ColourScheme& colours);
	extern void CheckSettingsAreSaved();
	extern void ShowAxis(size_t axis, bool b);
	extern void UpdateAxisPosition(size_t axis, float fval);
	extern void UpdateCurrentTemperature(size_t heater, float fval) pre(heater < maxHeaters);
	extern void ShowHeater(size_t heater, bool show) pre(heater < maxHeaters);
	extern void UpdateHeaterStatus(size_t heater, int ival) pre(heater < maxHeaters);
	extern void ChangeStatus(PrinterStatus oldStatus, PrinterStatus newStatus);
	extern void UpdateTimesLeft(size_t index, unsigned int seconds);
	extern bool ChangePage(ButtonBase *newTab);
	extern bool DoPolling();
	extern void Spin();
	extern void PrintStarted();
	extern void PrintingFilenameChanged(const char data[]);
	extern void ShowDefaultPage();
	extern void UpdatePrintingFields();
	extern void SetPrintProgressPercent(unsigned int percent);
	extern void UpdateGeometry(unsigned int numAxes, bool isDelta);
	extern void UpdateHomedStatus(int axis, bool isHomed);
	extern void UpdateZProbe(const char data[]);
	extern void UpdateMachineName(const char data[]);
	extern void ProcessAlert(const char data[]);
	extern void UpdateFileGeneratedByText(const char data[]);
	extern void UpdateFileObjectHeight(float f);
	extern void UpdateFileLayerHeight(float f);
	extern void UpdateFileSize(int size);
	extern void UpdateFileFilament(int len);
	extern void UpdateFanPercent(int rpm);
	extern void UpdateActiveTemperature(size_t index, int ival) pre(index < maxHeaters);
	extern void UpdateStandbyTemperature(size_t index, int ival) pre(index < maxHeaters);
	extern void UpdateExtrusionFactor(size_t index, int ival) pre(index + 1 < maxHeaters);
	extern void UpdateSpeedPercent(int ival);
	extern void FirmwareFeaturesChanged(FirmwareFeatures newFeatures);
	extern void ProcessTouch(ButtonPress bp);
	extern void ProcessTouchOutsidePopup(ButtonPress bp)
	pre(bp.IsValid());
	extern void OnButtonPressTimeout();
	extern bool IsDisplayingFileInfo();
	extern void UpdateFilesListTitle(int cardNumber, unsigned int numVolumes, bool isFilesList);
	extern void SetNumTools(unsigned int n);
	extern void FileListLoaded(int errCode);
	extern void EnableFileNavButtons(bool scrollEarlier, bool scrollLater, bool parentDir);
	extern unsigned int GetNumScrolledFiles();
	extern void SetBabystepOffset(float f);
}

#endif /* SRC_USERINTERFACE_HPP_ */
