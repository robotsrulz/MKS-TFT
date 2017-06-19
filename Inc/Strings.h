/*
 * Strings.hpp
 *
 *  Created on: 27 Feb 2017
 *      Author: David
 *
 * The encoding used for this file must be UTF-8 to ensure that accented characters are displayed correctly.
 */

#ifndef SRC_STRINGS_HPP_
#define SRC_STRINGS_HPP_

#include "ecv.h"

#define CSTRING const char * const array
#define Newline			"\n"
#define DegreeSymbol	"\u00B0"

struct StringTable
{
	// Language name
	CSTRING languageName;

	// Main page strings
	CSTRING control;
	CSTRING print;
	CSTRING console;
	CSTRING setup;
	CSTRING current;
	CSTRING active;
	CSTRING standby;
	CSTRING move;
	CSTRING extrusion;
	CSTRING macro;
	CSTRING stop;

	// Print page
	CSTRING extruderPercent;
	CSTRING speed;
	CSTRING fan;
	CSTRING timeRemaining;
	CSTRING file;
	CSTRING filament;
	CSTRING layer;
	CSTRING notAvailable;
	CSTRING pause;
	CSTRING babystep;
	CSTRING resume;
	CSTRING cancel;

	// Setup page
	CSTRING volume;
	CSTRING calibrateTouch;
	CSTRING rotateDisplay;
	CSTRING invertDisplay;
	CSTRING theme;
	CSTRING brightnessDown;
	CSTRING brightnessUp;
	CSTRING saveSettings;
	CSTRING clearSettings;
	CSTRING saveAndRestart;

	// Misc
	CSTRING confirmFactoryReset;
	CSTRING confirmRestart;
	CSTRING confirmFileDelete;
	CSTRING areYouSure;
	CSTRING touchTheSpot;
	CSTRING settingsNotSavedText;
	CSTRING restartNeededText;
	CSTRING moveHead;
	CSTRING extrusionAmount;
	CSTRING extrusionSpeed;
	CSTRING extrude;
	CSTRING retract;
	CSTRING babyStepping;
	CSTRING currentZoffset;
	CSTRING message;
	CSTRING messages;
	CSTRING firmwareVersion;

	// File popup
	CSTRING filesOnCard;
	CSTRING macros;
	CSTRING error;
	CSTRING accessingSdCard;
	CSTRING fileName;
	CSTRING fileSize;
	CSTRING layerHeight;
	CSTRING objectHeight;
	CSTRING filamentNeeded;
	CSTRING generatedBy;

	// Printer status strings
	CSTRING statusValues[11];

	// Colour theme names
	CSTRING colourSchemeNames[NumColourSchemes];
};

const StringTable LanguageTables[2] =
{
	// English
	{
		// Language name
		"English",

		// Main page strings
		"Control",
		"Print",
		"Console",
		"Setup",
		"Current" THIN_SPACE DEGREE_SYMBOL "C",
		"Active" THIN_SPACE DEGREE_SYMBOL "C",
		"Standby" THIN_SPACE DEGREE_SYMBOL "C",
		"Move",
		"Extrude",
		"Macro",
		"STOP",

		// Print page
		"Extruder" THIN_SPACE "%",
		"Spd" THIN_SPACE,							// note space at end
		"Fan" THIN_SPACE,								// note space at end
		"Time left: ",
		"file ",							// note space at end
		", filament ",						// note space at end
		", layer ",							// note space at end
		"n/a",
		"Pause",
		"Babystep",
		"Resume",
		"Cancel",

		// Setup page
		"Volume ",							// note space at end
		"Calibrate touch",
		"Rotate display",
		"Invert display",
		"Theme",
		"Brightness -",
		"Brightness +",
		"Save settings",
		"Clear settings",
		"Save & Restart",

		// Misc
		"Confirm factory reset",
		"Confirm restart",
		"Confirm file delete",
		"Are you sure?",
		"Touch the spot",
		"Some settings are not saved!",
		"Touch" THIN_SPACE "Save" THIN_SPACE "&" THIN_SPACE "Restart" THIN_SPACE "to" THIN_SPACE "activate",
		"Move head",
		"Extrude" THIN_SPACE "amount" THIN_SPACE "(mm)",
		"Speed (mm/s)",
		"Extrude",
		"Retract",
		"Babystep",
		"Current Z offset: ",
		"Message",
		"Messages",
		"PanelMKS firmware ",	// note space at end

		// File popup
		"Storage" THIN_SPACE,				// note the space on the end
		"Macros",
		"Error ",						// note the space at the end
		" accessing SD card",			// note the space at the start
		"Filename: ",
		"Size: ",
		"Layer height: ",
		"Object height: ",
		"Filament needed: ",
		"Sliced by: ",

		// Printer status strings
		{
			"Connecting",
			"Idle",
			"Printing",
			"Halted",
			"Starting up",
			"Paused",
			"Busy",
			"Pausing",
			"Resuming",
			"Firmware upload",
			"Changing tool"
		},

		// Theme names
		{
			"Light",
			"Dark"
		}
	},

	{
		// Language name
		"Русский",

		// Main page strings
		"Рулить",
		"Печать",
		"Консоль",
		"Настрой",
		"Текущая" THIN_SPACE DEGREE_SYMBOL "C",
		"Активная" THIN_SPACE DEGREE_SYMBOL "C",
		"Неактивн" THIN_SPACE DEGREE_SYMBOL "C",
		"Двигать",
		"Давить",
		"Макро",
		"СТОП",

		// Print page
		"Экструзия %",
		"Скор" THIN_SPACE, 	       			// note space at end
		"Вент" THIN_SPACE,					// note space at end
		"Осталось: ",
		"файл ",							// note space at end
		", пластик ",						// note space at end
		", слой ",							// note space at end
		"n/a",
		"Пауза",
		"Микрошаг",
		"Продолж",
		"Отмена",

		// Setup page
		"Громкость" THIN_SPACE,				// note space at end
		"Калибровка",
		"Перевернуть",
        "Зеркально",
		"Тема",
		"Яркость -",
		"Яркость +",
		"Сохранить",
		"Сбросить",
		"Сохран/Рестарт",

		// Misc
		"Подтвердить заводской сброс",
		"Подтвердить рестарт",
		"Подтвердить удаление файла",
		"Вы уверены?",
		"Дотроньтесь до точки",
		"Некоторые настройки не сохранены!",
		"Сохран/Рестарт для применения",
		"Двигать по осям",
		"Выдавить" THIN_SPACE "пластик" THIN_SPACE "(мм)",
		"Скорость (мм/с)",
		"Давить",
		"Откат",
		"Микрошаг",
		"Текущее смещение Z: ",
		"Сообщение",
		"Сообщения",
		"PanelMKS версия ",	// note space at end

		// File popup
		"На карте ",				// note the space on the end
		"Макро",
		"Ошибка ",						// note the space at the end
		" доступ к SD карте",			// note the space at the start
		"Файл: ",
		"Размер: ",
		"Высота слоя: ",
		"Высота объекта: ",
		"Количество пластика: ",
		"Слайсер: ",

		// Printer status strings
		{
			"Соединение",
			"Простой",
			"Печать",
			"Сброс",
			"Запуск печати",
			"Остановлен",
			"Занят",
			"Останавливается",
			"Возобновление",
			"Загрузка прошивки",
			"Смена инструмента"
		},

		// Theme names
		{
			"Светлая",
			"Темная"
		}
	},

#if 0	// Spanish not supported yet
	// Spanish
	{
		// Language name
		"Espanol",

		// Main page strings
		"Control",
		"Print",
		"Console",
		"Setup",
		"Current" THIN_SPACE DEGREE_SYMBOL "C",
		"Active" THIN_SPACE DEGREE_SYMBOL "C",
		"Standby" THIN_SPACE DEGREE_SYMBOL "C",
		"Move",
		"Extrusion",
		"Macro",
		"STOP",

		// Print page
		"Extruder" THIN_SPACE "%",
		"Speed ",							// note space at end
		"Fan ",								// note space at end
		"Time left: ",
		"file ",							// note space at end
		", filament ",						// note space at end
		", layer ",							// note space at end
		"n/a",
		"Pause",
		"Baby step",
		"Resume",
		"Cancel",

		// Setup page
		"Volume ",							// note space at end
		"Calibrate touch",
		"Mirror display",
		"Invert display",
		"Theme",
		"Brightness -",
		"Brightness +",
		"Save settings",
		"Clear settings",
		"Save & Restart",

		// Misc
		"Confirm factory reset",
		"Confirm restart",
		"Confirm file delete",
		"Are you sure?",
		"Touch the spot",
		"Some settings are not saved!",
		"Touch Save & Restart to use new settings",
		"Move head",
		"Extrusion amount (mm)",
		"Speed (mm/s)",
		"Extrude",
		"Retract",
		"Baby stepping",
		"Current Z offset: ",
		"Message",
		"Messages",
		"Panel Due firmware version ",	// note space at end

		// File popup
		"Files on card ",				// note the space on the end
		"Macros",
		"Error ",						// note the space at the end
		" accessing SD card",			// note the space at the start
		"Filename: ",
		"Size: ",
		"Layer height: ",
		"Object height: ",
		"Filament needed: ",
		"Sliced by: ",

		// Printer status strings
		{
			"conexión",
			"ocioso",
			"imprimiendo",
			"detuvo",
			"empezando",
			"pausado",
			"ocupado",
			"pausando",
			"reanudando",
			"carga del firmware",
			"herramienta de cambio"
		},

		// Theme names
		{
			"Light",
			"Dark"
		}
	},
#endif
};

#endif /* SRC_STRINGS_HPP_ */
