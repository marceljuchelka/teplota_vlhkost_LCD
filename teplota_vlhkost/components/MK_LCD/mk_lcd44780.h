//-----------------------------------------------------------------------------------------------------------
// *** Obs�uga wy�wietlaczy alfanumerycznych zgodnych z HD44780 ***
//
// - Sterowanie: tryb 4-bitowy
// - Dowolne przypisanie ka�dego sygna�u steruj�cego do dowolnego pinu mikrokontrolera
// - Praca z pinem RW pod��czonym do GND lub do mikrokontrolera (sprawdzanie BusyFLAG - szybkie operacje LCD)
//
//	Biblioteka ver: 1.0
//
// Pliki 			: lcd44780.c , lcd44780.h
// Mikrokontrolery 	: Atmel AVR
// Kompilator 		: avr-gcc
// �r�d�o 			: http://www.atnel.pl
// Data 			: 2019-04-23
// Autor 			: Miros�aw Karda�
//-----------------------------------------------------------------------------------------------------------
#ifndef LCD_H_
#define LCD_H_
#include "../MK_LCD/PCF8574.h"
//----------------------------------------------------------------------------------------
//
//		Parametry pracy sterownika
//
//----------------------------------------------------------------------------------------
// rozdzielczo�� wy�wietlacza LCD (wiersze/kolumny)
#define LCD_ROWS 2		// ilo�� wierszy wy�wietlacza LCD
#define LCD_COLS 16	// ilo�� kolumn wy�wietlacza LCD

// tu ustalamy za pomoc� zera lub jedynki czy sterujemy pinem RW
//	0 - pin RW pod��czony na sta�e do GND
//	1 - pin RW pod��czony do mikrokontrolera
#define USE_RW 1

#define USE_BACKLIGHT		1

//----------------------------------------------------------------------------------------
//
//		Wyb�r trybu pracy I2C / Standard
//
//----------------------------------------------------------------------------------------
// w��czenie obs�ugi magistrali I2C (1 - w��czone, 0 - wy��czone)
//
//	UWAGA! u�ycie magistrali I2C wymaga u�ycia odr�bnej biblioteki MK_I2C : https://sklep.atnel.pl/pl/p/0581_0582-MK-I2C-AVR-Biblioteka-C/244
//
#define USE_I2C		1

#define I2C_KHZ		800			// ustalamy pr�dko�� na magistrali I2C od 50 kHz do 400 kHz (standard to 100 kHz)

// ekspandery PCF8574(A) obs�uguj� standardowo 100 kHz mo�na jednak zwi�ksza� pr�dko�� spokojnie do 250-300 kHz
// natomiast je�li przewody I2C s� bardzo d�ugie mo�na zmniejszy� pr�dko�� do 50 kHz

//----------------------------------------------------------------------------------------
//
//	Ustawienia sprz�towe obs�ugi komunikacji I2C dla Ekspander�w PCF8574 oraz PCF8574A
//
//----------------------------------------------------------------------------------------
// Adres EXPANDERA
//#define PCF8574_LCD_ADDR 0x70	// PCF8574A gdy A0, A1 i A2 --> GND
#define PCF8574_LCD_ADDR 0x4E	// PCF8574  gdy A0, A1 i A2 --> GND

//----------------------------------------------------------------------------------------
//
//		Ustawienia sprz�towe po��cze� sterownika z mikrokontrolerem
//
//----------------------------------------------------------------------------------------
// tu konfigurujemy port i piny do jakich pod��czymy linie D7..D4 LCD
#if !USE_I2C
	#define LCD_D7PORT  A
	#define LCD_D7 6
	#define LCD_D6PORT  A
	#define LCD_D6 5
	#define LCD_D5PORT  A
	#define LCD_D5 4
	#define LCD_D4PORT  A
	#define LCD_D4 3


	// tu definiujemy piny procesora do kt�rych pod��czamy sygna�y RS,RW, E
	#define LCD_RSPORT A
	#define LCD_RS 0

	#define LCD_RWPORT A
	#define LCD_RW 1

	#define LCD_EPORT A
	#define LCD_E 2

#if USE_BACKLIGHT == 1
	#define LCD_LED_PORT  A		// POD�WIETLENIE LCD
	#define LCD_LED 7
#endif

#else
	// Tu definiujemy piny ekspandera do kt�rych pod��czamy sygna�y D7..D4 LCD
	#define LCD_D7 	7
	#define LCD_D6 	6
	#define LCD_D5 	5
	#define LCD_D4 	4

	// tu definiujemy piny ekspandera do kt�rych pod��czamy sygna�y RS,RW, E
	#define LCD_RS 	0
	#define LCD_RW 	1
	#define LCD_E 	2

#if USE_BACKLIGHT == 1
	#define LCD_LED 3		// POD�WIETLENIE LCD
#endif

//	#include "../MK_I2C/mk_i2c.h"

#endif


//------------------------------------------------  koniec ustawie� sprz�towych ---------------


//----------------------------------------------------------------------------------------
//****************************************************************************************
//*																						 *
//*		U S T A W I E N I A   KOMPILACJI												 *
//*																						 *
//*		W��czamy kompilacj� komend u�ywanych lub wy��czamy nieu�ywanych					 *
//*		(dzi�ki temu regulujemy zaj�to�� pami�ci FLASH po kompilacji)					 *
//*																						 *
//*		1 - oznacza W��CZENIE do kompilacji												 *
//*		0 - oznacza wy��czenie z kompilacji (funkcja niedost�pna)						 *
//*																						 *
//****************************************************************************************


#define USE_EEPROM		1


//------------------------------------------------  koniec ustawie�  ---------------


// definicje adres�w w DDRAM dla r�nych wy�wietlaczy
// inne s� w wy�wietlaczach 2wierszowych i w 4wierszowych
#if ( (LCD_ROWS == 4) && (LCD_COLS == 20) )
#define LCD_LINE1 0x00		// adres 1 znaku 1 wiersza
#define LCD_LINE2 0x28		// adres 1 znaku 2 wiersza
#define LCD_LINE3 0x14  	// adres 1 znaku 3 wiersza
#define LCD_LINE4 0x54  	// adres 1 znaku 4 wiersza
#else
#define LCD_LINE1 0x00		// adres 1 znaku 1 wiersza
#define LCD_LINE2 0x40		// adres 1 znaku 2 wiersza
#define LCD_LINE3 0x10  	// adres 1 znaku 3 wiersza
#define LCD_LINE4 0x50  	// adres 1 znaku 4 wiersza
#endif

//Makra uproszczaj�ce obs�ug� magistralii I2C
#define SEND_I2C 		pcf8574_write( PCF8574_LCD_ADDR, mpxLCD )
#define RECEIVE_I2C  	pcf8574_read( PCF8574_LCD_ADDR )

// Makra upraszczaj�ce dost�p do port�w
#if !USE_I2C
	// *** PORT
	#define PORT(x) SPORT(x)
	#define SPORT(x) (PORT##x)
	// *** PIN
	#define PIN(x) SPIN(x)
	#define SPIN(x) (PIN##x)
	// *** DDR
	#define DDR(x) SDDR(x)
	#define SDDR(x) (DDR##x)
#endif


// Komendy steruj�ce
#define LCDC_CLS					0x01
#define LCDC_HOME					0x02
#define LCDC_ENTRY					0x04
	#define LCDC_ENTRYR					0x02
	#define LCDC_ENTRYL					0
	#define LCDC_MOVE					0x01
#define LCDC_ONOFF					0x08
	#define LCDC_DISPLAYON				0x04
	#define LCDC_CURSORON				0x02
	#define LCDC_CURSOROFF				0
	#define LCDC_BLINKON				0x01
#define LCDC_SHIFT					0x10
	#define LCDC_SHIFTDISP				0x08
	#define LCDC_SHIFTR					0x04
	#define LCDC_SHIFTL					0
#define LCDC_FUNC					0x20
	#define LCDC_FUNC8B					0x10
	#define LCDC_FUNC4B					0
	#define LCDC_FUNC2L					0x08
	#define LCDC_FUNC1L					0
	#define LCDC_FUNC5x10				0x04
	#define LCDC_FUNC5x7				0
#define LCDC_SET_CGRAM				0x40
#define LCDC_SET_DDRAM				0x80

enum { _left, _center, _right };


// deklaracje funkcji na potrzeby innych modu��w
void lcd_init(void);								// W��CZONA na sta�e do kompilacji
void lcd_cls(void);									// W��CZONA na sta�e do kompilacji
void lcd_str(char * str);							// W��CZONA na sta�e do kompilacji
void lcd_str_al( uint8_t y, uint8_t x, char * str, uint8_t align );


void lcd_locate(uint8_t y, uint8_t x);				// domy�lnie W��CZONA z kompilacji w pliku lcd.c

void lcd_char(char c);								// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_str_P(const char * str);							// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_str_al_P( uint8_t y, uint8_t x, const char * str, uint8_t align );
void lcd_str_E(char * str);							// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_int( int32_t val );								// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_int_al( uint8_t y, uint8_t x, int32_t val, uint8_t align );
void lcd_long( uint32_t val );
void lcd_long_al( uint8_t y, uint8_t x, uint32_t val, uint8_t align );
void lcd_float(uint8_t y,uint8_t x,float val, uint8_t len,uint8_t align);
void lcd_bin( uint32_t val, uint8_t len );
void lcd_bin_al( uint8_t x, uint8_t y, uint32_t val, uint8_t len, uint8_t align );
void lcd_hex(int32_t val);								// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_hex_al( uint8_t y, uint8_t x, int32_t val, uint8_t align );
void lcd_defchar(uint8_t nr, uint8_t *def_znak);	// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_defchar_P(uint8_t nr, const uint8_t *def_znak);	// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_defchar_E(uint8_t nr, uint8_t *def_znak);	// domy�lnie wy��czona z kompilacji w pliku lcd.c

void lcd_home(void);								// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_cursor_on(void);							// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_cursor_off(void);							// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_blink_on(void);							// domy�lnie wy��czona z kompilacji w pliku lcd.c
void lcd_blink_off(void);							// domy�lnie wy��czona z kompilacji w pliku lcd.c

void lcd_LED( uint8_t enable );							// domy�lnie wy��czona z kompilacji w pliku lcd.c

#endif /* LCD_H_ */
