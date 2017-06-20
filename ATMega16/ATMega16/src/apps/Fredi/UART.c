/*
 * CFile1.c
 *
 * Created: 23.05.2017 10:56:06
 *  Author: Nils
 */ 
/*
   DDRD = 0b11111110;    //EIA 232
   
   
   
   uart_init();  // Serielle Schnittstelle starten
   
   
   // Global Interrupts aktivieren
   printf("Interruptfreigabe erteilt            \r\n");
   sei();
   

   // dann nutzbar mit printf("");

   
   #define F_CPU 16000000                                        //CPU-Speed auf 16Mhz gesetzt
   #define BAUD 38400UL                                         //Baudrate auf 38.4k gesetzt
   #define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)               //clever runden
   #define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))                 //Reale Baudrate
   #define BAUD_ERROR ((BAUD_REAL*1000)/BAUD)                     //Fehler in Promille, 1000 = kein Fehler.
   #include <util/setbaud.h>
   
   static FILE mystdout;
   int count;
   int ncount;
   
   
   int uart_putchar(char c, FILE *stream);                        //Deklaration der primitiven Ausgabefunktion
   //Umleiten der Standardausgabe stdout (Teil 1)
   static FILE mystdout = FDEV_SETUP_STREAM( uart_putchar, NULL, _FDEV_SETUP_WRITE );
   
   
   int uart_putchar( char c, FILE *stream )                    //Definition der Ausgabefunktion
   {
	   if( c == '\n' )
	   uart_putchar( '\r', stream );
	   
	   loop_until_bit_is_set( UCSRA, UDRE );
	   UDR = c;
	   return 0;
   }
   
   void uart_init(void)                                        //Serielle Schnittstelle initialisiert
   {
	   UBRRH = UBRR_VAL >> 8;
	   UBRRL = UBRR_VAL & 0xFF;
	   
	   UCSRB = (1<<RXCIE)|(1<<RXEN) |(1<<TXEN);                  // UART TX einschalten
	   UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);                  // Asynchron 8N1 Betrieb festlegen
	   stdout = &mystdout;
   }
   
   //Serielle Eingabe
   ISR(USART_RXC_vect)
   {
	   // Leeren des UDR und verschieben in die daten1 Variable
	   daten1 = UDR;
	   
	   // Augabe der Empfangenden Daten
	   printf("-%i-",daten1);
	   
	   // Von der Eingabe abhänige Funktion ausführen
	   switch (daten1)
	   {
		   case 0x48: hilfe();        //h
		   break;
		   case 0x68:    hilfe();    //H
		   break;
		   case 0x56:     info();        //v
		   break;
		   case 0x76:    info();        //V
		   break;
		   case 0x31:    profil = 1;    //1
		   profch();
		   break;
		   case 0x32:    profil = 2;    //2
		   profch();
		   break;
		   case 0x33:    profil = 3;    //3
		   profch();
		   break;
		   default:    printf("Falsche Eingabe 1,2 oder 3 oder (H)ilfe oder (V)ersion \?\r");
		   
	   }
	   
   }
*/