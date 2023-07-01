/*
Bardzo ważny błąd!... nie połączyłem mas Arduino i ekspandera. Przez to ekspander raz działał a raz nie. PAMIĘTAJ! ZAWSZE SPRAWDZAJ POŁĄCZENIE WSZYSTKICH MAS ZE SOBĄ!
 ----------------------------------------------------------------------------------------------------------------------------------------
Na ekspanderze wyjście plusowe daje bardzo mały prąd. Ledwo świeci dioda. Dlatego jeśli chcemy coś podłączyć do ekspandera to ustawiamy na nim stan niski i sterujemy urządzenie stanem niskim.
Zatem jesli chcemy wykorzystac tranzystory do ekspandera to PNP, albo MOSFETY. NPN nie nadaja sie bo beda sterowane malym pradem zatem nie wyczaruja z tego duzego pradu. nawet jesli 
zrodlo pradu bedzie dawalo wiele Amper to ja wykorzystam tylko troszke, tranzystor wzmacnia prad jaki jest na bramce poprzez wspolczynnik beta.
-------------------------------------------------------------------------------------------------------------------------------------------------
Jak w takim razie sterować tranzystor przez stan niski w ekspanderze?
Myślałem, że: 
Może dzielnik napięcia robić? - nie próbowałem ale dzielnik napięcia to zmniejszanie prądu a nam chodzi o to, by prąd płynął duży przez ten pierwszy tranzystor, aby 
poprzez wzmocnienie drugiego tranzystora otrzymać docelowo bardzo duży prąd na urzadzeniu. Nie próbowałem w każdym razie dzielnika.

Myślałem że można zrobić to tak, że sygnał z ekspandera idzie na NPN a ja z kolektora pociagne sygnal do bazy drugiego tranzystora i zadziala. Ale nie zadziala, tak to nie chula a przynajmniej
ja nie wiem jak to zrobic. Pomysl wydawal sie spoko, sygnal idzie na baze 1 NPN a potem sygnal z kolektora 1 NPN idzie na baze 2 NPN i nim steruje, jest to sygnal wzmocniony. Tak mi sie
wydawalo...nie dziala. 

Po to chyba zostały wymyślone tranzystory NPN i PNP bo pierwsze steruje sie sygnalem wysokim a drugie niskim. Zatem musze poszukac tranzystorow PNP aby expander stanem niskim 
wysterowywał PNP ktory bedzie obslugiwal urzadzenie. A.... chyba tez zadziala MOSFET, bo on nie potrzebuje pradu, dla niego liczy sie napiecie. Zaraz to sprawdze... 
Tak MOSFET DZIALA, poniewaz on na Zrodle "mierzy" napiecie, prad go nie intereusje. Skoro mam 5 volt na wyjsciu expandera to dla niego nie ma znaczenia czy expander daje 0,1A czy 10A.
Po raz kolejny MOSFET ratuje mi tyłek :-)
--------------------------------------------------------------------------------------------------------
Żeby wykorzystać sterowanie PWM na tranzystor to trzeba wykorzystać tranzystor MOSFET.On jest sterowany sygnałem napięcia. Zatem PWM na niego działa.Normalny NPN po prostu działa albo nie,
niezależnie jaki sygnał PWM damy na niego. Kiedy PWM za niski to tranzystor nie przewodzi a potem przewodzi niezależnie od PWM.
Okazało się że sterowane PWM wentylatory piszczą.Oczywiście sygnał idzie z Ardu, wyjście PWM na tranzystor MOSFET. Czyli zachowuja sie tak jak przypuszczalem zwalniaja i rozpedzaja sie,
ale piszcza. 
Natomiast diody led nie piszcza i bardzo ladnie swieca ciemniej, jasniej. Zatem MOSFET robi robote.
----------------------------------------------------------------------------------------------------------
Adresowanie pinow expanderow...to byla zabawa!
Glowilem sie jak adresuje sie w #define poszczegolne piny expandera a to takie proste... Wpisuje sie tylko i wylacznie pin na expanderze czyli np... #define urzadzenie 0..., a nastpenie w 
programie uzywa sie polecen expander.digitalWrite(.........) Obiekt expander, expander_0x21 czy jak go nazwie mowi Ardu gdzie ma szukac pinu zero do ktorego podlaczone
jest urzadzenie ...proste i genialne. Nigdzie tego nie było w necie opisanego. Wyczailem to z kodu ktory znajduje sie tutaj: http://technovade.pl/pcf8574p-ekspander-wyprowadzen-mikrokontrolera.html 
----------------------------------------------------------------------------------------------------------------------------------------------------------------
Przekaznika NIE MOZNA PODLACZYC DO EXPANDERA...poniewaz ekspander daje za maly prad i nie jest w stanie wysterowac przekaznika. Dlatego u mnie robote musi zrobic Arduino
i dlatego mata grzewcza załaczana przez przekaznik musi byc podlaczona do arduino
----------------------------------------------------------------------------------------------
Kolejny raz musialem uzyc MOSFETA, podlaczylem generator wilgoci przez PNP do expandera i nie dzialalo, znowu pomogl MOSFET. Biorac pod uwage ze reaguje on rowniez na PWM
to lepiej kupic mosfety niz PNP NPN, sa bardziej uniwersalne.
--------------------------------------------------------------------------------------------------
Tak jak w projekcie głównie wykorzystuje MOSFETY tak jeśli chodzi o przekaźniki to bardziej sprawdzą się te sterowane stanem niskim, ponieważ zarówno expander jak i arduino są
wysoko prądowe na stanie niskim i wysterują przekaźnik, natomiast expander jest nisko prądowy na stanie wysokim i nie wysteruje przekaźnika. Dlatego warto kupować
przekaźniki STROWANE STANEM NISKIM
---------------------------------------------------------------------------------------------------
W takirm projekcie jak ten zajeszybko zaczyna brakować pinów arduino dlatego wszystko co kupuje z elektroniki powinienem szukać na magistrali I2C, wszelkie czujniki, wyswietlacze,
cokolwiek...WSZYSTKO NA I2C
*/




/*
 PROBLEMY NA ETAPIE TOWRZENIA PŁYTKI:
 -bardzo słabo szło z fotometodą - nie udało się.
 -metoda termo sprawdziła się dużo lepiej, być może lepszy byłby papier kredowy błyszczący. Toner w płynie chyba nie działa. 
 -ustawione żelazko na pierwszą skalę i prasowanie ok 3 minut 

 SCHEMAT:
 -należy bardzo dokładnie przeanalizować schemat, robić go bardzo dokładnie, bo każdy błąd na tym etapie to porażka dalej.
 -robiłem zmiany w schemacie i nie sprawdziłem ścieżek, skończyło się dorabianiem druta do wytrawionej płytki.

 TWORZENIE PŁYTKI:
 -zacznij od odpowiedniego ustawienia elementów, zwróć uwage gdzie mają PLUS gdzie MASĘ, żeby nie wyszło, że wyświetlacz musi byc ustawiony do gory nogami a RTC odwrotnie
 -STWORZ SWOJE FOOTPRINTY i przyloz sie do tego...DUZE POLA LUTOWNICZE!!!, DUZE ODSTEPY MIEDZY SCIEZKAMI!!!, DUZE POLA NA PRZELOTKI!!!
 -dokladnie pomierz urzadzenia, aby nie dotykaly sie, aby bylo miejsce na radiatory, aby wszystko sie miescilo obok siebie
 -wszystko stawiaj NA PODSTAWKACH. Prawie zaden element ma nie być przylutowany do plytki procz podstawek. WSZYSTKO MA BYC WYMIENNE!!!
 P O L A     L U T O W N I C Z E    M A J A    B Y C     O G R O M N E   A  P R Z E R W A   Z   M A S A    P R Z E O G R O M N A    A   ODSTEPY MIEDZY PRZELOTKAMI DUUUUUUZE!!!!
 W S Z Y S T K O      M A     B Y C     W Y M I E N N E !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

 PROBLEMY NA PLYTCE:
 -nie dzialal RTC, uszkodzone byly dwa, na trzy moduly. Akurat ten dobry zniszczylem przy rozlutowywaniu i rzucilem w kat. To on uratowl mi skore potem bo dal sie zlutowac... na szczescie.
 Dodatkowo modul nie dal ustawic sobie czasu i daty bedac przylutowanym do analogowych nozek ardu. Okazalo sie ze musi byc do digital!  Ja na plytce stykowej jakos wgralem mu czas, ale 
 wtedy pewnie podlaczony był do DIGITAL. Z biegiem czasu przelozylem go do ANALOG. Na plytce byl juz na ANALOG, ale dzialal bo mial baterie i pamietal czas. Problem powstal, kiedy przy
 rozlutowywaniu uszkodzilem uklad i wlozylem nowy do nowej plytki. Ten nie mial wgranego czasu, a bedac podlaczonym do ANALOG wywalal czas i date 0/0/0/ godzina 0:0:0

-dziwnie zachowywal sie EKSPANDER. Przekaznik na nim dzialal, ale wentylator i nawilzacz przez transformatory juz nie. Na odpowiednich nozkach pojawial sie sygnal
wysoki w postaci 0,6V zamiast 5V
problemem byly tranzystory, ktore wymienilem.

-bardzo duzo problemow z lutowaniami dlatego UNIKAJ UMIESZKACZNIA PRZELOTEK POD PODSTAWKAMI!!!! pod modułami, podstawkami etc prowadz sciezki ale nie dawaj tam polaczen, przelotek!
 */

#include <LiquidCrystal_I2C.h>             //bilbioteka  wyswietlacza na I2C
#include <DHT.h>                          //biblioteka czujnika temp_wilg
#include <Wire.h>
#include <PCF8574.h>                    //biblioteka expanderow
#include <ThreeWire.h>                 //biblioteka do zdefiniowania portow RTC
#include <RtcDS1302.h>                //biblioteka do obslugi RTC
#include "Adafruit_SGP30.h"          //biblioteka niezbedna do obslugi miernika CO2
#include <avr/wdt.h>                //biblioteka do watchdoga




//-----------------------------------------------------------------definicja portow na expanderze_0x22------------------------------------------------------------------------------------------
#define wentylatorWilgoci                         4               //powietrze z pojemnika z woda
#define generatorWilgoci                          5              //zwiekszanie wilgoci
#define suszarkaPrzekaznik                        6             //grzanie suszarkaPrzekaznik



//-----------------------------------------------------------------definicja portow na expanderze_0x21------------------------------------------------------------------------------------------
#define przyciskUstawieniaParametrow              7
#define przyciskAktTempWilg                       4           //przyciski sterujace
#define przyciskPlus                              6
#define przyciskMinus                             5

#define boozer                                    0       //boozer
#define chlodzeniePeltier                         3      //sygnał na mostek H do chłodzenia


//---------------------------------------------------------------------definicja portow na ARDUINO----------------------------------------------------------------------------------------------
#define rtc_CLK                         A1// CLOCK   nie wiem jak zaadresowane piny expandera wpisac przy towrzeniu obiektu ThreeWire dlatego, musi byc na ardu   
#define rtc_DAT                         A2// DATA
#define rtc_RST                         A3// RESET


#define fotorezystor                    A7//dzielnik napiecia na fotorezystorze aby mierzyc poziom swiatla, musi byc analogowy na ardu
#define dht11                           A0//czujnik wilgotnosci i temperatury...nie wiem jak zadeklarowac adres na expanderze zatem musi pozostac na ardu

//SDA                                   A4 // łączność I2C z wyswietlaczem i expanderami
//SCL                                   A5



#define B                               11
#define R                               10        //sygnał sterujacy z tych pinow idzie na MOSFETY i PWM steruje janoscia poszczegolnych kolorow musi byc na ardu
#define G                               9

#define jasnoscLCD                      3       //podswietlenie wyswietlacza LCD poprzez PWM musi byc na ardu
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

PCF8574 expander_0x21;                               //expander podlaczony + - - co daje adres 0x21
PCF8574 expander_0x22;                              //expander podlaczony - + - co daje adres 0x22                                        
LiquidCrystal_I2C lcd(0x20,16,2);                  //wyswietlacz podlaczony do expandera - - - co daje adres 0x20, rodzaj wyswietlacza 16,2
DHT dht;                                          //deklaracja czujnika temperatury i wilgotnosci
ThreeWire myWire(rtc_DAT,rtc_CLK,rtc_RST);       // DAT(IO), CLK(SCLK), RST(CE) zdefiniowane poniżej
RtcDS1302<ThreeWire> Rtc(myWire);
Adafruit_SGP30 sgp;




int jasnosc = 0;                   //zmienna do ustawienia jasności wyświetlacza
int odczyt_fotorezystor = 0;      //zmienna do zmierzenia światła
bool stan_wyswietlacza = true;

int temperatura_pojemnika=0;
int wilgotnosc_pojemnika=0;     //zmienne wykorzystywane do pomiaru aktualnych warunkow GROWBOXA
int CO2_pojemnika=0;
int CO2_srednia_10pomiarow=0;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>ZADANE PARAMETRY>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
int temperatura_zadana=10;
int wilgotnosc_zadana=85;    //zmienne wykorzystywane w funkcji ustawiania parametrow GROWBOXA
int maxCO2=700;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


int stansuszarkaPrzekaznik=HIGH;             // zmienna wykorzystywana przy przelaczaniu stanu przekaznika suszarki 
                                             //przekaźnik sterowany stanem niskim zatem tutaj na poczatku daje stan wysoki
int stanPeltier=LOW;
int stanWentylatorWilgoci=LOW;

int godzina=0;
int minuta=0;                               //zmienne czasu wykorzystane do sterowania LED
int sekunda=0; 

float stanLed_R=0;
float stanLed_G=0;      //zmienne potrzebne funkcji swiatloLed() stanowiace poziom swiecenia poszczegolnych kolorow sterowanych sygnalem PWM przez tranzystory MOSFET
float stanLed_B=0;

int counter = 0;      //zmienna narzucona mi przez kod dla czujnika CO2, bedzie pracowac w funkcji dotyczacej CO2
int CO2=0;            //to jest moja zmienna, ktora bede wykorzystywal do sterowania CO2

int licznik_wlaczen_suszarki = 0;
int licznik_czujnikaCO2 = 0;

unsigned long aktualnyCzas=0;
unsigned long zapamietanyCzas=0;          //zmienne podstawowa wykorzystywana do odmierzania czasu we wszystkich while, gdzie jest pojedyncze odmierzanie czasu
unsigned long zapamietanyCzasLed_R=0;
unsigned long zapamietanyCzasLed_G=0;     //zmienne niezbedne dla dzialania funkcji swiatloLed()
unsigned long zapamietanyCzasLed_B=0;
unsigned long zapamietanyCzas_LicznikWlaczenSuszarki=0;
unsigned long zapamietanyCzas_Suszarki=0;        //zmienne potrzebna do dzialania w ifie temepratura growboxa za mala
unsigned long zapamietanyCzas_ProgramyLoopa=0;  //zmienna potrzebna do odliczania czasu co ktory beda wlaczane programy w loopie, niektore


unsigned long roznicaCzasu=0;           //zmienna potrzebna do zapisania roznicy miedzy aktualnym czasem a zapamietanym czasem

//..........................................................................................................................................................................................
      //to jest  ogolna formula na wyliczenie absolutnej wilgotnosci, potrzebna czujnikowi CO2 do prawidlowego wskazywania przy wpisanej temperaturze i wilgotnosci, wpis ten jest na dole 
      //w moim programie dwutlenekWegla()
      
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}
//...........................................................................................................................................................................................

 
void setup() {
  wdt_disable();
  Serial.begin(9600);
  expander_0x21.begin(0x21);
  expander_0x22.begin(0x22);
  Rtc.Begin();

  if (! sgp.begin()){
    //Serial.println("Sensor not found :(");
    while (1);
  }
//  Serial.print("Found SGP30 serial #");
//  Serial.print(sgp.serialnumber[0], HEX);
//  Serial.print(sgp.serialnumber[1], HEX);
//  Serial.println(sgp.serialnumber[2], HEX);


 //....................................................................................................................................................................... 
  
  expander_0x21.pinMode(przyciskUstawieniaParametrow, INPUT_PULLUP);//przycisk wejscia w opcje ustawienia parametrow
  expander_0x21.pinMode(przyciskAktTempWilg, INPUT_PULLUP);//przycisk wyswietlenia aktualnych warunkow Growbox
  expander_0x21.pinMode(przyciskPlus,INPUT_PULLUP);//przycisk plus (z jakiejs przyczyny te przyciski nie wykonuja input pullup jak je zadeklaruje jak przyciski powyzej
  expander_0x21.pinMode(przyciskMinus,INPUT_PULLUP);//przycisk minus
  expander_0x21.pinMode(boozer,OUTPUT);
  expander_0x21.pinMode(chlodzeniePeltier,OUTPUT);
        expander_0x22.pinMode(wentylatorWilgoci,OUTPUT);
        expander_0x22.pinMode(generatorWilgoci,OUTPUT);
        expander_0x22.pinMode(suszarkaPrzekaznik,OUTPUT);  


              pinMode(jasnoscLCD,OUTPUT);
              pinMode(R,OUTPUT);
              pinMode(G,OUTPUT);
              pinMode(B,OUTPUT);

  //...............................................................................................................................................................................
//  expander_0x21.digitalWrite(boozer,LOW);
//  delay(100);
  expander_0x21.digitalWrite(boozer,HIGH);

//  expander_0x21.digitalWrite(chlodzeniePeltier,LOW);     //przekaźnik sterowany stanem niskim(ponieważ expander na stanie niskim daje odpowiedni prad)
//      expander_0x22.digitalWrite(wentylatorWilgoci,HIGH);
//      expander_0x22.digitalWrite(suszarkaPrzekaznik,LOW); //przekaźnik sterowany stanem niskim  
//      expander_0x22.digitalWrite(generatorWilgoci,HIGH);
//  
//delay(500);                                                     //NIEKTORE RZECZY WYLACZYLEM BO WKURZALY PRZY CZESTYM WGRYWANIU PROGRAMU
//  
       expander_0x21.digitalWrite(chlodzeniePeltier,HIGH);
       expander_0x22.digitalWrite(generatorWilgoci,LOW);
       expander_0x22.digitalWrite(suszarkaPrzekaznik,HIGH);
       expander_0x22.digitalWrite(generatorWilgoci,LOW);

  
  

  analogWrite(R,50);
  analogWrite(G,50);
  analogWrite(B,50);
  delay(500);                                   //kontrolne zaswiecenie diod
  analogWrite(R,0);
  analogWrite(G,0);
  analogWrite(B,0);
  

//............................................................................................................................................................................
  
  dht.setup(dht11);                                                 //informacja dla biblioteki na ktorym pinie jest dht, u mnie na pinie 10 teraz (dht11 to jest nazwa czujnika)  
  lcd.init();                                                      //uruchomienie wyswietlacza
  lcd.clear();                                                    //Wymazanie wyswietlacza
 
  
}
//################################################################################ L O O P ##################################################################################################
void loop(){ 
      wszystkoLOW();  //program wylacza wszystko co niepotrzebne w loopie

     //Serial.println("poczatek funkcji loop");
     delay(2000);                                             //bez tego delaya program napitala jak głupi, ardu się męczy i poci niepotrzebnie. 
                                                                  //jak już Growbox będzie działał to można to nawet zwiększyć a zwłaszcza jeśli nauczę się robić przerwania
                                                                  //i wtedy naciśnięcie przycisków ustawienia albo wyswietlenia parametrow od razu zadziała. 
                                                                  //tak NAUCZ SIĘ ROBIĆ PRZERWANIA W ARDU. NAUCZ SIĘ TEŻ JAK TO ZROBIĆ ABY KAZDY PODPROGRAM BYL NA INNEJ ZAKLADCE

    pobranieTemperaturyWilgotnosci();
    dwutlenekWegla();
  

     aktualnyCzas=millis();
     if(aktualnyCzas - zapamietanyCzas_ProgramyLoopa>=10000UL){        //dzieki temu zalozeniu  co 10 sekund będą omiatane niektóre programy w loopie, nie musze....
     swiatloLED();                                              //sprawdzenie czy nie nalezy wlaczyc podswietlenie Growboxa
     porownanie_ParametryDocelowe_ParametryZadane();            //sprawdzenie czy parametry Growboxa odpowiadaja parametrom zadanym                                    
     zapamietanyCzas_ProgramyLoopa=aktualnyCzas;               //......używac tego delaya co jest powyzej i powinny szybko reagowac przyciski teraz                                                         
     }                                                                  

    


      

                                                               

     

                                         
                             
        

        
        if(expander_0x21.digitalRead(przyciskAktTempWilg)== LOW){               //dzialanie w przypadku wcisniecia przycisku akt temp wilg Growboxa
                     delay(500);
                     sygnalBoozer();
                     delay(1500);  
                                                                    //Serial.println("weszlismy do warunku if  nacisniecie przycisku akt temp wilg pojemnika");           
                     AktualneParametryGrowbox();
        }        
        if(expander_0x21.digitalRead(przyciskUstawieniaParametrow)==LOW){        //dzialanie w przypadku wcisniecia przycisku ustawienia parametrow Growboxa
                    delay(500);
                    sygnalBoozer();
                    delay(1000);
                                                                    //Serial.println("przycisk ustawienia parametrow zostal nacisniety");
                    UstawianieParametrow(); 
        }       
                                                                    //Serial.println("koniec funkcji loop");



     aktualnyCzas=millis();
     if(aktualnyCzas - zapamietanyCzas_LicznikWlaczenSuszarki>=600000UL){        //dzieki temu zalozeniu  co 1 godzine licznik wlaczen suszarki sie zeruje, suszarka znow moze wykonac cykl
     licznik_wlaczen_suszarki=0;       
     zapamietanyCzas_LicznikWlaczenSuszarki=aktualnyCzas;                                                            
     }        
}
 //##############################################################################################################################################################################################
//**********************************************************************************************************************************************************************************
int pobranieTemperaturyWilgotnosci(){                 //PROGRAM POBIERA Z CZUJNIKA I WYPLUWA TEMPERATURE I WILGOTNOSC W GROWBOXIE
                                               Serial.println("weszlismy do funkcji pobranie temepratury i wilgotnosci");
  
  //delay(dht.getMinimumSamplingPeriod());                           //Odczekanie wymaganego czasu do pobrania informacji z czujnika, wylaczony chyba tez dziala a niezle spowalnia program
  wilgotnosc_pojemnika = dht.getHumidity();                          //pobranie informacji o temperaturze i wilgotnosci
  temperatura_pojemnika = dht.getTemperature();  
                                              Serial.print("wilgotnosc pojemnika: ");
                                              Serial.println(wilgotnosc_pojemnika);
                                              //Serial.print("wilgotnosc zadana: ");
                                              //Serial.println(wilgotnosc_zadana);
                                      
                                              Serial.print("temperatura pojemnika: ");
                                              Serial.println(temperatura_pojemnika);
                                              //Serial.print("temperatura zadana: ");
                                              //Serial.println(temperatura_zadana);
                                              //delay(1500);
                                            
                                              //Serial.println("wychodzimy z funkcji pobranie temperatury");
  }
//*********************************************************************************************************************************************************************************************
void AktualneParametryGrowbox(){                     //PROGRAM WYSWIETLA NA EKRANIE AKTUALNE PARAMETRY PANUJACE W GROWBOXIE
                                             //Serial.println("weszlismy do funkcji AktualneParametryGrowbox");

  pobranieTemperaturyWilgotnosci();          //program pobiera z czujnika temperature i wilgotnosc
  dwutlenekWegla();                          //program pobiera z czujnika poziom CO2
  
  jasnosc_LCD();                            //funkcja ustawiajaca jasnosc wyswietlacza
  analogWrite(jasnoscLCD,jasnosc);          // wlaczenie diody  podswietlajacej LCD sygnalem PWM o wartosci (jasnosc)
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("00 st.C temp.");                //najpierw ustawiamy napis, to 00 na poczatku bedzie zaraz zastapione wynikiem z czujnika
  lcd.setCursor(0,1);                       //WSTEPNE USTAWIENIE NAPISOW NA WYSWIETLACZU
  lcd.print("00 % wilg."); 
  lcd.display();      
  lcd.setCursor(0,0);
  lcd.print(temperatura_pojemnika);         //tutaj nastepuje zastapienie tego 00 wynikiem z czujnika
  lcd.setCursor(0,1);
  lcd.print(wilgotnosc_pojemnika);          //WYSWIETLANIE TEMPERATURY I WILGOTNOSCI W POJEMNIKU
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Poziom CO2 :");
  lcd.setCursor(0,1);                       //WSTEPNE USTAWIENIE NAPISOW NA WYSWIETLACZU
  lcd.print(CO2_pojemnika);                                                
  delay(2000);
  lcd.clear();
  lcd.noDisplay();  
  analogWrite(jasnoscLCD,0);                //wylaczenie diody podwsietlajacej LCD
  

                               //Serial.println("koniec funkcji AktualneParametryGrowbox idziemy do LOOPa");
  loop();
  }
//******************************************************************************************************************************************************************************************
void UstawianieParametrow(){
                                                      //Serial.println("weszlismy do funkcji UstawienieParametrow()");

int ilosc_nacisniec=0;

    jasnosc_LCD();
    analogWrite(jasnoscLCD,jasnosc);                                  // wykonanie funkcji jasnosc() i ustalenie jasnosci wyswietlacza                                  
    lcd.display();
    lcd.clear();    

     while(ilosc_nacisniec==0){
                                                      //Serial.println("jestesmy w petli while dla ilosc nacisniec 1");                                  
    lcd.setCursor(0,0);
    lcd.print("nacisnij");                                         //w tej czesci bedziemy ustawiac temperaturę jaka chcemy trzymac w GROWBOXIE
    lcd.setCursor(0,1);
    lcd.print("przycisk");  
    ilosc_nacisniec=ilosc_nacisniec+1;
    lcd.clear();
    } 
                              
    while(ilosc_nacisniec==1){
                                                      //Serial.println("jestesmy w petli while dla ilosc nacisniec 1");                                  
    lcd.setCursor(0,0);
    lcd.print("Ustaw temp.");                                         //w tej czesci bedziemy ustawiac temperaturę jaka chcemy trzymac w GROWBOXIE
    lcd.setCursor(0,1);
    lcd.print(temperatura_zadana);  

      if(expander_0x21.digitalRead(przyciskPlus)==LOW){
        delay(250);
        temperatura_zadana = temperatura_zadana+1;   
      }else if(expander_0x21.digitalRead(przyciskMinus)==LOW){         //warunki dla przycisniecia przyciskow
        delay(250);
        temperatura_zadana = temperatura_zadana-1;        
      }else if(expander_0x21.digitalRead(przyciskUstawieniaParametrow)==LOW){
        delay(250);
        lcd.clear();
        ilosc_nacisniec=ilosc_nacisniec+1;
      } 
    }
     //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
     while(ilosc_nacisniec==2){
                                                        //Serial.println("jestesmy w petli while dla ilosc nacisniec 2");
       
       lcd.setCursor(0,0);
       lcd.print("Ustaw wilg.");                                       //ustawiamy wilgotnosc docelowa dla GROWBOXA
       lcd.setCursor(0,1);
       lcd.print(wilgotnosc_zadana);  
         
      if(expander_0x21.digitalRead(6)==LOW){        
        delay(250);
                                                         //Serial.println("został wcisniety przycisk zwiekszania ++++ ");
        wilgotnosc_zadana = wilgotnosc_zadana+1;   
      }else if(expander_0x21.digitalRead(5)==LOW){                       //warunki dla przycisniecia przyciskow
        delay(250);
                                                         //Serial.println("został wcisniety przycisk zmniejszania ---");
        wilgotnosc_zadana = wilgotnosc_zadana-1;        
      }else if(expander_0x21.digitalRead(7)==LOW){
        delay(250);
        lcd.clear();
       ilosc_nacisniec=ilosc_nacisniec+1;
      } 
     }
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
      while(ilosc_nacisniec==3){
                                                          //Serial.println("jestesmy w petli while dla ilosc nacisniec 3");
       
      lcd.setCursor(0,0);
      lcd.print("MAX CO2");                                           //ustawiamy CO2 docelowe dla GROWBOXA
      lcd.setCursor(0,1);
      lcd.print(maxCO2);  
     
      if(expander_0x21.digitalRead(6)==LOW){
        delay(250);
        maxCO2 = maxCO2+10;   
      }else if(expander_0x21.digitalRead(5)==LOW){                   //warunki dla przycisniecia przyciskow
        delay(250);
        maxCO2 = maxCO2-10;        
      }else if(expander_0x21.digitalRead(7)==LOW){
        delay(250);
        lcd.clear();
       ilosc_nacisniec=ilosc_nacisniec+1;
      } 
    }
    
    if (ilosc_nacisniec == 4){                            //Serial.println("jestesmy w funkcji if dla ilosc nacisniec 4");      
      lcd.setCursor(0,0);
      lcd.print("ustawiona temp:");
      lcd.setCursor(0,1);              
      lcd.print(temperatura_zadana);
      delay(2000);

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ustawiona wilg:");
      lcd.setCursor(0,1);                                             //wyswietlaja sie ustawione przez nas parametry
      lcd.print(wilgotnosc_zadana);
      delay(2000);

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ustawione CO2:");
      lcd.setCursor(0,1);              
      lcd.print(maxCO2);
      delay(2000);   

      lcd.clear();
      lcd.noDisplay();
                                                                   //wylaczamy wyswietlacz
      analogWrite(jasnoscLCD,0);                                  // wylaczamy podswietlenie wyswietlacza                                          
    }
   loop();                                                       //przechodzimy do LOOP
}

//**********************************************************************************************************************************************************************************************
int jasnosc_LCD(){                                   //DOSTOSOWANIE DIODY PODSWIETLAJACEJ WYSWIETLACZA DO JASNOSCI POMIESZCZENIA
                                                                               //Serial.println("wszedlem do funkcji jasnoscLCD");
  odczyt_fotorezystor = analogRead(fotorezystor);
                                                                                //Serial.print("odczyt z fotorezystora: ");
                                                                               // Serial.println(odczyt_fotorezystor);
  jasnosc = map(odczyt_fotorezystor,0,1000,0,255);                                                  
                                                                               // Serial.print("jasnosc po zmapowaniu: ");
                                                                               // Serial.println(jasnosc);  
  if(jasnosc < 0){
  jasnosc = 0;
  }else if(jasnosc>=0&&jasnosc<85){
    jasnosc = 85;   
  }else if(jasnosc>=85&&jasnosc<170){
    jasnosc = 170;
  }else if(jasnosc>=170){
    jasnosc = 255;
  }
                                                                                   // Serial.print("jasnosc po przejsciu przez moje warunki if: ");
                                                                                   // Serial.println(jasnosc);
                                                                                   // Serial.println("koniec funkcji jasnoscLCD");
  return(jasnosc); 
}
//**********************************************************************************************************************************************************************************************
void sygnalBoozer(){
  expander_0x21.digitalWrite(boozer,LOW);
  delay(10);
  expander_0x21.digitalWrite(boozer,HIGH);
}
//*********************************************************************************************************************************************************************************************

void porownanie_ParametryDocelowe_ParametryZadane(){
//                                                         Serial.println("wejscie do funkcji porownanie ParametryDocelowe_ParametryZadane"); 
   pobranieTemperaturyWilgotnosci();                       //pobranie temperatury i wilgotnosci
   dwutlenekWegla();                                       //tutaj musze dodac funkcje pobrania CO2
                                                           //tutaj jest miejsce na funkcje pobierajaca cokolwiek innego do kontroli                                                                             
//                                                                              Serial.print("temperatura pojemnika: ");
//                                                                              Serial.println(temperatura_pojemnika);
//                                                                              Serial.print("temperatura zadana: ");
//                                                                              Serial.println(temperatura_zadana);
//                                                                              delay(3000);
//                                                                              Serial.print("wilgotnosc pojemnika: ");
//                                                                              Serial.println(wilgotnosc_pojemnika);
//                                                                              Serial.print("wilgotnosc zadana: ");
//                                                                              Serial.println(wilgotnosc_zadana);
//                                                                              delay(5000);
//                                                                              Serial.print("CO2 pojemnika: ");
//                                                                              Serial.println(CO2_pojemnika);
//                                                                              Serial.print("C02 zadane: ");
//                                                                              Serial.println(maxCO2);
//                                                                            delay(1500);
  //...........................................................................................................................................................................................
   
    if((temperatura_pojemnika<(temperatura_zadana-1))&&(licznik_wlaczen_suszarki<10)){                            //TEMPERATURA POJEMNIKA JEST MNIEJSZA NIZ TEMPERATURA ZADANA

//                                          Serial.println("jestem w ifie temepratura pojemnika < niż temperatura zadana");     
    

     lcd.display();                      //musimy oczyscic ekran, bo byc moze jakis inny program cos na nim zostawi. Nie mozemy jednak zrobic tego w while poniwaz funkcja clear()
     lcd.clear();                        //bedzie powodowac miganie ekranu. CLEAR NIE MOZE BYC W SRODKU WHILE
                                                            //Serial.println("jestem w if temperatura_pojemnika<temperatura_zadana-1");                                                    
                                                           
    while((temperatura_pojemnika<temperatura_zadana+1)&&(licznik_wlaczen_suszarki<=10)){         //dopoki temperatura i licznik sa odpowiednie wykonuj while(

//                                                  Serial.print("licznik wlaczen suszarki w WHILE wynosi: ");
//                                                  Serial.println(licznik_wlaczen_suszarki);
      if (licznik_wlaczen_suszarki>10){                                         //jesli licznik wlaczen suszarki przekroczy ...  a ty jestes w while to wyjdz z niego
                                                                                //licznik ten sterowany jest programem wlaczenieSuszarki()
        loop();  
      }
//                                                           Serial.println("wejscie w while temppoj<tempzad");                                                                              
    pobranieTemperaturyWilgotnosci();                                 //pobieraj w petli aktualna temperature aby wiedziec kiedy wyjsc z petli
    
    analogWrite(jasnoscLCD,50);                                     
    lcd.display();
    lcd.setCursor(0,0);
    lcd.print("temp poj<zad");
                          
     int roznica;                        //zmienna potrzebna po to aby wybrnac z problemu pozostawania zera na ekranie kiedy roznica miedzy wartosciami spadnie ponizej 10
                                         //czyli bylo na przyklad 11 potem 10 potem jest 90 zamiast 9
                                         //nie mozna uzyc lcd.clear() poniewaz czyszczenie ekranu  tak czeste powoduje slabe wyswietlanie obrazu
                          
     roznica = (temperatura_zadana - temperatura_pojemnika);
        if(roznica>=10){
           lcd.setCursor(0,1);
           lcd.print(roznica);
        }else{
           lcd.setCursor(0,1);
           lcd.print("00");
           lcd.setCursor(1,1);
           lcd.print(roznica);
          }
                          
          
                                 
        
        
                                 
                      
          expander_0x22.digitalWrite(generatorWilgoci,HIGH); //wlacz generator wilgoci niech juz robi w pojemniku chmury
                            
          aktualnyCzas=millis();
          if((aktualnyCzas - zapamietanyCzas)>=8000UL){
              stanWentylatorWilgoci = HIGH;
              expander_0x22.digitalWrite(wentylatorWilgoci,stanWentylatorWilgoci);      //pracy szuszarki bedzie towarzyszyc ta sama sekwencja co przy utracie wilgoci ponieważ suszarka
              delay(2000);                                                              //wydmuchuje powietrze z Growboxa i po jej dzialaniu poziom wilgoci mocno spada.
              stanWentylatorWilgoci = LOW;                                              //zatem podczas dzialania suszarki od razu dorzucam dzialanie wentylatora wilgoci i generatora wilgoci
              expander_0x22.digitalWrite(wentylatorWilgoci,stanWentylatorWilgoci); 
              zapamietanyCzas = aktualnyCzas;                  
            }

            aktualnyCzas=millis();
            if(aktualnyCzas - zapamietanyCzas_Suszarki>=20000UL){      //dzieki temu zalozeniu program wlaczenie suszarki powinien wlaczac sie co 15 sekund                      
          
                wlaczenieSuszarki();    //czas wlaczenia suszarki i ilosc cykli wlaczenia suszarki sterowane przez program wlaczenieSuszarki()
           
                wylaczenieSuszarki();                                              
             zapamietanyCzas_Suszarki = aktualnyCzas;
             }

            

     
          if(expander_0x21.digitalRead(przyciskUstawieniaParametrow)==LOW){
          sygnalBoozer();
                                                                      //Serial.println("przycisk ustawienia parametrow zostal nacisniety");                                                                      
          delay(1500);
          UstawianieParametrow();
         }else if(expander_0x21.digitalRead(przyciskAktTempWilg)== LOW){
          sygnalBoozer();  
                                                                     //Serial.println("weszlismy do warunku if  nacisniecie przycisku akt temp wilg pojemnika");
          delay(1500);                                                           
          AktualneParametryGrowbox();
                              }
                                                                     //Serial.print("temperatura pojemnika: ");
                                                                     //Serial.println(temperatura_pojemnika);
                                                                     //Serial.print("temperatura zadana: ");
                                                                     //Serial.println(temperatura_zadana);
                                                                     //delay(2000);
                                                                     //Serial.println("koniec while");
         }     
         lcd.noDisplay();                                                 //wylacz wyswietlacz
         loop();                                                          //idź do LOOPa, nie sprawdzaj innych parametrow od razu, w LOOPie bedziesz musial przejsc przez wszystkoLow(), dzieki temu nie bedzie sytuacji
                                                                         //ze podczas bardzo szybkiej zmiany parametrow, np. temperatury, przeskoczysz do innego while, np. od wilgotnosci a mata grzewcza i wentylator nadal 
                                                                         //beda ON.
 }


////...........................................................................................................................................................................................
  
   else if(temperatura_pojemnika>(temperatura_zadana+5)){                           //TEMPERATURA POJEMNIKA JEST WIEKSZA NIZ TEMPERATURA ZADANA
                                                                     //Serial.println("jestem w if temperatura_pojemnika>temperatura_zadana+2");
   lcd.display();                      //musimy oczyscic ekran, bo byc moze jakis inny program cos na nim zostawi. Nie mozemy jednak zrobic tego w while poniwaz funkcja clear()
   lcd.clear();                        //bedzie powodowac miganie ekranu. CLEAR NIE MOZE BYC W SRODKU WHILE
  
                                
         
    while(temperatura_pojemnika>temperatura_zadana){ 
        
      pobranieTemperaturyWilgotnosci();
      analogWrite(jasnoscLCD,50); 
      lcd.display();
      lcd.setCursor(0,0);
      lcd.print("temp.poj>zad");

                              
      int roznica;                                          //zmienna potrzebna po to aby wybrnac z problemu pozostawania zera na ekranie kiedy roznica miedzy wartosciami spadnie ponizej 10
                                                            //czyli bylo na przyklad 11 potem 10 potem jest 90 zamiast 9
                                                            //nie mozna uzyc lcd.clear() poniewaz czyszczenie ekranu  tak czeste powoduje slabe wyswietlanie obrazu
                        
     roznica = (temperatura_pojemnika - temperatura_zadana);
     if(roznica>=10){
     lcd.setCursor(0,1);
     lcd.print(roznica);
     }else{
     lcd.setCursor(0,1);
     lcd.print("00");
     lcd.setCursor(1,1);
     lcd.print(roznica);
     }

     aktualnyCzas=millis();
     if(aktualnyCzas - zapamietanyCzas>=1000UL){                                //dzieki temu zalozeniu Peltier powinien wlaczac sie i wylaczac co 1 sekund (docelowo np 60s.)
     stanPeltier = !stanPeltier;                                               //zmiana stanu... 
                      
     expander_0x21.digitalWrite(chlodzeniePeltier,stanPeltier);                                                    
     //expander_0x22.digitalWrite(wentylatorWilgoci,HIGH);                   //wspomaganie wentylatorem wilgoci, pewnie bedzie dawal chlodniejsze powietrze 
       
     zapamietanyCzas=aktualnyCzas;                                                            
     }        
      
     

    if(expander_0x21.digitalRead(przyciskUstawieniaParametrow)==LOW){
    sygnalBoozer();
                                                                    //Serial.println("przycisk ustawienia parametrow zostal nacisniety");
    delay(1500);
    UstawianieParametrow();
    }else if(expander_0x21.digitalRead(przyciskAktTempWilg)== LOW){
    sygnalBoozer();  
                                                                   //Serial.println("weszlismy do warunku if  nacisniecie przycisku akt temp wilg pojemnika"); // NACISNIECIE PRZYCISKU AKTUALNYCH WARUNKOW W GROWBOX
    delay(1500);
    AktualneParametryGrowbox();
    }
   }
   lcd.noDisplay();
   loop();    
 }    
  //...........................................................................................................................................................................................
  

 if (wilgotnosc_pojemnika>=wilgotnosc_zadana+9){                      //WILGOTNOSC POJEMNIKA JEST WIESZKA NIZ WILGOTNOSC ZADANA
                                                                      //Serial.println("wilgotnosc pojemnika wieksza niz wilgotnosc zadana + 4");   

    lcd.display();                      //musimy oczyscic ekran, bo byc moze jakis inny program cos na nim zostawi. Nie mozemy jednak zrobic tego w while poniwaz funkcja clear()
    lcd.clear();                        //bedzie powodowac miganie ekranu. CLEAR NIE MOZE BYC W SRODKU WHILE
                                                                                                           
   
   while(wilgotnosc_pojemnika>wilgotnosc_zadana){
    
   pobranieTemperaturyWilgotnosci();
                                                                     //Serial.println("robie while'a wilgotnosc pojemnika wieksza niz wilgotnosc zadana");
   analogWrite(jasnoscLCD,50); //moj zamysl - kiedy dziala while to ekranik swieci sie i widze zmiane parametrow
   lcd.display();  
   lcd.setCursor(0,0);
   lcd.print("wilg poj>zad");
     
int roznica;                                         //zmienna potrzebna po to aby wybrnac z problemu pozostawania zera na ekranie kiedy roznica miedzy wartosciami spadnie ponizej 10
                                                    //czyli bylo na przyklad 11 potem 10 potem jest 90 zamiast 9
                                                   //nie mozna uzyc lcd.clear() poniewaz czyszczenie ekranu  tak czeste powoduje slabe wyswietlanie obrazu
  roznica = (wilgotnosc_pojemnika - wilgotnosc_zadana);
                                                                  //Serial.println(roznica);
  if(roznica>=10){
  lcd.setCursor(0,1);
  lcd.print(roznica);
  }else{
  lcd.setCursor(0,1);
  lcd.print("00");
  lcd.setCursor(1,1);
  lcd.print(roznica);
  }                 
                                                                          //Serial.print("wilgotnosc pojemnika: ");
                                                                          //Serial.println(wilgotnosc_pojemnika);
                                                                          //Serial.print("wilgotnosc zadana: ");
                                                                          //Serial.println(wilgotnosc_zadana);
                                                                          //delay(1500);

//   aktualnyCzas=millis();
//   if(aktualnyCzas - zapamietanyCzas>=5000UL){                    //dzieki temu zalozeniu wentylator wilgoci powinien wlaczac sie co 5s na 10s
     stanWentylatorWilgoci = HIGH;                       
     expander_0x22.digitalWrite(wentylatorWilgoci,stanWentylatorWilgoci);
//   zapamietanyCzas=aktualnyCzas;
//   }  

     if(expander_0x21.digitalRead(przyciskUstawieniaParametrow)==LOW){
        sygnalBoozer();
                                                            //Serial.println("przycisk ustawienia parametrow zostal nacisniety");
       delay(1500);
       UstawianieParametrow();
       }else if(expander_0x21.digitalRead(przyciskAktTempWilg)== LOW){
       sygnalBoozer();  
                                                          //Serial.println("weszlismy do warunku if  nacisniecie przycisku akt temp wilg pojemnika"); // NACISNIECIE PRZYCISKU AKTUALNYCH WARUNKOW W GROWBOX
      delay(1500);
      AktualneParametryGrowbox();
      }
     }   
   lcd.noDisplay(); 
   loop();  
   }  
  //...........................................................................................................................................................................................
   
  else 
  if (wilgotnosc_pojemnika<=(wilgotnosc_zadana-2)){                        //WILGOTNOSC POJEMNIKA JEST MNIEJSZA NIZ WILGOTNOSC ZADANA
//                                                                  Serial.println("wilgotnosc pojemnika mniejsza niz wilgotnosc zadana -2");  
                                                                                                        
    lcd.display();                      //musimy oczyscic ekran, bo byc moze jakis inny program cos na nim zostawi. Nie mozemy jednak zrobic tego w while poniwaz funkcja clear()
    lcd.clear();                        //bedzie powodowac miganie ekranu. CLEAR NIE MOZE BYC W SRODKU WHILE
    
    while(wilgotnosc_pojemnika<wilgotnosc_zadana){
      
     pobranieTemperaturyWilgotnosci();
      
      analogWrite(jasnoscLCD,50); //moj zamysl - kiedy dziala while to ekranik swieci sie i widze zmiane parametrow
      lcd.display();
      lcd.setCursor(0,0);
      lcd.print("wilg poj<zad");
                                      
 int roznica;                                                //zmienna potrzebna po to aby wybrnac z problemu pozostawania zera na ekranie kiedy roznica miedzy wartosciami spadnie ponizej 10
                                                           //czyli bylo na przyklad 11 potem 10 potem jest 90 zamiast 9
                                                          //nie mozna uzyc lcd.clear() poniewaz czyszczenie ekranu  tak czeste powoduje slabe wyswietlanie obrazu
                          
     roznica = (wilgotnosc_zadana - wilgotnosc_pojemnika);
                                                              //Serial.println(roznica);
     if(roznica>=10){
        lcd.setCursor(0,1);
        lcd.print(roznica);
     }else{
        lcd.setCursor(0,1);
        lcd.print("00");
        lcd.setCursor(1,1);
        lcd.print(roznica);
        }
       
        aktualnyCzas=millis();
        if((aktualnyCzas - zapamietanyCzas)>=10000UL){                                  //dzieki temu zalozeniu wentylatorWilgoci powinien wlaczac sie i wylaczac co 10 sekund na dwie sekundy
        stanWentylatorWilgoci = HIGH;
        expander_0x22.digitalWrite(wentylatorWilgoci,stanWentylatorWilgoci);      
        delay(2000);
        stanWentylatorWilgoci = LOW;
        expander_0x22.digitalWrite(wentylatorWilgoci,stanWentylatorWilgoci);      
        zapamietanyCzas=aktualnyCzas;   
        }                             
                      
        expander_0x22.digitalWrite(generatorWilgoci,HIGH);                         //a generator wilgoci wlacza sie i dziala non stop az while jest prawdziwy
        
        if(expander_0x21.digitalRead(przyciskUstawieniaParametrow)==LOW){
           sygnalBoozer();
                                                          //Serial.println("przycisk ustawienia parametrow zostal nacisniety");//gdybym w petli chcial zmienic parametry zadane
           delay(1500);
           UstawianieParametrow();
          }else if(expander_0x21.digitalRead(przyciskAktTempWilg)== LOW){
           sygnalBoozer ();  
                                                          //Serial.println("weszlismy do warunku if  nacisniecie przycisku akt temp wilg pojemnika"); // NACISNIECIE PRZYCISKU AKTUALNYCH WARUNKOW W GROWBOX
           delay(1500);
           AktualneParametryGrowbox();
          }
      }   
   lcd.noDisplay(); 
  }  
//............................................................................................................................................................................................  
 
 if(CO2_pojemnika>(maxCO2)){                                       //CO2 POJEMNIKA JEST WIESZE NIZ maxCO2 ZADANE


      
    lcd.display();                      //musimy oczyscic ekran, bo byc moze jakis inny program cos na nim zostawi. Nie mozemy jednak zrobic tego w while poniwaz funkcja clear()
    lcd.clear();                        //bedzie powodowac miganie ekranu. CLEAR NIE MOZE BYC W SRODKU WHILE
    
   while(CO2_pojemnika>maxCO2){  

         dwutlenekWegla(); 
    
         analogWrite(jasnoscLCD,50); //moj zamysl - kiedy dziala while to ekranik swieci sie i widze zmiane parametrow
         lcd.display();
         lcd.setCursor(0,0);
         lcd.print("CO2poj>CO2max");

 int roznica;                                                 //zmienna potrzebna po to aby wybrnac z problemu pozostawania zera na ekranie kiedy roznica miedzy wartosciami spadnie ponizej 10
                                                             //czyli bylo na przyklad 11 potem 10 potem jest 90 zamiast 9
                                                            //nie mozna uzyc lcd.clear() poniewaz czyszczenie ekranu  tak czeste powoduje slabe wyswietlanie obrazu
         roznica = (CO2_pojemnika - maxCO2);
                                                          //Serial.println(roznica);
         if(roznica>=10){
         lcd.setCursor(0,1);
         lcd.print(roznica);
         }else{
         lcd.setCursor(0,1);
         lcd.print("00");
         lcd.setCursor(1,1);
         lcd.print(roznica);
         }
         expander_0x22.digitalWrite(generatorWilgoci,HIGH); //wlacza sie generator wilgoci i dziala non stop az while jest prawdziwy
         delay(10000);


         
         /* w tej sytuacji postepowanie bedzie podobne jak przy utracie wilgotnosci, poniewaz kiedy bede wietrzyl samym powietrzem pozbede sie wilgoci, roznica bedzie polegac tylko na tym
         ze czesciej zapuszcze wentylator, nie raz na 10 sekund a moze raz na 5 sekund...zobaczymy*/              
         aktualnyCzas=millis();
        if((aktualnyCzas - zapamietanyCzas)>=5000UL){                       //dzieki temu zalozeniu wentylatorWilgoci powinien wlaczac sie i wylaczac co 5 sekund
        stanWentylatorWilgoci = HIGH;
        expander_0x22.digitalWrite(wentylatorWilgoci,stanWentylatorWilgoci);      // na 4 sekundy
        delay(6000);
        stanWentylatorWilgoci = LOW;
        expander_0x22.digitalWrite(wentylatorWilgoci,stanWentylatorWilgoci);      
        zapamietanyCzas=aktualnyCzas;   
        }                             
        
         
         expander_0x22.digitalWrite(generatorWilgoci,HIGH);                         //wlacza sie generator wilgoci i dziala non stop az while jest prawdziwy
                    

         if(expander_0x21.digitalRead(przyciskUstawieniaParametrow)==LOW){
         sygnalBoozer();
                                                                //Serial.println("przycisk ustawienia parametrow zostal nacisniety");
         delay(1500);
         UstawianieParametrow();
         }else if(expander_0x21.digitalRead(przyciskAktTempWilg)== LOW){
          sygnalBoozer ();  
                                                          //Serial.println("weszlismy do warunku if  nacisniecie przycisku akt temp wilg pojemnika");
                                                          // NACISNIECIE PRZYCISKU AKTUALNYCH WARUNKOW W GROWBOX
           delay(1500);
           AktualneParametryGrowbox();
         } 
        
    }   
 }
  //---------------jesli wszystkie parametry GROWBOX sa rowne parametrom zadanym to kod ponizej pozwala zareagowac na nacisniecie przycisku w tej funkcji

  
             if(expander_0x21.digitalRead(przyciskUstawieniaParametrow)==LOW){
                sygnalBoozer();
                                                                //Serial.println("przycisk ustawienia parametrow zostal nacisniety");     //na koncu funkcji wrzucam nacisniecia przyciskow,bo kiedy wszystkie parametry sa ok tez chce moc podejrzec 
                delay(100);
                UstawianieParametrow();
                }
              else if(expander_0x21.digitalRead(przyciskAktTempWilg)== LOW){
                sygnalBoozer();  
                                                                //Serial.println("weszlismy do warunku if  nacisniecie przycisku akt temp wilg pojemnika");
                delay(100);                                              
                AktualneParametryGrowbox();
                } 
                                                                //Serial.println("wychodze z porownania parametrow");
 }
//*********************************************************************************************************************************************************************************************
void wszystkoLOW(){
                                                                  //Serial.println("wszystko LOW");
      analogWrite(jasnoscLCD,0);                                 //dalem tutaj wylaczenie podswietlenia wyswietlacza poniewaz czasami po dostrojeniu sie parametrow Growboxa
                                                                 //i powrocie do funkcji loop, podswietlenie pozostawalo. A wszystkoLOW jest pierwsza funkcja sprawdzana w loopie zatem
                                                                 //wyswietlacz zostanie wygaszony w loopie jesli gdzies tam w innych podprogramach tego nie dopatrzylem

                                                                 
      expander_0x21.digitalWrite(boozer,HIGH);           //ponieważ expander_0x21 daje maly prad na plusie i duzy na minusie to boozer jest sterowany minusem zatem stan WYSOKI nie dziala, NISKI dziala
      expander_0x21.digitalWrite(chlodzeniePeltier,HIGH); //przekaźnik PELTIER sterowany minusem zatem wyłaczamy go stanem wysokim 

      
      expander_0x22.digitalWrite(wentylatorWilgoci,LOW);//poniewaz wentylator musi byc sterowany stanem WYSOKIM bo stan ten idzie na tranzystory, zatem przy wentylatorze stan WYSOKI dziala, NISKI nie dziala
      expander_0x22.digitalWrite(generatorWilgoci,LOW);
      expander_0x22.digitalWrite(suszarkaPrzekaznik,HIGH);//przekaźnik suszarki sterowany minusem zatem wyłaczamy go stanem wysokim  

}
//*********************************************************************************************************************************************************************************************

void czas(const RtcDateTime& dt)                                //FUNKCJA ZWRACA CZAS I DATE
{
                                                                   // Serial.println("weszlismy do funkcji czas");


            dt.Month()
            dt.Day()
            dt.Year()
            dt.Hour()
            dt.Minute()
            dt.Second()


    
    godzina = dt.Hour();    
    minuta=dt.Minute();

}
//*********************************************************************************************************************************************************************************************
void swiatloLED(){                                            //FUNKCJA WLACZAJACA DIODY LED OSWIETLENIE GROWBOX 
                                                                //Serial.println("wszedlem do funkcji swiatloLED"); 

         czas(Rtc.GetDateTime());                                    //pobranie czasu systemowego na poczatku bo np. swiatloLED() go potrzebuje
                                                                       //Serial.print("godzina: ");
                                                                       //Serial.println(godzina);    

          odczyt_fotorezystor=analogRead(fotorezystor);                //sprawdzam poziom swiatla
//                                                                      Serial.print("odczyt z fotorezystora: ");
//                                                                      Serial.println(odczyt_fotorezystor); 
          
                       
                                                               
          if(odczyt_fotorezystor<190){   
          /* Zatrzymajmy sie tutaj na chwile :-) bo tu mi kopara opadła, to jest przykład jak programista o wszystkim musi myslec. Jak pierwszy raz uruchamia się looop i wchodzi
          tutaj do funkcji swiatlo to odczyt z fotorezystora ma np 4 bo jest ciemno, ale jak wchodzi kolejny raz to odczyt ma juz 175, a potem jak wchodzi ponownie to znowu 4...o co
          chodzi???? ano o to, że jak wchodzi pierwszy raz to swiatlo growboxa sie nie pali, jest 4 na fotorezystorze i odpalany jest wschod slonca, ale jak wchodzi kolejny raz
          to growbox juz pali bo jest po wschodzi slonca i swiatla jest na fotorezystorze 175...dlatego prog zadzialania musi byc wiekszy niz prog jaki daja lampki growboxa :-)
          W firmie, trzeba by dać fotorezystor za okno, zeby sprawdzal jakie jest naswietlenie od slonca, a jesli bedzie to budynek bez okien to sterowanie tylko godzina :-)*/                                                    
                                                                                      
             if(godzina>=8 && godzina<18){                     //switlo bedzie sie palilo od 8 do 18 czyli 10 godzin               
                 wschodSlonca();
             }else{
              zachodSlonca();
             }
            }else{                                               
               zachodSlonca();                                        //Serial.println("zadzialal if ELSE");        
            } 
} 
       
 //******************************************************************************************************************************************************************************************
  
float wschodSlonca(){                                               //FUNKCJA IMITUJACA WSCHOD SLONCA PODCZAS WLACZENIA DIOD LED
  
int zapamietanyCzasLed=0;                                                           //zmienna potrzebna do odmierzania czasu

      

        while((stanLed_R + stanLed_G + stanLed_B) < 665){                          //byly rozne wersje tej linijki ale ta dziala, no i wynika z tego, ze 255+255+155==765
            
                                                            //Serial.println("wszedlem do while wschod slonca");
                                                                   
          aktualnyCzas=millis();   
          if(aktualnyCzas - zapamietanyCzasLed>=50UL){                              //kiedy czas zmieni sie o 50 milisekkund
              stanLed_R = stanLed_R + 3;
              stanLed_G = stanLed_G + 1;                                           //pododawaj to do zmiennych poszczegolnych diod, czerwona zapala najszybciej
              stanLed_B = stanLed_B + 2;
                              
            if(stanLed_R>=155){
              stanLed_R=155;        // zmniejszam poziom czerwonego aby swiatło było bardziej zielono niebieskie                                       
              }
            if(stanLed_G>=255){
              stanLed_G=255;                                               
              }
            if(stanLed_B>=255){
              stanLed_B=255;                                               
              }
                                          
             analogWrite(R,stanLed_R);
             analogWrite(G,stanLed_G);     
             analogWrite(B,stanLed_B);  
//                                                       Serial.println("stan LED  R G B wyglada nastepujaco");
//                                                       Serial.println(stanLed_R);
//                                                       Serial.println(stanLed_G);
//                                                       Serial.println(stanLed_B);
//                                                       delay(100);
                                                      
             zapamietanyCzasLed=aktualnyCzas;

            
             } 
                    
      }
       return stanLed_R;
       return stanLed_G;
       return stanLed_B; 
}

//*******************************************************************************************************************************************************************************************
float zachodSlonca(){                                   //FUNKCJA IMITUJACA ZACHOD SLONCA PODCZAS WYLACZENIA DIOD LED

                                                   // Serial.println("wszedlem do funkcji zachod slonca");
    

 int zapamietanyCzasLed=0;                                                    //zmienna potrzebna do odmierzania czasu

    while((stanLed_R>0) || (stanLed_G>0) || (stanLed_B>0)){   //tu jest innna wersja warunku, który jest w funkcji wschod slonca. Byly problemy, kombinowalem z roznymi opcjami     
    aktualnyCzas=millis();                              
                                                   //Serial.println("wszedlem do while zachod slonca");
                                                              
    if(aktualnyCzas - zapamietanyCzasLed>=10L){ 
         stanLed_R = stanLed_R-1;
         stanLed_G = stanLed_G-2;           //pododawaj to do zmiennych poszczegolnych diod, czerwona zapala najszybciej
         stanLed_B = stanLed_B-1;
                              
            if(stanLed_R<=0){
              stanLed_R=0;                                               
              }
            if(stanLed_G<=0){
              stanLed_G=0;                                               
              }
            if(stanLed_B<=0){
              stanLed_B=0;                                               
              }

                            
             analogWrite(R,stanLed_R);
             analogWrite(G,stanLed_G);     
             analogWrite(B,stanLed_B);  
                                                      
             zapamietanyCzasLed=aktualnyCzas;                                               
             }
     }
      return stanLed_R;
      return stanLed_G;
      return stanLed_B; 
}

 //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
         
int  dwutlenekWegla(){                                //FUNKCJA OBSLUGUJACA CZUJNIK SGP 30 ZWRACA POZIOM DWUTLENKU WEGLA
                                                    Serial.println("weszlismy do funkcji dwutlenek wegla");
  

  float temperature = 18; // [°C]
  float humidity = 85; // [%RH]
  sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  if (! sgp.IAQmeasure()) {
    return;}
  if (! sgp.IAQmeasureRaw()) {                               //te wszystkie ify i inne rzeczy musza byc, inaczej nie zwroci tego co chce.Nie wnikam w to co to dokładnie robi, nie ma sensu.
    return;}
    counter++;
  if (counter == 30) {
    counter = 0;
   uint16_t TVOC_base, eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {      
      return;
    }
  }


  CO2_pojemnika=sgp.eCO2;
    delay(300);
                                                 
                                                  Serial.print("moje CO2 wynosi: ");                 
                                                  Serial.println(CO2_pojemnika);
                                                  
  
  int pomiar[10];

  for(int i=0; i<10; i++){
    delay(1000);
    CO2_pojemnika=sgp.eCO2;
    pomiar[i]=CO2_pojemnika;
    Serial.print("pomiar ");                    //TUTAJ PRZYKLAD TABLICY, DZIALA TABLICA ale sama srednia nie dziala poniewaz ten czu
    Serial.print(i);
    Serial.print(" wynosi :");
    Serial.println(pomiar[i]);
    
  }

  return(CO2_pojemnika);                                                //ograniczam sie do wydobycia dwutlenku wegla do mojej zmiennej, ktora bede mogl monitorowac
/* TO CIEKAWE, PAMIETAJ O TYM...RETURN W FUNKCJI ZWRACA JAKAS WARTOSC ALE PROGRAM PO RETURNIE SIE NIE WYKONUJE DALEJ. KIEDY MIALEM RETURNA PRZED PETLA FOR TO PETLA SIE NIE WYKONYWALA
CZYLI RETURN DZIALA TROCHE JAK BREAK TYLKO ZE BREAK WYCHODZI I NIC NIE ZWRACA A RETURN ZWRACA*/

}

//*******************************************************************************************************************************************************************************************
void wlaczenieSuszarki(){          //ZADANIEM PROGRAMU JEST WLACZANIE SUSZARKI ALE TEZ PILNOWANIE BY NIE WLACZALA SIE ZA CZESTO JESLI WYSYPIE SIE CZUJNIK TEMPERATURY
                                  // docelowo program powinien wspolpracowac a Arduino2 ktore bedzie kontrolowac Arduino1
                
    licznik_wlaczen_suszarki++;
//                                Serial.print("licznik wlaczen suszarki wynosi: ");
//                                Serial.println(licznik_wlaczen_suszarki);

      if(licznik_wlaczen_suszarki<=10){         //tyle razy suszarka moze sie wlaczyc w ciagu czasu jaki ustawiony jest w loopie (na ta chwile jest to 1 godzina)           
      expander_0x22.digitalWrite(suszarkaPrzekaznik,LOW);  //wlaczenie przekaznika suszarki, ktory sterowany jest minusem, poniewaz expander na minusie daje odpowiedni prad, a na plusie nie
      delay(2000);                               //suszarka wlacza sie na 3 sekundy
      }else{
        wylaczenieSuszarki();
        licznik_wlaczen_suszarki=11;
      }
      
  
}
void wylaczenieSuszarki(){

      expander_0x22.digitalWrite(suszarkaPrzekaznik,HIGH);
  
}
