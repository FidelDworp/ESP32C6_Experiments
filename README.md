# ESP32C6_Experiments

ESP32C6_TEST.ino = Pin Test voor controller ESP32-C6 gemonteerd op een "Freenove Breakout Board" met ledjes per pin.

Beide types ESP32-C6 controllers kunnen getest worden.
In serial monitor: Typ '30' of '32' voor board versie. Daarna Typ aantal seconden per pin (1-20).
=> Alle pins worden rondom getest.

Dit zijn de twee pin configuraties:

"	30-pin RECHTS	30-pin LINKS	
R12	4	2	L14
R11	5	3	L13
R10	6	16	L12
R09	7	17	L11
R08	0	15	L10
R07	1	23	L09
R06	8	22	L08
R05	10	21	L07
R04	11	20	L06
R03	12	19	L05
R02	13	18	L04
		9	L03"

"	32-pin RECHTS	32-pin LINKS	
R12	4	16	L14
R11	5	17	L13
R10	6	15	L12
R09	7	23	L11
R08	0	22	L10
R07	1	21	L09
R06	8	20	L08
R05	10	19	L07
R04	11	18	L06
R03	2	9	L05
R02	3	13	L03
		12	L02"

